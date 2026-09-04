package CPUSTC.backend.execute

import chisel3._
import chisel3.util._

import CPUSTC.config._
import CPUSTC.config.Execute.IntPortParams
import CPUSTC.config.FunctionUnit._
import CPUSTC.config.OPSource._
import CPUSTC.config.RegisterFile._
import CPUSTC.config.Branch._
import CPUSTC.config.MulOp
import CPUSTC.config.DivOp
import CPUSTC.config.CsrOp
import CPUSTC.config.CtrlFlowInstr._
import CPUSTC.config.MemoryException._
import CPUSTC.config.RenameConfig._
import CPUSTC.backend.branch.BranchMask
import CPUSTC.backend.execute.fu.{ALU, BranchUnit, CounterUnit, CSRUnit, DivUnit, MulUnit, PredictedTargetCompare}
import CPUSTC.config.Consts.CSR_TID
import CPUSTC.backend.{AddressFastMap, AddressTranslationState, CSRDebugState}
import CPUSTC.memory.MemSysConfig

class IntExecutePort(
    params: IntPortParams,
    memSysConfig: MemSysConfig = MemSysConfig()
) extends Module {
    val io = IO(new IntExecutePortIO)

    require(!params.div || params.mul)

    private val specialGroups = Seq(
        params.jmp,
        params.csr || params.system || params.cnt,
        params.mul || params.div
    )
    require(specialGroups.count(identity) <= 1)

    private val supportedFuMaskValue =
        FU_ALU.litValue |
        (if (params.jmp) FU_JMP.litValue else BigInt(0)) |
        (if (params.csr) FU_CSR.litValue else BigInt(0)) |
        (if (params.system) FU_SYS.litValue else BigInt(0)) |
        (if (params.cnt) FU_CNT.litValue else BigInt(0)) |
        (if (params.mul) FU_MUL.litValue else BigInt(0)) |
        (if (params.div) FU_DIV.litValue else BigInt(0))

    io.status.supportedFuMask := supportedFuMaskValue.U(FUC_SZ.W)

    val resolveMask = Mux(
        io.branchUpdate.valid,
        io.branchUpdate.bits.resolveMask,
        0.U(maxBrCount.W)
    )

    val mispredictMask = Mux(
        io.branchUpdate.valid,
        io.branchUpdate.bits.mispredictMask,
        0.U(maxBrCount.W)
    )
    val recoveryMispredict = io.branchUpdate.valid && mispredictMask.orR

    def killedByRecovery(mask: UInt): Bool =
        BranchMask.isKilled(mask, mispredictMask)

    def forwardedOperand(
        enable: Bool,
        psrc: UInt,
        original: UInt
    ): (UInt, Vec[Bool]) = {
        val hits = VecInit(io.fastForward.map { forward =>
            enable &&
            psrc =/= 0.U &&
            forward.valid &&
            forward.bits.pdest === psrc
        })

        when(io.in.valid && enable) {
            assert(PopCount(hits) <= 1.U)
        }
        (
            Mux(
                hits.asUInt.orR,
                Mux1H(hits, io.fastForward.map(_.bits.data)),
                original
            ),
            hits
        )
    }

    for (forward <- io.fastForward) {
        when(forward.valid) {
            assert(forward.bits.pdest =/= 0.U)
        }
    }

    val (src1Data, src1ForwardHits) = forwardedOperand(
        io.in.bits.uop.reg.lsrc1Valid,
        io.in.bits.uop.reg.psrc1,
        io.in.bits.src1Data
    )
    val (src2Data, _) = forwardedOperand(
        io.in.bits.uop.reg.lsrc2Valid,
        io.in.bits.uop.reg.psrc2,
        io.in.bits.src2Data
    )

    val op1 = LookupTreeDefault(
        io.in.bits.ctrl.op1Sel,
        0.U(dataWidth.W),
        Seq(
            OP1_RS1  -> src1Data,
            OP1_ZERO -> 0.U(dataWidth.W),
            OP1_PC   -> io.in.bits.uop.meta.pc
        )
    )

    val op2 = LookupTreeDefault(
        io.in.bits.ctrl.op2Sel,
        0.U(dataWidth.W),
        Seq(
            OP2_RS2  -> src2Data,
            OP2_IMM  -> io.in.bits.uop.imm,
            OP2_ZERO -> 0.U(dataWidth.W),
            OP2_NTPC -> 4.U(dataWidth.W)
        )
    )

    val isAlu = io.in.bits.uop.ctrl.fuType === FU_ALU
    val isJmp = io.in.bits.uop.ctrl.fuType === FU_JMP
    val isCnt = io.in.bits.uop.ctrl.fuType === FU_CNT
    val isCsr = io.in.bits.uop.ctrl.fuType === FU_CSR
    val isSys = io.in.bits.uop.ctrl.fuType === FU_SYS
    val isMul = io.in.bits.uop.ctrl.fuType === FU_MUL
    val isDiv = io.in.bits.uop.ctrl.fuType === FU_DIV

    val alu = Module(new ALU)

    alu.io.fn  := io.in.bits.ctrl.fuOp
    alu.io.op1 := op1
    alu.io.op2 := op2

    val cntResult = WireDefault(0.U(dataWidth.W))

    if (params.cnt) {
        val cnt = Module(new CounterUnit)

        cnt.io.valid        := io.in.valid && isCnt
        cnt.io.fn           := io.in.bits.ctrl.fuOp
        cnt.io.counterValue := io.counterValue

        cntResult := cnt.io.result
    }

    val aluResult = Wire(new ExecuteResult)

    aluResult.robPtr := io.in.bits.uop.robPtr
    aluResult.pdest  := io.in.bits.uop.reg.pdest
    aluResult.rfWen  := io.in.bits.uop.reg.rfWen
    aluResult.data   := Mux(isCnt, cntResult, alu.io.result)
    aluResult.brMask :=
        BranchMask.clearResolved(
            io.in.bits.uop.spec.brMask,
            resolveMask
        )
    aluResult.exceptionValid := false.B
    aluResult.exceptionCause := EXC_NONE
    aluResult.exceptionBadvValid := false.B
    aluResult.exceptionBadv := 0.U

    val branch = Module(new BranchUnit)

    branch.io.brType := io.in.bits.ctrl.brType
    branch.io.pc     := io.in.bits.uop.meta.pc
    branch.io.src1   := src1Data
    branch.io.imm    := io.in.bits.uop.imm
    branch.io.cmp    := alu.io.cmp

    val isControlFlow = isJmp &&
        io.in.bits.ctrl.brType =/= BR_N &&
        io.in.bits.uop.br.cfiType =/= CFI_X

    val directionWrong =
        io.in.bits.uop.meta.predTaken =/= branch.io.actualTaken

    val isJirl = io.in.bits.uop.br.cfiType === CFI_JIRL
    val jirlTargetMatches = PredictedTargetCompare.matchesForwardedBase(
        io.in.bits.uop.meta.predTarget,
        io.in.bits.src1Data,
        src1ForwardHits,
        io.fastForward.map(_.bits.data)
    )
    val predictedTargetMatches = Mux(
        isJirl,
        jirlTargetMatches,
        io.in.bits.uop.meta.predTarget === branch.io.branchTarget
    )

    val targetWrong =
        branch.io.actualTaken &&
        io.in.bits.uop.meta.predTaken &&
        (
            !io.in.bits.uop.meta.predTargetValid ||
            !predictedTargetMatches
        )
    val branchMispredict = isControlFlow && (directionWrong || targetWrong)

    aluResult.branchResolved := isControlFlow

    val resultValid = RegInit(false.B)
    val resultBits  = Reg(new ExecuteResult)

    // Writeback already receives resultBits on io.result.bits.  Export only
    // the pre-recovery validity needed by the ROB; keeping this tap one bit
    // wide avoids duplicating the execute payload and its placement cone.
    val robRawValid = WireDefault(resultValid && io.result.ready)
    io.robRawValid := robRawValid

    val resultKilled =
        resultValid &&
        killedByRecovery(resultBits.brMask)

    val resultNewMask =
        BranchMask.clearResolved(
            resultBits.brMask,
            resolveMask
        )

    val resultCanAccept =
        !resultValid ||
        io.result.ready ||
        resultKilled

    val resultCapacity =
        !resultValid ||
        io.result.ready

    val executeCanAccept = Wire(Bool())
    val inputCanAccept   = Wire(Bool())

    val inputKilled = killedByRecovery(io.in.bits.uop.spec.brMask)

    val isCounterRead = isCnt || (isCsr && io.in.bits.ctrl.fuOp === CsrOp.CNTID)
    io.counterDebug.valid := io.in.fire && isCounterRead && !inputKilled &&
        !recoveryMispredict && !io.flush
    io.counterDebug.bits.robPtr := io.in.bits.uop.robPtr
    io.counterDebug.bits.value := io.counterValue

    val csrReqReady = WireDefault(false.B)
    val csrRespValid = WireDefault(false.B)
    val csrRespBits = WireDefault(0.U.asTypeOf(new ExecuteResult))
    val csrRespReady = WireDefault(false.B)

    io.csrInterruptPending := false.B
    io.csrArchRedirect := 0.U.asTypeOf(io.csrArchRedirect)
    io.csrIdle := false.B
    io.csrIdleResumePc := 0.U
    io.csrBusy := false.B
    io.csrCommitDone := false.B
    io.csrResultFire := false.B
    io.csrAddressState := 0.U.asTypeOf(new AddressTranslationState)
    io.csrAddressState.crmd.da := 1.U
    io.csrFastAddressMap := 0.U.asTypeOf(new AddressFastMap)
    io.csrDebugState := 0.U.asTypeOf(new CSRDebugState)
    io.csrDebugErtn := false.B
    io.csrDebugInterrupt := 0.U
    io.llbitClear := false.B
    io.sysHeadReq := 0.U.asTypeOf(io.sysHeadReq)
    io.sysMemCmd.valid := false.B
    io.sysMemCmd.bits := 0.U.asTypeOf(io.sysMemCmd.bits)
    io.sysMemResp.ready := false.B

    if (params.csr) {
        val csr = Module(new CSRUnit(memSysConfig))

        csr.io.flush := io.flush
        csr.io.commit := io.csrCommit
        csr.io.headGrant := io.sysHeadGrant
        csr.io.exception := io.csrException
        csr.io.hardwareInterrupt := io.hardwareInterrupt
        csr.io.llbitValue := io.llbitValue

        csr.io.req.valid :=
            io.in.valid &&
            (isCsr || isSys) &&
            !inputKilled &&
            !recoveryMispredict &&
            !io.flush
        csr.io.req.bits.system  := isSys
        csr.io.req.bits.op      := io.in.bits.ctrl.fuOp
        csr.io.req.bits.robPtr  := io.in.bits.uop.robPtr
        csr.io.req.bits.pc      := io.in.bits.uop.meta.pc
        csr.io.req.bits.csrAddr := Mux(
            io.in.bits.ctrl.fuOp === CsrOp.CNTID,
            CSR_TID,
            io.in.bits.uop.imm(13, 0)
        )
        csr.io.req.bits.csrData := src1Data
        csr.io.req.bits.csrMask := src2Data
        csr.io.req.bits.sysImm  := io.in.bits.uop.imm
        csr.io.req.bits.auxOp   := io.in.bits.uop.meta.sysAux
        csr.io.req.bits.pdest   := io.in.bits.uop.reg.pdest
        csr.io.req.bits.rfWen   := io.in.bits.uop.reg.rfWen
        csr.io.req.bits.brMask  := BranchMask.clearResolved(
            io.in.bits.uop.spec.brMask,
            resolveMask
        )

        csr.io.resp.ready := csrRespReady
        csr.io.sysMemResp.valid := io.sysMemResp.valid
        csr.io.sysMemResp.bits := io.sysMemResp.bits
        io.sysMemResp.ready := csr.io.sysMemResp.ready
        io.sysMemCmd.valid := csr.io.sysMemCmd.valid
        io.sysMemCmd.bits := csr.io.sysMemCmd.bits
        csr.io.sysMemCmd.ready := io.sysMemCmd.ready
        csrReqReady := csr.io.req.ready
        csrRespValid := csr.io.resp.valid
        csrRespBits := csr.io.resp.bits

        io.csrInterruptPending := csr.io.interruptPending
        io.csrArchRedirect := csr.io.archRedirect
        io.csrIdle := csr.io.idle
        io.csrIdleResumePc := csr.io.idleResumePc
        io.csrBusy := csr.io.busy
        io.csrCommitDone := csr.io.commitDone
        io.csrResultFire := csr.io.resp.fire
        io.csrAddressState := csr.io.addressState
        io.csrFastAddressMap := csr.io.fastAddressMap
        io.csrDebugState := csr.io.debugState
        io.csrDebugErtn := csr.io.debugErtn
        io.csrDebugInterrupt := csr.io.debugInterrupt
        io.llbitClear := csr.io.llbitClear
        io.sysHeadReq := csr.io.headReq
    }

    val outputValid = WireDefault(
        resultValid &&
        !io.flush &&
        !resultKilled
    )
    val outputBits = WireDefault(resultBits)
    outputBits.brMask := resultNewMask

    // A killed producer may still wake IntIQ: every matching consumer is
    // younger and is killed by the same recovery. Keeping this identity raw
    // prevents recovery from entering the IQ wakeup path.
    val rawWakeupIdentityValid = WireDefault(
        resultValid && io.result.ready && !io.flush
    )
    val rawWakeupIdentityBits = WireDefault(resultBits)

    // A killed producer may remain on the one-cycle forwarding bus: every
    // consumer of its physical destination is younger and is killed by the
    // same recovery. Keeping this valid independent of resultKilled prevents
    // branch recovery from entering the execute forwarding data path.
    val earlyResultValid = WireDefault(resultValid && !io.flush)
    val earlyResultBits = WireDefault(resultBits)

    io.result.valid := outputValid
    io.result.bits  := outputBits

    val isSupported =
        isAlu ||
        (params.jmp.B && isJmp) ||
        (params.csr.B && isCsr) ||
        (params.system.B && isSys) ||
        (params.cnt.B && isCnt) ||
        (params.mul.B && isMul) ||
        (params.div.B && isDiv)

    io.in.ready :=
        isSupported &&
        inputCanAccept &&
        !recoveryMispredict &&
        !io.flush

    if (params.mul) {
        val stage0Valid   = RegInit(false.B)
        val stage0Bits    = Reg(new ExecuteResult)
        val stage0IsMul   = Reg(Bool())

        val stage0Killed =
            stage0Valid &&
            killedByRecovery(stage0Bits.brMask)

        val stage0NewMask =
            BranchMask.clearResolved(
                stage0Bits.brMask,
                resolveMask
            )

        val stage0CanAccept =
            !stage0Valid ||
            resultCanAccept ||
            stage0Killed

        val stage0Capacity =
            !stage0Valid ||
            resultCapacity

        executeCanAccept := stage0Capacity

        val mul = Module(new MulUnit)
        mul.io.load := io.in.fire && isMul && !inputKilled
        mul.io.fn   := io.in.bits.ctrl.fuOp
        mul.io.src1 := src1Data
        mul.io.src2 := src2Data

        if (params.div) {
            val div = Module(new DivUnit)
            val divMetaValid = RegInit(false.B)
            val divMetaBits  = Reg(new ExecuteResult)

            val divMetaKilled =
                divMetaValid &&
                killedByRecovery(divMetaBits.brMask)

            val divMetaNewMask =
                BranchMask.clearResolved(
                    divMetaBits.brMask,
                    resolveMask
                )

            val divKill = io.flush || divMetaKilled

            div.io.kill := divKill
            div.io.req.valid :=
                io.in.valid &&
                isDiv &&
                !io.flush &&
                !inputKilled &&
                !recoveryMispredict &&
                !divMetaValid
            div.io.req.bits.fn       := io.in.bits.ctrl.fuOp
            div.io.req.bits.dividend := src1Data
            div.io.req.bits.divisor  := src2Data

            val fixedInputCanAccept =
                stage0Capacity &&
                !div.io.respPending

            val divInputCanAccept =
                !div.io.busy &&
                !divMetaValid

            inputCanAccept := Mux(
                isDiv,
                divInputCanAccept,
                fixedInputCanAccept
            )

            val fixedEarlyValid = resultValid && !io.flush
            val fixedOutputValid = fixedEarlyValid && !resultKilled

            val divOutputValid =
                div.io.resp.valid &&
                divMetaValid &&
                !divMetaKilled &&
                !io.flush &&
                !resultValid &&
                !stage0Valid

            val divOutputBits = WireDefault(divMetaBits)
            divOutputBits.data   := div.io.resp.bits
            divOutputBits.brMask := divMetaNewMask

            // A recovery-killed fixed or DIV result has no architectural
            // side effects, but its registered identity may still complete
            // the now-unreachable ROB slot.  Keep that identity on the shared
            // bits bus while normal outputValid remains recovery-filtered.
            val divIdentityAvailable =
                div.io.respPending &&
                divMetaValid &&
                !resultValid &&
                !stage0Valid

            // Arbitrate only on registered occupancy, never recovery status.
            // A killed fixed identity completes its unreachable ROB slot while
            // a simultaneous surviving DIV waits one cycle in sResponse.
            val selectFixedIdentity = resultValid
            val fixedRobRawValid = resultValid && io.result.ready
            val divRobRawValid =
                divIdentityAvailable &&
                io.result.ready

            robRawValid := fixedRobRawValid || divRobRawValid
            outputValid := fixedOutputValid || divOutputValid
            outputBits := Mux(
                selectFixedIdentity,
                resultBits,
                divOutputBits
            )
            outputBits.brMask := Mux(
                selectFixedIdentity,
                resultNewMask,
                divMetaNewMask
            )
            rawWakeupIdentityValid := robRawValid && !io.flush
            rawWakeupIdentityBits := outputBits

            div.io.resp.ready :=
                divMetaValid &&
                !resultValid &&
                !stage0Valid &&
                io.result.ready

            when(io.flush || divMetaKilled) {
                divMetaValid := false.B
            }.otherwise {
                when(div.io.req.fire) {
                    divMetaValid       := true.B
                    divMetaBits.robPtr := io.in.bits.uop.robPtr
                    divMetaBits.pdest  := io.in.bits.uop.reg.pdest
                    divMetaBits.rfWen  := io.in.bits.uop.reg.rfWen
                    divMetaBits.data   := 0.U
                    divMetaBits.brMask := BranchMask.clearResolved(
                        io.in.bits.uop.spec.brMask,
                        resolveMask
                    )
                    divMetaBits.exceptionValid := false.B
                    divMetaBits.exceptionCause := EXC_NONE
                    divMetaBits.exceptionBadvValid := false.B
                    divMetaBits.exceptionBadv := 0.U
                    divMetaBits.branchResolved := false.B
                }.elsewhen(div.io.resp.fire) {
                    divMetaValid := false.B
                }.elsewhen(io.branchUpdate.valid && divMetaValid) {
                    divMetaBits.brMask := divMetaNewMask
                }
            }

            val fixedReadyMask = Mux(
                fixedInputCanAccept &&
                !io.flush,
                (FU_ALU.litValue | FU_MUL.litValue).U(FUC_SZ.W),
                0.U(FUC_SZ.W)
            )

            val divReadyMask = Mux(
                divInputCanAccept &&
                !io.flush,
                FU_DIV,
                0.U(FUC_SZ.W)
            )

            io.status.readyFuMask := fixedReadyMask | divReadyMask

            when(div.io.req.fire) {
                assert(isDiv)
                assert(params.div.B)
                assert(io.in.fire)
            }

            when(div.io.resp.valid) {
                assert(divMetaValid)
            }

            when(div.io.respPending) {
                assert(!io.in.fire)
            }

            when(divMetaKilled) {
                assert(!divOutputValid)
                assert(!div.io.resp.fire)
            }

            when(resultKilled) {
                assert(!fixedOutputValid)
                when(io.result.valid) {
                    assert(divOutputValid)
                }
            }
        } else {
            inputCanAccept := stage0CanAccept
        }

        when(io.flush) {
            stage0Valid := false.B
            resultValid := false.B
        }.otherwise {
            when(resultCanAccept) {
                resultValid := stage0Valid && !stage0Killed
                when(stage0Valid && !stage0Killed) {
                    resultBits := stage0Bits
                    resultBits.data := Mux(stage0IsMul, mul.io.result, stage0Bits.data)
                    resultBits.brMask := stage0NewMask
                }
            }.elsewhen(io.branchUpdate.valid) {
                resultBits.brMask := resultNewMask
            }

            when(stage0CanAccept) {
                val fixedInputFire = io.in.fire && !isDiv && !inputKilled
                stage0Valid := fixedInputFire
                when(fixedInputFire) {
                    stage0Bits := aluResult
                    stage0Bits.brMask := BranchMask.clearResolved(
                        io.in.bits.uop.spec.brMask,
                        resolveMask
                    )
                    stage0IsMul := isMul
                }
            }.elsewhen(io.branchUpdate.valid) {
                stage0Bits.brMask := stage0NewMask
            }
        }

        val stage0StalledLastCycle = RegNext(
            stage0Valid && !stage0CanAccept,
            false.B
        )
        val stage0BitsLastCycle = RegNext(stage0Bits.asUInt)
        val stage0IsMulLastCycle = RegNext(stage0IsMul)
        val stage0ControlChangedLastCycle = RegNext(
            io.flush || io.branchUpdate.valid,
            false.B
        )

        when(
            stage0StalledLastCycle &&
            !io.flush &&
            !io.branchUpdate.valid &&
            !stage0ControlChangedLastCycle
        ) {
            assert(stage0Valid)
            assert(stage0Bits.asUInt === stage0BitsLastCycle)
            assert(stage0IsMul === stage0IsMulLastCycle)
        }

    } else {
        if (params.csr) {
            val fixedOutputValid =
                resultValid && !io.flush && !resultKilled
            val csrOutputValid = csrRespValid && !io.flush
            val fixedEarlyValid = resultValid && !io.flush
            val fixedIdentityPresent = fixedEarlyValid
            val selectCsrResponse =
                !fixedIdentityPresent && csrOutputValid

            outputValid := fixedOutputValid || selectCsrResponse
            // A fixed result owns this physical port even when recovery has
            // killed its architectural valid. Its raw identity still
            // completes the unreachable ROB slot, while a simultaneous CSR
            // response remains held in CSRUnit for a later cycle.
            outputBits := Mux(fixedIdentityPresent, resultBits, csrRespBits)
            outputBits.brMask := Mux(
                fixedIdentityPresent,
                resultNewMask,
                csrRespBits.brMask
            )
            earlyResultValid := fixedEarlyValid
            earlyResultBits := resultBits
            rawWakeupIdentityValid :=
                (fixedIdentityPresent || selectCsrResponse) &&
                io.result.ready &&
                !io.flush
            rawWakeupIdentityBits := Mux(
                fixedIdentityPresent,
                resultBits,
                csrRespBits
            )
            csrRespReady := !fixedIdentityPresent && io.result.ready

            val fixedCanAccept = resultCapacity
            executeCanAccept := fixedCanAccept
            inputCanAccept := Mux(isCsr || isSys, csrReqReady, fixedCanAccept)

            val expectedFixedOutput = WireInit(resultBits)
            expectedFixedOutput.brMask := resultNewMask
            when(csrRespValid && fixedIdentityPresent) {
                assert(!csrRespReady)
                assert(outputBits.asUInt === expectedFixedOutput.asUInt)
            }

            val csrRespStalledLastCycle = RegNext(
                csrRespValid && !csrRespReady,
                false.B
            )
            val csrRespBitsLastCycle = RegNext(csrRespBits.asUInt)
            val csrRespControlLastCycle = RegNext(io.flush, false.B)
            when(
                csrRespStalledLastCycle &&
                !io.flush &&
                !csrRespControlLastCycle
            ) {
                assert(csrRespValid)
                assert(csrRespBits.asUInt === csrRespBitsLastCycle)
            }

            when(
                io.in.valid &&
                (isCsr || isSys) &&
                !inputKilled &&
                !recoveryMispredict &&
                !io.flush
            ) {
                assert(
                    csrReqReady,
                    "ROB-head CSR/SYS did not enter its independent slow path"
                )
            }

            when(io.flush) {
                resultValid := false.B
            }.elsewhen(resultCanAccept) {
                val inputFire =
                    io.in.fire && !isCsr && !isSys && !inputKilled
                resultValid := inputFire
                when(inputFire) {
                    resultBits := aluResult
                }
            }.elsewhen(io.branchUpdate.valid) {
                resultBits.brMask := resultNewMask
            }
        } else {
            executeCanAccept := resultCapacity
            inputCanAccept   := resultCapacity

            when(io.flush) {
                resultValid := false.B
            }.elsewhen(resultCanAccept) {
                val inputFire = io.in.fire && !inputKilled
                resultValid := inputFire
                when(inputFire) {
                    resultBits := aluResult
                }
            }.elsewhen(io.branchUpdate.valid) {
                resultBits.brMask := resultNewMask
            }
        }
    }

    io.recoveryIndependentReady :=
        isSupported && inputCanAccept && !io.flush

    io.rawWakeup.valid :=
        rawWakeupIdentityValid &&
        rawWakeupIdentityBits.rfWen &&
        !rawWakeupIdentityBits.exceptionValid &&
        rawWakeupIdentityBits.pdest =/= 0.U
    io.rawWakeup.bits := rawWakeupIdentityBits.pdest

    io.earlyForward.valid :=
        earlyResultValid &&
        earlyResultBits.rfWen &&
        !earlyResultBits.exceptionValid &&
        earlyResultBits.pdest =/= 0.U
    io.earlyForward.bits.pdest := earlyResultBits.pdest
    io.earlyForward.bits.data  := earlyResultBits.data

    // Operand selection is allowed to observe a registered producer during a
    // flush. The consumer cannot fire in that cycle, but keeping this identity
    // independent of flush prevents recovery from crossing the ALU/branch
    // payload cone through an otherwise invalid input.
    io.operandForward.valid :=
        resultValid &&
        resultBits.rfWen &&
        !resultBits.exceptionValid &&
        resultBits.pdest =/= 0.U
    io.operandForward.bits.pdest := resultBits.pdest
    io.operandForward.bits.data  := resultBits.data

    if (!params.div) {
        val staticFuMask =
            FU_ALU.litValue |
            (if (params.jmp) FU_JMP.litValue else BigInt(0)) |
            (if (params.csr) FU_CSR.litValue else BigInt(0)) |
            (if (params.system) FU_SYS.litValue else BigInt(0)) |
            (if (params.cnt) FU_CNT.litValue else BigInt(0)) |
            (if (params.mul) FU_MUL.litValue else BigInt(0))

        io.status.readyFuMask := Mux(
            executeCanAccept && !io.flush,
            staticFuMask.U(FUC_SZ.W),
            0.U(FUC_SZ.W)
        )
    }

    io.branchResolve.valid :=
        isControlFlow &&
        io.in.fire &&
        !inputKilled

    io.branchResolve.bits := 0.U.asTypeOf(new BranchResolve)

    io.branchResolve.bits.robPtr    := io.in.bits.uop.robPtr
    io.branchResolve.bits.ftqPtr    := io.in.bits.uop.meta.ftqPtr
    io.branchResolve.bits.ftqOffset := io.in.bits.uop.meta.ftqOffset
    io.branchResolve.bits.pc        := io.in.bits.uop.meta.pc

    io.branchResolve.bits.actualTaken  := branch.io.actualTaken
    io.branchResolve.bits.branchTarget := branch.io.branchTarget
    io.branchResolve.bits.actualNextPc := branch.io.actualNextPc
    io.branchResolve.bits.predTaken    := io.in.bits.uop.meta.predTaken
    io.branchResolve.bits.mispredict   := branchMispredict
    io.branchResolve.bits.directionWrong := directionWrong
    io.branchResolve.bits.targetWrong    := targetWrong

    io.branchResolve.bits.cfiType := io.in.bits.uop.br.cfiType
    io.branchResolve.bits.isCall  := io.in.bits.uop.br.isCall
    io.branchResolve.bits.isRet   := io.in.bits.uop.br.isRet

    io.branchResolve.bits.brMask := aluResult.brMask
    io.branchResolve.bits.brTag  := io.in.bits.uop.spec.brTag

    when(io.in.fire && isJmp) {
        assert(
            (io.in.bits.ctrl.brType =/= BR_N) ===
            (io.in.bits.uop.br.cfiType =/= CFI_X)
        )
    }

    when(io.branchResolve.valid) {
        assert(params.jmp.B)
        assert(io.branchResolve.bits.cfiType =/= CFI_X)
    }

    when(io.in.fire && inputKilled) {
        assert(!io.branchResolve.valid)
    }

    when(recoveryMispredict && !io.flush) {
        assert(!io.in.fire)
        assert(!io.branchResolve.valid)
    }

    when(!recoveryMispredict) {
        assert(io.recoveryIndependentReady === io.in.ready)
        assert(
            io.rawWakeup.valid ===
                (io.result.fire &&
                    io.result.bits.rfWen &&
                    !io.result.bits.exceptionValid &&
                    io.result.bits.pdest =/= 0.U)
        )
    }

    when(io.rawWakeup.valid) {
        assert(io.rawWakeup.bits =/= 0.U)
    }

    when(io.earlyForward.valid) {
        assert(io.earlyForward.bits.pdest =/= 0.U)
    }

    when(io.in.valid && !io.flush) {
        assert(isSupported)
        assert(PopCount(io.in.bits.uop.ctrl.fuType) === 1.U)
    }

    when(io.in.fire && io.in.bits.uop.reg.rfWen) {
        assert(io.in.bits.uop.reg.ldestValid)
        assert(io.in.bits.uop.reg.pdest =/= 0.U)
    }

    when(io.in.fire && isCnt) {
        assert(params.cnt.B)
        assert(io.in.bits.uop.reg.rfWen)
        assert(io.in.bits.uop.reg.ldestValid)
        assert(io.in.bits.uop.reg.pdest =/= 0.U)
        assert(!io.in.bits.uop.reg.lsrc1Valid)
        assert(!io.in.bits.uop.reg.lsrc2Valid)
        assert(io.in.bits.ctrl.brType === BR_N)
    }

    when(io.in.fire && isCsr) {
        assert(params.csr.B)
        assert(!io.in.bits.uop.spec.brMask.orR)
        assert(io.in.bits.ctrl.brType === BR_N)
    }

    when(io.in.fire && isMul) {
        assert(params.mul.B)
        assert(io.in.bits.uop.reg.rfWen)
        assert(io.in.bits.uop.reg.ldestValid)
        assert(io.in.bits.uop.reg.pdest =/= 0.U)
        assert(io.in.bits.uop.reg.lsrc1Valid)
        assert(io.in.bits.uop.reg.lsrc2Valid)
        assert(io.in.bits.ctrl.op1Sel === OP1_RS1)
        assert(io.in.bits.ctrl.op2Sel === OP2_RS2)
        assert(io.in.bits.ctrl.brType === BR_N)
        assert(
            io.in.bits.ctrl.fuOp === MulOp.MUL ||
            io.in.bits.ctrl.fuOp === MulOp.MULH ||
            io.in.bits.ctrl.fuOp === MulOp.MULHU
        )
    }

    when(io.in.fire && isDiv) {
        assert(params.div.B)
        assert(io.in.bits.uop.reg.rfWen)
        assert(io.in.bits.uop.reg.ldestValid)
        assert(io.in.bits.uop.reg.pdest =/= 0.U)
        assert(io.in.bits.uop.reg.lsrc1Valid)
        assert(io.in.bits.uop.reg.lsrc2Valid)
        assert(io.in.bits.ctrl.op1Sel === OP1_RS1)
        assert(io.in.bits.ctrl.op2Sel === OP2_RS2)
        assert(io.in.bits.ctrl.brType === BR_N)
        assert(
            io.in.bits.ctrl.fuOp === DivOp.DIV ||
            io.in.bits.ctrl.fuOp === DivOp.MOD ||
            io.in.bits.ctrl.fuOp === DivOp.DIVU ||
            io.in.bits.ctrl.fuOp === DivOp.MODU
        )
    }

    when(io.result.valid) {
        assert(!io.result.bits.rfWen || io.result.bits.pdest =/= 0.U)
    }

    when(io.robRawValid) {
        assert(!io.result.bits.exceptionValid)
    }

    val stalledLastCycle = RegNext(
        io.result.valid && !io.result.ready,
        false.B
    )
    val bitsLastCycle = RegNext(io.result.bits.asUInt)
    val controlChangedLastCycle = RegNext(
        io.flush || io.branchUpdate.valid,
        false.B
    )

    when(
        stalledLastCycle &&
        !io.flush &&
        !io.branchUpdate.valid &&
        !controlChangedLastCycle
    ) {
        assert(io.result.valid)
        assert(io.result.bits.asUInt === bitsLastCycle)
    }

    if (!params.div) {
        when(resultKilled) {
            assert(!io.result.valid)
            assert(!io.result.fire)
        }
    }
}
