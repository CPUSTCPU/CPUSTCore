package CPUSTC.backend.regfile

import chisel3._
import chisel3.util._

import CPUSTC.config.Issue._
import CPUSTC.config.FunctionUnit._
import CPUSTC.config.MemIssueOp._
import CPUSTC.config.RegisterFile._
import CPUSTC.config.Execute._
import CPUSTC.config.RenameConfig._
import CPUSTC.config.CtrlFlowInstr.CFI_JIRL
import CPUSTC.config.WritebackConfig._
import CPUSTC.backend.branch.BranchMask
import CPUSTC.decode.FuDecoder
import CPUSTC.backend.execute.{IntExecuteInput, MemExecuteInput}
import CPUSTC.backend.execute.fu.{AddressGenerationUnit, PredictedTargetCompare}

class RegisterRead extends Module {
    val io = IO(new RegisterReadIO)

    require(nis == intNissue + memNissue)
    require(nRead == intNissue * 2 + memNissue)
    require(intPorts.length == intNissue)

    val issuePorts = io.intIssue.toSeq ++ io.memIssue.toSeq

    val headValid = RegInit(VecInit(Seq.fill(nis)(false.B)))
    // P0 only serves fixed-latency ALU/JMP operations and its execute result
    // is never backpressured.  Keep skid entries only for the other ports.
    val tailValid = RegInit(VecInit(Seq.fill(nis - 1)(false.B)))
    val intHeadBits = Reg(Vec(intNissue, new IntExecuteInput))
    val intTailBits = Reg(Vec(intNissue - 1, new IntExecuteInput))
    val memHeadBits = Reg(Vec(memNissue, new MemExecuteInput))
    val memTailBits = Reg(Vec(memNissue, new MemExecuteInput))

    // Load bypasses arrive from the registered Writeback relay. They repair
    // this boundary directly, while the raw DCache response stays local to
    // Writeback and its scheduling/ROB completion paths.
    val loadRelayBypass = io.bypass.slice(nIntWb, nDataWb)

    for (i <- 0 until nLoadWb; j <- i + 1 until nLoadWb) {
        when(loadRelayBypass(i).valid && loadRelayBypass(j).valid) {
            assert(
                loadRelayBypass(i).bits.pdest =/=
                    loadRelayBypass(j).bits.pdest
            )
        }
    }

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

    val headKilled = Wire(Vec(nis, Bool()))
    val tailKilled = Wire(Vec(nis - 1, Bool()))

    for (p <- 0 until intNissue) {
        headKilled(p) :=
            headValid(p) &&
            BranchMask.isKilled(
                intHeadBits(p).uop.spec.brMask,
                mispredictMask
            )
    }

    for (p <- 1 until intNissue) {
        val t = p - 1
        tailKilled(t) :=
            tailValid(t) &&
            BranchMask.isKilled(
                intTailBits(t).uop.spec.brMask,
                mispredictMask
            )
    }

    for (m <- 0 until memNissue) {
        val p = intNissue + m

        headKilled(p) :=
            headValid(p) &&
            BranchMask.isKilled(
                memHeadBits(m).uop.spec.brMask,
                mispredictMask
            )
        tailKilled(p - 1) :=
            tailValid(p - 1) &&
            BranchMask.isKilled(
                memTailBits(m).uop.spec.brMask,
                mispredictMask
            )
    }

    issuePorts(0).ready := !io.flush
    for (p <- 1 until intNissue) {
        issuePorts(p).ready := !tailValid(p - 1) && !io.flush
    }

    for (m <- 0 until memNissue) {
        val p = intNissue + m

        issuePorts(p).ready := !tailValid(p - 1) && !io.flush
    }

    for (p <- 0 until intNissue) {
        io.intExecute(p).valid := headValid(p) && !io.flush
        io.intExecute(p).bits  := intHeadBits(p)

        val headFuMask = Mux(
            headValid(p),
            intHeadBits(p).uop.ctrl.fuType,
            0.U(FUC_SZ.W)
        )
        if (p == 0) {
            io.intBufferedFuMask(p) := headFuMask
        } else {
            val tailFuMask = Mux(
                tailValid(p - 1),
                intTailBits(p - 1).uop.ctrl.fuType,
                0.U(FUC_SZ.W)
            )
            io.intBufferedFuMask(p) := headFuMask | tailFuMask
        }
    }

    val p1FixedIssue =
        issuePorts(1).fire &&
        (issuePorts(1).bits.uop.ctrl.fuType === FU_ALU ||
            issuePorts(1).bits.uop.ctrl.fuType === FU_CNT)
    val p1FixedPromiseValid = RegInit(false.B)
    val p1FixedPromisePdest = Reg(UInt(wpreg.W))
    val p1FixedPromiseRobPtr =
        Reg(chiselTypeOf(issuePorts(1).bits.uop.robPtr))

    when(io.flush || mispredictMask.orR) {
        p1FixedPromiseValid := false.B
    }.otherwise {
        p1FixedPromiseValid := p1FixedIssue
        when(p1FixedIssue) {
            p1FixedPromisePdest := issuePorts(1).bits.uop.reg.pdest
            p1FixedPromiseRobPtr := issuePorts(1).bits.uop.robPtr
        }
    }

    when(p1FixedPromiseValid && !io.flush && !mispredictMask.orR) {
        assert(
            io.intExecute(1).fire,
            "P1 fixed producer did not enter execute on schedule"
        )
        assert(
            io.intExecute(1).bits.uop.reg.pdest === p1FixedPromisePdest
        )
        assert(
            io.intExecute(1).bits.uop.robPtr.asUInt ===
                p1FixedPromiseRobPtr.asUInt
        )
    }

    when(!io.flush && !mispredictMask.orR) {
        assert(
            !tailValid(0),
            "P1 fixed-latency contract must never require the RR tail buffer"
        )
    }

    for (m <- 0 until memNissue) {
        val p = intNissue + m

        io.memExecute(m).valid := headValid(p) && !io.flush
        io.memExecute(m).bits  := memHeadBits(m)
    }

    for (r <- 0 until nRead) {
        io.rfReadReq(r).en   := false.B
        io.rfReadReq(r).addr := 0.U
        io.rfReadReq(r).speculative := false.B
    }

    def readRfOperand(
        payloadEnable: Bool,
        psrc: UInt,
        rfData: UInt
    ): UInt = Mux(!payloadEnable || psrc === 0.U, 0.U, rfData)

    def forwardLoadRelay(
        enable: Bool,
        psrc: UInt,
        current: UInt
    ): (UInt, Bool) = {
        val hits = VecInit((0 until nLoadWb).map { l =>
            enable &&
            psrc =/= 0.U &&
            loadRelayBypass(l).valid &&
            loadRelayBypass(l).bits.pdest === psrc
        })
        val hit = hits.asUInt.orR

        when(enable) {
            assert(PopCount(hits) <= 1.U)
        }

        (
            Mux(hit, Mux1H(hits, loadRelayBypass.map(_.bits.data)), current),
            hit
        )
    }

    def captureFastForward(
        enable: Bool,
        psrc: UInt,
        current: UInt
    ): (UInt, Bool) = {
        val fastBypass = io.bypass.take(nFastIntWb)
        val hits = VecInit(fastBypass.map { bypass =>
            enable &&
            psrc =/= 0.U &&
            bypass.valid &&
            bypass.bits.pdest === psrc
        })
        val hit = hits.asUInt.orR

        when(enable) {
            assert(PopCount(hits) <= 1.U)
        }

        (
            Mux(hit, Mux1H(hits, fastBypass.map(_.bits.data)), current),
            hit
        )
    }

    for (b <- io.bypass) {
        when (b.valid) {
            assert(b.bits.pdest =/= 0.U)
        }
    }

    val intNextBits = Wire(Vec(intNissue, new IntExecuteInput))
    val memNextBits = Wire(Vec(memNissue, new MemExecuteInput))

    intNextBits := 0.U.asTypeOf(intNextBits)
    memNextBits := 0.U.asTypeOf(memNextBits)

    for (p <- 0 until intNissue) {
        val in = io.intIssue(p)
        val r1 = p * 2
        val r2 = p * 2 + 1
        val decoder = Module(new FuDecoder(intPorts(p)))

        decoder.io.uop := in.bits.uop.ctrl.uop

        val src1PayloadEn = in.bits.src1Read
        val src1Accepted = in.fire && src1PayloadEn
        io.rfReadReq(r1).en   := src1Accepted
        io.rfReadReq(r1).addr := Mux(src1PayloadEn, in.bits.uop.reg.psrc1, 0.U)
        io.rfReadReq(r1).speculative := src1Accepted && in.bits.src1FastWakeup

        val src2PayloadEn = in.bits.src2Read
        val src2Accepted = in.fire && src2PayloadEn
        io.rfReadReq(r2).en   := src2Accepted
        io.rfReadReq(r2).addr := Mux(src2PayloadEn, in.bits.uop.reg.psrc2, 0.U)
        io.rfReadReq(r2).speculative := src2Accepted && in.bits.src2FastWakeup

        intNextBits(p).uop.connectFrom(in.bits.uop)
        intNextBits(p).uop.spec.brMask :=
            BranchMask.clearResolved(
                in.bits.uop.spec.brMask,
                resolveMask
            )
        intNextBits(p).ctrl := decoder.io.out.ctrl

        if (p == 0) {
            val predictionAligned =
                io.ftqPredictionReadResp.valid &&
                io.ftqPredictionReadResp.bits.ptr.idx === in.bits.uop.meta.ftqPtr.idx &&
                io.ftqPredictionReadResp.bits.ptr.high === in.bits.uop.meta.ftqPtr.high
            val predictionTaken =
                predictionAligned &&
                io.ftqPredictionReadResp.bits.cfiIdx.valid &&
                io.ftqPredictionReadResp.bits.cfiIdx.bits === in.bits.uop.meta.ftqOffset
            val isJirl = in.bits.uop.br.cfiType === CFI_JIRL
            val encodedPredictedTarget = PredictedTargetCompare.encode(
                io.ftqPredictionReadResp.bits.target,
                in.bits.uop.imm,
                isJirl
            )

            intNextBits(p).uop.meta.predTaken       := predictionTaken
            intNextBits(p).uop.meta.predTargetValid := predictionTaken
            intNextBits(p).uop.meta.predTarget := Mux(
                predictionTaken,
                encodedPredictedTarget,
                0.U
            )

            when(in.fire && (in.bits.uop.ctrl.fuType & FU_JMP).orR) {
                assert(predictionAligned)
            }
        }

        val src1RfData = readRfOperand(
            src1PayloadEn,
            in.bits.uop.reg.psrc1,
            io.rfReadData(r1)
        )

        val src2RfData = readRfOperand(
            src2PayloadEn,
            in.bits.uop.reg.psrc2,
            io.rfReadData(r2)
        )
        // The architectural PRF write is relayed by one cycle. A new
        // dependent can therefore arrive here alongside that write and must
        // see the relay directly rather than the old RF array value.
        val (src1Data, _) = forwardLoadRelay(
            src1PayloadEn,
            in.bits.uop.reg.psrc1,
            src1RfData
        )
        val (src2Data, _) = forwardLoadRelay(
            src2PayloadEn,
            in.bits.uop.reg.psrc2,
            src2RfData
        )
        intNextBits(p).src1Data := src1Data
        intNextBits(p).src2Data := src2Data

        when(in.fire) {
            assert(in.bits.memOp === MEM_X)
            assert(
                decoder.io.out.recognized,
                p"RegisterRead: P${p} cannot decode PC=0x${Hexadecimal(in.bits.uop.meta.pc)} " +
                    p"uop=${in.bits.uop.ctrl.uop} fuType=0x${Hexadecimal(in.bits.uop.ctrl.fuType)}"
            )
            assert(PopCount(in.bits.uop.ctrl.fuType) === 1.U)
        }
    }

    for (m <- 0 until memNissue) {
        val in = io.memIssue(m)
        val rp = intNissue * 2 + m

        val readAny  = in.bits.src1Read || in.bits.src2Read
        val readAddr = Mux(
            in.bits.src2Read,
            in.bits.uop.reg.psrc2,
            in.bits.uop.reg.psrc1
        )

        val readAccepted = in.fire && readAny
        val selectedFastWakeup = Mux(
            in.bits.src2Read,
            in.bits.src2FastWakeup,
            in.bits.src1FastWakeup
        )

        io.rfReadReq(rp).en   := readAccepted
        io.rfReadReq(rp).addr := Mux(readAny, readAddr, 0.U)

        val rfData = readRfOperand(readAny, readAddr, io.rfReadData(rp))

        val loadRelayHits = VecInit((0 until nLoadWb).map { l =>
            readAccepted &&
                readAddr =/= 0.U &&
                loadRelayBypass(l).valid &&
                loadRelayBypass(l).bits.pdest === readAddr
        })
        val loadRelayHit = loadRelayHits.asUInt.orR
        val loadRelayData = Mux1H(
            loadRelayHits,
            loadRelayBypass.map(_.bits.data)
        )

        // Memory PRF read ports have no integer write-through. Only static PRF
        // data or the registered Load relay may feed the pre-AGU carry chain.
        val preAguBase = Mux(loadRelayHit, loadRelayData, rfData)

        val intWriteHits = VecInit(io.bypass.take(nIntWb).map { bypass =>
            readAccepted &&
                readAddr =/= 0.U &&
                bypass.valid &&
                bypass.bits.pdest === readAddr
        })
        val intWriteCollision = intWriteHits.asUInt.orR
        val intWriteData = Mux1H(
            intWriteHits,
            io.bypass.take(nIntWb).map(_.bits.data)
        )

        // Preserve the correct operand for the registered repair path, without
        // allowing the same-cycle integer result to reach the pre-AGU.
        val operandData = Mux(intWriteCollision, intWriteData, preAguBase)

        io.rfReadReq(rp).speculative :=
            readAccepted && (selectedFastWakeup || intWriteCollision)

        val preAgu = Module(new AddressGenerationUnit)
        preAgu.io.base    := preAguBase
        preAgu.io.offset  := in.bits.uop.imm
        preAgu.io.memType := in.bits.uop.mem.memType

        val isAddrOp = in.bits.memOp === MEM_LD || in.bits.memOp === MEM_STA
        val fastAddress =
            io.fastAddressMap.byVseg(preAgu.io.vaddr(31, 29))

        memNextBits(m).uop.connectFrom(in.bits.uop)
        memNextBits(m).uop.spec.brMask :=
            BranchMask.clearResolved(
                in.bits.uop.spec.brMask,
                resolveMask
        )
        memNextBits(m).memOp       := in.bits.memOp
        memNextBits(m).operandData := operandData
        memNextBits(m).preVaddr := preAgu.io.vaddr
        memNextBits(m).preFastPseg := fastAddress.pseg
        memNextBits(m).preSizeMask := preAgu.io.sizeMask
        memNextBits(m).preMisaligned := isAddrOp && preAgu.io.addrMisaligned
        memNextBits(m).preTranslationResolved := fastAddress.resolved
        memNextBits(m).preFastCacheable := fastAddress.cacheable
        // A registered Load relay is final at this boundary. ALU fast wakeup
        // and same-cycle integer writes are repaired after RegisterRead.
        memNextBits(m).addrSpeculative :=
            isAddrOp &&
                !loadRelayHit &&
                (selectedFastWakeup || intWriteCollision)

        when(readAccepted) {
            assert(PopCount(loadRelayHits) <= 1.U)
            assert(PopCount(intWriteHits) <= 1.U)
            assert(!(loadRelayHit && intWriteCollision))
        }

        when(in.fire) {
            assert(in.bits.memOp =/= MEM_X)
            assert(in.bits.src1Read =/= in.bits.src2Read)

            when(in.bits.memOp === MEM_LD || in.bits.memOp === MEM_STA) {
                assert(in.bits.src1Read && !in.bits.src2Read)
            }

            when(in.bits.memOp === MEM_STD) {
                assert(!in.bits.src1Read && in.bits.src2Read)
            }

            when(intWriteCollision && isAddrOp) {
                assert(memNextBits(m).addrSpeculative)
                assert(memNextBits(m).operandData === intWriteData)
            }
        }
    }

    for (p <- 0 until intNissue) {
        val headBitsUpdated = WireInit(intHeadBits(p))
        headBitsUpdated.uop.spec.brMask := BranchMask.clearResolved(
            intHeadBits(p).uop.spec.brMask,
            resolveMask
        )

        val (headSrc1FastData, headSrc1FastForwarded) = captureFastForward(
            headValid(p) && intHeadBits(p).uop.reg.lsrc1Valid,
            intHeadBits(p).uop.reg.psrc1,
            intHeadBits(p).src1Data
        )
        val (headSrc2Data, headSrc2Forwarded) = captureFastForward(
            headValid(p) && intHeadBits(p).uop.reg.lsrc2Valid,
            intHeadBits(p).uop.reg.psrc2,
            intHeadBits(p).src2Data
        )

        val (headSrc1LoadData, headSrc1LoadForwarded) =
            forwardLoadRelay(
                headValid(p) && intHeadBits(p).uop.reg.lsrc1Valid,
                intHeadBits(p).uop.reg.psrc1,
                intHeadBits(p).src1Data
            )
        val (headSrc2LoadData, headSrc2LoadForwarded) =
            forwardLoadRelay(
                headValid(p) && intHeadBits(p).uop.reg.lsrc2Valid,
                intHeadBits(p).uop.reg.psrc2,
                intHeadBits(p).src2Data
            )

        // Keep the current integer fast-forward network off the execute
        // output.  Registered load events repair both the current output and
        // resident state; integer fast forwarding only repairs resident state.
        io.intExecute(p).bits.src1Data := headSrc1LoadData
        io.intExecute(p).bits.src2Data := headSrc2LoadData

        val headSrc1Data = Mux(
            headSrc1LoadForwarded,
            headSrc1LoadData,
            headSrc1FastData
        )
        val headSrc2FinalData = Mux(
            headSrc2LoadForwarded,
            headSrc2LoadData,
            headSrc2Data
        )

        when(headValid(p)) {
            assert(!(headSrc1LoadForwarded && headSrc1FastForwarded))
            assert(!(headSrc2LoadForwarded && headSrc2Forwarded))
        }

        headBitsUpdated.src1Data := headSrc1Data
        headBitsUpdated.src2Data := headSrc2FinalData
        val headForwarded =
            headSrc1FastForwarded ||
            headSrc2Forwarded ||
            headSrc1LoadForwarded ||
            headSrc2LoadForwarded

        val headRemains =
            headValid(p) && !headKilled(p) && !io.intExecute(p).fire

        if (p == 0) {
            val nextHeadValid = headRemains || issuePorts(p).fire

            when(io.flush) {
                headValid(p) := false.B
            }.otherwise {
                assert(!(headRemains && issuePorts(p).fire))
                headValid(p) := nextHeadValid
                when(headRemains) {
                    intHeadBits(p) := headBitsUpdated
                }.elsewhen(issuePorts(p).fire) {
                    intHeadBits(p) := intNextBits(p)
                }
            }

            when(issuePorts(p).fire) {
                assert(
                    issuePorts(p).bits.uop.ctrl.fuType === FU_ALU ||
                        issuePorts(p).bits.uop.ctrl.fuType === FU_JMP
                )
            }
        } else {
            val t = p - 1
            val tailBitsUpdated = WireInit(intTailBits(t))
            tailBitsUpdated.uop.spec.brMask := BranchMask.clearResolved(
                intTailBits(t).uop.spec.brMask,
                resolveMask
            )

            val (tailSrc1FastData, tailSrc1FastForwarded) =
                captureFastForward(
                    tailValid(t) && intTailBits(t).uop.reg.lsrc1Valid,
                    intTailBits(t).uop.reg.psrc1,
                    intTailBits(t).src1Data
                )
            val (tailSrc2Data, _) = captureFastForward(
                tailValid(t) && intTailBits(t).uop.reg.lsrc2Valid,
                intTailBits(t).uop.reg.psrc2,
                intTailBits(t).src2Data
            )
            val (tailSrc1LoadData, tailSrc1LoadForwarded) =
                forwardLoadRelay(
                    tailValid(t) && intTailBits(t).uop.reg.lsrc1Valid,
                    intTailBits(t).uop.reg.psrc1,
                    intTailBits(t).src1Data
                )
            val (tailSrc2LoadData, tailSrc2LoadForwarded) =
                forwardLoadRelay(
                    tailValid(t) && intTailBits(t).uop.reg.lsrc2Valid,
                    intTailBits(t).uop.reg.psrc2,
                    intTailBits(t).src2Data
                )

            tailBitsUpdated.src1Data := Mux(
                tailSrc1LoadForwarded,
                tailSrc1LoadData,
                tailSrc1FastData
            )
            tailBitsUpdated.src2Data := Mux(
                tailSrc2LoadForwarded,
                tailSrc2LoadData,
                tailSrc2Data
            )

            when(tailValid(t)) {
                assert(!(tailSrc1LoadForwarded && tailSrc1FastForwarded))
            }

            val tailRemains = tailValid(t) && !tailKilled(t)
            val packedHeadValid = headRemains || tailRemains
            val packedTailValid = headRemains && tailRemains
            val packedHeadBits =
                Mux(headRemains, headBitsUpdated, tailBitsUpdated)

            val nextHeadValid = packedHeadValid || issuePorts(p).fire
            val nextTailValid = packedTailValid ||
                (packedHeadValid && issuePorts(p).fire)
            val nextHeadBits = WireDefault(packedHeadBits)
            val nextTailBits = WireDefault(tailBitsUpdated)

            when(issuePorts(p).fire) {
                when(!packedHeadValid) {
                    nextHeadBits := intNextBits(p)
                }.elsewhen(!packedTailValid) {
                    nextTailBits := intNextBits(p)
                }
            }

            when(io.flush) {
                headValid(p) := false.B
                tailValid(t) := false.B
            }.otherwise {
                headValid(p) := nextHeadValid
                tailValid(t) := nextTailValid
                when(nextHeadValid) {
                    intHeadBits(p) := nextHeadBits
                }
                when(nextTailValid) {
                    intTailBits(t) := nextTailBits
                }
            }

            assert(!tailValid(t) || headValid(p))
        }

        val stalledLastCycle = RegNext(
            io.intExecute(p).valid && !io.intExecute(p).ready,
            false.B
        )
        val bitsLastCycle = RegNext(io.intExecute(p).bits.asUInt)
        val headForwardedLastCycle = RegNext(headForwarded, false.B)
        val recoveryLastCycle = RegNext(
            io.flush || io.branchUpdate.valid,
            false.B
        )

        when(
            stalledLastCycle &&
            !io.flush &&
            !io.branchUpdate.valid &&
            !recoveryLastCycle &&
            !headForwardedLastCycle
        ) {
            assert(io.intExecute(p).valid)
            assert(io.intExecute(p).bits.asUInt === bitsLastCycle)
        }

        when(headKilled(p)) {
            assert(BranchMask.isKilled(
                io.intExecute(p).bits.uop.spec.brMask,
                mispredictMask
            ))
        }

        if (p == 1) {
            val fixedHead =
                io.intExecute(p).valid &&
                (io.intExecute(p).bits.uop.ctrl.fuType === FU_ALU ||
                    io.intExecute(p).bits.uop.ctrl.fuType === FU_CNT)
            when(
                fixedHead &&
                !headKilled(p) &&
                !mispredictMask.orR &&
                !io.flush
            ) {
                assert(
                    io.intExecute(p).ready,
                    "P1 fixed producer stalled after selection-time wake"
                )
            }
        }
    }

    for (m <- 0 until memNissue) {
        val p = intNissue + m
        val t = p - 1

        val headBitsUpdated = WireInit(memHeadBits(m))
        val tailBitsUpdated = WireInit(memTailBits(m))
        headBitsUpdated.uop.spec.brMask := BranchMask.clearResolved(
            memHeadBits(m).uop.spec.brMask,
            resolveMask
        )
        tailBitsUpdated.uop.spec.brMask := BranchMask.clearResolved(
            memTailBits(m).uop.spec.brMask,
            resolveMask
        )

        val headPsrc = Mux(
            memHeadBits(m).memOp === MEM_STD,
            memHeadBits(m).uop.reg.psrc2,
            memHeadBits(m).uop.reg.psrc1
        )
        val headOperandEnable = Mux(
            memHeadBits(m).memOp === MEM_STD,
            memHeadBits(m).uop.reg.lsrc2Valid,
            memHeadBits(m).uop.reg.lsrc1Valid
        )
        val tailPsrc = Mux(
            memTailBits(m).memOp === MEM_STD,
            memTailBits(m).uop.reg.psrc2,
            memTailBits(m).uop.reg.psrc1
        )
        val tailOperandEnable = Mux(
            memTailBits(m).memOp === MEM_STD,
            memTailBits(m).uop.reg.lsrc2Valid,
            memTailBits(m).uop.reg.lsrc1Valid
        )

        val (headFastData, headFastForwarded) = captureFastForward(
            headValid(p) && headOperandEnable,
            headPsrc,
            memHeadBits(m).operandData
        )
        val (tailFastData, tailFastForwarded) = captureFastForward(
            tailValid(t) && tailOperandEnable,
            tailPsrc,
            memTailBits(m).operandData
        )

        val (headLoadData, headLoadForwarded) = forwardLoadRelay(
            headValid(p) && headOperandEnable,
            headPsrc,
            memHeadBits(m).operandData
        )
        val (tailLoadData, tailLoadForwarded) = forwardLoadRelay(
            tailValid(t) && tailOperandEnable,
            tailPsrc,
            memTailBits(m).operandData
        )

        // MemExecutePort applies same-cycle integer forwarding. Only the
        // registered load relay bypasses the resident head at this boundary.
        io.memExecute(m).bits.operandData := headLoadData

        val headResidentData = Mux(
            headLoadForwarded,
            headLoadData,
            headFastData
        )
        val tailResidentData = Mux(
            tailLoadForwarded,
            tailLoadData,
            tailFastData
        )

        headBitsUpdated.operandData := headResidentData
        tailBitsUpdated.operandData := tailResidentData

        when(headValid(p)) {
            assert(!(headLoadForwarded && headFastForwarded))
        }
        when(tailValid(t)) {
            assert(!(tailLoadForwarded && tailFastForwarded))
        }

        val headForwarded = headLoadForwarded || headFastForwarded

        val headRemains = headValid(p) && !headKilled(p) && !io.memExecute(m).fire
        val tailRemains = tailValid(t) && !tailKilled(t)
        val packedHeadValid = headRemains || tailRemains
        val packedTailValid = headRemains && tailRemains
        val packedHeadBits = Mux(headRemains, headBitsUpdated, tailBitsUpdated)

        val nextHeadValid = packedHeadValid || issuePorts(p).fire
        val nextTailValid = packedTailValid ||
            (packedHeadValid && issuePorts(p).fire)
        val nextHeadBits = WireDefault(packedHeadBits)
        val nextTailBits = WireDefault(tailBitsUpdated)

        when(issuePorts(p).fire) {
            when(!packedHeadValid) {
                nextHeadBits := memNextBits(m)
            }.elsewhen(!packedTailValid) {
                nextTailBits := memNextBits(m)
            }
        }

        when(io.flush) {
            headValid(p) := false.B
            tailValid(t) := false.B
        }.otherwise {
            headValid(p) := nextHeadValid
            tailValid(t) := nextTailValid
            when(nextHeadValid) {
                memHeadBits(m) := nextHeadBits
            }
            when(nextTailValid) {
                memTailBits(m) := nextTailBits
            }
        }

        val stalledLastCycle = RegNext(
            io.memExecute(m).valid && !io.memExecute(m).ready,
            false.B
        )
        val bitsLastCycle = RegNext(io.memExecute(m).bits.asUInt)
        val headForwardedLastCycle = RegNext(headForwarded, false.B)
        val recoveryLastCycle = RegNext(
            io.flush || io.branchUpdate.valid,
            false.B
        )

        when(
            stalledLastCycle &&
            !io.flush &&
            !io.branchUpdate.valid &&
            !recoveryLastCycle &&
            !headForwardedLastCycle
        ) {
            assert(io.memExecute(m).valid)
            assert(io.memExecute(m).bits.asUInt === bitsLastCycle)
        }

        when(headKilled(p)) {
            assert(BranchMask.isKilled(
                io.memExecute(m).bits.uop.spec.brMask,
                mispredictMask
            ))
        }

        assert(!tailValid(t) || headValid(p))
    }

    when(io.flush) {
        assert(!VecInit(issuePorts.map(_.fire)).asUInt.orR)
    }

}
