package CPUSTC.backend.issue

import chisel3._
import chisel3.util._

import CPUSTC.config._
import CPUSTC.config.Decode._
import CPUSTC.config.FunctionUnit._
import CPUSTC.config.Issue._
import CPUSTC.config.IssueQueue._
import CPUSTC.config.MemIssueOp._
import CPUSTC.config.RegisterFile._
import CPUSTC.config.RenameConfig._
import CPUSTC.config.WritebackConfig._
import CPUSTC.backend.branch.BranchMask
import CPUSTC.backend.dispatch.DispatchUop
import CPUSTC.utils.{ClusterFreeIndexQueue, XilinxFdreWakeCatcher}

class IntIssueQueue extends Module {
    val io = IO(new IntIssueQueueIO)

    require(intNissue == 3)
    require(nFastIntWb == 2)
    require(intNiq > ndcd)

    private def pickThreeRotOHCarry(req: UInt, baseOH: UInt): Vec[UInt] = {
        require(req.getWidth == baseOH.getWidth)

        def pickThreeLow(input: UInt): Vec[UInt] = {
            val width = input.getWidth
            val grants = Wire(Vec(3, UInt(width.W)))
            var remaining = input

            for (i <- 0 until 3) {
                val grant = remaining & ((~remaining).asUInt + 1.U(width.W))
                grants(i) := grant
                remaining = remaining & (~grant).asUInt
            }

            grants
        }

        val fromBaseMask = MaskUpper(baseOH)
        val highGrants = pickThreeLow(req & fromBaseMask)
        val lowGrants = pickThreeLow(req & (~fromBaseMask).asUInt)
        val highValid = VecInit(highGrants.map(_.orR))

        VecInit(Seq(
            Mux(highValid(0), highGrants(0), lowGrants(0)),
            Mux(highValid(1), highGrants(1),
                Mux(highValid(0), lowGrants(0), lowGrants(1))),
            Mux(highValid(2), highGrants(2),
                Mux(highValid(1), lowGrants(0),
                    Mux(highValid(0), lowGrants(1), lowGrants(2))))
        ))
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

    val mispredict =
        io.branchUpdate.valid &&
        mispredictMask.orR

    def makeFuMask(fu:UInt): UInt = {
        VecInit(entries.map { e => 
            e.valid && (e.uop.ctrl.fuType & fu).orR
        }).asUInt
    }

    val entries = RegInit(
        VecInit(Seq.fill(intNiq)(0.U.asTypeOf(new IntIssueEntry)))
    )

    // A recovery-independent P2 would-wake may precede the producer's real
    // acceptance. Keep only the affected resident consumers blocked until
    // that exact producer fires; recovery itself only reaches this sidecar D.
    val p2DeferredBlockMask = RegInit(0.U(intNiq.W))
    val p2DeferredPdest = RegInit(0.U(wpreg.W))

    private val load0WakePort = nIntWb
    require(load0WakePort < nwkp)
    private val restWakePorts = (0 until nwkp).filter(_ != load0WakePort)
    val load0Wake = io.wakeup(load0WakePort)
    val pendingLoad0WakeSrc1 = Wire(Vec(intNiq, Bool()))
    val pendingLoad0WakeSrc2 = Wire(Vec(intNiq, Bool()))

    for (i <- 0 until intNiq) {
        io.residentIntProducers(i).valid :=
            entries(i).valid &&
            entries(i).uop.reg.rfWen &&
            entries(i).uop.reg.pdest =/= 0.U
        io.residentIntProducers(i).bits := entries(i).uop.reg.pdest
    }

    val ageSelect = Module(new IssueAgeMatrix(
        numEntries = intNiq,
        enqWidth = ndcd,
        selectWidth = 2
    ))
    ageSelect.io.flush := io.flush

    val selectBaseOH = RegInit(
        VecInit(Seq.fill(intNissue)(1.U(intNiq.W)))
    )

    val aluAgeSeedOH = RegInit(1.U(intNiq.W))

    val issueRegValid = RegInit(
        VecInit(Seq.fill(intNissue)(false.B))
    )

    val issueRegBits = Reg(Vec(intNissue, new IssueOut))

    // P1/P2 carry only the selected identity in the normal issue path.  The
    // wide register is used as a skid buffer only when a live selection first
    // sees backpressure or must survive a branch recovery.
    val liveIssueValid = RegInit(
        VecInit(Seq.fill(intNissue)(false.B))
    )
    val liveIssueOH = Reg(Vec(intNissue, UInt(intNiq.W)))
    val liveIssueBrMask = Reg(Vec(intNissue, UInt(maxBrCount.W)))
    val liveIssuePsrc1 = Reg(Vec(intNissue, UInt(wpreg.W)))
    val liveIssuePsrc2 = Reg(Vec(intNissue, UInt(wpreg.W)))
    val liveIssueSrc1Read = Reg(Vec(intNissue, Bool()))
    val liveIssueSrc2Read = Reg(Vec(intNissue, Bool()))
    val liveIssueSrc1FastWakeup = Reg(Vec(intNissue, Bool()))
    val liveIssueSrc2FastWakeup = Reg(Vec(intNissue, Bool()))
    val liveIssueIsHeadSerialized = Reg(Vec(intNissue, Bool()))
    val liveIssueIsDiv = Reg(Vec(intNissue, Bool()))

    val divIssueDelay = RegInit(false.B)
    val divInIssueReg =
        (liveIssueValid(2) && liveIssueIsDiv(2)) ||
        (issueRegValid(2) &&
            (issueRegBits(2).uop.ctrl.fuType & FU_DIV).orR)
    val suppressDivIssue = divInIssueReg || divIssueDelay

    val headSerializedFu = FU_CSR | FU_SYS
    val headSerializedIssueDelay = RegInit(false.B)
    val headSerializedInIssueReg =
        (liveIssueValid(1) && liveIssueIsHeadSerialized(1)) ||
        (issueRegValid(1) &&
            (issueRegBits(1).uop.ctrl.fuType & headSerializedFu).orR)
    val suppressP1Issue =
        headSerializedInIssueReg || headSerializedIssueDelay

    val effectivePortCaps = WireInit(io.portCaps)
    effectivePortCaps(1) := Mux(
        suppressP1Issue,
        0.U(FUC_SZ.W),
        io.portCaps(1)
    )
    effectivePortCaps(2) := io.portCaps(2) &
        (~Mux(suppressDivIssue, FU_DIV, 0.U(FUC_SZ.W))).asUInt

    val issueRegKilled = Wire(Vec(intNissue, Bool()))
    val issueRegNewMask = Wire(Vec(intNissue, UInt(maxBrCount.W)))
    val liveIssueKilled = Wire(Vec(intNissue, Bool()))
    val liveIssueNewMask = Wire(Vec(intNissue, UInt(maxBrCount.W)))

    for (p <- 0 until intNissue) {
        issueRegKilled(p) :=
            issueRegValid(p) &&
            BranchMask.isKilled(
                issueRegBits(p).uop.spec.brMask,
                mispredictMask
            )

        issueRegNewMask(p) :=
            BranchMask.clearResolved(
                issueRegBits(p).uop.spec.brMask,
                resolveMask
            )

        liveIssueKilled(p) :=
            liveIssueValid(p) &&
            BranchMask.isKilled(
                liveIssueBrMask(p),
                mispredictMask
            )

        liveIssueNewMask(p) :=
            BranchMask.clearResolved(
                liveIssueBrMask(p),
                resolveMask
            )
    }

    val issueRegCanAccept = Wire(Vec(intNissue, Bool()))

    for (p <- 0 until intNissue) {
        issueRegCanAccept(p) :=
            (!issueRegValid(p) && !liveIssueValid(p)) ||
                io.issue(p).ready
    }

    val wakeSrc1 = Wire(Vec(intNiq, Bool()))
    val wakeSrc2 = Wire(Vec(intNiq, Bool()))
    val fastWakeSrc1 = Wire(Vec(intNiq, Bool()))
    val fastWakeSrc2 = Wire(Vec(intNiq, Bool()))
    val intProducerFastWakeSrc1 = Wire(Vec(intNiq, Bool()))
    val intProducerFastWakeSrc2 = Wire(Vec(intNiq, Bool()))
    val legacyLocalFastWakeSrc1 = Wire(Vec(intNiq, Bool()))
    val legacyLocalFastWakeSrc2 = Wire(Vec(intNiq, Bool()))
    val localFastWakeup = WireDefault(
        0.U.asTypeOf(Vec(nFastIntWb, Valid(new IssueWakeup)))
    )
    val loadPredWakeSrc1 = Wire(Vec(intNiq, UInt(memNissue.W)))
    val loadPredWakeSrc2 = Wire(Vec(intNiq, UInt(memNissue.W)))

    val loadPredResolveMask = VecInit(io.loadPredResolve.map(_.valid)).asUInt
    val loadPredSuccessMask = VecInit(io.loadPredResolve.map { resolve =>
        resolve.valid && resolve.bits.success
    }).asUInt
    val loadPredCancelMask = VecInit(io.loadPredResolve.map { resolve =>
        resolve.valid && !resolve.bits.success
    }).asUInt

    val src1LoadCanceled = VecInit(entries.map { entry =>
        (entry.src1LoadPoison & loadPredCancelMask).orR
    })
    val src2LoadCanceled = VecInit(entries.map { entry =>
        (entry.src2LoadPoison & loadPredCancelMask).orR
    })
    for (i <- 0 until intNiq) {
        wakeSrc1(i) := restWakePorts.map { port =>
            val w = io.wakeup(port)
            w.valid &&
            w.bits.pdest === entries(i).uop.reg.psrc1
        }.reduce(_ || _)

        wakeSrc2(i) := restWakePorts.map { port =>
            val w = io.wakeup(port)
            w.valid &&
            w.bits.pdest === entries(i).uop.reg.psrc2
        }.reduce(_ || _)

        fastWakeSrc1(i) := restWakePorts.map { port =>
            val w = io.wakeup(port)
            w.valid &&
            w.bits.fast &&
            w.bits.pdest === entries(i).uop.reg.psrc1
        }.reduce(_ || _)

        fastWakeSrc2(i) := restWakePorts.map { port =>
            val w = io.wakeup(port)
            w.valid &&
            w.bits.fast &&
            w.bits.pdest === entries(i).uop.reg.psrc2
        }.reduce(_ || _)

        legacyLocalFastWakeSrc1(i) := localFastWakeup.map { w =>
            w.valid && w.bits.pdest === entries(i).uop.reg.psrc1
        }.reduce(_ || _)
        legacyLocalFastWakeSrc2(i) := localFastWakeup.map { w =>
            w.valid && w.bits.pdest === entries(i).uop.reg.psrc2
        }.reduce(_ || _)

        loadPredWakeSrc1(i) := VecInit(io.loadPredWake.map { wake =>
            wake.valid &&
            !entries(i).src1Ready &&
            entries(i).uop.reg.lsrc1Valid &&
            entries(i).uop.reg.psrc1 =/= 0.U &&
            wake.bits.pdest === entries(i).uop.reg.psrc1
        }).asUInt

        loadPredWakeSrc2(i) := VecInit(io.loadPredWake.map { wake =>
            wake.valid &&
            !entries(i).src2Ready &&
            entries(i).uop.reg.lsrc2Valid &&
            entries(i).uop.reg.psrc2 =/= 0.U &&
            wake.bits.pdest === entries(i).uop.reg.psrc2
        }).asUInt
    }

    val effectiveSrc1Ready = VecInit((0 until intNiq).map { i =>
        entries(i).src1Ready || pendingLoad0WakeSrc1(i)
    })
    val effectiveSrc2Ready = VecInit((0 until intNiq).map { i =>
        entries(i).src2Ready || pendingLoad0WakeSrc2(i)
    })
    val effectiveSrc1FastWakeup = VecInit((0 until intNiq).map { i =>
        Mux(
            pendingLoad0WakeSrc1(i),
            false.B,
            entries(i).src1FastWakeup
        )
    })
    val effectiveSrc2FastWakeup = VecInit((0 until intNiq).map { i =>
        Mux(
            pendingLoad0WakeSrc2(i),
            false.B,
            entries(i).src2FastWakeup
        )
    })
    val effectiveSrc1LoadPoison = VecInit((0 until intNiq).map { i =>
        Mux(
            pendingLoad0WakeSrc1(i),
            0.U(memNissue.W),
            entries(i).src1LoadPoison
        )
    })
    val effectiveSrc2LoadPoison = VecInit((0 until intNiq).map { i =>
        Mux(
            pendingLoad0WakeSrc2(i),
            0.U(memNissue.W),
            entries(i).src2LoadPoison
        )
    })

    val baseReadyVec = Wire(Vec(intNiq, Bool()))

    val entryKilled = Wire(Vec(intNiq, Bool()))

    for (i <- 0 until intNiq) {
        baseReadyVec(i) :=
            entries(i).valid &&
            entries(i).uop.ctrl.legal &&
            effectiveSrc1Ready(i) &&
            effectiveSrc2Ready(i)
    }

    val baseReady = baseReadyVec.asUInt
    val schedReady = baseReady & (~p2DeferredBlockMask).asUInt

    io.validMask := VecInit((0 until intNiq).map { i =>
        entries(i).valid && !entryKilled(i)
    }).asUInt
    io.canIssueMask := schedReady & (~entryKilled.asUInt).asUInt
    io.full := io.validMask.andR
    val liveIssueRegMask = VecInit((0 until intNissue).map { p =>
        (issueRegValid(p) && !issueRegKilled(p)) ||
        (liveIssueValid(p) && !liveIssueKilled(p))
    }).asUInt
    io.empty := !io.validMask.orR && !liveIssueRegMask.orR

    val portReq = Wire(Vec(intNissue, UInt(intNiq.W)))

    for (p <- 0 until intNissue) {
        portReq(p) := VecInit((0 until intNiq).map { i =>
            schedReady(i) &&
            (entries(i).uop.ctrl.fuType & effectivePortCaps(p)).orR
        }).asUInt
    }

    val jmpMask = makeFuMask(FU_JMP)
    val headSerializedMask = makeFuMask(headSerializedFu)
    val cntMask = makeFuMask(FU_CNT)
    val headSerializedEligibleNow = VecInit(entries.zipWithIndex.map { case (entry, i) =>
        val localRobHead = io.robHead(i)
        entry.valid &&
        (entry.uop.ctrl.fuType & headSerializedFu).orR &&
        localRobHead.valid &&
        entry.uop.robPtr.qidx === localRobHead.bits.qidx &&
        entry.uop.robPtr.offset === localRobHead.bits.offset &&
        entry.uop.robPtr.high === localRobHead.bits.high &&
        entry.uop.robPtr.epoch === localRobHead.bits.epoch &&
        !entry.uop.spec.brMask.orR
    }).asUInt
    // Keep the wide per-entry ROB-head comparison out of P1 selection and the
    // ordinary ALU fast-wakeup cone. CSR/system operations are serializing and
    // may wait one cycle after becoming the architectural head; a generation
    // token prevents an old snapshot from issuing a newer operation.
    val headSerializedEligible = RegInit(0.U(intNiq.W))
    val headSerializedTokenGeneration = RegInit(false.B)
    when(io.flush || !io.robHead(0).valid) {
        headSerializedEligible := 0.U
        headSerializedTokenGeneration := false.B
    }.otherwise {
        headSerializedEligible := headSerializedEligibleNow
        headSerializedTokenGeneration := io.robHeadGeneration
    }
    val headSerializedTokenCurrent =
        io.robHead(0).valid &&
            (headSerializedTokenGeneration === io.robHeadGeneration)
    val headSerializedCntMask =
        (headSerializedMask & headSerializedEligible &
            Fill(intNiq, headSerializedTokenCurrent)) |
            cntMask
    val mulDivMask = makeFuMask(FU_MUL | FU_DIV)

    val portCanSelect = VecInit((0 until intNissue).map { p =>
        issueRegCanAccept(p) && !io.flush
    })

    val specialMask = Seq(jmpMask, headSerializedCntMask, mulDivMask)
    val specialCandidates = Wire(Vec(intNissue, UInt(intNiq.W)))
    val specialSelectOH = Wire(Vec(intNissue, UInt(intNiq.W)))

    specialCandidates(0) := portReq(0) & specialMask(0)
    ageSelect.io.request(0) := specialCandidates(0)

    specialSelectOH(0) := Mux(
        portCanSelect(0),
        ageSelect.io.oldestOH(0),
        0.U(intNiq.W)
    )

    for (p <- 1 until intNissue) {
        specialCandidates(p) := portReq(p) & specialMask(p)
        specialSelectOH(p) := Mux(
            portCanSelect(p),
            PickRotOH(specialCandidates(p), selectBaseOH(p)),
            0.U(intNiq.W)
        )
    }

    // Keep current-cycle readiness out of the age matrix.  The registered
    // winner is only a rotation seed for the next cycle's ready-only picker.
    val residentAluMask = makeFuMask(FU_ALU)
    ageSelect.io.request(1) := residentAluMask

    when(io.flush) {
        aluAgeSeedOH := 1.U
    }.otherwise {
        aluAgeSeedOH := Mux(
            residentAluMask.orR,
            ageSelect.io.oldestOH(1),
            1.U
        )
    }

    val aluMask = residentAluMask & schedReady
    val aluGrantOH = pickThreeRotOHCarry(
        req    = aluMask,
        baseOH = aluAgeSeedOH
    )

    // A stale head-serialized token reserves P1 for one cycle before it is
    // rebuilt. This keeps serialization independent of the token check.
    val p1SpecialReserved =
        (portReq(1) &
            ((headSerializedMask & headSerializedEligible) | cntMask)).orR
    val aluPortAvailable = VecInit((0 until intNissue).map { p =>
        portCanSelect(p) &&
        !(if (p == 1) p1SpecialReserved else specialCandidates(p).orR) &&
        (effectivePortCaps(p) & FU_ALU).orR
    })

    // P2's rotating-base state only needs to know whether a selection exists.
    // Keep the full one-hot selection cone off that register's clock enable.
    val p2AluRequestCount = PopCount(aluMask)
    val p2PriorAluPortAny =
        aluPortAvailable(0) || aluPortAvailable(1)
    val p2PriorAluPortBoth =
        aluPortAvailable(0) && aluPortAvailable(1)
    val p2DirectSelectAdvance =
        (portCanSelect(2) && specialCandidates(2).orR) ||
        (aluPortAvailable(2) && Mux(
            p2PriorAluPortBoth,
            p2AluRequestCount > 2.U,
            Mux(
                p2PriorAluPortAny,
                p2AluRequestCount > 1.U,
                p2AluRequestCount.orR
            )
        ))

    val aluSelectOH = Wire(Vec(intNissue, UInt(intNiq.W)))
    aluSelectOH(0) := Mux(
        aluPortAvailable(0),
        aluGrantOH(0),
        0.U(intNiq.W)
    )
    aluSelectOH(1) := Mux(
        aluPortAvailable(1),
        Mux(aluPortAvailable(0), aluGrantOH(1), aluGrantOH(0)),
        0.U(intNiq.W)
    )
    aluSelectOH(2) := Mux(
        aluPortAvailable(2),
        Mux(
            aluPortAvailable(0) && aluPortAvailable(1),
            aluGrantOH(2),
            Mux(
                aluPortAvailable(0) || aluPortAvailable(1),
                aluGrantOH(1),
                aluGrantOH(0)
            )
        ),
        0.U(intNiq.W)
    )

    val selectOH = Wire(Vec(intNissue, UInt(intNiq.W)))
    for (p <- 0 until intNissue) {
        selectOH(p) := specialSelectOH(p) | aluSelectOH(p)
    }

    val entryLoadPoison = VecInit((0 until intNiq).map { i =>
        effectiveSrc1LoadPoison(i) | effectiveSrc2LoadPoison(i)
    })
    val predictionResolvedMask = VecInit(entryLoadPoison.map { poison =>
        (poison & (~loadPredSuccessMask).asUInt) === 0.U
    }).asUInt
    val poisonedEntryMask = VecInit(entryLoadPoison.map(_.orR)).asUInt
    val fastWakeEligibleMask = VecInit(entries.map { entry =>
        !(entry.uop.ctrl.fuType & headSerializedFu).orR &&
        entry.uop.reg.rfWen &&
        entry.uop.reg.pdest =/= 0.U
    }).asUInt

    val selectedBits = Wire(Vec(intNissue, new IssueOut))
    val selectedPdest = Wire(Vec(intNissue, UInt(wpreg.W)))

    for (p <- 0 until intNissue) {
        val selectedValid = selectOH(p).orR

        val selectedUop = Mux(
            selectedValid,
            Mux1H(selectOH(p), VecInit(entries.map(_.uop))),
            0.U.asTypeOf(new DispatchUop)
        )
        selectedPdest(p) := Mux(
            selectedValid,
            Mux1H(selectOH(p), VecInit(entries.map(_.uop.reg.pdest))),
            0.U
        )
        selectedBits(p) := 0.U.asTypeOf(new IssueOut)
        selectedBits(p).uop := selectedUop
        selectedBits(p).uop.spec.brMask :=
            BranchMask.clearResolved(
                selectedUop.spec.brMask,
                resolveMask
            )
        selectedBits(p).memOp := MEM_X
        selectedBits(p).src1Read := selectedUop.reg.lsrc1Valid
        selectedBits(p).src2Read := selectedUop.reg.lsrc2Valid
        selectedBits(p).src1FastWakeup := Mux(
            selectedValid,
            Mux1H(selectOH(p), effectiveSrc1FastWakeup),
            false.B
        )
        selectedBits(p).src2FastWakeup := Mux(
            selectedValid,
            Mux1H(selectOH(p), effectiveSrc2FastWakeup),
            false.B
        )

    }

    val liveIssueBits = Wire(Vec(intNissue, new IssueOut))
    for (p <- 0 until intNissue) {
        val liveUop = Mux1H(
            liveIssueOH(p),
            VecInit(entries.map(_.uop))
        )

        liveIssueBits(p) := 0.U.asTypeOf(new IssueOut)
        liveIssueBits(p).uop := liveUop
        // The selected source identity was captured beside liveIssueOH.  Keep
        // the entry-payload mux out of the following cycle's asynchronous PRF
        // address path without adding an issue stage.
        liveIssueBits(p).uop.reg.psrc1 := liveIssuePsrc1(p)
        liveIssueBits(p).uop.reg.psrc2 := liveIssuePsrc2(p)
        liveIssueBits(p).uop.spec.brMask := liveIssueNewMask(p)
        liveIssueBits(p).memOp := MEM_X
        liveIssueBits(p).src1Read := liveIssueSrc1Read(p)
        liveIssueBits(p).src2Read := liveIssueSrc2Read(p)
        liveIssueBits(p).src1FastWakeup :=
            liveIssueSrc1FastWakeup(p)
        liveIssueBits(p).src2FastWakeup :=
            liveIssueSrc2FastWakeup(p)
    }

    val selectAdvance = VecInit((0 until intNissue).map { p =>
        selectOH(p).orR &&
            issueRegCanAccept(p) &&
            !io.flush
    })

    for (p <- 0 until intNissue) {
        assert(selectAdvance(p) === selectOH(p).orR)
    }

    val selectCandidateOH = VecInit((0 until intNissue).map { p =>
        selectOH(p) & predictionResolvedMask
    })
    val selectFireOH = VecInit((0 until intNissue).map { p =>
        Mux(
            !mispredict,
            selectCandidateOH(p),
            0.U(intNiq.W)
        )
    })
    val selectFire = VecInit((0 until intNissue).map { p =>
        selectFireOH(p).orR
    })

    // Only liveIssueValid is architectural state.  Sidecar contents are
    // unobservable whenever it is false, so write them every cycle and keep
    // the selection cone off their clock-enable pins.
    for (p <- 1 until intNissue) {
        liveIssueOH(p) := selectFireOH(p)
        liveIssueBrMask(p) := selectedBits(p).uop.spec.brMask
        liveIssuePsrc1(p) := selectedBits(p).uop.reg.psrc1
        liveIssuePsrc2(p) := selectedBits(p).uop.reg.psrc2
        liveIssueSrc1Read(p) := selectedBits(p).src1Read
        liveIssueSrc2Read(p) := selectedBits(p).src2Read
        liveIssueSrc1FastWakeup(p) :=
            selectedBits(p).src1FastWakeup
        liveIssueSrc2FastWakeup(p) :=
            selectedBits(p).src2FastWakeup
        liveIssueIsHeadSerialized(p) :=
            (selectedBits(p).uop.ctrl.fuType & headSerializedFu).orR
        liveIssueIsDiv(p) :=
            (selectedBits(p).uop.ctrl.fuType & FU_DIV).orR
    }

    io.ftqPredictionReadReq.valid :=
        selectFire(0) &&
        (selectedBits(0).uop.ctrl.fuType & FU_JMP).orR
    io.ftqPredictionReadReq.bits := selectedBits(0).uop.meta.ftqPtr

    io.loadPredIssueCount := PopCount(VecInit((0 until intNissue).map { p =>
        (selectFireOH(p) & poisonedEntryMask).orR
    }))

    for (p <- 0 until nFastIntWb) {
        val selectedUop = selectedBits(p).uop

        localFastWakeup(p).valid :=
            (selectFireOH(p) & fastWakeEligibleMask).orR
        localFastWakeup(p).bits.pdest := selectedPdest(p)
        localFastWakeup(p).bits.fast := true.B

        when(localFastWakeup(p).valid) {
            assert(selectedUop.reg.ldestValid)
            assert(selectedUop.reg.pdest =/= 0.U)
        }
    }
    io.fastWakeup := localFastWakeup

    val selectFireMask = selectFireOH.reduce(_ | _)
    val fastSelectedProducerMask =
        (selectFireOH(0) | selectFireOH(1)) &
            fastWakeEligibleMask
    val selectCandidateMask = selectCandidateOH.reduce(_ | _)
    val fastCandidateProducerMask =
        (selectCandidateOH(0) | selectCandidateOH(1)) &
            fastWakeEligibleMask

    assert((p2DeferredBlockMask & selectCandidateMask) === 0.U)

    when(localFastWakeup(1).valid) {
        assert(
            selectedBits(1).uop.ctrl.fuType === FU_ALU ||
                selectedBits(1).uop.ctrl.fuType === FU_CNT
        )
        assert(io.issue(1).ready)
    }

    // A P1 selection-time wake is a fixed-latency contract, not a prediction.
    // With no intervening recovery, the selected identity must leave the IQ
    // on the following cycle. RegisterRead and IntExecutePort assert the next
    // fixed-latency boundary independently.
    val p1FastPromiseValid = RegInit(false.B)
    val p1FastPromisePdest = Reg(UInt(wpreg.W))
    val p1FastPromiseRobPtr = Reg(chiselTypeOf(selectedBits(1).uop.robPtr))

    when(io.flush || mispredict) {
        p1FastPromiseValid := false.B
    }.otherwise {
        p1FastPromiseValid := localFastWakeup(1).valid
        when(localFastWakeup(1).valid) {
            p1FastPromisePdest := selectedBits(1).uop.reg.pdest
            p1FastPromiseRobPtr := selectedBits(1).uop.robPtr
        }
    }

    when(p1FastPromiseValid && !io.flush && !mispredict) {
        assert(
            io.issue(1).fire,
            "P1 selection-time wake did not reach RegisterRead on schedule"
        )
        assert(io.issue(1).bits.uop.reg.pdest === p1FastPromisePdest)
        assert(
            io.issue(1).bits.uop.robPtr.asUInt ===
                p1FastPromiseRobPtr.asUInt
        )
    }

    io.producerFastWakeMask := fastSelectedProducerMask
    io.producerReleaseMask := selectFireMask

    for (i <- 0 until intNiq) {
        intProducerFastWakeSrc1(i) :=
            (entries(i).src1IntProducerOH & fastSelectedProducerMask).orR
        intProducerFastWakeSrc2(i) :=
            (entries(i).src2IntProducerOH & fastSelectedProducerMask).orR

        when (
            entries(i).valid &&
            entries(i).uop.reg.lsrc1Valid &&
            !entries(i).src1Ready
        ) {
            assert(
                intProducerFastWakeSrc1(i) === legacyLocalFastWakeSrc1(i),
                "IntIssueQueue: src1 producer-slot wake differs from tag wake"
            )
        }
        when (
            entries(i).valid &&
            entries(i).uop.reg.lsrc2Valid &&
            !entries(i).src2Ready
        ) {
            assert(
                intProducerFastWakeSrc2(i) === legacyLocalFastWakeSrc2(i),
                "IntIssueQueue: src2 producer-slot wake differs from tag wake"
            )
        }
    }

    val divSelectedThisCycle =
        selectFire(2) &&
        (selectedBits(2).uop.ctrl.fuType & FU_DIV).orR

    val divLeavesIssueReg =
        io.issue(2).fire &&
        Mux(
            liveIssueValid(2),
            liveIssueIsDiv(2),
            (issueRegBits(2).uop.ctrl.fuType & FU_DIV).orR
        )

    val headSerializedLeavesIssueReg =
        io.issue(1).fire &&
        Mux(
            liveIssueValid(1),
            liveIssueIsHeadSerialized(1),
            (issueRegBits(1).uop.ctrl.fuType & headSerializedFu).orR
        )

    when(io.flush) {
        divIssueDelay := false.B
        headSerializedIssueDelay := false.B
    }.otherwise {
        divIssueDelay := divLeavesIssueReg
        headSerializedIssueDelay := headSerializedLeavesIssueReg
    }

    // P0 retains the ordinary wide issue register.  Its payload feeds the
    // JIRL/redirect path, so this experiment isolates P1/P2 without moving
    // that already-sensitive boundary.
    when (io.flush) {
        issueRegValid(0) := false.B
    }.elsewhen(issueRegKilled(0)) {
        issueRegValid(0) := false.B
    }.elsewhen(mispredict) {
        issueRegBits(0).uop.spec.brMask := issueRegNewMask(0)
    }.elsewhen (issueRegCanAccept(0)) {
        issueRegValid(0) := selectFire(0)

        when (selectFire(0)) {
            issueRegBits(0) := selectedBits(0)
        }
    }.elsewhen(io.branchUpdate.valid) {
        issueRegBits(0).uop.spec.brMask := issueRegNewMask(0)
    }

    io.issue(0).valid :=
        issueRegValid(0) &&
        !io.flush &&
        !mispredict &&
        !issueRegKilled(0)
    io.issue(0).bits := issueRegBits(0)
    io.issue(0).bits.uop.spec.brMask := issueRegNewMask(0)

    def captureLiveSelection(p: Int): Unit = {
        liveIssueValid(p) := selectFire(p)
    }

    for (p <- 1 until intNissue) {
        when (io.flush) {
            liveIssueValid(p) := false.B
            issueRegValid(p) := false.B
        }.elsewhen(mispredict) {
            when(liveIssueValid(p)) {
                liveIssueValid(p) := false.B
                when(liveIssueKilled(p)) {
                    issueRegValid(p) := false.B
                }.otherwise {
                    issueRegValid(p) := true.B
                    issueRegBits(p) := liveIssueBits(p)
                    issueRegBits(p).uop.spec.brMask :=
                        liveIssueNewMask(p)
                }
            }.elsewhen(issueRegValid(p)) {
                liveIssueValid(p) := false.B
                when(issueRegKilled(p)) {
                    issueRegValid(p) := false.B
                }.otherwise {
                    issueRegBits(p).uop.spec.brMask :=
                        issueRegNewMask(p)
                }
            }.otherwise {
                liveIssueValid(p) := false.B
                issueRegValid(p) := false.B
            }
        }.elsewhen(liveIssueValid(p)) {
            when(io.issue(p).ready) {
                issueRegValid(p) := false.B
                captureLiveSelection(p)
            }.otherwise {
                liveIssueValid(p) := false.B
                issueRegValid(p) := true.B
                issueRegBits(p) := liveIssueBits(p)
            }
        }.elsewhen(issueRegValid(p)) {
            liveIssueValid(p) := false.B
            when(io.issue(p).ready) {
                issueRegValid(p) := false.B
                captureLiveSelection(p)
            }.elsewhen(io.branchUpdate.valid) {
                issueRegBits(p).uop.spec.brMask := issueRegNewMask(p)
            }
        }.otherwise {
            issueRegValid(p) := false.B
            captureLiveSelection(p)
        }

        val issueKilled = Mux(
            liveIssueValid(p),
            liveIssueKilled(p),
            issueRegKilled(p)
        )
        io.issue(p).valid :=
            (liveIssueValid(p) || issueRegValid(p)) &&
            !io.flush &&
            !mispredict &&
            !issueKilled
        io.issue(p).bits := Mux(
            liveIssueValid(p),
            liveIssueBits(p),
            issueRegBits(p)
        )
        io.issue(p).bits.uop.spec.brMask := Mux(
            liveIssueValid(p),
            liveIssueNewMask(p),
            issueRegNewMask(p)
        )

        assert(!(liveIssueValid(p) && issueRegValid(p)))
        when(liveIssueValid(p)) {
            assert(PopCount(liveIssueOH(p)) === 1.U)
            assert(
                liveIssuePsrc1(p) ===
                    Mux1H(liveIssueOH(p), VecInit(entries.map(_.uop.reg.psrc1)))
            )
            assert(
                liveIssuePsrc2(p) ===
                    Mux1H(liveIssueOH(p), VecInit(entries.map(_.uop.reg.psrc2)))
            )
            assert(
                liveIssueSrc1Read(p) ===
                    Mux1H(liveIssueOH(p), VecInit(entries.map(_.uop.reg.lsrc1Valid)))
            )
            assert(
                liveIssueSrc2Read(p) ===
                    Mux1H(liveIssueOH(p), VecInit(entries.map(_.uop.reg.lsrc2Valid)))
            )
        }
    }

    liveIssueValid(0) := false.B

    when(divSelectedThisCycle) {
        assert(!suppressDivIssue)
    }

    when(suppressDivIssue) {
        assert((effectivePortCaps(2) & FU_DIV) === 0.U)
    }


    when(suppressP1Issue) {
        assert(effectivePortCaps(1) === 0.U)
        assert(!selectFire(1))
    }

    when(selectFire(1) &&
        (selectedBits(1).uop.ctrl.fuType & headSerializedFu).orR) {
        assert(Mux1H(selectOH(1), io.robHead.map(_.valid)))
        assert(headSerializedTokenGeneration === io.robHeadGeneration)
        assert(!selectedBits(1).uop.spec.brMask.orR)
        assert(!localFastWakeup(1).valid)
    }

    val storedValidMask = VecInit(entries.map(_.valid)).asUInt
    val freeIndices = Module(new ClusterFreeIndexQueue(
        numEntries = intNiq,
        allocWidth = ndcd,
        useCompactBankPayload = true
    ))
    val allocOH = freeIndices.io.allocOH
    io.dispatchAllocOH := allocOH

    io.dispatch.canAccept :=
        !io.flush &&
        freeIndices.io.canAllocate

    val enqFire = VecInit((0 until ndcd).map { i =>
        io.dispatch.enq(i).valid && io.dispatch.canAccept && !io.flush && !mispredict
    })
    val rawEnqCount = OHToUInt(io.dispatch.reqCountOH)
    val rawEnqLane = VecInit((0 until ndcd).map { lane =>
        lane.U < rawEnqCount
    })
    val payloadWriteOH = VecInit((0 until ndcd).map { lane =>
        Mux(
            rawEnqLane(lane) && io.dispatch.canAccept && !io.flush,
            allocOH(lane),
            0.U(intNiq.W)
        )
    })

    for (lane <- 0 until ndcd) {
        ageSelect.io.enqOH(lane) := Mux(
            enqFire(lane),
            allocOH(lane),
            0.U(intNiq.W)
        )
    }

    freeIndices.io.flush := io.flush
    freeIndices.io.allocCount := OHToUInt(io.dispatch.reqCountOH)
    freeIndices.io.allocate := enqFire.asUInt.orR
    freeIndices.io.fastReleaseOH := selectFireOH
    freeIndices.io.registeredReturnOH := VecInit.fill(ndcd)(0.U(intNiq.W))
    freeIndices.io.registeredReturnCount := 0.U
    freeIndices.io.normalReleaseMask := 0.U
    freeIndices.io.releaseMask := entryKilled.asUInt

    def sameCycleWakeup(preg: UInt): Bool = {
        restWakePorts.map { port =>
            val w = io.wakeup(port)
            w.valid && w.bits.pdest === preg
        }.reduce(_ || _)
    }

    def sameCycleFastWakeup(preg: UInt): Bool = {
        restWakePorts.map { port =>
            val w = io.wakeup(port)
            w.valid && w.bits.fast && w.bits.pdest === preg
        }.reduce(_ || _)
    }

    def legacySameCycleLocalFastWakeup(preg: UInt): Bool = {
        localFastWakeup.map { w =>
            w.valid && w.bits.pdest === preg
        }.reduce(_ || _)
    }

    def sameCycleLoadPredWake(preg: UInt, valid: Bool): UInt = {
        VecInit(io.loadPredWake.map { wake =>
            valid && preg =/= 0.U && wake.valid && wake.bits.pdest === preg
        }).asUInt
    }

    val dispatchLoad0Src1Match = VecInit(io.dispatch.enq.map { enq =>
        load0Wake.bits.pdest === enq.bits.reg.psrc1
    })
    val dispatchLoad0Src2Match = VecInit(io.dispatch.enq.map { enq =>
        load0Wake.bits.pdest === enq.bits.reg.psrc2
    })

    for (i <- 0 until intNiq) {
        val enqToSlot = VecInit((0 until ndcd).map { e =>
            payloadWriteOH(e)(i)
        })
        val incomingOwner = enqToSlot.asUInt.orR
        val load0Src1Match = Mux(
            incomingOwner,
            Mux1H(enqToSlot, dispatchLoad0Src1Match),
            load0Wake.bits.pdest === entries(i).uop.reg.psrc1
        )
        val load0Src2Match = Mux(
            incomingOwner,
            Mux1H(enqToSlot, dispatchLoad0Src2Match),
            load0Wake.bits.pdest === entries(i).uop.reg.psrc2
        )

        val load0Src1Catcher = Module(new XilinxFdreWakeCatcher)
        load0Src1Catcher.io.clock := clock
        load0Src1Catcher.io.enable := true.B
        load0Src1Catcher.io.data := load0Wake.valid && load0Src1Match
        load0Src1Catcher.io.reset := false.B
        pendingLoad0WakeSrc1(i) := load0Src1Catcher.io.out

        val load0Src2Catcher = Module(new XilinxFdreWakeCatcher)
        load0Src2Catcher.io.clock := clock
        load0Src2Catcher.io.enable := true.B
        load0Src2Catcher.io.data := load0Wake.valid && load0Src2Match
        load0Src2Catcher.io.reset := false.B
        pendingLoad0WakeSrc2(i) := load0Src2Catcher.io.out
    }

    val nextEntries = WireInit(entries)

    for (i <- 0 until intNiq) {
        entryKilled(i) :=
            entries(i).valid &&
            BranchMask.isKilled(
                entries(i).uop.spec.brMask,
                mispredictMask
            )

        when(entries(i).valid) {
            nextEntries(i).uop.spec.brMask :=
                BranchMask.clearResolved(
                    entries(i).uop.spec.brMask,
                    resolveMask
                )
        }

        when(entryKilled(i)) {
            nextEntries(i).valid := false.B
        }
    }

    val killedProducerMask = entryKilled.asUInt

    val p2DeferredMatchMask = VecInit((0 until intNiq).map { i =>
        val src1Match =
            entries(i).uop.reg.lsrc1Valid &&
            !effectiveSrc1Ready(i) &&
            entries(i).uop.reg.psrc1 === io.wakeup(2).bits.pdest
        val src2Match =
            entries(i).uop.reg.lsrc2Valid &&
            !effectiveSrc2Ready(i) &&
            entries(i).uop.reg.psrc2 === io.wakeup(2).bits.pdest

        io.p2DeferredWake &&
        entries(i).valid &&
        !entryKilled(i) &&
        (src1Match || src2Match)
    }).asUInt

    val p2DeferredSurvivorMask =
        p2DeferredBlockMask & (~entryKilled.asUInt).asUInt
    val p2DeferredAccepted =
        p2DeferredBlockMask.orR &&
        io.p2FixedWakeAccepted &&
        io.wakeup(2).bits.pdest === p2DeferredPdest
    val p2DeferredNextMask =
        Mux(p2DeferredAccepted, 0.U, p2DeferredSurvivorMask) |
        p2DeferredMatchMask

    when(io.flush) {
        p2DeferredBlockMask := 0.U
    }.otherwise {
        p2DeferredBlockMask := p2DeferredNextMask

        when(
            p2DeferredMatchMask.orR &&
            !p2DeferredSurvivorMask.orR
        ) {
            p2DeferredPdest := io.wakeup(2).bits.pdest
        }
    }

    when(io.p2DeferredWake) {
        assert(mispredict)
        assert(!io.p2FixedWakeAccepted)
        assert(io.wakeup(2).valid)
        assert(io.wakeup(2).bits.pdest =/= 0.U)

        when(p2DeferredSurvivorMask.orR) {
            assert(io.wakeup(2).bits.pdest === p2DeferredPdest)
        }
    }

    when(p2DeferredBlockMask.orR && io.p2FixedWakeAccepted) {
        assert(io.wakeup(2).bits.pdest === p2DeferredPdest)
    }

    for (i <- 0 until intNiq) {
        when(p2DeferredBlockMask(i)) {
            assert(entries(i).valid)
        }
    }

    for (i <- 0 until intNiq) {
        when (
            entries(i).valid &&
            (
                (entries(i).src1IntProducerOH & killedProducerMask).orR ||
                (entries(i).src2IntProducerOH & killedProducerMask).orR
            )
        ) {
            assert(
                entryKilled(i),
                "IntIssueQueue: a surviving consumer depends on a killed producer"
            )
        }
    }

    for (i <- 0 until intNiq) {
        when (entries(i).valid) {
            val src1RestDeterministicWake =
                wakeSrc1(i) || intProducerFastWakeSrc1(i)
            val src2RestDeterministicWake =
                wakeSrc2(i) || intProducerFastWakeSrc2(i)
            val src1DeterministicWake =
                src1RestDeterministicWake || pendingLoad0WakeSrc1(i)
            val src2DeterministicWake =
                src2RestDeterministicWake || pendingLoad0WakeSrc2(i)
            val src1IntProducerAfterRelease =
                entries(i).src1IntProducerOH & (~selectFireMask).asUInt
            val src2IntProducerAfterRelease =
                entries(i).src2IntProducerOH & (~selectFireMask).asUInt

            nextEntries(i).src1Ready := Mux(
                src1DeterministicWake || loadPredWakeSrc1(i).orR,
                true.B,
                Mux(src1LoadCanceled(i), false.B, entries(i).src1Ready)
            )
            nextEntries(i).src2Ready := Mux(
                src2DeterministicWake || loadPredWakeSrc2(i).orR,
                true.B,
                Mux(src2LoadCanceled(i), false.B, entries(i).src2Ready)
            )

            nextEntries(i).src1FastWakeup := Mux(
                pendingLoad0WakeSrc1(i),
                false.B,
                Mux(
                    src1RestDeterministicWake,
                    fastWakeSrc1(i) || intProducerFastWakeSrc1(i),
                    Mux(loadPredWakeSrc1(i).orR, true.B,
                        Mux(src1LoadCanceled(i), false.B, entries(i).src1FastWakeup))
                )
            )
            nextEntries(i).src2FastWakeup := Mux(
                pendingLoad0WakeSrc2(i),
                false.B,
                Mux(
                    src2RestDeterministicWake,
                    fastWakeSrc2(i) || intProducerFastWakeSrc2(i),
                    Mux(loadPredWakeSrc2(i).orR, true.B,
                        Mux(src2LoadCanceled(i), false.B, entries(i).src2FastWakeup))
                )
            )

            nextEntries(i).src1LoadPoison :=
                Mux(
                    src1DeterministicWake,
                    0.U,
                    (entries(i).src1LoadPoison & (~loadPredResolveMask).asUInt) |
                        loadPredWakeSrc1(i)
                )
            nextEntries(i).src2LoadPoison :=
                Mux(
                    src2DeterministicWake,
                    0.U,
                    (entries(i).src2LoadPoison & (~loadPredResolveMask).asUInt) |
                        loadPredWakeSrc2(i)
                )
            nextEntries(i).src1IntProducerOH := Mux(
                entryKilled(i),
                0.U,
                src1IntProducerAfterRelease
            )
            nextEntries(i).src2IntProducerOH := Mux(
                entryKilled(i),
                0.U,
                src2IntProducerAfterRelease
            )

            when (
                !entryKilled(i) &&
                (wakeSrc1(i) || pendingLoad0WakeSrc1(i) ||
                    loadPredWakeSrc1(i).orR)
            ) {
                assert(!src1IntProducerAfterRelease.orR)
            }
            when (
                !entryKilled(i) &&
                (wakeSrc2(i) || pendingLoad0WakeSrc2(i) ||
                    loadPredWakeSrc2(i).orR)
            ) {
                assert(!src2IntProducerAfterRelease.orR)
            }
        }
    }

    for (i <- 0 until intNiq) {
        when (selectFireMask(i)) {
            nextEntries(i).valid := false.B
        }
    }

    for (e <- 0 until ndcd) {
        val inUop = io.dispatch.enq(e).bits
        val src1IntProducerOH = io.dispatch.src1IntProducerOH(e)
        val src2IntProducerOH = io.dispatch.src2IntProducerOH(e)
        val src1NeedsProducer =
            inUop.reg.lsrc1Valid &&
                inUop.reg.psrc1 =/= 0.U &&
                !inUop.reg.psrc1Ready
        val src2NeedsProducer =
            inUop.reg.lsrc2Valid &&
                inUop.reg.psrc2 =/= 0.U &&
                !inUop.reg.psrc2Ready
        val src1IntProducerFastWake =
            (src1IntProducerOH & fastCandidateProducerMask).orR
        val src2IntProducerFastWake =
            (src2IntProducerOH & fastCandidateProducerMask).orR
        val src1IntProducerAfterRelease = Mux(
            src1NeedsProducer,
            src1IntProducerOH & (~selectCandidateMask).asUInt,
            0.U
        )
        val src2IntProducerAfterRelease = Mux(
            src2NeedsProducer,
            src2IntProducerOH & (~selectCandidateMask).asUInt,
            0.U
        )

        val src1ReadyInit =
            !inUop.reg.lsrc1Valid ||
            inUop.reg.psrc1 === 0.U ||
            inUop.reg.psrc1Ready ||
            sameCycleWakeup(inUop.reg.psrc1) ||
            src1IntProducerFastWake

        val src2ReadyInit =
            !inUop.reg.lsrc2Valid ||
            inUop.reg.psrc2 === 0.U ||
            inUop.reg.psrc2Ready ||
            sameCycleWakeup(inUop.reg.psrc2) ||
            src2IntProducerFastWake

        val src1FastWakeupInit =
            inUop.reg.lsrc1Valid &&
            !inUop.reg.psrc1Ready &&
            (
                sameCycleFastWakeup(inUop.reg.psrc1) ||
                src1IntProducerFastWake
            )

        val src2FastWakeupInit =
            inUop.reg.lsrc2Valid &&
            !inUop.reg.psrc2Ready &&
            (
                sameCycleFastWakeup(inUop.reg.psrc2) ||
                src2IntProducerFastWake
            )

        val src1LoadPredWakeRaw = sameCycleLoadPredWake(
            inUop.reg.psrc1,
            inUop.reg.lsrc1Valid
        )
        val src2LoadPredWakeRaw = sameCycleLoadPredWake(
            inUop.reg.psrc2,
            inUop.reg.lsrc2Valid
        )
        val src1LoadPredWakeInit = Mux(
            src1NeedsProducer,
            src1LoadPredWakeRaw,
            0.U
        )
        val src2LoadPredWakeInit = Mux(
            src2NeedsProducer,
            src2LoadPredWakeRaw,
            0.U
        )
        val src1ReadyAfterLoadPred =
            src1ReadyInit || src1LoadPredWakeRaw.orR
        val src2ReadyAfterLoadPred =
            src2ReadyInit || src2LoadPredWakeRaw.orR

        for (i <- 0 until intNiq) {
            when (payloadWriteOH(e)(i)) {
                nextEntries(i).uop := inUop
                nextEntries(i).src1Ready := src1ReadyAfterLoadPred
                nextEntries(i).src2Ready := src2ReadyAfterLoadPred
                nextEntries(i).src1FastWakeup :=
                    src1FastWakeupInit || src1LoadPredWakeInit.orR
                nextEntries(i).src2FastWakeup :=
                    src2FastWakeupInit || src2LoadPredWakeInit.orR
                nextEntries(i).src1LoadPoison := Mux(
                    sameCycleWakeup(inUop.reg.psrc1) ||
                        src1IntProducerFastWake,
                    0.U,
                    src1LoadPredWakeInit
                )
                nextEntries(i).src2LoadPoison := Mux(
                    sameCycleWakeup(inUop.reg.psrc2) ||
                        src2IntProducerFastWake,
                    0.U,
                    src2LoadPredWakeInit
                )
                nextEntries(i).src1IntProducerOH :=
                    src1IntProducerAfterRelease
                nextEntries(i).src2IntProducerOH :=
                    src2IntProducerAfterRelease

                when(src1ReadyAfterLoadPred) {
                    assert(!src1IntProducerAfterRelease.orR)
                }
                when(src2ReadyAfterLoadPred) {
                    assert(!src2IntProducerAfterRelease.orR)
                }
            }

            when (enqFire(e) && allocOH(e)(i)) {
                nextEntries(i).valid := true.B
            }
        }

        when (
            enqFire(e) &&
            inUop.reg.lsrc1Valid &&
            !inUop.reg.psrc1Ready
        ) {
            assert(
                src1IntProducerFastWake ===
                    legacySameCycleLocalFastWakeup(inUop.reg.psrc1),
                "IntIssueQueue: enqueued src1 producer-slot wake differs from tag wake"
            )
        }
        when (
            enqFire(e) &&
            inUop.reg.lsrc2Valid &&
            !inUop.reg.psrc2Ready
        ) {
            assert(
                src2IntProducerFastWake ===
                    legacySameCycleLocalFastWakeup(inUop.reg.psrc2),
                "IntIssueQueue: enqueued src2 producer-slot wake differs from tag wake"
            )
        }
    }

    when (io.flush) {
        entries := 0.U.asTypeOf(entries)
    }.otherwise {
        entries := nextEntries
    }

    for (p <- 0 until intNissue - 1) {
        when (selectAdvance(p)) {
            selectBaseOH(p) := ShiftAdd1(selectOH(p))
        }
    }
    when (p2DirectSelectAdvance) {
        selectBaseOH(2) := ShiftAdd1(selectOH(2))
    }

    assert(p2DirectSelectAdvance === selectAdvance(2))

    when (io.flush) {
        selectBaseOH := VecInit(
            Seq.fill(intNissue)(1.U(intNiq.W))
        )
    }

    assert(PopCount(io.dispatch.reqCountOH) === 1.U)
    assert(PopCount(aluAgeSeedOH) === 1.U)
    assert(freeIndices.io.allocatedMask === storedValidMask)
    when(load0Wake.valid) {
        assert(!load0Wake.bits.fast)
    }

    for (i <- 0 until ndcd) {
        assert(PopCount(allocOH(i)) <= 1.U)
        assert((allocOH(i) & storedValidMask) === 0.U)
    }

    for (i <- 0 until ndcd; j <- i + 1 until ndcd) {
        assert((allocOH(i) & allocOH(j)) === 0.U)

        when(!allocOH(i).orR) {
            assert(!allocOH(j).orR)
        }
    }

    for (p <- 0 until intNissue) {
        assert(PopCount(selectBaseOH(p)) === 1.U)
        assert(PopCount(selectOH(p)) <= 1.U)
        assert((specialSelectOH(p) & aluSelectOH(p)) === 0.U)
        assert(
            specialSelectOH(p).orR ===
                (portCanSelect(p) && specialCandidates(p).orR)
        )

        when(selectOH(p).orR) {
            assert((selectOH(p) & portReq(p)).orR)
        }

        when(selectFire(p)) {
            assert(
                (selectFireOH(p) & (~predictionResolvedMask).asUInt) === 0.U,
                "IntIssueQueue: unresolved predictive Load dependency issued"
            )
        }

        assert(selectedPdest(p) === selectedBits(p).uop.reg.pdest)

        val stalledLastCycle = RegNext(
            io.issue(p).valid && !io.issue(p).ready,
            false.B
        )
        // Keep the Decoupled stability assertion in machine-word-sized chunks.
        // A single wide history register makes recent Verilator versions emit a
        // separate constant-pool/PCH build that conflicts with ChiselTest's
        // forced top-header include.
        val issueBitChunks = io.issue(p).bits.asUInt.asBools
            .grouped(64)
            .map(bits => VecInit(bits).asUInt)
            .toSeq
        val bitsLastCycle = issueBitChunks.map(RegNext(_))

        when (
            stalledLastCycle &&
            !io.flush &&
            !io.branchUpdate.valid
        ) {
            assert(io.issue(p).valid)
            issueBitChunks.zip(bitsLastCycle).foreach { case (bits, previous) =>
                assert(bits === previous)
            }
        }

        when(issueRegKilled(p)) {
            assert(!io.issue(p).valid)
            assert(!io.issue(p).fire)
        }
    }

    for (p <- 0 until intNissue; q <- p + 1 until intNissue) {
        assert((selectOH(p) & selectOH(q)) === 0.U)
    }

    for (e <- 0 until ndcd) {
        assert(PopCount(allocOH(e)) <= 1.U)
        assert(PopCount(payloadWriteOH(e)) <= 1.U)

        when(payloadWriteOH(e).orR) {
            assert((payloadWriteOH(e) & storedValidMask) === 0.U)
        }

        when (enqFire(e)) {
            assert(io.dispatch.enq(e).bits.ctrl.iqType === IQT_INT)
            assert(rawEnqLane(e))
            assert(payloadWriteOH(e) === allocOH(e))
        }

        if (e > 0) {
            when (io.dispatch.enq(e).valid) {
                assert(io.dispatch.enq(e - 1).valid)
            }
        }
    }

    for (e <- 0 until ndcd; f <- e + 1 until ndcd) {
        assert((allocOH(e) & allocOH(f)) === 0.U)
    }

    val dispatchEnqValid = VecInit(io.dispatch.enq.map(_.valid))
    when (dispatchEnqValid.asUInt.orR) {
        assert(OHToUInt(io.dispatch.reqCountOH) === PopCount(dispatchEnqValid))
    }

}
