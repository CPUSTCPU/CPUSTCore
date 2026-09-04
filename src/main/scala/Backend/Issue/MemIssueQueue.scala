package CPUSTC.backend.issue

import chisel3._
import chisel3.util._

import CPUSTC.config._
import CPUSTC.config.Decode._
import CPUSTC.config.Issue._
import CPUSTC.config.IssueQueue._
import CPUSTC.config.MemIssueOp._
import CPUSTC.config.RenameConfig._
import CPUSTC.config.WritebackConfig._
import CPUSTC.backend.branch.BranchMask
import CPUSTC.backend.dispatch.DispatchUop
import CPUSTC.utils.{ClusterFreeIndexQueue, XilinxFdreWakeCatcher}

class MemIssueQueue extends Module {
    val io = IO(new MemIssueQueueIO)

    require(memNissue == 2)
    require(memNiq > ndcd)

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

    val entries = RegInit(
        VecInit(Seq.fill(memNiq)(0.U.asTypeOf(new MemIssueEntry)))
    )
    val releasedQ = RegInit(0.U(memNiq.W))
    val physicalValidMask = VecInit(entries.map(_.valid)).asUInt
    val liveValidMask = physicalValidMask & (~releasedQ).asUInt

    val addressAgeSelect = Module(new IssueAgeMatrix(
        numEntries = memNiq,
        enqWidth = ndcd,
        selectWidth = 1
    ))
    addressAgeSelect.io.flush := io.flush

    val addressSelectBaseOH = RegInit(1.U(memNiq.W))
    val dataSelectBaseOH    = RegInit(1.U(memNiq.W))

    // Each issue port has a one-entry elastic register. An IQ entry is released
    // when its sub-operation enters this register, not when execution completes.
    val issueRegValid = RegInit(
        VecInit(Seq.fill(memNissue)(false.B))
    )
    val issueRegBits = Reg(Vec(memNissue, new IssueOut))
    val issueRegKilled = Wire(Vec(memNissue, Bool()))
    val issueRegNewMask = Wire(Vec(memNissue, UInt(maxBrCount.W)))

    for (p <- 0 until memNissue) {
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
    }

    val issueRegCanAccept = Wire(Vec(memNissue, Bool()))
    for (p <- 0 until memNissue) {
        issueRegCanAccept(p) :=
            !issueRegValid(p) ||
                io.issue(p).ready
    }

    // Collapse tag/producer matching into one local bit per entry before it
    // reaches the age selector. The bit is bypassed during the same cycle in
    // which a conventional registered ready flag would first be visible.
    val restWakeCaptureSrc1 = WireInit(VecInit.fill(memNiq)(false.B))
    val restWakeCaptureSrc2 = WireInit(VecInit.fill(memNiq)(false.B))
    val fastCaptureSrc1 = WireInit(VecInit.fill(memNiq)(false.B))
    val fastCaptureSrc2 = WireInit(VecInit.fill(memNiq)(false.B))
    val pendingRestWakeSrc1 = RegInit(VecInit.fill(memNiq)(false.B))
    val pendingRestWakeSrc2 = RegInit(VecInit.fill(memNiq)(false.B))
    val pendingLoad0WakeSrc1 = Wire(Vec(memNiq, Bool()))
    val pendingLoad0WakeSrc2 = Wire(Vec(memNiq, Bool()))
    val pendingLoad1WakeSrc1 = Wire(Vec(memNiq, Bool()))
    val pendingLoad1WakeSrc2 = Wire(Vec(memNiq, Bool()))
    val pendingFastSrc1 = RegInit(VecInit.fill(memNiq)(false.B))
    val pendingFastSrc2 = RegInit(VecInit.fill(memNiq)(false.B))
    when(io.flush) {
        pendingFastSrc1 := VecInit.fill(memNiq)(false.B)
        pendingFastSrc2 := VecInit.fill(memNiq)(false.B)
    }.otherwise {
        pendingFastSrc1 := fastCaptureSrc1
        pendingFastSrc2 := fastCaptureSrc2
    }

    val wakeSrc1 = Wire(Vec(memNiq, Bool()))
    val wakeSrc2 = Wire(Vec(memNiq, Bool()))
    val fastWakeSrc1 = Wire(Vec(memNiq, Bool()))
    val fastWakeSrc2 = Wire(Vec(memNiq, Bool()))
    val intProducerFastWakeSrc1 = Wire(Vec(memNiq, Bool()))
    val intProducerFastWakeSrc2 = Wire(Vec(memNiq, Bool()))
    val legacyIntFastWakeSrc1 = Wire(Vec(memNiq, Bool()))
    val legacyIntFastWakeSrc2 = Wire(Vec(memNiq, Bool()))
    val loadPredWakeSrc1 = Wire(Vec(memNiq, UInt(memNissue.W)))
    val loadPredWakeSrc2 = Wire(Vec(memNiq, UInt(memNissue.W)))

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
    for (i <- 0 until memNiq) {
        wakeSrc1(i) :=
            pendingRestWakeSrc1(i) || pendingFastSrc1(i) ||
                pendingLoad0WakeSrc1(i) || pendingLoad1WakeSrc1(i)
        wakeSrc2(i) :=
            pendingRestWakeSrc2(i) || pendingFastSrc2(i) ||
                pendingLoad0WakeSrc2(i) || pendingLoad1WakeSrc2(i)
        fastWakeSrc1(i) := pendingFastSrc1(i)
        fastWakeSrc2(i) := pendingFastSrc2(i)

        legacyIntFastWakeSrc1(i) := io.intFastWakeup.map { w =>
            w.valid && w.bits.pdest === entries(i).uop.reg.psrc1
        }.reduce(_ || _)
        legacyIntFastWakeSrc2(i) := io.intFastWakeup.map { w =>
            w.valid && w.bits.pdest === entries(i).uop.reg.psrc2
        }.reduce(_ || _)

        intProducerFastWakeSrc1(i) :=
            (entries(i).src1IntProducerOH & io.intProducerFastWakeMask).orR
        intProducerFastWakeSrc2(i) :=
            (entries(i).src2IntProducerOH & io.intProducerFastWakeMask).orR

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

        when (
            liveValidMask(i) &&
            entries(i).uop.reg.lsrc1Valid &&
            !entries(i).src1Ready
        ) {
            assert(
                intProducerFastWakeSrc1(i) === legacyIntFastWakeSrc1(i),
                "MemIssueQueue: src1 producer-slot wake differs from tag wake"
            )
        }
        when (
            liveValidMask(i) &&
            entries(i).uop.reg.lsrc2Valid &&
            !entries(i).src2Ready
        ) {
            assert(
                intProducerFastWakeSrc2(i) === legacyIntFastWakeSrc2(i),
                "MemIssueQueue: src2 producer-slot wake differs from tag wake"
            )
        }
    }

    val effectiveSrc1Ready = VecInit((0 until memNiq).map { i =>
        entries(i).src1Ready || wakeSrc1(i)
    })
    val effectiveSrc2Ready = VecInit((0 until memNiq).map { i =>
        entries(i).src2Ready || wakeSrc2(i)
    })
    val effectiveSrc1FastWakeup = VecInit((0 until memNiq).map { i =>
        Mux(
            wakeSrc1(i),
            fastWakeSrc1(i),
            entries(i).src1FastWakeup
        )
    })
    val effectiveSrc2FastWakeup = VecInit((0 until memNiq).map { i =>
        Mux(
            wakeSrc2(i),
            fastWakeSrc2(i),
            entries(i).src2FastWakeup
        )
    })
    val effectiveSrc1LoadPoison = VecInit((0 until memNiq).map { i =>
        Mux(
            wakeSrc1(i),
            0.U(memNissue.W),
            entries(i).src1LoadPoison
        )
    })
    val effectiveSrc2LoadPoison = VecInit((0 until memNiq).map { i =>
        Mux(
            wakeSrc2(i),
            0.U(memNissue.W),
            entries(i).src2LoadPoison
        )
    })

    val entryKilled = Wire(Vec(memNiq, Bool()))

    for (i <- 0 until memNiq) {
        entryKilled(i) :=
            liveValidMask(i) &&
            BranchMask.isKilled(
                entries(i).uop.spec.brMask,
                mispredictMask
            )
    }

    val staEarlyAcceptedMask = io.staEarlyAccepted.map { accepted =>
        Mux(accepted.valid, accepted.bits.oh, 0.U)
    }.reduce(_ | _)
    val staAcceptedMask = io.staAccepted.map { accepted =>
        Mux(accepted.valid, accepted.bits.oh, 0.U)
    }.reduce(_ | _)
    val staDependencyClearMask = staEarlyAcceptedMask | staAcceptedMask

    val ldMask = VecInit((0 until memNiq).map { i =>
        val e = entries(i)
        liveValidMask(i) && e.uop.mem.isLoad && effectiveSrc1Ready(i) &&
            !e.uop.stDepMask.orR
    }).asUInt

    val staMask = VecInit((0 until memNiq).map { i =>
        val e = entries(i)
        liveValidMask(i) && e.uop.mem.isStore && !e.staIssued && effectiveSrc1Ready(i)
    }).asUInt

    val stdMask = VecInit((0 until memNiq).map { i =>
        val e = entries(i)
        liveValidMask(i) && e.uop.mem.isStore && !e.stdIssued && effectiveSrc2Ready(i)
    }).asUInt

    val addressMask = ldMask | staMask
    val dataMask    = stdMask
    val anyMask     = addressMask | dataMask

    addressAgeSelect.io.request(0) := addressMask

    io.validMask := VecInit((0 until memNiq).map { i =>
        liveValidMask(i) && !entryKilled(i)
    }).asUInt
    io.canIssueMask := anyMask & (~entryKilled.asUInt).asUInt
    io.full := io.validMask.andR
    val liveIssueRegMask = VecInit((0 until memNissue).map { p =>
        issueRegValid(p) && !issueRegKilled(p)
    }).asUInt
    io.empty := !io.validMask.orR && !liveIssueRegMask.orR

    val portReq = Wire(Vec(memNissue, UInt(memNiq.W)))
    for (p <- 0 until memNissue) {
        portReq(p) := VecInit((0 until memNiq).map { i =>
            anyMask(i) &&
            (entries(i).uop.ctrl.fuType & io.portCaps(p)).orR
        }).asUInt
    }

    // M0 prefers address operations, while M1 prefers store data. Each class
    // computes both grants in parallel, so neither port depends on the other
    // port's selected entry.
    val selectOH      = Wire(Vec(memNissue, UInt(memNiq.W)))
    val selectedLdOH  = Wire(Vec(memNissue, UInt(memNiq.W)))
    val selectedStaOH = Wire(Vec(memNissue, UInt(memNiq.W)))
    val selectedStdOH = Wire(Vec(memNissue, UInt(memNiq.W)))

    val oldestAddressOH = addressAgeSelect.io.oldestOH(0)
    // Compute the RR pair beside exact-oldest.  Selecting the first RR grant
    // that is not oldest is equivalent to RR(addressMask & ~oldest), but avoids
    // putting the age result in front of the secondary picker.
    val rrAddressGrantOH = PickNRotOHParallel(
        req    = addressMask,
        baseOH = addressSelectBaseOH,
        count  = memNissue
    )
    val rrFirstIsOldest = (rrAddressGrantOH(0) & oldestAddressOH).orR
    val secondaryAddressGrantOH = Mux(
        rrFirstIsOldest,
        rrAddressGrantOH(1),
        rrAddressGrantOH(0)
    )
    val addressGrantOH = Wire(Vec(memNissue, UInt(memNiq.W)))
    addressGrantOH(0) := oldestAddressOH
    addressGrantOH(1) := secondaryAddressGrantOH

    val dataGrantOH = PickNRotOHParallel(
        req    = dataMask,
        baseOH = dataSelectBaseOH,
        count  = memNissue
    )

    // Arbitration state is allowed to advance during a mispredict recovery.
    // Actual issue remains suppressed below, but keeping recovery out of this
    // eligibility term prevents branch feedback from driving the round-robin
    // state registers' enables.
    val portCanSelect = VecInit((0 until memNissue).map { p =>
        issueRegCanAccept(p) && !io.flush
    })
    val portAddressAvailable = VecInit((0 until memNissue).map { p =>
        (portReq(p) & addressMask).orR
    })
    val portDataAvailable = VecInit((0 until memNissue).map { p =>
        (portReq(p) & dataMask).orR
    })

    val portUsesAddress = Wire(Vec(memNissue, Bool()))
    val portUsesData    = Wire(Vec(memNissue, Bool()))

    portUsesAddress(0) := portCanSelect(0) && portAddressAvailable(0)
    portUsesData(0) :=
        portCanSelect(0) && !portUsesAddress(0) && portDataAvailable(0)

    portUsesData(1) := portCanSelect(1) && portDataAvailable(1)
    portUsesAddress(1) :=
        portCanSelect(1) && !portUsesData(1) && portAddressAvailable(1)

    selectOH(0) := Mux(
        portUsesAddress(0),
        addressGrantOH(0),
        Mux(portUsesData(0), dataGrantOH(0), 0.U(memNiq.W))
    )
    selectOH(1) := Mux(
        portUsesAddress(1),
        Mux(portUsesAddress(0), addressGrantOH(1), addressGrantOH(0)),
        Mux(
            portUsesData(1),
            Mux(portUsesData(0), dataGrantOH(1), dataGrantOH(0)),
            0.U(memNiq.W)
        )
    )

    for (p <- 0 until memNissue) {
        selectedLdOH(p) := Mux(
            portUsesAddress(p),
            selectOH(p) & ldMask,
            0.U(memNiq.W)
        )
        selectedStaOH(p) := Mux(
            portUsesAddress(p),
            selectOH(p) & staMask,
            0.U(memNiq.W)
        )
        selectedStdOH(p) := Mux(
            portUsesData(p),
            selectOH(p) & stdMask,
            0.U(memNiq.W)
        )
    }

    val selectedMemOp = Wire(Vec(memNissue, UInt(MEMQ_SZ.W)))
    for (p <- 0 until memNissue) {
        selectedMemOp(p) :=
            Mux(selectedLdOH(p).orR, MEM_LD,
                Mux(selectedStaOH(p).orR, MEM_STA,
                    Mux(selectedStdOH(p).orR, MEM_STD, MEM_X)))
    }

    val selectedBits = Wire(Vec(memNissue, new IssueOut))
    val selectedLoadPoison = Wire(Vec(memNissue, UInt(memNissue.W)))
    for (p <- 0 until memNissue) {
        val selectedValid = selectOH(p).orR
        // Mux1H naturally produces zero when no entry is selected. Keep that
        // don't-care case inside the one-hot mux instead of factoring a shared
        // entry-0 fallback across the entire wide payload.
        val selectedUop = Mux1H(
            selectOH(p),
            VecInit(entries.map(_.uop))
        )

        selectedBits(p) := 0.U.asTypeOf(new IssueOut)
        selectedBits(p).uop := selectedUop
        selectedBits(p).uop.spec.brMask :=
            BranchMask.clearResolved(
                selectedUop.spec.brMask,
                resolveMask
            )
        selectedBits(p).uop.stDepMask := selectedUop.stDepMask
        selectedBits(p).memOp := selectedMemOp(p)
        selectedBits(p).src1Read :=
            selectedMemOp(p) === MEM_LD || selectedMemOp(p) === MEM_STA
        selectedBits(p).src2Read := selectedMemOp(p) === MEM_STD
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
        selectedLoadPoison(p) := Mux(
            selectedMemOp(p) === MEM_STD,
            Mux1H(selectOH(p), effectiveSrc2LoadPoison),
            Mux1H(selectOH(p), effectiveSrc1LoadPoison)
        )

    }

    val selectAdvance = VecInit((0 until memNissue).map { p =>
        selectOH(p).orR &&
            issueRegCanAccept(p) &&
            !io.flush
    })
    val selectAdvanceOH = VecInit((0 until memNissue).map { p =>
        Mux(selectAdvance(p), selectOH(p), 0.U(memNiq.W))
    })
    val legacySelectFireOH = VecInit((0 until memNissue).map { p =>
        val loadPredictionResolved =
            (selectedLoadPoison(p) & (~loadPredSuccessMask).asUInt) === 0.U
        Mux(
            !mispredict && loadPredictionResolved,
            selectAdvanceOH(p),
            0.U(memNiq.W)
        )
    })

    // Settle the two address and data grant ranks independently. State updates
    // can then use these narrow class-local events instead of selecting a
    // payload, reconstructing its MemOp, and decoding it back into LD/STA/STD.
    val addressGrantAdvance = VecInit(Seq(
        portUsesAddress.asUInt.orR && addressGrantOH(0).orR,
        portUsesAddress.asUInt.andR && addressGrantOH(1).orR
    ))
    val dataGrantAdvance = VecInit(Seq(
        portUsesData.asUInt.orR && dataGrantOH(0).orR,
        portUsesData.asUInt.andR && dataGrantOH(1).orR
    ))

    val addressPredResolvedMask = VecInit((0 until memNiq).map { i =>
        (effectiveSrc1LoadPoison(i) &
            (~loadPredSuccessMask).asUInt) === 0.U
    }).asUInt
    val dataPredResolvedMask = VecInit((0 until memNiq).map { i =>
        (effectiveSrc2LoadPoison(i) &
            (~loadPredSuccessMask).asUInt) === 0.U
    }).asUInt

    val addressGrantFireOH = VecInit((0 until memNissue).map { rank =>
        Mux(
            addressGrantAdvance(rank) && !mispredict,
            addressGrantOH(rank) & addressPredResolvedMask,
            0.U(memNiq.W)
        )
    })
    val dataGrantFireOH = VecInit((0 until memNissue).map { rank =>
        Mux(
            dataGrantAdvance(rank) && !mispredict,
            dataGrantOH(rank) & dataPredResolvedMask,
            0.U(memNiq.W)
        )
    })

    val selectFireOH = Wire(Vec(memNissue, UInt(memNiq.W)))
    selectFireOH(0) := Mux(
        portUsesAddress(0),
        addressGrantFireOH(0),
        Mux(portUsesData(0), dataGrantFireOH(0), 0.U(memNiq.W))
    )
    selectFireOH(1) := Mux(
        portUsesAddress(1),
        Mux(portUsesAddress(0), addressGrantFireOH(1), addressGrantFireOH(0)),
        Mux(
            portUsesData(1),
            Mux(portUsesData(0), dataGrantFireOH(1), dataGrantFireOH(0)),
            0.U(memNiq.W)
        )
    )
    val selectFire = VecInit((0 until memNissue).map { p =>
        selectFireOH(p).orR
    })

    val addressPoisonPresentMask = VecInit(
        effectiveSrc1LoadPoison.map(_.orR)
    ).asUInt
    val dataPoisonPresentMask = VecInit(
        effectiveSrc2LoadPoison.map(_.orR)
    ).asUInt
    io.loadPredIssueCount := PopCount(VecInit(
        (0 until memNissue).flatMap { rank =>
            Seq(
                (addressGrantFireOH(rank) & addressPoisonPresentMask).orR,
                (dataGrantFireOH(rank) & dataPoisonPresentMask).orR
            )
        }
    ))
    io.loadPredLoadIssueCount := PopCount(VecInit(
        (0 until memNissue).map { rank =>
            (addressGrantFireOH(rank) & ldMask &
                addressPoisonPresentMask).orR
        }
    ))

    val addressFireMask = addressGrantFireOH.reduce(_ | _)
    val dataFireMask = dataGrantFireOH.reduce(_ | _)
    val selectFireMask = addressFireMask | dataFireMask

    val ldFireOH = VecInit((0 until memNissue).map { p =>
        Mux(
            selectFire(p) && selectedMemOp(p) === MEM_LD,
            selectOH(p),
            0.U(memNiq.W)
        )
    })

    val staFireOH = VecInit((0 until memNissue).map { p =>
        Mux(
            selectFire(p) && selectedMemOp(p) === MEM_STA,
            selectOH(p),
            0.U(memNiq.W)
        )
    })

    val stdFireOH = VecInit((0 until memNissue).map { p =>
        Mux(
            selectFire(p) && selectedMemOp(p) === MEM_STD,
            selectOH(p),
            0.U(memNiq.W)
        )
    })

    val ldFireMask  = addressFireMask & ldMask
    val staFireMask = addressFireMask & staMask
    val stdFireMask = dataFireMask

    val staIssuedMask = VecInit((0 until memNiq).map { i =>
        entries(i).staIssued
    }).asUInt
    val stdIssuedMask = VecInit((0 until memNiq).map { i =>
        entries(i).stdIssued
    }).asUInt

    // A live store can never have both halves marked issued: issuing the
    // second half releases it in this cycle. Complete stores directly from
    // the issue events so releasedQ does not depend on live/type decoding.
    val storeCompleteMask =
        (staFireMask & (stdIssuedMask | stdFireMask)) |
        (stdFireMask & staIssuedMask)

    // Loads leave after LD issue. Stores stay resident until both STA and STD
    // have entered an Issue Register, possibly in different cycles.
    val releaseMask = ldFireMask | storeCompleteMask

    // Normal releases already stop in the free-index queue's registered return
    // stage. Keep the backing valid bit for one extra cycle, but make the entry
    // logically dead immediately. This removes the issue/release arbitration
    // cone from entries.valid while preserving the existing reuse cycle.
    when(io.flush) {
        releasedQ := 0.U
    }.otherwise {
        releasedQ := releaseMask
    }

    for (p <- 0 until memNissue) {
        when (io.flush) {
            issueRegValid(p) := false.B
        }.elsewhen(issueRegKilled(p)) {
            issueRegValid(p) := false.B
        }.elsewhen(mispredict) {
            // Output valid is globally suppressed during recovery, so downstream
            // ready cannot consume an independent entry in this cycle.
            issueRegBits(p).uop.spec.brMask := issueRegNewMask(p)
        }.elsewhen (issueRegCanAccept(p)) {
            issueRegValid(p) := selectFire(p)
            // Payload is don't-care whenever valid is clear. Writing it on every
            // accepted cycle keeps the wide register bank off selectFire's CE cone.
            issueRegBits(p) := selectedBits(p)
        }.elsewhen(io.branchUpdate.valid) {
            issueRegBits(p).uop.spec.brMask := issueRegNewMask(p)
        }

        io.issue(p).valid :=
            issueRegValid(p) &&
            !io.flush &&
            !mispredict &&
            !issueRegKilled(p)
        io.issue(p).bits := issueRegBits(p)
        io.issue(p).bits.uop.spec.brMask := issueRegNewMask(p)
    }

    val storedValidMask = liveValidMask
    val freeIndices = Module(new ClusterFreeIndexQueue(
        numEntries = memNiq,
        allocWidth = ndcd,
        useExternalRegisteredReturn = true
    ))
    val allocOH = freeIndices.io.allocOH

    io.dispatch.canAccept :=
        !io.flush &&
        freeIndices.io.canAllocate

    val enqFire = VecInit((0 until ndcd).map { i =>
        io.dispatch.enq(i).valid && io.dispatch.canAccept && !io.flush && !mispredict
    })

    freeIndices.io.flush := io.flush
    freeIndices.io.allocCount := OHToUInt(io.dispatch.reqCountOH)
    freeIndices.io.allocate := enqFire.asUInt.orR

    // releasedQ is the registered set of entries released in the preceding
    // cycle. Current issue/select logic therefore stops at releasedQ and cannot
    // reach the free-index counters or bank write controls.
    val registeredReturnOH = WireInit(
        VecInit.fill(ndcd)(0.U(memNiq.W))
    )
    val releasedCandidates = PickNRotOHParallel(
        req = releasedQ,
        baseOH = 1.U(memNiq.W),
        count = memNissue
    )
    for (lane <- 0 until memNissue) {
        registeredReturnOH(lane) := releasedCandidates(lane)
    }
    freeIndices.io.fastReleaseOH := VecInit.fill(ndcd)(0.U(memNiq.W))
    freeIndices.io.registeredReturnOH := registeredReturnOH
    val registeredReturnCount = Wire(UInt(log2Ceil(ndcd + 1).W))
    registeredReturnCount := PopCount(releasedQ)
    freeIndices.io.registeredReturnCount := registeredReturnCount
    freeIndices.io.normalReleaseMask := releaseMask
    freeIndices.io.releaseMask := entryKilled.asUInt

    for (lane <- 0 until ndcd) {
        addressAgeSelect.io.enqOH(lane) := Mux(
            enqFire(lane),
            allocOH(lane),
            0.U(memNiq.W)
        )
    }

    def sameCycleLoadPredWake(preg: UInt, valid: Bool): UInt = {
        VecInit(io.loadPredWake.map { wake =>
            valid && preg =/= 0.U && wake.valid && wake.bits.pdest === preg
        }).asUInt
    }

    private val load0WakePort = nIntWb
    private val load1WakePort = nIntWb + 1
    require(nLoadWb == 2)
    require(load0WakePort < nwkp)
    require(load1WakePort < nwkp)

    def sameCycleWakeupFrom(
        ports: Seq[Int],
        preg: UInt,
        valid: Bool
    ): Bool = {
        ports.map { port =>
            val wake = io.wakeup(port)
            valid && preg =/= 0.U && wake.valid && wake.bits.pdest === preg
        }.reduce(_ || _)
    }

    def sameCycleRestWakeup(preg: UInt, valid: Bool): Bool =
        sameCycleWakeupFrom(
            (0 until nwkp).filter { port =>
                port != load0WakePort && port != load1WakePort
            },
            preg,
            valid
        )

    def sameCycleFastWakeup(preg: UInt, valid: Bool): Bool = {
        io.wakeup.map { wake =>
            valid && preg =/= 0.U && wake.valid && wake.bits.fast &&
                wake.bits.pdest === preg
        }.reduce(_ || _)
    }

    val load0Wake = io.wakeup(load0WakePort)
    val load1Wake = io.wakeup(load1WakePort)
    val dispatchLoad0Src1Match = VecInit(io.dispatch.enq.map { enq =>
        load0Wake.bits.pdest === enq.bits.reg.psrc1
    })
    val dispatchLoad0Src2Match = VecInit(io.dispatch.enq.map { enq =>
        load0Wake.bits.pdest === enq.bits.reg.psrc2
    })
    val dispatchLoad1Src1Match = VecInit(io.dispatch.enq.map { enq =>
        load1Wake.bits.pdest === enq.bits.reg.psrc1
    })
    val dispatchLoad1Src2Match = VecInit(io.dispatch.enq.map { enq =>
        load1Wake.bits.pdest === enq.bits.reg.psrc2
    })

    for (i <- 0 until memNiq) {
        val enqToSlot = VecInit((0 until ndcd).map { e =>
            enqFire(e) && allocOH(e)(i)
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
        val load1Src1Match = Mux(
            incomingOwner,
            Mux1H(enqToSlot, dispatchLoad1Src1Match),
            load1Wake.bits.pdest === entries(i).uop.reg.psrc1
        )
        val load1Src2Match = Mux(
            incomingOwner,
            Mux1H(enqToSlot, dispatchLoad1Src2Match),
            load1Wake.bits.pdest === entries(i).uop.reg.psrc2
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

        val load1Src1Catcher = Module(new XilinxFdreWakeCatcher)
        load1Src1Catcher.io.clock := clock
        load1Src1Catcher.io.enable := true.B
        load1Src1Catcher.io.data := load1Wake.valid && load1Src1Match
        load1Src1Catcher.io.reset := false.B
        pendingLoad1WakeSrc1(i) := load1Src1Catcher.io.out

        val load1Src2Catcher = Module(new XilinxFdreWakeCatcher)
        load1Src2Catcher.io.clock := clock
        load1Src2Catcher.io.enable := true.B
        load1Src2Catcher.io.data := load1Wake.valid && load1Src2Match
        load1Src2Catcher.io.reset := false.B
        pendingLoad1WakeSrc2(i) := load1Src2Catcher.io.out

        val residentSrc1Valid =
            liveValidMask(i) && entries(i).uop.reg.lsrc1Valid
        val residentSrc2Valid =
            liveValidMask(i) && entries(i).uop.reg.lsrc2Valid
        val residentRestWakeSrc1 = sameCycleRestWakeup(
            entries(i).uop.reg.psrc1,
            residentSrc1Valid
        )
        val residentRestWakeSrc2 = sameCycleRestWakeup(
            entries(i).uop.reg.psrc2,
            residentSrc2Valid
        )
        val residentFastSrc1 = sameCycleFastWakeup(
            entries(i).uop.reg.psrc1,
            residentSrc1Valid
        )
        val residentFastSrc2 = sameCycleFastWakeup(
            entries(i).uop.reg.psrc2,
            residentSrc2Valid
        )

        val incomingWakeSrc1 = VecInit((0 until ndcd).map { e =>
            val inUop = io.dispatch.enq(e).bits
            enqFire(e) && allocOH(e)(i) && sameCycleRestWakeup(
                inUop.reg.psrc1,
                inUop.reg.lsrc1Valid && !inUop.reg.psrc1Ready
            )
        }).asUInt.orR
        val incomingWakeSrc2 = VecInit((0 until ndcd).map { e =>
            val inUop = io.dispatch.enq(e).bits
            enqFire(e) && allocOH(e)(i) && sameCycleRestWakeup(
                inUop.reg.psrc2,
                inUop.reg.lsrc2Valid && !inUop.reg.psrc2Ready
            )
        }).asUInt.orR
        val incomingFastSrc1 = VecInit((0 until ndcd).map { e =>
            val inUop = io.dispatch.enq(e).bits
            enqFire(e) && allocOH(e)(i) && sameCycleFastWakeup(
                inUop.reg.psrc1,
                inUop.reg.lsrc1Valid && !inUop.reg.psrc1Ready
            )
        }).asUInt.orR
        val incomingFastSrc2 = VecInit((0 until ndcd).map { e =>
            val inUop = io.dispatch.enq(e).bits
            enqFire(e) && allocOH(e)(i) && sameCycleFastWakeup(
                inUop.reg.psrc2,
                inUop.reg.lsrc2Valid && !inUop.reg.psrc2Ready
            )
        }).asUInt.orR
        val incomingIntFastSrc1 = VecInit((0 until ndcd).map { e =>
            enqFire(e) && allocOH(e)(i) &&
                (io.dispatch.src1IntProducerOH(e) &
                    io.intProducerFastWakeMask).orR
        }).asUInt.orR
        val incomingIntFastSrc2 = VecInit((0 until ndcd).map { e =>
            enqFire(e) && allocOH(e)(i) &&
                (io.dispatch.src2IntProducerOH(e) &
                    io.intProducerFastWakeMask).orR
        }).asUInt.orR

        val residentIntFastSrc1 =
            liveValidMask(i) && intProducerFastWakeSrc1(i)
        val residentIntFastSrc2 =
            liveValidMask(i) && intProducerFastWakeSrc2(i)

        restWakeCaptureSrc1(i) :=
            residentRestWakeSrc1 || incomingWakeSrc1
        restWakeCaptureSrc2(i) :=
            residentRestWakeSrc2 || incomingWakeSrc2
        fastCaptureSrc1(i) :=
            residentFastSrc1 || residentIntFastSrc1 ||
                incomingFastSrc1 || incomingIntFastSrc1
        fastCaptureSrc2(i) :=
            residentFastSrc2 || residentIntFastSrc2 ||
                incomingFastSrc2 || incomingIntFastSrc2
    }

    when(io.flush) {
        pendingRestWakeSrc1 := VecInit.fill(memNiq)(false.B)
        pendingRestWakeSrc2 := VecInit.fill(memNiq)(false.B)
    }.otherwise {
        pendingRestWakeSrc1 := restWakeCaptureSrc1
        pendingRestWakeSrc2 := restWakeCaptureSrc2
    }

    val nextEntries = WireInit(entries)

    for (i <- 0 until memNiq) {
        when (liveValidMask(i)) {
            val src1DeterministicWake = wakeSrc1(i)
            val src2DeterministicWake = wakeSrc2(i)
            val src1IntProducerAfterRelease =
                entries(i).src1IntProducerOH &
                    (~io.intProducerReleaseMask).asUInt
            val src2IntProducerAfterRelease =
                entries(i).src2IntProducerOH &
                    (~io.intProducerReleaseMask).asUInt

            nextEntries(i).uop.spec.brMask :=
                BranchMask.clearResolved(
                    entries(i).uop.spec.brMask,
                    resolveMask
                )
            nextEntries(i).uop.stDepMask :=
                entries(i).uop.stDepMask & (~staDependencyClearMask).asUInt
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
                src1DeterministicWake,
                fastWakeSrc1(i) || intProducerFastWakeSrc1(i),
                Mux(loadPredWakeSrc1(i).orR, true.B,
                    Mux(src1LoadCanceled(i), false.B, entries(i).src1FastWakeup))
            )
            nextEntries(i).src2FastWakeup := Mux(
                src2DeterministicWake,
                fastWakeSrc2(i) || intProducerFastWakeSrc2(i),
                Mux(loadPredWakeSrc2(i).orR, true.B,
                    Mux(src2LoadCanceled(i), false.B, entries(i).src2FastWakeup))
            )
            nextEntries(i).src1LoadPoison :=
                Mux(
                    src1DeterministicWake,
                    0.U,
                    (entries(i).src1LoadPoison &
                        (~loadPredResolveMask).asUInt) | loadPredWakeSrc1(i)
                )
            nextEntries(i).src2LoadPoison :=
                Mux(
                    src2DeterministicWake,
                    0.U,
                    (entries(i).src2LoadPoison &
                        (~loadPredResolveMask).asUInt) | loadPredWakeSrc2(i)
                )
            nextEntries(i).src1IntProducerOH := Mux(
                entryKilled(i),
                0.U,
                Mux(src1DeterministicWake, 0.U, src1IntProducerAfterRelease)
            )
            nextEntries(i).src2IntProducerOH := Mux(
                entryKilled(i),
                0.U,
                Mux(src2DeterministicWake, 0.U, src2IntProducerAfterRelease)
            )

            when (
                !entryKilled(i) &&
                (wakeSrc1(i) || loadPredWakeSrc1(i).orR)
            ) {
                assert(!src1IntProducerAfterRelease.orR)
            }
            when (
                !entryKilled(i) &&
                (wakeSrc2(i) || loadPredWakeSrc2(i).orR)
            ) {
                assert(!src2IntProducerAfterRelease.orR)
            }
            nextEntries(i).staIssued := entries(i).staIssued || staFireMask(i)
            nextEntries(i).stdIssued := entries(i).stdIssued || stdFireMask(i)
        }

        when (releasedQ(i)) {
            nextEntries(i).valid := false.B
        }

        when(entryKilled(i)) {
            nextEntries(i).valid := false.B
        }
    }

    for (e <- 0 until ndcd) {
        val inUop = io.dispatch.enq(e).bits
        val src1IntProducerOH = io.dispatch.src1IntProducerOH(e)
        val src2IntProducerOH = io.dispatch.src2IntProducerOH(e)
        val src1IntProducerAfterRelease =
            src1IntProducerOH & (~io.intProducerReleaseMask).asUInt
        val src2IntProducerAfterRelease =
            src2IntProducerOH & (~io.intProducerReleaseMask).asUInt

        val src1ReadyInit =
            !inUop.reg.lsrc1Valid ||
            inUop.reg.psrc1 === 0.U ||
            inUop.reg.psrc1Ready

        val src2ReadyInit =
            !inUop.reg.lsrc2Valid ||
            inUop.reg.psrc2 === 0.U ||
            inUop.reg.psrc2Ready

        val src1LoadPredWakeInit = sameCycleLoadPredWake(
            inUop.reg.psrc1,
            inUop.reg.lsrc1Valid && !inUop.reg.psrc1Ready
        )
        val src2LoadPredWakeInit = sameCycleLoadPredWake(
            inUop.reg.psrc2,
            inUop.reg.lsrc2Valid && !inUop.reg.psrc2Ready
        )

        for (i <- 0 until memNiq) {
            when (enqFire(e) && allocOH(e)(i)) {
                nextEntries(i).valid := true.B
                nextEntries(i).uop := inUop
                nextEntries(i).uop.stDepMask :=
                    inUop.stDepMask & (~staDependencyClearMask).asUInt
                nextEntries(i).src1Ready :=
                    src1ReadyInit || src1LoadPredWakeInit.orR
                nextEntries(i).src2Ready :=
                    src2ReadyInit || src2LoadPredWakeInit.orR
                nextEntries(i).src1FastWakeup :=
                    src1LoadPredWakeInit.orR
                nextEntries(i).src2FastWakeup :=
                    src2LoadPredWakeInit.orR
                nextEntries(i).src1LoadPoison := src1LoadPredWakeInit
                nextEntries(i).src2LoadPoison := src2LoadPredWakeInit
                nextEntries(i).src1IntProducerOH := src1IntProducerAfterRelease
                nextEntries(i).src2IntProducerOH := src2IntProducerAfterRelease

                when(
                    src1ReadyInit || src1LoadPredWakeInit.orR
                ) {
                    assert(!src1IntProducerAfterRelease.orR)
                }
                when(
                    src2ReadyInit || src2LoadPredWakeInit.orR
                ) {
                    assert(!src2IntProducerAfterRelease.orR)
                }
                nextEntries(i).staIssued := false.B
                nextEntries(i).stdIssued := false.B
            }
        }

    }

    when (io.flush) {
        entries := 0.U.asTypeOf(entries)
    }.otherwise {
        entries := nextEntries
    }

    val lastAddressAdvanceOH = Mux(
        addressGrantAdvance(1),
        addressGrantOH(1),
        Mux(addressGrantAdvance(0), addressGrantOH(0), 0.U(memNiq.W))
    )
    val lastDataAdvanceOH = Mux(
        dataGrantAdvance(1),
        dataGrantOH(1),
        Mux(dataGrantAdvance(0), dataGrantOH(0), 0.U(memNiq.W))
    )

    when(portUsesAddress.asUInt.orR) {
        addressSelectBaseOH := ShiftAdd1(lastAddressAdvanceOH)
    }
    when(portUsesData.asUInt.orR) {
        dataSelectBaseOH := ShiftAdd1(lastDataAdvanceOH)
    }

    when (io.flush) {
        addressSelectBaseOH := 1.U
        dataSelectBaseOH    := 1.U
    }

    assert(PopCount(io.dispatch.reqCountOH) === 1.U)
    assert(freeIndices.io.allocatedMask === storedValidMask)
    assert((releasedQ & (~physicalValidMask).asUInt) === 0.U)
    assert((releaseMask & releasedQ) === 0.U)

    for (p <- 0 until memNissue) {
        when(io.staEarlyAccepted(p).valid) {
            assert(PopCount(io.staEarlyAccepted(p).bits.oh) === 1.U)
        }
        when(io.staAccepted(p).valid) {
            assert(PopCount(io.staAccepted(p).bits.oh) === 1.U)
        }
    }
    assert(PopCount(releasedQ) <= memNissue.U)
    assert(registeredReturnOH.reduce(_ | _) === releasedQ)
    assert(registeredReturnCount === PopCount(releasedQ))
    when(io.wakeup(load0WakePort).valid) {
        assert(!io.wakeup(load0WakePort).bits.fast)
    }
    when(io.wakeup(load1WakePort).valid) {
        assert(!io.wakeup(load1WakePort).bits.fast)
    }
    when(io.flush) {
        assert(!io.wakeup(load0WakePort).valid)
        assert(!io.wakeup(load1WakePort).valid)
    }

    for (i <- 0 until ndcd) {
        assert(PopCount(allocOH(i)) <= 1.U)
        assert((allocOH(i) & storedValidMask) === 0.U)
        assert(PopCount(registeredReturnOH(i)) <= 1.U)
    }

    for (i <- 0 until ndcd; j <- i + 1 until ndcd) {
        assert((allocOH(i) & allocOH(j)) === 0.U)
        assert((registeredReturnOH(i) & registeredReturnOH(j)) === 0.U)

        when(!allocOH(i).orR) {
            assert(!allocOH(j).orR)
        }
    }

    assert((ldFireMask | staFireMask | stdFireMask) === selectFireMask)
    for (p <- 0 until memNissue) {
        assert(selectFireOH(p) === legacySelectFireOH(p))
    }
    assert(portUsesAddress.asUInt.orR === lastAddressAdvanceOH.orR)
    assert(portUsesData.asUInt.orR === lastDataAdvanceOH.orR)
    assert((ldFireMask & staFireMask) === 0.U)
    assert((ldFireMask & stdFireMask) === 0.U)
    val liveStoreMask = VecInit((0 until memNiq).map { i =>
        liveValidMask(i) && entries(i).uop.mem.isStore
    }).asUInt
    val legacyStoreCompleteMask =
        liveStoreMask &
        (staIssuedMask | staFireMask) &
        (stdIssuedMask | stdFireMask)
    assert((liveStoreMask & staIssuedMask & stdIssuedMask) === 0.U)
    assert((staFireMask & staIssuedMask) === 0.U)
    assert((stdFireMask & stdIssuedMask) === 0.U)
    assert(storeCompleteMask === legacyStoreCompleteMask)
    when(mispredict) {
        assert(!selectFire.asUInt.orR)
        assert(!releaseMask.orR)
    }

    assert(PopCount(addressSelectBaseOH) === 1.U)
    assert(PopCount(dataSelectBaseOH) === 1.U)

    for (p <- 0 until memNissue) {
        assert(PopCount(selectOH(p)) <= 1.U)

        when (selectFire(p)) {
            assert(selectedMemOp(p) =/= MEM_X)
            assert((selectOH(p) & portReq(p)) === selectOH(p))
            assert(
                (selectedLoadPoison(p) & (~loadPredSuccessMask).asUInt) === 0.U,
                "MemIssueQueue: unresolved predictive Load dependency issued"
            )
        }

        val stalledLastCycle = RegNext(
            io.issue(p).valid && !io.issue(p).ready,
            false.B
        )
        val bitsLastCycle = RegNext(io.issue(p).bits.asUInt)

        when (
            stalledLastCycle &&
            !io.flush &&
            !io.branchUpdate.valid
        ) {
            assert(io.issue(p).valid)
            assert(io.issue(p).bits.asUInt === bitsLastCycle)
        }

        when(issueRegKilled(p)) {
            assert(!io.issue(p).valid)
            assert(!io.issue(p).fire)
        }
    }

    for (p <- 0 until memNissue; q <- p + 1 until memNissue) {
        assert((ldFireOH(p) & ldFireOH(q)) === 0.U)
        assert((staFireOH(p) & staFireOH(q)) === 0.U)
        assert((stdFireOH(p) & stdFireOH(q)) === 0.U)

        val legalStorePair =
            (staFireOH(p) & stdFireOH(q)) |
            (stdFireOH(p) & staFireOH(q))

        assert((selectFireOH(p) & selectFireOH(q)) === legalStorePair)
    }

    for (i <- 0 until memNiq) {
        when (releaseMask(i) && entries(i).uop.mem.isStore) {
            assert(entries(i).staIssued || staFireMask(i))
            assert(entries(i).stdIssued || stdFireMask(i))
        }
    }

    for (e <- 0 until ndcd) {
        assert(PopCount(allocOH(e)) <= 1.U)

        when (enqFire(e)) {
            assert(io.dispatch.enq(e).bits.ctrl.iqType === IQT_MEM)
            assert(
                io.dispatch.enq(e).bits.mem.isLoad =/=
                io.dispatch.enq(e).bits.mem.isStore
            )
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
