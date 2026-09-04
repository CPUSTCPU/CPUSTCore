package CPUSTC.backend.dispatch

import chisel3._
import chisel3.util._

import CPUSTC.config.Decode._
import CPUSTC.config.Commit._
import CPUSTC.config.LoadStoreQueue._
import CPUSTC.config.RenameConfig._
import CPUSTC.config.Issue._
import CPUSTC.backend.branch.BranchUpdate

class LsqAllocatorIO extends Bundle {
    val flush = Input(Bool())

    val dispatch = new LsqDispatchIO

    val ldqRelease = Input(Vec(ncmt, Bool()))
    val stqFreed = Flipped(Valid(UInt(nstq.W)))
    val stqCommitPtr = Input(new StqPtr)
    val stqCommittedMask = Input(UInt(nstq.W))
    val staAccepted = Input(Vec(memNissue, Valid(new StqPtr)))

    val ldqFull = Output(Bool())
    val stqFull = Output(Bool())

    val branchUpdate = Flipped(Valid(new BranchUpdate))

    val liveState = Output(new LsqLiveState)
    val writebackLiveState = Output(new LsqLiveState)
    val stqHeadCurrent = Output(new StqPtr)
    val ldqFlushMask = Output(UInt(nldq.W))
    val stqFlushMask = Output(UInt(nstq.W))
    val stqRecoveryKillMask = Output(UInt(nstq.W))
}

class LsqAllocator extends Module {
    val io = IO(new LsqAllocatorIO)

    val ldqTail = RegInit(LdqPtr.init)
    val ldqHead = RegInit(LdqPtr.init)

    val stqTail = RegInit(StqPtr.init)
    val stqHead = RegInit(StqPtr.init)

    val ldqValidMask = RegInit(0.U(nldq.W))
    val stqValidMask = RegInit(0.U(nstq.W))
    val stAddrReadyMask = RegInit(0.U(nstq.W))

    val ldqHighMask = RegInit(0.U(nldq.W))
    val stqHighMask = RegInit(0.U(nstq.W))

    // Admission only needs queue capacity, not the identity of every live
    // slot. Keep compact registered occupancy beside the allocator and retain
    // the masks below as the architectural source of pointer/liveness truth.
    val ldOcc = RegInit(0.U(log2Ceil(nldq + 1).W))
    val stOcc = RegInit(0.U(log2Ceil(nstq + 1).W))

    val ldReq = VecInit((0 until ndcd).map { i =>
        io.dispatch.req(i).valid && io.dispatch.req(i).bits.isLoad
    })

    val stReq = VecInit((0 until ndcd).map { i =>
        io.dispatch.req(i).valid && io.dispatch.req(i).bits.isStore
    })

    val ldTailAfterBr = Reg(Vec(maxBrCount, new LdqPtr))
    val stTailAfterBr = Reg(Vec(maxBrCount, new StqPtr))
    val ldAllocsAfterBr = RegInit(VecInit.fill(maxBrCount)(0.U(nldq.W)))
    val stAllocsAfterBr = RegInit(VecInit.fill(maxBrCount)(0.U(nstq.W)))
    val snapshotValid = RegInit(0.U(maxBrCount.W))

    def nextStqPtr(ptr: StqPtr): StqPtr = {
        val out = Wire(new StqPtr)

        val wrap = ptr.oh(nstq - 1)
        out.oh   := Cat(ptr.oh(nstq - 2, 0), ptr.oh(nstq - 1))
        out.flag := ptr.flag ^ wrap

        out
    }

    def nextLdqPtr(ptr: LdqPtr): LdqPtr = {
        val out = Wire(new LdqPtr)

        val wrap = ptr.oh(nldq - 1)
        out.oh   := Cat(ptr.oh(nldq - 2, 0), ptr.oh(nldq - 1))
        out.flag := ptr.flag ^ wrap

        out
    }

    val ldCursor = Wire(Vec(ndcd + 1, new LdqPtr))
    val stCursor = Wire(Vec(ndcd + 1, new StqPtr))

    ldCursor(0) := ldqTail
    stCursor(0) := stqTail

    for (i <- 0 until ndcd) {
        io.dispatch.resp(i).bits.ldqIdx := ldCursor(i)
        io.dispatch.resp(i).bits.stqIdx := stCursor(i)

        ldCursor(i + 1) := Mux(ldReq(i), nextLdqPtr(ldCursor(i)), ldCursor(i))
        stCursor(i + 1) := Mux(stReq(i), nextStqPtr(stCursor(i)), stCursor(i))
    }

    val ldRelCursor = Wire(Vec(ncmt + 1, new LdqPtr))
    val stRelCursor = Wire(Vec(nstq + 1, new StqPtr))

    ldRelCursor(0) := ldqHead
    stRelCursor(0) := stqHead

    val ldReleaseMaskVec = Wire(Vec(ncmt, UInt(nldq.W)))
    val stReleaseCount = Mux(io.stqFreed.valid, PopCount(io.stqFreed.bits), 0.U)
    val stReleaseMaskVec = Wire(Vec(nstq, UInt(nstq.W)))

    for (i <- 0 until ncmt) {
        ldReleaseMaskVec(i) := Mux(io.ldqRelease(i), ldRelCursor(i).oh, 0.U(nldq.W))
        ldRelCursor(i + 1) := Mux(io.ldqRelease(i), nextLdqPtr(ldRelCursor(i)), ldRelCursor(i))
    }

    for (i <- 0 until nstq) {
        val releaseThis = i.U < stReleaseCount
        stReleaseMaskVec(i) := Mux(releaseThis, stRelCursor(i).oh, 0.U(nstq.W))
        stRelCursor(i + 1) := Mux(
            releaseThis,
            nextStqPtr(stRelCursor(i)),
            stRelCursor(i)
        )
    }

    val ldReleaseMask = ldReleaseMaskVec.reduce(_ | _)
    val stReleaseMask = stReleaseMaskVec.reduce(_ | _)
    val stHeadAfterRelease = stRelCursor(nstq)

    val ldAliveMask = ldqValidMask & ~ldReleaseMask.asUInt
    val stAliveMask = stqValidMask & ~stReleaseMask.asUInt

    // CPUSTC.memory owns the Store commit/drain state. Intersect its registered
    // truth with CPU-side liveness so a released slot is never resurrected by
    // a temporarily skewed commit pointer during architectural recovery.
    val committedSurvivors = io.stqCommittedMask & stAliveMask
    val hardFlushKillMask = stAliveMask & (~committedSurvivors).asUInt
    val stHeadAfterHardFlush = Mux(
        committedSurvivors.orR,
        stHeadAfterRelease,
        io.stqCommitPtr
    )

    val ldConflictVec = Wire(Vec(ndcd, Bool()))
    val stConflictVec = Wire(Vec(ndcd, Bool()))

    for (i <- 0 until ndcd) {
        // Load releases arrive from the ROB's registered commit pipe. Let that
        // credit fund a same-cycle replacement allocation so registering the
        // ROB-to-LSQ boundary does not add a full-queue dispatch bubble.
        ldConflictVec(i) := ldReq(i) && (ldCursor(i).oh & ldAliveMask).orR
        stConflictVec(i) := stReq(i) && (stCursor(i).oh & stqValidMask).orR
    }

    val ldConflict = ldConflictVec.asUInt.orR
    val stConflict = stConflictVec.asUInt.orR

    val ldDemand = PopCount(ldReq)
    val stDemand = PopCount(stReq)
    val ldReleaseCount = PopCount(io.ldqRelease)

    // Load commit credits are registered and may fund same-cycle reuse. Store
    // frees are deliberately excluded here: the SQ contract exposes that slot
    // to dispatch only on the following cycle.
    val ldCanAccept =
        (ldOcc +& ldDemand) <= (nldq.U +& ldReleaseCount)
    val stCanAccept =
        (stOcc +& stDemand) <= nstq.U

    val mispredict = io.branchUpdate.valid && io.branchUpdate.bits.mispredictMask.orR
    val canAccept = !io.flush && ldCanAccept && stCanAccept
    val doAlloc = io.dispatch.doAllocate && canAccept && !mispredict
    val acceptedLdCount = Mux(doAlloc, ldDemand, 0.U)
    val acceptedStCount = Mux(doAlloc, stDemand, 0.U)

    io.dispatch.canAccept := canAccept
    io.ldqFull := (ldqTail.oh & ldAliveMask).orR
    io.stqFull := (stqTail.oh & stAliveMask).orR

    val liveMask = Wire(Vec(ndcd + 1, UInt(nstq.W)))
    val orderMask = Wire(Vec(ndcd + 1, UInt(nstq.W)))
    val staAcceptedMask = io.staAccepted.map { accepted =>
        Mux(accepted.valid, accepted.bits.oh, 0.U(nstq.W))
    }.reduce(_ | _)
    val visibleAddrReady =
        (stAddrReadyMask | staAcceptedMask) & stAliveMask

    liveMask(0) := stAliveMask & (~visibleAddrReady).asUInt
    orderMask(0) := stAliveMask

    for (i <- 0 until ndcd) {
        io.dispatch.resp(i).bits.stDepMask := liveMask(i)
        io.dispatch.resp(i).bits.stOrderMask := orderMask(i)

        val thisStoreMask = Mux(stReq(i), stCursor(i).oh, 0.U(nstq.W))
        liveMask(i + 1) := liveMask(i) | thisStoreMask
        orderMask(i + 1) := orderMask(i) | thisStoreMask
    }

    val ldAllocSuffix = Wire(Vec(ndcd + 1, UInt(nldq.W)))

    val stAllocSuffix = Wire(Vec(ndcd + 1, UInt(nstq.W)))

    ldAllocSuffix(ndcd) := 0.U
    stAllocSuffix(ndcd) := 0.U

    for (i <- (0 until ndcd).reverse) {
        val thisLdMask = Mux(
            ldReq(i),
            ldCursor(i).oh,
            0.U(nldq.W)
        )

        val thisStMask = Mux(
            stReq(i),
            stCursor(i).oh,
            0.U(nstq.W)
        )

        ldAllocSuffix(i) := thisLdMask | ldAllocSuffix(i + 1)
        stAllocSuffix(i) := thisStMask | stAllocSuffix(i + 1)
    }

    val ldAllocMask = ldAllocSuffix(0)
    val stAllocMask = stAllocSuffix(0)

    val ldAllocHighMask = (0 until ndcd).map { i =>
        Mux(ldReq(i) && ldCursor(i).flag, ldCursor(i).oh, 0.U(nldq.W))
    }.reduce(_ | _)

    val stAllocHighMask = (0 until ndcd).map { i =>
        Mux(stReq(i) && stCursor(i).flag, stCursor(i).oh, 0.U(nstq.W))
    }.reduce(_ | _)

    val ldHighAfterRelease = ldqHighMask & ldAliveMask
    val stHighAfterRelease = stqHighMask & stAliveMask

    val ldHighAfterAlloc = (ldHighAfterRelease & (~ldAllocMask).asUInt) | ldAllocHighMask
    val stHighAfterAlloc = (stHighAfterRelease & (~stAllocMask).asUInt) | stAllocHighMask

    val newBrTagOH = Wire(Vec(ndcd, UInt(maxBrCount.W)))

    for (i <- 0 until ndcd) {
        newBrTagOH(i) := Mux(
            io.dispatch.brSnapshotReqs(i).valid && canAccept,
            UIntToOH(
                io.dispatch.brSnapshotReqs(i).bits,
                maxBrCount
            ),
            0.U(maxBrCount.W)
        )
    }

    val newSnapshotMask = newBrTagOH.reduce(_ | _)
    val resolveMask = Mux(
        io.branchUpdate.valid,
        io.branchUpdate.bits.resolveMask,
        0.U(maxBrCount.W)
    )

    val selectedLdRollback = Mux1H(
        resolveMask.asBools,
        ldAllocsAfterBr
    )

    val selectedStRollback = Mux1H(
        resolveMask.asBools,
        stAllocsAfterBr
    )

    val ldRollbackMask = Mux(
        mispredict,
        selectedLdRollback,
        0.U(nldq.W)
    )

    val stRollbackMask = Mux(
        mispredict,
        selectedStRollback,
        0.U(nstq.W)
    )

    // Writeback only needs same-cycle speculative rollback information. Hard
    // flush is carried separately, and queue release must not feed this path.
    io.stqRecoveryKillMask := Mux(io.flush, 0.U, stRollbackMask)

    val recoveredLdTail = Mux1H(
        resolveMask.asBools,
        ldTailAfterBr
    )

    val recoveredStTail = Mux1H(
        resolveMask.asBools,
        stTailAfterBr
    )

    val ldRollbackAliveMask =
        ldAliveMask & (~ldRollbackMask).asUInt

    val stRollbackAliveMask =
        stAliveMask & (~stRollbackMask).asUInt

    val effectiveLdValidMask = Mux(
        io.flush,
        0.U(nldq.W),
        Mux(mispredict, ldRollbackAliveMask, ldAliveMask)
    )

    val effectiveStValidMask = Mux(
        io.flush,
        committedSurvivors,
        Mux(mispredict, stRollbackAliveMask, stAliveMask)
    )

    io.ldqFlushMask := Mux(
        io.flush,
        // Registered ROB commit credits may coincide with a later full flush.
        // Flush every slot that was live at the start of this cycle, including
        // those credits, because the memory-side head pointer is reset rather
        // than advanced through the released prefix.
        ldqValidMask,
        Mux(mispredict, ldRollbackMask, 0.U)
    )
    io.stqFlushMask := Mux(
        io.flush,
        hardFlushKillMask,
        Mux(mispredict, stRollbackMask, 0.U)
    )

    io.liveState.ldqHead := ldRelCursor(ncmt)
    io.liveState.stqHead := Mux(io.flush, stHeadAfterHardFlush, stHeadAfterRelease)
    io.liveState.ldqTail := Mux(mispredict, recoveredLdTail, ldqTail)
    io.liveState.stqTail := Mux(
        io.flush,
        io.stqCommitPtr,
        Mux(mispredict, recoveredStTail, stqTail)
    )
    io.liveState.ldqValidMask := effectiveLdValidMask
    io.liveState.stqValidMask := effectiveStValidMask
    io.liveState.ldqHighMask := ldqHighMask & effectiveLdValidMask
    io.liveState.stqHighMask := stqHighMask & effectiveStValidMask
    io.stqHeadCurrent := stqHead

    // Writeback uses cycle-start LQ liveness. Commit-side releases update these
    // registers at the edge, keeping ROB ready/commit logic out of the terminal
    // Load writeback path without delaying physical LQ slot reuse. Recovery is
    // filtered separately with the recovery-only kill masks above.
    io.writebackLiveState.ldqHead := ldqHead
    io.writebackLiveState.stqHead := stHeadAfterRelease
    io.writebackLiveState.ldqTail := ldqTail
    io.writebackLiveState.stqTail := stqTail
    io.writebackLiveState.ldqValidMask := ldqValidMask
    io.writebackLiveState.stqValidMask := stAliveMask
    io.writebackLiveState.ldqHighMask := ldqHighMask
    io.writebackLiveState.stqHighMask := stqHighMask & stAliveMask

    when(io.flush) {
        io.liveState.ldqHead := LdqPtr.init
        io.liveState.ldqTail := LdqPtr.init

        io.writebackLiveState.ldqHead := LdqPtr.init
        io.writebackLiveState.ldqTail := LdqPtr.init
        io.writebackLiveState.stqHead := stHeadAfterHardFlush
        io.writebackLiveState.stqTail := io.stqCommitPtr
        io.writebackLiveState.ldqValidMask := 0.U
        io.writebackLiveState.stqValidMask := committedSurvivors
        io.writebackLiveState.ldqHighMask := 0.U
        io.writebackLiveState.stqHighMask :=
            stHighAfterRelease & committedSurvivors
    }

    for (tag <- 0 until maxBrCount) {
        val createLaneOH = VecInit(
            (0 until ndcd).map { i =>
                newBrTagOH(i)(tag)
            }
        ).asUInt

        val snapshotLdTail = Mux1H(
            createLaneOH.asBools,
            VecInit(
                (0 until ndcd).map(i => ldCursor(i + 1))
            )
        )

        val snapshotStTail = Mux1H(
            createLaneOH.asBools,
            VecInit(
                (0 until ndcd).map(i => stCursor(i + 1))
            )
        )

        val sameCycleYoungerLd = Mux1H(
            createLaneOH.asBools,
            VecInit(
                (0 until ndcd).map(i => ldAllocSuffix(i + 1))
            )
        )

        val sameCycleYoungerSt = Mux1H(
            createLaneOH.asBools,
            VecInit(
                (0 until ndcd).map(i => stAllocSuffix(i + 1))
            )
        )

        val carriedLdMask = Mux(
            snapshotValid(tag),
            (ldAllocsAfterBr(tag) &
                (~ldRollbackMask).asUInt) |
                Mux(doAlloc, ldAllocMask, 0.U(nldq.W)),
            0.U(nldq.W)
        )

        val carriedStMask = Mux(
            snapshotValid(tag),
            (stAllocsAfterBr(tag) &
                (~stRollbackMask).asUInt) |
                Mux(doAlloc, stAllocMask, 0.U(nstq.W)),
            0.U(nstq.W)
        )

        when(io.flush) {
            ldAllocsAfterBr(tag) := 0.U
            stAllocsAfterBr(tag) := 0.U
        }.elsewhen(
            createLaneOH.orR &&
            !mispredict
        ) {
            ldTailAfterBr(tag) := snapshotLdTail
            stTailAfterBr(tag) := snapshotStTail

            ldAllocsAfterBr(tag) := sameCycleYoungerLd
            stAllocsAfterBr(tag) := sameCycleYoungerSt
        }.otherwise {
            ldAllocsAfterBr(tag) := carriedLdMask
            stAllocsAfterBr(tag) := carriedStMask
        }
    }

    when(io.flush) {
        snapshotValid := 0.U
    }.elsewhen(mispredict) {
        snapshotValid := io.branchUpdate.bits.recoverMask
    }.otherwise {
        snapshotValid :=
            (snapshotValid & (~resolveMask).asUInt) |
                newSnapshotMask
    }

    when(io.flush) {
        ldOcc := 0.U
        stOcc := PopCount(committedSurvivors)

        ldqTail := LdqPtr.init
        ldqHead := LdqPtr.init

        stqTail := io.stqCommitPtr
        stqHead := stHeadAfterHardFlush

        ldqValidMask := 0.U
        stqValidMask := committedSurvivors
        stAddrReadyMask := visibleAddrReady & committedSurvivors
        ldqHighMask := 0.U
        stqHighMask := stHighAfterRelease & committedSurvivors
    }.elsewhen(mispredict) {
        ldOcc := PopCount(ldRollbackAliveMask)
        stOcc := PopCount(stRollbackAliveMask)

        ldqTail := recoveredLdTail
        stqTail := recoveredStTail

        ldqHead := ldRelCursor(ncmt)
        stqHead := stHeadAfterRelease

        ldqValidMask := ldRollbackAliveMask
        stqValidMask := stRollbackAliveMask
        stAddrReadyMask := visibleAddrReady & stRollbackAliveMask
        ldqHighMask := ldqHighMask & ldRollbackAliveMask
        stqHighMask := stqHighMask & stRollbackAliveMask
    }.otherwise {
        ldOcc := (ldOcc - ldReleaseCount) + acceptedLdCount
        stOcc := (stOcc - stReleaseCount) + acceptedStCount

        ldqHead := ldRelCursor(ncmt)
        stqHead := stHeadAfterRelease

        when(doAlloc) {
            ldqTail := ldCursor(ndcd)
            stqTail := stCursor(ndcd)
        }

        ldqValidMask :=
            ldAliveMask |
            Mux(doAlloc, ldAllocMask, 0.U(nldq.W))

        stqValidMask :=
            stAliveMask |
            Mux(doAlloc, stAllocMask, 0.U(nstq.W))

        stAddrReadyMask :=
            visibleAddrReady &
            (~Mux(doAlloc, stAllocMask, 0.U(nstq.W))).asUInt

        ldqHighMask := Mux(
            doAlloc,
            ldHighAfterAlloc,
            ldHighAfterRelease
        )

        stqHighMask := Mux(
            doAlloc,
            stHighAfterAlloc,
            stHighAfterRelease
        )
    }

    for (i <- 0 until ndcd) {
        val memReq = ldReq(i) || stReq(i)
        io.dispatch.resp(i).valid := io.dispatch.req(i).valid && memReq && canAccept
    }

    when(io.branchUpdate.valid && !io.flush) {
        assert(PopCount(io.branchUpdate.bits.resolveMask) === 1.U)
        assert(
            (io.branchUpdate.bits.mispredictMask &
                (~io.branchUpdate.bits.resolveMask).asUInt) === 0.U
        )
    }

    when(io.flush || !mispredict) {
        assert(io.stqRecoveryKillMask === 0.U)
    }

    when(io.stqFreed.valid) {
        assert(
            io.stqFreed.bits === stReleaseMask,
            p"LsqAllocator: non-prefix Store release freed=${Hexadecimal(io.stqFreed.bits)} " +
                p"expected=${Hexadecimal(stReleaseMask)} head=${Hexadecimal(stqHead.oh)} " +
                p"valid=${Hexadecimal(stqValidMask)} high=${Hexadecimal(stqHighMask)}"
        )
    }

    when(io.ldqRelease.asUInt.orR) {
        assert(
            (ldReleaseMask & (~ldqValidMask).asUInt) === 0.U,
            p"LsqAllocator: Load release exceeds live prefix release=${Hexadecimal(ldReleaseMask)} " +
                p"valid=${Hexadecimal(ldqValidMask)} head=${Hexadecimal(ldqHead.oh)}"
        )
    }

    when(mispredict && !io.flush) {
        assert((snapshotValid & resolveMask).orR)
        assert(!doAlloc)
        assert((ldRollbackMask & (~ldAliveMask).asUInt) === 0.U)
        assert((stRollbackMask & (~stAliveMask).asUInt) === 0.U)
        assert((stRollbackMask & committedSurvivors) === 0.U)
        assert(PopCount(recoveredLdTail.oh) === 1.U)
        assert(PopCount(recoveredStTail.oh) === 1.U)
    }

    when(io.flush) {
        assert((committedSurvivors & (~visibleAddrReady).asUInt) === 0.U)
        assert((hardFlushKillMask & committedSurvivors) === 0.U)
    }

    for (i <- 0 until memNissue) {
        when(io.staAccepted(i).valid) {
            assert(PopCount(io.staAccepted(i).bits.oh) === 1.U)
            assert((io.staAccepted(i).bits.oh & stqValidMask).orR)
            assert((io.staAccepted(i).bits.oh & stqHighMask).orR ===
                io.staAccepted(i).bits.flag)
        }
    }

    for (i <- 0 until ndcd) {
        assert(PopCount(newBrTagOH(i)) <= 1.U)

        when(io.dispatch.brSnapshotReqs(i).valid) {
            assert(canAccept)
            assert(!io.flush)
            assert(!mispredict)
        }
    }

    for (i <- 0 until ndcd; j <- i + 1 until ndcd) {
        assert((newBrTagOH(i) & newBrTagOH(j)) === 0.U)
    }

    assert((ldqHighMask & (~ldqValidMask).asUInt) === 0.U)
    assert((stqHighMask & (~stqValidMask).asUInt) === 0.U)
    assert((stAddrReadyMask & (~stqValidMask).asUInt) === 0.U)
    assert(ldOcc === PopCount(ldqValidMask))
    assert(stOcc === PopCount(stqValidMask))
    assert(ldOcc <= nldq.U)
    assert(stOcc <= nstq.U)
    assert(ldReleaseCount <= ldOcc)
    assert(stReleaseCount <= stOcc)
    assert(ldCanAccept === !ldConflict)
    assert(stCanAccept === !stConflict)
    assert(PopCount(io.stqCommitPtr.oh) === 1.U)
    assert((committedSurvivors & (~stAliveMask).asUInt) === 0.U)
    assert((io.liveState.ldqHighMask &
        (~io.liveState.ldqValidMask).asUInt) === 0.U)
    assert((io.liveState.stqHighMask &
        (~io.liveState.stqValidMask).asUInt) === 0.U)
}
