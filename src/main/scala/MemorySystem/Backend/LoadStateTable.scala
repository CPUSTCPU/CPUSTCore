package CPUSTC.memory.backend

import chisel3._
import chisel3.util._
import CPUSTC.memory._
import CPUSTC.memory.MemoryPointerUtils._
import CPUSTC.backend.rob.RobPtr
import CPUSTC.config.RegisterFile._


class LoadWaitType extends Bundle {
    val mshrFull = Bool()
    val storeData = Bool()
    val storeComplete = Bool()
}

class DcacheLoadFailBus extends Bundle {
    val valid = Bool()
    val inst = new BackendInst
    val waitSqindex = UInt(StoreQueueConfig.length.W)
    val waitSqindexHigh = Bool()
    val mshrFull = Bool()
    val storeData = Bool()
    val partialOverlap = Bool()
    val waitStoreData = Bool()
}

class StoreReadyEvent extends Bundle {
    val paddr = UInt(32.W)
    val sqindex = UInt(StoreQueueConfig.length.W)
    val sqindexHigh = Bool()
    val sqMask = UInt(StoreQueueConfig.length.W)
    val sqHighMask = UInt(StoreQueueConfig.length.W)
}

class LoadIssueInfo extends LoadIndex {
    val robPtr = new RobPtr
}

class DcacheStoreRetryBus extends Bundle {
    // Oldest Store whose issue must be withdrawn. StoreQueue derives the
    // complete younger issued suffix from this resident ring identity.
    val sqindex = UInt(StoreQueueConfig.length.W)
    val sqindexHigh = Bool()
}

class LoadCompleteInfo extends LoadIssueInfo {
    val forwarded = Bool()
    val forwardSqindex = UInt(StoreQueueConfig.length.W)
    val forwardSqindexHigh = Bool()
    val forwardCommitted = Bool()
}

class DispatchPtrCtrl extends Bundle {
    val nextHeadPtr = UInt(LoadStateTableConfig.length.W)
    val nextHeadSuffixMask = UInt(LoadStateTableConfig.length.W)
    val nextTailPtr = UInt(LoadStateTableConfig.length.W)
    val nextHeadPtrHigh = Bool()
    val nextTailPtrHigh = Bool()
    val flushMask = UInt(LoadStateTableConfig.length.W)
    val redirect = Bool()
}

class MemoryEntranceEntry extends BackendInst

class LoadResidentInst extends Bundle {
    val pc = UInt(32.W)
    val paddr = UInt(32.W)
    val uncache = Bool()
    val mask = UInt(4.W)
    val valid = Bool()
    val signed = Bool()
    val sqindex = UInt(StoreQueueConfig.length.W)
    val sqindexHigh = Bool()
    val storeDepMask = UInt(StoreQueueConfig.length.W)
    val ldindexHigh = Bool()
    val soreceReg = UInt(6.W)
    val Poisoned = Bool()
    val exception = UInt(8.W)
    val exceptionBadvValid = Bool()
    val robPtr = new RobPtr
    val pdest = UInt(wpreg.W)
    val rfWen = Bool()
}

class InternalReplayBus extends Bundle {
    val mshrProgress = Bool()
    val storeReady = Vec(6, Valid(new StoreReadyEvent))
}

class TableEntry extends Bundle {
    val valid = Bool()
    val inst = new LoadResidentInst
    val executing = Bool()
    val complete = Bool()
    val waitType = new LoadWaitType
    val waitSqindex = UInt(StoreQueueConfig.length.W)
    val waitSqindexHigh = Bool()
    val forwarded = Bool()
    val forwardSqindex = UInt(StoreQueueConfig.length.W)
    val forwardSqindexHigh = Bool()
    val forwardCommitted = Bool()
}

class HeadLoadPerfState extends Bundle {
    val tracked = Bool()
    val missing = Bool()
    val readyIssued = Bool()
    val readyWait = Bool()
    val executing = Bool()
    val executingAfterStoreReplay = Bool()
    val waitMshrFull = Bool()
    val waitStoreData = Bool()
    val complete = Bool()
    val other = Bool()
}

class LoadStateTableIO extends Bundle {
    val loadStoreBack = Input(Vec(DcacheConfig.nPorts, new DcacheLoadFailBus))
    val loadMshrBack = Input(Vec(DcacheConfig.nPorts, new DcacheLoadFailBus))
    val ptrCtrl = Input(new DispatchPtrCtrl)
    val lsqLive = Input(new MemoryLsqLiveState)
    val entry = Input(Vec(LoadQueueConfig.EnqNum, new MemoryEntranceEntry))
    val entryIssued = Input(Vec(LoadQueueConfig.EnqNum, Bool()))
    val complete = Input(Vec(DcacheConfig.nPorts, Valid(new LoadCompleteInfo)))
    // Only an uncached response can outlive the local memory pipeline that
    // issued it. Cached DCache and MSHR results are validated at their owning
    // pipeline boundaries and must not feed this resident-table lookup into
    // the common load wakeup path.
    val uncacheResultCheck = Input(new BackendInst)
    val uncacheResultCurrent = Output(Bool())
    val sqHeadOH = Input(UInt(StoreQueueConfig.length.W))
    val sqHeadHigh = Input(Bool())
    val pendingUncacheStore = Input(Bool())
    val interReplay = Input(new InternalReplayBus)
    val storeWaitState = Input(new StoreWaitState)
    val storeFreedMask = Input(UInt(StoreQueueConfig.length.W))
    val issueInsts = Vec(DcacheConfig.nPorts, Decoupled(new BackendInst))
    val robHeadLoad = Flipped(Valid(new RobHeadLoadInfo))
    val llCommit = Flipped(Valid(new RobPtr))
    val llCommitPaddr = Output(Valid(UInt(32.W)))
    val uncacheReq = Decoupled(new BackendInst)
    val full = Output(Bool())
    val occupancy = Output(UInt(log2Ceil(LoadStateTableConfig.length + 1).W))
    val headPerf = Output(new HeadLoadPerfState)
}

private[backend] object LoadIssuePicker {
    private val groupSize = 3

    private def selectFirstOfThree(candidates: UInt): UInt = {
        val a = candidates(0)
        val b = candidates(1)
        val c = candidates(2)
        Cat(!a && !b && c, !a && b, a)
    }

    private def expandGroupOH(groupOH: Vec[UInt], length: Int): Vec[UInt] = {
        val groupCount = (length + groupSize - 1) / groupSize
        VecInit.tabulate(groupCount) { group =>
            VecInit.tabulate(length) { index =>
                if (index / groupSize == group) {
                    groupOH(group)(index % groupSize)
                } else {
                    false.B
                }
            }.asUInt
        }
    }

    private def selectFirstFixed(candidates: UInt, length: Int): UInt = {
        val groupCount = (length + groupSize - 1) / groupSize
        val paddedCandidates = candidates.pad(groupCount * groupSize)
        val groupFirstOH = VecInit.tabulate(groupCount) { group =>
            selectFirstOfThree(
                paddedCandidates(
                    group * groupSize + groupSize - 1,
                    group * groupSize
                )
            )
        }
        val groupHasFirst = VecInit(groupFirstOH.map(_.orR)).asUInt
        val firstGroupOH = PriorityEncoderOH(groupHasFirst)
        Mux1H(firstGroupOH, expandGroupOH(groupFirstOH, length))
    }

    def oldest(
        candidates: UInt,
        atOrAfterHead: UInt,
        length: Int
    ): UInt = {
        val suffixFirstOH = selectFirstFixed(
            candidates & atOrAfterHead,
            length
        )
        val prefixFirstOH = selectFirstFixed(
            candidates & (~atOrAfterHead).asUInt,
            length
        )
        Mux(suffixFirstOH.orR, suffixFirstOH, prefixFirstOH)
    }

}

class LoadStateTable(enableDebug: Boolean = false) extends Module {
    val io = IO(new LoadStateTableIO)

    private val length = LoadStateTableConfig.length
    val entries = RegInit(VecInit.fill(length)(0.U.asTypeOf(new TableEntry)))
    // Keep issueability in one narrow sidecar. Store wakeups and ordinary wake
    // events update this bit at the same edge, while the exact-oldest picker
    // reads a single registered state instead of ready | pending.
    val issueReady = RegInit(0.U(length.W))
    // Isolate SQ array state from LST's per-entry update fanout. This snapshot
    // is never frozen by recovery; same-cycle STD/completion events provide the
    // bypass while the level state crosses this register boundary.
    val storeWaitStateReg = RegNext(
        io.storeWaitState,
        0.U.asTypeOf(new StoreWaitState)
    )
    val trackedHeadOH = RegInit(1.U(length.W))
    val trackedHeadHigh = RegInit(false.B)

    def toResident(inst: BackendInst): LoadResidentInst = {
        val resident = Wire(new LoadResidentInst)
        resident.pc := inst.pc
        resident.paddr := inst.paddr
        resident.uncache := inst.uncache
        resident.mask := inst.mask
        resident.valid := inst.valid
        resident.signed := inst.signed
        resident.sqindex := inst.sqindex
        resident.sqindexHigh := inst.sqindexHigh
        resident.storeDepMask := inst.storeDepMask
        resident.ldindexHigh := inst.ldindexHigh
        resident.soreceReg := inst.soreceReg
        resident.Poisoned := inst.Poisoned
        resident.exception := inst.exception
        resident.exceptionBadvValid := inst.exceptionBadvValid
        resident.robPtr := inst.robPtr
        resident.pdest := inst.pdest
        resident.rfWen := inst.rfWen
        resident
    }

    def restoreLoad(resident: LoadResidentInst, ldindex: UInt): BackendInst = {
        val inst = WireDefault(0.U.asTypeOf(new BackendInst))
        inst.uop.isLD := true.B
        inst.pc := resident.pc
        inst.paddr := resident.paddr
        inst.uncache := resident.uncache
        inst.mask := resident.mask
        inst.valid := resident.valid
        inst.signed := resident.signed
        inst.sqindex := resident.sqindex
        inst.sqindexHigh := resident.sqindexHigh
        inst.storeDepMask := resident.storeDepMask
        inst.ldindex := ldindex
        inst.ldindexHigh := resident.ldindexHigh
        inst.soreceReg := resident.soreceReg
        inst.Poisoned := resident.Poisoned
        inst.exception := resident.exception
        inst.exceptionBadvValid := resident.exceptionBadvValid
        inst.exceptionBadv := Mux(
            resident.exceptionBadvValid,
            resident.pc,
            0.U
        )
        inst.robPtr := resident.robPtr
        inst.pdest := resident.pdest
        inst.rfWen := resident.rfWen
        inst
    }

    def storeWakeMatches(
        waitSqindex: UInt,
        waitSqindexHigh: Bool,
        wake: StoreReadyEvent
    ): Bool = {
        val generationMask = Mux(waitSqindexHigh, wake.sqHighMask, ~wake.sqHighMask)
        (waitSqindex & wake.sqMask & generationMask).orR
    }

    // STD and completion both prove that Store data is available, while only
    // DCache/MSHR completion proves that a partial-overlap Store is no longer
    // memory-pending. Keep the two wake classes separate so an STD pulse cannot
    // release a partial-overlap replay.
    val storeDataWakeLowMask = io.interReplay.storeReady.map { wake =>
        Mux(
            wake.valid,
            wake.bits.sqMask & (~wake.bits.sqHighMask).asUInt,
            0.U(StoreQueueConfig.length.W)
        )
    }.reduce(_ | _)
    val storeDataWakeHighMask = io.interReplay.storeReady.map { wake =>
        Mux(
            wake.valid,
            wake.bits.sqMask & wake.bits.sqHighMask,
            0.U(StoreQueueConfig.length.W)
        )
    }.reduce(_ | _)
    val storeCompleteWakeLowMask = io.interReplay.storeReady.drop(4).map { wake =>
        Mux(
            wake.valid,
            wake.bits.sqMask & (~wake.bits.sqHighMask).asUInt,
            0.U(StoreQueueConfig.length.W)
        )
    }.reduce(_ | _)
    val storeCompleteWakeHighMask = io.interReplay.storeReady.drop(4).map { wake =>
        Mux(
            wake.valid,
            wake.bits.sqMask & wake.bits.sqHighMask,
            0.U(StoreQueueConfig.length.W)
        )
    }.reduce(_ | _)

    def storeWakeMaskMatches(
        waitSqindex: UInt,
        waitSqindexHigh: Bool,
        wakeLowMask: UInt,
        wakeHighMask: UInt
    ): Bool = {
        val wakeMask = Mux(
            waitSqindexHigh,
            wakeHighMask,
            wakeLowMask
        )
        (waitSqindex & wakeMask).orR
    }

    def anyStoreDataWakeMatches(
        waitSqindex: UInt,
        waitSqindexHigh: Bool
    ): Bool = storeWakeMaskMatches(
        waitSqindex,
        waitSqindexHigh,
        storeDataWakeLowMask,
        storeDataWakeHighMask
    )

    def anyStoreCompleteWakeMatches(
        waitSqindex: UInt,
        waitSqindexHigh: Bool
    ): Bool = storeWakeMaskMatches(
        waitSqindex,
        waitSqindexHigh,
        storeCompleteWakeLowMask,
        storeCompleteWakeHighMask
    )

    def sameCycleStoreWake(back: DcacheLoadFailBus): Bool = Mux(
        back.partialOverlap,
        anyStoreCompleteWakeMatches(
            back.waitSqindex,
            back.waitSqindexHigh
        ),
        anyStoreDataWakeMatches(
            back.waitSqindex,
            back.waitSqindexHigh
        )
    )

    def storeStateMatches(
        waitSqindex: UInt,
        waitSqindexHigh: Bool,
        stateMask: UInt
    ): Bool = {
        val generationMask = Mux(
            waitSqindexHigh,
            storeWaitStateReg.highMask,
            (~storeWaitStateReg.highMask).asUInt
        )
        (waitSqindex & stateMask & generationMask).orR
    }

    val liveMask = VecInit((0 until length).map { index =>
        val indexOH = (BigInt(1) << index).U(length.W)
        pointerAlive(
            indexOH,
            entries(index).inst.ldindexHigh,
            io.lsqLive.ldqValidMask,
            io.lsqLive.ldqHighMask
        ) && !io.ptrCtrl.flushMask(index)
    }).asUInt

    // ROB commit is already registered. The matching LST entry is still
    // resident in this cycle, before the registered LDQ head advance releases
    // it. Use the complete ROB identity rather than the separately registered
    // liveness masks so their boundary skew cannot hide a retiring LL.
    val llCommitHits = VecInit((0 until length).map { index =>
        io.llCommit.valid &&
            entries(index).valid &&
            entries(index).complete &&
            entries(index).inst.robPtr.asUInt === io.llCommit.bits.asUInt
    })
    io.llCommitPaddr.valid := llCommitHits.asUInt.orR
    io.llCommitPaddr.bits := Mux(
        llCommitHits.asUInt.orR,
        Mux1H(llCommitHits, entries.map(_.inst.paddr)),
        0.U
    )

    when(io.llCommit.valid) {
        assert(PopCount(llCommitHits) <= 1.U,
            "LoadStateTable: one LL commit matched multiple resident loads")
    }

    def entryMatches(
        index: Int,
        indexOH: UInt,
        indexHigh: Bool,
        robPtr: RobPtr
    ): Bool = {
        entries(index).valid && liveMask(index) && indexOH(index) &&
            entries(index).inst.ldindexHigh === indexHigh &&
            entries(index).inst.robPtr.asUInt === robPtr.asUInt
    }

    // Replay and completion feedback belongs to the resident LST entry.  The
    // global LDQ live mask may advance at a different registered boundary, so
    // using it here can discard the final response of a still-resident Load.
    def residentMatches(
        index: Int,
        indexOH: UInt,
        indexHigh: Bool,
        robPtr: RobPtr
    ): Bool = {
        entries(index).valid && !io.ptrCtrl.flushMask(index) && indexOH(index) &&
            entries(index).inst.ldindexHigh === indexHigh &&
            entries(index).inst.robPtr.asUInt === robPtr.asUInt
    }

    val allocationMask = io.entry.map { entry =>
        Mux(entry.valid, entry.ldindex, 0.U(length.W))
    }.reduce(_ | _)

    val residentStoreWakeSetMask = VecInit((0 until length).map { index =>
        val waitsForStoreComplete = entries(index).waitType.storeComplete
        val storeWake = Mux(
            waitsForStoreComplete,
            anyStoreCompleteWakeMatches(
                entries(index).waitSqindex,
                entries(index).waitSqindexHigh
            ),
            anyStoreDataWakeMatches(
                entries(index).waitSqindex,
                entries(index).waitSqindexHigh
            )
        )
        val waitsForStore = entries(index).waitType.storeData ||
            entries(index).waitType.storeComplete
        entries(index).valid && !entries(index).inst.uncache &&
            !issueReady(index) && !entries(index).executing &&
            !entries(index).complete && waitsForStore && storeWake
    }).asUInt

    // A Store wake may coincide with the first DCache replay feedback. The
    // entry is still executing then, so capture the complete feedback identity
    // separately from resident waiters.
    val replayStoreWakeSetMask = VecInit((0 until length).map { index =>
        io.loadStoreBack.map { back =>
            back.storeData && !allocationMask(index) &&
                sameCycleStoreWake(back) &&
                residentMatches(
                    index,
                    back.inst.ldindex,
                    back.inst.ldindexHigh,
                    back.inst.robPtr
                )
        }.reduce(_ || _)
    }).asUInt
    val storeWakeSetMask = residentStoreWakeSetMask | replayStoreWakeSetMask

    // Keep T15's source-ordered updates. Allocation is connected again after
    // all old-resident events so a reused slot always belongs to the new load.
    val nextIssueReady = Wire(Vec(length, Bool()))
    nextIssueReady := VecInit((issueReady | storeWakeSetMask).asBools)
    val readyMask = issueReady & (~io.ptrCtrl.flushMask).asUInt

    val headLoadStalled = io.robHeadLoad.valid && io.robHeadLoad.bits.waiting
    val headRobMatches = VecInit((0 until length).map { index =>
        headLoadStalled && entries(index).valid && liveMask(index) &&
            entries(index).inst.robPtr.asUInt === io.robHeadLoad.bits.robPtr.asUInt
    }).asUInt
    val headTracked = headRobMatches.orR
    val headEntry = Mux1H(headRobMatches, entries)
    val delayedMshrBack = RegInit(VecInit.fill(DcacheConfig.nPorts)(
        0.U.asTypeOf(new DcacheLoadFailBus)
    ))
    for (port <- 0 until DcacheConfig.nPorts) {
        delayedMshrBack(port) := 0.U.asTypeOf(new DcacheLoadFailBus)
        when(io.loadMshrBack(port).valid && io.loadMshrBack(port).mshrFull) {
            delayedMshrBack(port) := io.loadMshrBack(port)
        }
    }

    val headAdvanced = !pointerEqual(trackedHeadOH, trackedHeadHigh,
        io.ptrCtrl.nextHeadPtr, io.ptrCtrl.nextHeadPtrHigh)
    val retiredMask = VecInit((0 until length).map { index =>
        val indexOH = (BigInt(1) << index).U(length.W)
        headAdvanced && pointerInRange(
            indexOH,
            entries(index).inst.ldindexHigh,
            trackedHeadOH,
            trackedHeadHigh,
            io.ptrCtrl.nextHeadPtr,
            io.ptrCtrl.nextHeadPtrHigh
        )
    }).asUInt
    // this is for dubug assert
    val occupiedMask = VecInit((0 until length).map(index => entries(index).valid)).asUInt
    io.occupancy := PopCount(occupiedMask)
    io.full := occupiedMask.andR

    io.uncacheResultCurrent := VecInit((0 until length).map { index =>
        entryMatches(
            index,
            io.uncacheResultCheck.ldindex,
            io.uncacheResultCheck.ldindexHigh,
            io.uncacheResultCheck.robPtr
        ) && entries(index).inst.uncache && entries(index).executing &&
            !entries(index).complete
    }).asUInt.orR

    //=============================== ordered selection mechanism =====================================
    private val replayBankCount = DcacheConfig.nPorts
    require(replayBankCount == 2)
    require(length % replayBankCount == 0)
    private val replayBankDepth = length / replayBankCount
    private val replayBankIndices = Seq.tabulate(replayBankCount) { bank =>
        (0 until length).filter(_ % replayBankCount == bank)
    }
    private val replayBankMasks = replayBankIndices.map { indices =>
        indices.foldLeft(BigInt(0)) { case (mask, index) =>
            mask | (BigInt(1) << index)
        }.U(length.W)
    }
    require(replayBankIndices.forall(_.length == replayBankDepth))

    // Each residue bank has its own compact oldest selector and fixed output.
    // The global suffix mask only supplies the local circular starting point;
    // winners are never compared or transferred between banks.
    val selectedLocalOH = VecInit((0 until replayBankCount).map { bank =>
        val localReady = VecInit(
            replayBankIndices(bank).map(index => readyMask(index))
        ).asUInt
        val localAtOrAfterHead = VecInit(
            replayBankIndices(bank).map { index =>
                io.ptrCtrl.nextHeadSuffixMask(index)
            }
        ).asUInt
        val suffixReady = localReady & localAtOrAfterHead
        val prefixReady = localReady & (~localAtOrAfterHead).asUInt
        Mux(
            suffixReady.orR,
            PriorityEncoderOH(suffixReady),
            PriorityEncoderOH(prefixReady)
        )
    })
    val selectedOH = VecInit((0 until replayBankCount).map { bank =>
        VecInit.tabulate(length) { index =>
            if (index % replayBankCount == bank) {
                selectedLocalOH(bank)(index / replayBankCount)
            } else {
                false.B
            }
        }.asUInt
    })
    val selectedResidents = VecInit((0 until replayBankCount).map { bank =>
        Mux1H(
            selectedLocalOH(bank),
            replayBankIndices(bank).map(index => entries(index).inst)
        )
    })

    for (port <- 0 until DcacheConfig.nPorts) {
        val issueBits = restoreLoad(selectedResidents(port), selectedOH(port))
        io.issueInsts(port).valid := selectedOH(port).orR
        io.issueInsts(port).bits := issueBits
        assert((selectedOH(port) & ~replayBankMasks(port)).asUInt === 0.U,
            s"LoadStateTable: replay port $port selected the wrong bank")
    }
    assert((selectedOH(0) & selectedOH(1)) === 0.U,
        "LoadStateTable: replay banks must select distinct entries")

    val issueFireMask = VecInit((0 until length).map { index =>
        VecInit((0 until DcacheConfig.nPorts).map { port =>
            io.issueInsts(port).fire && selectedOH(port)(index)
        }).asUInt.orR
    }).asUInt

    val headIssuedNow = VecInit((0 until DcacheConfig.nPorts).map { port =>
        io.issueInsts(port).fire &&
            io.issueInsts(port).bits.robPtr.asUInt === io.robHeadLoad.bits.robPtr.asUInt
    }).asUInt.orR
    val headLogicalReady = (headRobMatches & issueReady).orR
    val headReady = headTracked && headLogicalReady &&
        !headEntry.executing && !headEntry.complete
    val headWaitMshrFull = headTracked && headEntry.waitType.mshrFull &&
        !headLogicalReady && !headEntry.executing && !headEntry.complete
    val headStoreWait = headEntry.waitType.storeData ||
        headEntry.waitType.storeComplete
    val headWaitStoreData = headTracked && headStoreWait &&
        !headLogicalReady && !headEntry.executing && !headEntry.complete
    val headExecutingAfterStoreReplay = headTracked && headEntry.executing &&
        headStoreWait
    val headExecutingClean = headTracked && headEntry.executing &&
        !headStoreWait

    io.headPerf := 0.U.asTypeOf(new HeadLoadPerfState)
    io.headPerf.tracked := headTracked
    io.headPerf.missing := headLoadStalled && !headTracked
    io.headPerf.readyIssued := headReady && headIssuedNow
    io.headPerf.readyWait := headReady && !headIssuedNow
    io.headPerf.executing := headExecutingClean
    io.headPerf.executingAfterStoreReplay := headExecutingAfterStoreReplay
    io.headPerf.waitMshrFull := headWaitMshrFull
    io.headPerf.waitStoreData := headWaitStoreData
    io.headPerf.complete := headTracked && headEntry.complete
    io.headPerf.other := headTracked && !headReady && !headEntry.executing &&
        !headEntry.complete && !headWaitMshrFull &&
        !headWaitStoreData

    //=============================== uncache req =======================================
    val headLoadOH = io.ptrCtrl.nextHeadPtr
    val headLoadEntry = Mux1H(headLoadOH, entries)
    val headLoadFlushed = (headLoadOH & io.ptrCtrl.flushMask).orR
    val headLoadGenerationMatch = headLoadEntry.inst.ldindexHigh === io.ptrCtrl.nextHeadPtrHigh
    val uncacheCanIssue = headLoadEntry.valid && headLoadGenerationMatch && !headLoadFlushed &&
        headLoadEntry.inst.uncache && !headLoadEntry.executing && !headLoadEntry.complete &&
        !io.pendingUncacheStore
    io.uncacheReq.valid := io.robHeadLoad.valid && uncacheCanIssue
    io.uncacheReq.bits := restoreLoad(headLoadEntry.inst, headLoadOH)
    val uncacheIssueMask = Mux(io.uncacheReq.fire, headLoadOH, 0.U(length.W))

    when(io.uncacheReq.fire) {
        for (index <- 0 until length) {
            when(headLoadOH(index)) {
                nextIssueReady(index) := false.B
                entries(index).executing := true.B
                entries(index).complete := false.B
            }
        }
    }

    //=================================== enque filling & bypass ================================
    val allocationResidents = VecInit(io.entry.map(toResident))
    for (port <- 0 until LoadQueueConfig.EnqNum) {
        when(io.entry(port).valid) {
            when(io.entry(port).exceptionBadvValid) {
                assert(io.entry(port).exception.orR,
                    "LoadStateTable: BADV is valid only for an exception")
                assert(io.entry(port).exceptionBadv === io.entry(port).pc,
                    "LoadStateTable: memory exception BADV must equal vaddr")
            }
            assert(PopCount(io.entry(port).ldindex) === 1.U,
                "LoadStateTable: allocated ldindex must be one-hot")
            for (index <- 0 until length) {
                when(io.entry(port).ldindex(index)) {
                    entries(index) := 0.U.asTypeOf(new TableEntry)
                    entries(index).valid := true.B
                    entries(index).executing := io.entryIssued(port)
                    entries(index).inst := allocationResidents(port)
                    entries(index).inst.storeDepMask := io.entry(port).storeDepMask & ~io.storeFreedMask
                }
            }
        }
    }

    def consumeStoreReplayBack(back: DcacheLoadFailBus): Unit = {
        when(back.storeData) {
            val waitForCompletion = back.partialOverlap
            val unresolvedState = Mux(
                waitForCompletion,
                storeWaitStateReg.memoryPendingMask,
                storeWaitStateReg.dataMissingMask
            )
            val storeStillBlocked = storeStateMatches(
                back.waitSqindex,
                back.waitSqindexHigh,
                unresolvedState
            )
            assert(back.valid,
                "LoadStateTable: Store replay must carry valid feedback")
            assert(back.partialOverlap ^ back.waitStoreData,
                "LoadStateTable: Store replay must have exactly one wait reason")
            assert(PopCount(back.inst.ldindex) === 1.U,
                "LoadStateTable: replayed ldindex must be one-hot")
            for (index <- 0 until length) {
                when(!allocationMask(index) && residentMatches(
                    index,
                    back.inst.ldindex,
                    back.inst.ldindexHigh,
                    back.inst.robPtr
                )) {
                    entries(index).executing := false.B
                    nextIssueReady(index) :=
                        sameCycleStoreWake(back) || !storeStillBlocked
                    entries(index).waitType.mshrFull := false.B
                    entries(index).waitType.storeData := back.waitStoreData
                    entries(index).waitType.storeComplete := back.partialOverlap
                    entries(index).waitSqindex := back.waitSqindex
                    entries(index).waitSqindexHigh := back.waitSqindexHigh
                }
            }
        }
    }

    def consumeMshrReplayBack(back: DcacheLoadFailBus): Unit = {
        when(back.valid) {
            assert(back.mshrFull,
                "LoadStateTable: MSHR replay feedback must report full")
            assert(PopCount(back.inst.ldindex) === 1.U,
                "LoadStateTable: replayed ldindex must be one-hot")
            for (index <- 0 until length) {
                when(!allocationMask(index) && residentMatches(
                    index,
                    back.inst.ldindex,
                    back.inst.ldindexHigh,
                    back.inst.robPtr
                )) {
                    entries(index).executing := false.B
                    nextIssueReady(index) := io.interReplay.mshrProgress
                    entries(index).waitType.mshrFull := true.B
                    entries(index).waitType.storeData := false.B
                    entries(index).waitType.storeComplete := false.B
                    entries(index).waitSqindex := back.waitSqindex
                    entries(index).waitSqindexHigh := back.waitSqindexHigh
                }
            }
        }
    }

    //=============================== selected issue update =========================================
    for (port <- 0 until DcacheConfig.nPorts) {
        when(io.issueInsts(port).fire) {
            for (index <- 0 until length) {
                when(selectedOH(port)(index)) {
                    nextIssueReady(index) := false.B
                    entries(index).executing := true.B
                    entries(index).complete := false.B
                }
            }
        }

    //==================================== backed inst replay manage ===================================
        val storeBack = io.loadStoreBack(port)
        val mshrBack = delayedMshrBack(port)
        when(storeBack.storeData && mshrBack.valid) {
            val sameIdentity = storeBack.inst.ldindex === mshrBack.inst.ldindex &&
                storeBack.inst.ldindexHigh === mshrBack.inst.ldindexHigh &&
                storeBack.inst.robPtr.asUInt === mshrBack.inst.robPtr.asUInt
            assert(!sameIdentity,
                "LoadStateTable: Store and delayed MSHR replay feedback target one Load")
        }
        consumeStoreReplayBack(storeBack)
        consumeMshrReplayBack(mshrBack)

        //========================= keep trace of outing load ========================
        when(io.complete(port).valid) {
            for (index <- 0 until length) {
                when(!allocationMask(index) && residentMatches(
                    index,
                    io.complete(port).bits.ldindex,
                    io.complete(port).bits.ldindexHigh,
                    io.complete(port).bits.robPtr
                )) {
                    entries(index).executing := false.B
                    entries(index).complete := true.B
                    nextIssueReady(index) := false.B
                    entries(index).waitType := 0.U.asTypeOf(new LoadWaitType)
                    entries(index).forwarded := io.complete(port).bits.forwarded
                    entries(index).forwardSqindex := io.complete(port).bits.forwardSqindex
                    entries(index).forwardSqindexHigh := io.complete(port).bits.forwardSqindexHigh
                    entries(index).forwardCommitted := io.complete(port).bits.forwardCommitted
                }
            }
        }
    }

    val completionMask = (0 until DcacheConfig.nPorts).map { port =>
        Mux(
            io.complete(port).valid,
            io.complete(port).bits.ldindex,
            0.U(LoadStateTableConfig.length.W)
        )
    }.reduce(_ | _)

    // =================================== wake up =======================================
    for (index <- 0 until length) {
        val canWake = entries(index).valid && !entries(index).inst.uncache &&
            !entries(index).executing && !entries(index).complete &&
            !allocationMask(index) && !issueFireMask(index) &&
            !completionMask(index)
        when(canWake && entries(index).waitType.mshrFull &&
            io.interReplay.mshrProgress) {
            nextIssueReady(index) := true.B
        }
        val waitsForStoreComplete = entries(index).waitType.storeComplete
        val unresolvedState = Mux(
            waitsForStoreComplete,
            storeWaitStateReg.memoryPendingMask,
            storeWaitStateReg.dataMissingMask
        )
        val storeStillBlocked = storeStateMatches(
            entries(index).waitSqindex,
            entries(index).waitSqindexHigh,
            unresolvedState
        )
        val waitsForStore = entries(index).waitType.storeData ||
            entries(index).waitType.storeComplete
        when(canWake && waitsForStore && !storeStillBlocked) {
            nextIssueReady(index) := true.B
        }
    }

    for (index <- 0 until length) {
        when(io.storeFreedMask.orR && !allocationMask(index)) {
            entries(index).inst.storeDepMask := entries(index).inst.storeDepMask & ~io.storeFreedMask
        }
        when(io.ptrCtrl.flushMask(index) && !allocationMask(index)) {
            entries(index).valid := false.B
            nextIssueReady(index) := false.B
            entries(index).executing := false.B
            entries(index).complete := false.B
            entries(index).waitType := 0.U.asTypeOf(new LoadWaitType)
        }.elsewhen(io.ptrCtrl.redirect && entries(index).valid &&
            entries(index).executing && !entries(index).complete &&
            !entries(index).inst.uncache) {
            nextIssueReady(index) := true.B
            entries(index).executing := false.B
            entries(index).waitType := 0.U.asTypeOf(new LoadWaitType)
            entries(index).waitSqindex := 0.U
            entries(index).waitSqindexHigh := false.B
        }.elsewhen(retiredMask(index) && !allocationMask(index)) {
            entries(index) := 0.U.asTypeOf(new TableEntry)
            nextIssueReady(index) := false.B
        }
    }

    // Final connection: all signals above describe the old resident. A new
    // allocation owns both payload and issueability for the reused slot.
    for (port <- 0 until LoadQueueConfig.EnqNum) {
        when(io.entry(port).valid) {
            for (index <- 0 until length) {
                when(io.entry(port).ldindex(index)) {
                    nextIssueReady(index) :=
                        !io.entry(port).uncache && !io.entryIssued(port)
                }
            }
        }
    }

    issueReady := nextIssueReady.asUInt

    when(headAdvanced) {
        trackedHeadOH := io.ptrCtrl.nextHeadPtr
        trackedHeadHigh := io.ptrCtrl.nextHeadPtrHigh
    }

    if (enableDebug) {
        val debugCycle = RegInit(0.U(64.W))
        debugCycle := debugCycle + 1.U
        val previousEntries = RegNext(
            entries,
            VecInit.fill(length)(0.U.asTypeOf(new TableEntry))
        )
        val heartbeat = debugCycle(7, 0) === 0.U

        when(debugCycle >= 300000.U) {
        for (port <- 0 until LoadQueueConfig.EnqNum) {
            when(io.entry(port).valid) {
                printf(
                    p"[DBG][LST][ALLOC] cycle=${debugCycle} port=${port.U} " +
                        p"pc=0x${Hexadecimal(io.entry(port).pc)} " +
                        p"robQ=${io.entry(port).robPtr.qidx} " +
                        p"robOff=${io.entry(port).robPtr.offset} " +
                        p"robH=${io.entry(port).robPtr.high} " +
                        p"robEpoch=${io.entry(port).robPtr.epoch} " +
                        p"ldOH=0x${Hexadecimal(io.entry(port).ldindex)} " +
                        p"ldH=${io.entry(port).ldindexHigh} " +
                        p"sqOH=0x${Hexadecimal(io.entry(port).sqindex)} " +
                        p"sqH=${io.entry(port).sqindexHigh} " +
                        p"dep=0x${Hexadecimal(io.entry(port).storeDepMask)} " +
                        p"ready=${!io.entry(port).uncache && !io.entryIssued(port)} " +
                        p"executing=${io.entryIssued(port)}\n"
                )
            }
        }

        for (port <- 0 until DcacheConfig.nPorts) {
            val storeBack = io.loadStoreBack(port)
            val storeReplayMatches = VecInit((0 until length).map { index =>
                storeBack.inst.ldindex(index) &&
                    entries(index).inst.ldindexHigh === storeBack.inst.ldindexHigh &&
                    entries(index).inst.robPtr.asUInt === storeBack.inst.robPtr.asUInt
            }).asUInt
            val sameCycleStoreWake = Mux(
                storeBack.partialOverlap,
                anyStoreCompleteWakeMatches(
                    storeBack.waitSqindex,
                    storeBack.waitSqindexHigh
                ),
                anyStoreDataWakeMatches(
                    storeBack.waitSqindex,
                    storeBack.waitSqindexHigh
                )
            )
            when(storeBack.storeData) {
                printf(
                    p"[DBG][LST][STORE_REPLAY] cycle=${debugCycle} port=${port.U} " +
                        p"pc=0x${Hexadecimal(storeBack.inst.pc)} " +
                        p"robQ=${storeBack.inst.robPtr.qidx} " +
                        p"robOff=${storeBack.inst.robPtr.offset} " +
                        p"robH=${storeBack.inst.robPtr.high} " +
                        p"robEpoch=${storeBack.inst.robPtr.epoch} " +
                        p"ldOH=0x${Hexadecimal(storeBack.inst.ldindex)} " +
                        p"ldH=${storeBack.inst.ldindexHigh} " +
                        p"match=0x${Hexadecimal(storeReplayMatches)} " +
                        p"waitSQ=0x${Hexadecimal(storeBack.waitSqindex)} " +
                        p"waitSQH=${storeBack.waitSqindexHigh} " +
                        p"sameWake=${sameCycleStoreWake}\n"
                )
            }

            val mshrBack = delayedMshrBack(port)
            val mshrReplayMatches = VecInit((0 until length).map { index =>
                mshrBack.inst.ldindex(index) &&
                    entries(index).inst.ldindexHigh === mshrBack.inst.ldindexHigh &&
                    entries(index).inst.robPtr.asUInt === mshrBack.inst.robPtr.asUInt
            }).asUInt
            when(mshrBack.valid) {
                printf(
                    p"[DBG][LST][MSHR_REPLAY] cycle=${debugCycle} port=${port.U} " +
                        p"pc=0x${Hexadecimal(mshrBack.inst.pc)} " +
                        p"robQ=${mshrBack.inst.robPtr.qidx} " +
                        p"robOff=${mshrBack.inst.robPtr.offset} " +
                        p"robH=${mshrBack.inst.robPtr.high} " +
                        p"robEpoch=${mshrBack.inst.robPtr.epoch} " +
                        p"ldOH=0x${Hexadecimal(mshrBack.inst.ldindex)} " +
                        p"ldH=${mshrBack.inst.ldindexHigh} " +
                        p"match=0x${Hexadecimal(mshrReplayMatches)} " +
                        p"full=${mshrBack.mshrFull} progress=${io.interReplay.mshrProgress}\n"
                )
            }

            when(io.issueInsts(port).fire) {
                val inst = io.issueInsts(port).bits
                printf(
                    p"[DBG][LST][ISSUE] cycle=${debugCycle} port=${port.U} " +
                        p"pc=0x${Hexadecimal(inst.pc)} robQ=${inst.robPtr.qidx} " +
                        p"robOff=${inst.robPtr.offset} robH=${inst.robPtr.high} " +
                        p"robEpoch=${inst.robPtr.epoch} " +
                        p"ldOH=0x${Hexadecimal(inst.ldindex)} ldH=${inst.ldindexHigh} " +
                        p"dep=0x${Hexadecimal(inst.storeDepMask)}\n"
                )
            }

            when(io.complete(port).valid) {
                val complete = io.complete(port).bits
                printf(
                    p"[DBG][LST][COMPLETE] cycle=${debugCycle} port=${port.U} " +
                        p"robQ=${complete.robPtr.qidx} robOff=${complete.robPtr.offset} " +
                        p"robH=${complete.robPtr.high} robEpoch=${complete.robPtr.epoch} " +
                        p"ldOH=0x${Hexadecimal(complete.ldindex)} ldH=${complete.ldindexHigh} " +
                        p"forwarded=${complete.forwarded} " +
                        p"forwardSQ=0x${Hexadecimal(complete.forwardSqindex)} " +
                        p"forwardSQH=${complete.forwardSqindexHigh}\n"
                )
            }
        }

        when(io.interReplay.mshrProgress) {
            val matchingLoads = VecInit((0 until length).map { index =>
                entries(index).valid && entries(index).waitType.mshrFull
            }).asUInt
            printf(
                p"[DBG][LST][MSHR_PROGRESS] cycle=${debugCycle} " +
                    p"matchingLoads=0x${Hexadecimal(matchingLoads)}\n"
            )
        }

        for (wakeIndex <- io.interReplay.storeReady.indices) {
            val wake = io.interReplay.storeReady(wakeIndex)
            val matchingLoads = VecInit((0 until length).map { index =>
                entries(index).valid &&
                    (entries(index).waitType.storeData ||
                        entries(index).waitType.storeComplete) &&
                    storeWakeMatches(
                        entries(index).waitSqindex,
                        entries(index).waitSqindexHigh,
                        wake.bits
                    )
            }).asUInt
            when(wake.valid) {
                printf(
                    p"[DBG][LST][STORE_WAKE] cycle=${debugCycle} wake=${wakeIndex.U} " +
                        p"paddr=0x${Hexadecimal(wake.bits.paddr)} " +
                        p"sqOH=0x${Hexadecimal(wake.bits.sqindex)} " +
                        p"sqH=${wake.bits.sqindexHigh} " +
                        p"sqMask=0x${Hexadecimal(wake.bits.sqMask)} " +
                        p"sqHighMask=0x${Hexadecimal(wake.bits.sqHighMask)} " +
                        p"matchingLoads=0x${Hexadecimal(matchingLoads)}\n"
                )
            }
        }

        for (index <- 0 until length) {
            val entry = entries(index)
            val previous = previousEntries(index)
            val stateChanged = entry.asUInt =/= previous.asUInt
            when((entry.valid || previous.valid) && (stateChanged || (entry.valid && heartbeat))) {
                printf(
                    p"[DBG][LST][STATE] cycle=${debugCycle} entry=${index.U} " +
                        p"valid=${entry.valid} live=${liveMask(index)} ready=${issueReady(index)} " +
                        p"executing=${entry.executing} complete=${entry.complete} " +
                        p"mshrFull=${entry.waitType.mshrFull} " +
                        p"storeData=${entry.waitType.storeData} " +
                        p"storeComplete=${entry.waitType.storeComplete} " +
                        p"pc=0x${Hexadecimal(entry.inst.pc)} " +
                        p"robQ=${entry.inst.robPtr.qidx} robOff=${entry.inst.robPtr.offset} " +
                        p"robH=${entry.inst.robPtr.high} robEpoch=${entry.inst.robPtr.epoch} " +
                        p"ldOH=0x${Hexadecimal((BigInt(1) << index).U(length.W))} " +
                        p"ldH=${entry.inst.ldindexHigh} " +
                        p"sqOH=0x${Hexadecimal(entry.inst.sqindex)} " +
                        p"sqH=${entry.inst.sqindexHigh} " +
                        p"dep=0x${Hexadecimal(entry.inst.storeDepMask)} " +
                        p"waitSQ=0x${Hexadecimal(entry.waitSqindex)} " +
                        p"waitSQH=${entry.waitSqindexHigh}\n"
                )
            }
            when(io.ptrCtrl.flushMask(index) && entry.valid) {
                printf(p"[DBG][LST][FLUSH] cycle=${debugCycle} entry=${index.U}\n")
            }
            when(retiredMask(index) && entry.valid && !allocationMask(index)) {
                printf(p"[DBG][LST][RETIRE] cycle=${debugCycle} entry=${index.U}\n")
            }
        }
        }
    }

    when(!reset.asBool) {
        assert(PopCount(io.ptrCtrl.nextHeadPtr) === 1.U,
            "LoadStateTable: head pointer must be one-hot")
        assert(
            io.ptrCtrl.nextHeadSuffixMask ===
                VecInit.tabulate(length) { index =>
                    io.ptrCtrl.nextHeadPtr(index, 0).orR
                }.asUInt,
            "LoadStateTable: registered head suffix mask must match head pointer"
        )
        assert(PopCount(io.ptrCtrl.nextTailPtr) === 1.U,
            "LoadStateTable: tail pointer must be one-hot")
        for (port <- 0 until LoadQueueConfig.EnqNum) {
            val selected = Mux1H(io.entry(port).ldindex, entries)
            val sameGenerationOccupied = (io.entry(port).ldindex & occupiedMask).orR &&
                selected.inst.ldindexHigh === io.entry(port).ldindexHigh
            assert(!(io.entry(port).valid && sameGenerationOccupied),
                s"LoadStateTable: allocation port $port cannot overwrite a live generation")
            for (index <- 0 until length) {
                when(io.entry(port).valid && io.entry(port).ldindex(index)) {
                    assert(nextIssueReady(index) ===
                        (!io.entry(port).uncache && !io.entryIssued(port)),
                        "LoadStateTable: allocation must own reused entry readiness")
                }
            }
        }
        assert(!(io.entry(0).valid && io.entry(1).valid &&
            (io.entry(0).ldindex & io.entry(1).ldindex).orR),
            "LoadStateTable: two allocations cannot target the same entry")
        assert((allocationMask & issueFireMask) === 0.U,
            "LoadStateTable: allocation cannot reuse an entry issuing in the same cycle")
        assert((allocationMask & uncacheIssueMask) === 0.U,
            "LoadStateTable: allocation cannot reuse an uncache entry issuing in the same cycle")
        assert(PopCount(issueFireMask) <= DcacheConfig.nPorts.U,
            "LoadStateTable: issue fire mask exceeds DCache port count")
        for (port <- 0 until DcacheConfig.nPorts) {
            assert(!(selectedOH(port) & (~readyMask).asUInt).orR,
                s"LoadStateTable: issue port $port selected a non-ready Load")
            assert(selectedOH(port).orR ===
                (readyMask & replayBankMasks(port)).orR,
                s"LoadStateTable: replay port $port must preserve bank readiness")
        }
        assert(!(selectedOH(0) & selectedOH(1)).orR,
            "LoadStateTable: one resident load selected by both ports")
        for (index <- 0 until length) {
            when(issueReady(index)) {
                assert(entries(index).valid,
                    "LoadStateTable: ready entry must be valid")
                assert(!entries(index).inst.uncache,
                    "LoadStateTable: uncache entry cannot use cached issue selection")
                assert(!entries(index).executing && !entries(index).complete,
                    "LoadStateTable: ready entry cannot be executing or complete")
            }
            when(entries(index).executing) {
                assert(entries(index).valid && !issueReady(index) &&
                    !entries(index).complete,
                    "LoadStateTable: executing state must be exclusively live")
            }
            when(entries(index).complete) {
                assert(entries(index).valid && !issueReady(index) &&
                    !entries(index).executing,
                    "LoadStateTable: complete state must be exclusively live")
            }
            val waitsForStore = entries(index).waitType.storeData ||
                entries(index).waitType.storeComplete
            assert(!(entries(index).waitType.storeData &&
                entries(index).waitType.storeComplete),
                "LoadStateTable: one Load cannot wait for Store data and completion")
            when(entries(index).valid && waitsForStore) {
                assert(PopCount(entries(index).waitSqindex) === 1.U,
                    "LoadStateTable: Store wait identity must be one-hot")
            }
            assert(!(io.ptrCtrl.flushMask(index) && entries(index).valid && entries(index).executing &&
                entries(index).inst.uncache),
                "LoadStateTable: an executing uncache Load cannot be flushed")
        }
    }
}
