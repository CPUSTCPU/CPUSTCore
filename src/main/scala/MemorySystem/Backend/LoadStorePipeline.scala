package CPUSTC.memory.backend

import chisel3._
import chisel3.util._
import CPUSTC.memory._
import CPUSTC.memory.external._
import _root_.circt.stage.ChiselStage
import CPUSTC.perf.MemoryPerfEvents
import CPUSTC.backend.rob.RobPtr

class LoadStorePipelineIO(enablePerfCounters: Boolean = false) extends Bundle {
    val backendInst = Vec(LoadQueueConfig.EnqNum, Flipped(Decoupled(new BackendInst)))
    val directCachedLoad = Vec(
        LoadQueueConfig.EnqNum,
        Flipped(Decoupled(new DirectCachedLoad))
    )
    val commitStore = Flipped(Valid(Bool()))
    val robHeadLoad = Flipped(Valid(new RobHeadLoadInfo))
    val llCommit = Flipped(Valid(new RobPtr))
    val llCommitPaddr = Output(Valid(UInt(32.W)))
    val atomicStoreReq = Flipped(Decoupled(new AtomicStoreRequest))
    val atomicStoreDone = Output(Bool())
    val atomicBusy = Output(Bool())
    val dcacheMaintenanceReq = Flipped(Decoupled(new DcacheMaintenanceRequest))
    val dcacheMaintenanceResp = Decoupled(new DcacheMaintenanceResponse)
    val sqHeadOH = Input(UInt(StoreQueueConfig.length.W))
    val sqHeadHigh = Input(Bool())
    val sqFlushMask = Input(UInt(StoreQueueConfig.length.W))
    val sqLiveMask = Output(UInt(StoreQueueConfig.length.W))
    val sqFreedMask = Output(Valid(UInt(StoreQueueConfig.length.W)))
    val sqCommitPtrOH = Output(UInt(StoreQueueConfig.length.W))
    val sqCommitPtrHigh = Output(Bool())
    val sqCommittedMask = Output(UInt(StoreQueueConfig.length.W))
    val loadPtrCtrl = Input(new DispatchPtrCtrl)
    val lsqLive = Input(new MemoryLsqLiveState)
    val flush = Input(Bool())
    val loadRecovery = Input(Bool())
    val quiesce = Input(Bool())
    val drainDone = Output(Bool())
    val loadResult = Output(Vec(DcacheConfig.nPorts, Valid(new LoadResult)))
    val loadPredWake = Output(Vec(
        DcacheConfig.nPorts,
        Valid(new LoadPredictInfo)
    ))
    val loadPredResolve = Output(Vec(
        DcacheConfig.nPorts,
        Valid(new LoadPredictResolve)
    ))
    val storeComplete = Output(Vec(
        StoreQueueConfig.EnqNum,
        Valid(new StoreCompletionToken)
    ))
    val storeException = Output(Vec(
        LoadQueueConfig.EnqNum,
        Valid(new StoreExceptionEvent)
    ))
    val storeCommitTrace = Output(Valid(new StoreCommitTrace))
    val memory = new MshrMemoryIO
    val uncache = new UncacheMemoryIO
    val loadQueueFull = Output(Bool())
    val uncacheFull = Output(Bool())
    val perf = if (enablePerfCounters) {
        Some(Output(new MemoryPerfEvents))
    } else {
        None
    }
}

class LoadStorePipeline(
    useBlackBoxRam: Boolean = true,
    enablePerfCounters: Boolean = false
) extends Module {
    val io = IO(new LoadStorePipelineIO(enablePerfCounters))

    val storeQueue = Module(new StoreQueue)
    val storeIssueBuffer = Module(new StoreIssueBuffer)
    val loadStateTable = Module(new LoadStateTable(enableDebug = false))
    val dcache = Module(new DCache(useBlackBoxRam))
    val mshr = Module(new MissStatusHoldingRegister(enableDebug = false))

    object AtomicState extends ChiselEnum {
        val Idle, CachedIssue, CachedWait, UncachedIssue, UncachedWait = Value
    }

    val atomicState = RegInit(AtomicState.Idle)
    val atomicReq = Reg(new AtomicStoreRequest)
    val atomicTag = 1.U(StoreQueueConfig.length.W)
    val atomicActive = atomicState =/= AtomicState.Idle
    val baseDrainDone =
        !storeQueue.io.committedMask.orR &&
        dcache.io.idle &&
        mshr.io.idle

    // Capture the serialized request before presenting it to the DCache.  The
    // normal queues are already drained while quiesced, so the registered
    // boundary keeps that drain decision out of the demand-response cone.
    val maintenancePendingValid = RegInit(false.B)
    val maintenancePendingReq = Reg(new DcacheMaintenanceRequest)
    val maintenanceCaptureReady =
        !maintenancePendingValid &&
        !atomicActive &&
        io.quiesce &&
        baseDrainDone
    io.dcacheMaintenanceReq.ready := maintenanceCaptureReady
    when(io.dcacheMaintenanceReq.fire) {
        maintenancePendingValid := true.B
        maintenancePendingReq := io.dcacheMaintenanceReq.bits
    }

    dcache.io.maintenanceReq.valid := maintenancePendingValid
    dcache.io.maintenanceReq.bits := maintenancePendingReq
    when(dcache.io.maintenanceReq.fire) {
        maintenancePendingValid := false.B
    }

    // A dirty maintenance line first traverses the ordinary WBB/L2 path.  Do
    // not expose its response until that write has completed, so the system
    // controller can safely launch the second, direct-to-memory write.
    io.dcacheMaintenanceResp.valid :=
        dcache.io.maintenanceResp.valid && mshr.io.idle
    io.dcacheMaintenanceResp.bits := dcache.io.maintenanceResp.bits
    dcache.io.maintenanceResp.ready :=
        io.dcacheMaintenanceResp.ready && mshr.io.idle

    io.atomicStoreReq.ready :=
        atomicState === AtomicState.Idle && io.quiesce && baseDrainDone
    io.atomicStoreDone := false.B
    io.atomicBusy := atomicActive

    when(io.atomicStoreReq.fire) {
        atomicReq := io.atomicStoreReq.bits
        atomicState := Mux(
            io.atomicStoreReq.bits.uncache,
            AtomicState.UncachedIssue,
            AtomicState.CachedIssue
        )
    }

    storeQueue.io.flush := io.flush
    storeQueue.io.commit := io.commitStore
    storeQueue.io.dispatch.headPtrNext := io.sqHeadOH
    storeQueue.io.dispatch.headPtrNextHigh := io.sqHeadHigh
    storeQueue.io.dispatch.flushMask := io.sqFlushMask
    storeQueue.io.lsqLive := io.lsqLive
    io.sqLiveMask := storeQueue.io.liveMask
    io.sqFreedMask := storeQueue.io.freedMask
    io.sqCommitPtrOH := storeQueue.io.commitPtrOH
    io.sqCommitPtrHigh := storeQueue.io.commitPtrHigh
    io.sqCommittedMask := storeQueue.io.committedMask
    io.storeComplete := storeQueue.io.normalComplete
    io.storeException := storeQueue.io.exceptionComplete
    io.storeCommitTrace := storeQueue.io.commitTrace
    val uncacheStoreReq = WireDefault(0.U.asTypeOf(new BackendInst))
    uncacheStoreReq.uop.isSTD := true.B
    uncacheStoreReq.paddr := storeQueue.io.dequeue.bits.addr
    uncacheStoreReq.operateData := storeQueue.io.dequeue.bits.wdata
    uncacheStoreReq.mask := storeQueue.io.dequeue.bits.wen
    uncacheStoreReq.uncache := true.B
    uncacheStoreReq.valid := storeQueue.io.dequeue.bits.valid
    uncacheStoreReq.sqindex := storeQueue.io.dequeue.bits.sqindex
    uncacheStoreReq.sqindexHigh := storeQueue.io.dequeue.bits.sqindexHigh
    val atomicUncachedIssue = atomicState === AtomicState.UncachedIssue
    val atomicStoreInst = WireDefault(0.U.asTypeOf(new BackendInst))
    atomicStoreInst.uop.isSTD := true.B
    atomicStoreInst.paddr := atomicReq.paddr
    atomicStoreInst.operateData := atomicReq.data
    atomicStoreInst.mask := "b1111".U
    atomicStoreInst.uncache := atomicReq.uncache
    atomicStoreInst.valid := true.B
    atomicStoreInst.sqindex := atomicTag

    val normalUncacheStoreValid =
        !atomicActive &&
        storeQueue.io.dequeue.valid &&
        storeQueue.io.dequeue.bits.uncache
    io.uncache.storeReq.valid := atomicUncachedIssue || normalUncacheStoreValid
    io.uncache.storeReq.bits := Mux(
        atomicUncachedIssue,
        atomicStoreInst,
        uncacheStoreReq
    )

    val loadEntrance = Wire(Vec(LoadQueueConfig.EnqNum, new BackendInst))
    val loadEntranceFire = Wire(Vec(LoadQueueConfig.EnqNum, Bool()))
    val loadFlush = io.flush || io.loadPtrCtrl.redirect
    // Backend observes branch recovery one cycle before the registered memory
    // pointer control. Gate result side effects immediately, while the normal
    // registered redirect remains responsible for replay state transitions.
    val loadResultKill = loadFlush || io.loadRecovery

    for (port <- 0 until LoadQueueConfig.EnqNum) {
        val entrance = io.backendInst(port)
        val direct = io.directCachedLoad(port)
        val isLoad = entrance.bits.uop.isLD
        val isStd = entrance.bits.uop.isSTD
        val isSta = entrance.bits.uop.isSTA
        val stdPort = port
        val staPort = port + LoadQueueConfig.EnqNum

        storeQueue.io.enqueue(stdPort).valid := entrance.valid && isStd && !io.quiesce
        storeQueue.io.enqueue(stdPort).bits := entrance.bits
        storeQueue.io.enqueue(staPort).valid := entrance.valid && isSta && !io.quiesce
        storeQueue.io.enqueue(staPort).bits := entrance.bits

        val storeReady = Mux(isStd, storeQueue.io.enqueue(stdPort).ready,
            Mux(isSta, storeQueue.io.enqueue(staPort).ready, false.B))
        entrance.ready := !io.quiesce && Mux(isLoad, !loadFlush, storeReady)

        // An allocated LDQ slot can always be captured by LST. DCache
        // availability only controls the entrance bypass, never this ready.
        direct.ready := !io.quiesce && !loadFlush

        val directInst = WireDefault(0.U.asTypeOf(new BackendInst))
        directInst.uop.isLD := true.B
        directInst.valid := true.B
        directInst.pc := direct.bits.vaddr
        directInst.paddr := direct.bits.paddr
        directInst.translationPending := false.B
        directInst.uncache := false.B
        directInst.mask := direct.bits.mask
        directInst.signed := direct.bits.signed
        directInst.sqindex := direct.bits.sqindex
        directInst.sqindexHigh := direct.bits.sqindexHigh
        directInst.storeDepMask := direct.bits.storeDepMask
        directInst.ldindex := direct.bits.ldindex
        directInst.ldindexHigh := direct.bits.ldindexHigh
        directInst.soreceReg := direct.bits.pdest
        directInst.robPtr := direct.bits.robPtr
        directInst.pdest := direct.bits.pdest
        directInst.rfWen := direct.bits.rfWen

        loadEntrance(port) := Mux(direct.fire, directInst, entrance.bits)
        val slowLoadFire = entrance.valid && isLoad && !io.quiesce && !loadFlush
        loadEntranceFire(port) :=
            direct.fire || slowLoadFire

        assert(!(entrance.valid && direct.valid),
            s"LoadStorePipeline: lane $port presented slow and direct requests together")
        assert(slowLoadFire === (entrance.fire && isLoad),
            s"LoadStorePipeline: lane $port slow Load acceptance diverged from ready")
        when(direct.valid) {
            assert(PopCount(direct.bits.sqindex) === 1.U)
            assert(PopCount(direct.bits.ldindex) === 1.U)
        }
    }
    io.loadQueueFull := loadStateTable.io.full
    io.uncacheFull := io.uncache.full

    loadStateTable.io.ptrCtrl := io.loadPtrCtrl
    loadStateTable.io.lsqLive := io.lsqLive
    loadStateTable.io.sqHeadOH := io.sqHeadOH
    loadStateTable.io.sqHeadHigh := io.sqHeadHigh
    loadStateTable.io.pendingUncacheStore := storeQueue.io.pendingUncacheStore
    loadStateTable.io.storeWaitState := storeQueue.io.waitState
    loadStateTable.io.llCommit := io.llCommit
    io.llCommitPaddr := loadStateTable.io.llCommitPaddr
    loadStateTable.io.robHeadLoad := io.robHeadLoad
    io.uncache.loadReq.valid :=
        !io.quiesce && loadStateTable.io.uncacheReq.valid
    io.uncache.loadReq.bits := loadStateTable.io.uncacheReq.bits
    loadStateTable.io.uncacheReq.ready :=
        !io.quiesce && io.uncache.loadReq.ready
    for (port <- 0 until LoadQueueConfig.EnqNum) {
        loadStateTable.io.entry(port) := loadEntrance(port)
        loadStateTable.io.entry(port).valid := loadEntranceFire(port)
        assert(loadStateTable.io.entry(port).valid === loadEntranceFire(port))
    }

    dcache.io.forwardSources := storeQueue.io.forward
    for (port <- 0 until LoadQueueConfig.EnqNum) {
        dcache.io.stdBypass(port).valid := storeQueue.io.storeReady(port).valid
        dcache.io.stdBypass(port).bits.data := io.backendInst(port).bits.operateData
        dcache.io.stdBypass(port).bits.sqindex := io.backendInst(port).bits.sqindex
        dcache.io.stdBypass(port).bits.sqindexHigh := io.backendInst(port).bits.sqindexHigh
    }
    val forwardResultFire = Wire(Vec(DcacheConfig.nPorts, Bool()))
    val staleResultFire   = Wire(Vec(DcacheConfig.nPorts, Bool()))

    val headLoadResultNow = Wire(Vec(DcacheConfig.nPorts, Bool()))
    for (port <- 0 until DcacheConfig.nPorts) {
        loadStateTable.io.loadStoreBack(port) := dcache.io.loadStoreFail(port)
        loadStateTable.io.loadMshrBack(port) := dcache.io.loadMshrFail(port)
        io.loadPredWake(port) := dcache.io.loadPredWake(port)
        io.loadPredWake(port).valid :=
            dcache.io.loadPredWake(port).valid && !loadResultKill
        io.loadPredResolve(port) := dcache.io.loadPredResolve(port)
        // Resolve belongs to the previous cycle's exported wake. Recovery must
        // cancel that wake rather than dropping the resolve and orphaning IQ
        // poison state.
        io.loadPredResolve(port).bits.success :=
            dcache.io.loadPredResolve(port).bits.success && !loadResultKill
    }
    dcache.io.flush := loadFlush
    dcache.io.ldqValidMask := io.lsqLive.ldqValidMask
    dcache.io.ldqHighMask := io.lsqLive.ldqHighMask

    mshr.io.req0 <> dcache.io.mshrIO.req0
    mshr.io.req1 <> dcache.io.mshrIO.req1
    dcache.io.mshrIO.storeAdmissionReady := mshr.io.storeAdmissionReady
    mshr.io.loadWaiterFlush := loadResultKill

    def rebuildMshrLoadResult(compact: MshrLoadResult): LoadResult = {
        val result = WireDefault(0.U.asTypeOf(new LoadResult))
        val metadata = compact.waiter.metadata
        val paddr = Cat(compact.linePaddr, compact.waiter.byteOffset)

        result.inst.uop.isLD := true.B
        result.inst.pc := Cat(
            metadata.vaddrVpn,
            paddr(MshrLoadWaiterConfig.pageOffsetWidth - 1, 0)
        )
        result.inst.paddr := paddr
        result.inst.mask := MshrLoadFormat.mask(metadata.format)
        result.inst.valid := true.B
        result.inst.signed := MshrLoadFormat.signed(metadata.format)
        // The post-result path only requires a legal one-hot SQ value. Store
        // dependency state has already been resolved before MSHR admission.
        result.inst.sqindex := 1.U
        result.inst.ldindex := UIntToOH(
            metadata.token,
            LoadStateTableConfig.length
        )
        result.inst.ldindexHigh := metadata.ldindexHigh
        result.inst.robPtr := metadata.robPtr
        result.inst.pdest := metadata.pdest
        result.inst.rfWen := metadata.pdest =/= 0.U
        result.data := compact.data
        result.exception := 0.U
        result
    }

    dcache.io.mshrIO.victimAvailable := mshr.io.victimAvailable
    mshr.io.victimReq <> dcache.io.mshrIO.victimReq
    dcache.io.mshrIO.resp <> mshr.io.refill
    io.memory <> mshr.io.memory
    io.drainDone := baseDrainDone && !atomicActive

    val atomicCachedActive =
        atomicState === AtomicState.CachedIssue ||
        atomicState === AtomicState.CachedWait
    val normalDcacheStoreComplete = WireDefault(dcache.io.storeComplete)
    val normalMshrStoreComplete = WireDefault(mshr.io.storeComplete)
    val normalDcacheStoreRetry = WireDefault(dcache.io.storeRetry)
    normalDcacheStoreComplete.valid :=
        dcache.io.storeComplete.valid && !atomicCachedActive
    normalMshrStoreComplete.valid :=
        mshr.io.storeComplete.valid && !atomicCachedActive
    normalDcacheStoreRetry.valid :=
        dcache.io.storeRetry.valid && !atomicCachedActive

    // DCache returns only the oldest rejected Store identity. StoreQueue uses
    // that boundary to withdraw every resident issued younger Store; clearing
    // the elastic buffer in the same registered wave removes its payload copy.
    val storeRetryBarrier = normalDcacheStoreRetry.valid
    storeIssueBuffer.io.clear := storeRetryBarrier

    storeQueue.io.complete(0) := normalDcacheStoreComplete
    storeQueue.io.complete(1) := normalMshrStoreComplete
    storeQueue.io.retry := normalDcacheStoreRetry

    val delayedMshrProgress = RegNext(mshr.io.progress, false.B)
    loadStateTable.io.interReplay.mshrProgress := delayedMshrProgress
    val storeMshrBlocked = RegInit(false.B)
    when(delayedMshrProgress) {
        storeMshrBlocked := false.B
    }.elsewhen(storeRetryBarrier) {
        storeMshrBlocked := true.B
    }
    loadStateTable.io.storeFreedMask := Mux(storeQueue.io.freedMask.valid,
        storeQueue.io.freedMask.bits, 0.U)
    loadStateTable.io.interReplay.storeReady(0) := storeQueue.io.storeReady(0)
    loadStateTable.io.interReplay.storeReady(1) := storeQueue.io.storeReady(1)
    // STA completion is not a store-data wake. Before its write edge the SQ
    // address is invisible to forwarding; afterwards address and data are both
    // resident, so no Load can be waiting for data on this one-shot event.
    for (port <- 2 until 4) {
        loadStateTable.io.interReplay.storeReady(port) :=
            0.U.asTypeOf(Valid(new StoreReadyEvent))
    }
    val delayedDcacheStoreComplete = RegNext(normalDcacheStoreComplete,
        0.U.asTypeOf(Valid(new StoreReadyEvent)))
    val delayedMshrStoreComplete = RegNext(normalMshrStoreComplete,
        0.U.asTypeOf(Valid(new StoreReadyEvent)))
    loadStateTable.io.interReplay.storeReady(4) := delayedDcacheStoreComplete
    loadStateTable.io.interReplay.storeReady(5) := delayedMshrStoreComplete

    val selectedValid = WireInit(VecInit.fill(DcacheConfig.nPorts)(false.B))
    val selectedReq = Wire(Vec(DcacheConfig.nPorts, new DcachePpReq))
    selectedReq := VecInit.fill(DcacheConfig.nPorts)(0.U.asTypeOf(new DcachePpReq))

    val storeBufferEnqueueReq = WireDefault(0.U.asTypeOf(new DcachePpReq))
    storeBufferEnqueueReq.uop.isSTD := true.B
    storeBufferEnqueueReq.paddr := storeQueue.io.dequeue.bits.addr
    storeBufferEnqueueReq.operateData := storeQueue.io.dequeue.bits.wdata
    storeBufferEnqueueReq.mask := storeQueue.io.dequeue.bits.wen
    storeBufferEnqueueReq.valid := storeQueue.io.dequeue.bits.valid
    storeBufferEnqueueReq.sqindex := storeQueue.io.dequeue.bits.sqindex
    storeBufferEnqueueReq.sqindexHigh := storeQueue.io.dequeue.bits.sqindexHigh
    storeIssueBuffer.io.enqueue.valid := !atomicActive && !storeRetryBarrier &&
        !storeMshrBlocked &&
        storeQueue.io.dequeue.valid &&
        !storeQueue.io.dequeue.bits.uncache
    storeIssueBuffer.io.enqueue.bits := storeBufferEnqueueReq

    val storeReq = storeIssueBuffer.io.dequeue.bits
    val atomicCachedReq = WireDefault(atomicStoreInst.asTypeOf(new DcachePpReq))
    atomicCachedReq.uncache := false.B
    val atomicCachedIssue = atomicState === AtomicState.CachedIssue
    val cachedStoreValid = !atomicActive && storeIssueBuffer.io.dequeue.valid
    val loadPathReady = VecInit((0 until DcacheConfig.nPorts).map { port =>
        dcache.io.requestAvailable(port) &&
            !loadFlush &&
            !io.quiesce
    })
    val issueLoadValid = VecInit((0 until DcacheConfig.nPorts).map { port =>
        loadStateTable.io.issueInsts(port).valid
    })
    val issueLoadReq = VecInit((0 until DcacheConfig.nPorts).map { port =>
        loadStateTable.io.issueInsts(port).bits.asTypeOf(new DcachePpReq)
    })
    // Recovery is already registered at the CPUSTC.memory boundary. Stall both
    // issue candidates for that one cycle instead of comparing the selected
    // payload against global LDQ masks; the latter folds the oldest-selection
    // network into every DCache RAM enable and S1 clock enable.
    val issueLoadCurrent = VecInit.fill(DcacheConfig.nPorts)(
        !io.loadPtrCtrl.redirect
    )
    val entranceLoadValid = Wire(Vec(LoadQueueConfig.EnqNum, Bool()))
    val entranceLoadReq = Wire(Vec(LoadQueueConfig.EnqNum, new DcachePpReq))
    for (lane <- 0 until LoadQueueConfig.EnqNum) {
        entranceLoadValid(lane) := loadStateTable.io.entry(lane).valid &&
            !loadStateTable.io.entry(lane).uncache
        entranceLoadReq(lane) := loadStateTable.io.entry(lane).asTypeOf(new DcachePpReq)
    }

    val channel1GrantIssue0 = issueLoadValid(0)
    val channel1GrantEntrance0 = !issueLoadValid(0) && entranceLoadValid(0)
    val channel1GrantEntrance1 = !issueLoadValid(0) &&
        !entranceLoadValid(0) && entranceLoadValid(1)
    val channel1LoadGrant = VecInit(
        channel1GrantIssue0,
        channel1GrantEntrance0,
        channel1GrantEntrance1
    )

    val channel0GrantIssue1 = issueLoadValid(1)
    val channel0GrantEntrance0 = issueLoadValid(0) &&
        !issueLoadValid(1) && entranceLoadValid(0)
    val channel0GrantEntrance1 =
        (issueLoadValid(0) && !issueLoadValid(1) &&
            !entranceLoadValid(0) && entranceLoadValid(1)) ||
        (!issueLoadValid(0) && !issueLoadValid(1) &&
            entranceLoadValid(0) && entranceLoadValid(1))
    val channel0LoadGrant = VecInit(
        channel0GrantIssue1,
        channel0GrantEntrance0,
        channel0GrantEntrance1
    )

    selectedReq(1) := Mux1H(channel1LoadGrant, Seq(
        issueLoadReq(0),
        entranceLoadReq(0),
        entranceLoadReq(1)
    ))
    val channel1BlockedIssue = channel1GrantIssue0 && !issueLoadCurrent(0)
    selectedValid(1) := channel1LoadGrant.asUInt.orR &&
        loadPathReady(1) && !channel1BlockedIssue

    val channel0BlockedIssue = channel0GrantIssue1 && !issueLoadCurrent(1)
    val channel0LoadCanIssue = channel0LoadGrant.asUInt.orR &&
        loadPathReady(0) && !channel0BlockedIssue
    val normalStoreGrant = cachedStoreValid && !channel0LoadCanIssue &&
        !storeRetryBarrier && !storeMshrBlocked
    val port0StoreValid = atomicCachedIssue || normalStoreGrant

    when(atomicCachedIssue) {
        selectedValid(0) := true.B
        selectedReq(0) := atomicCachedReq
    }.elsewhen(channel0LoadCanIssue) {
        selectedValid(0) := true.B
        selectedReq(0) := Mux1H(channel0LoadGrant, Seq(
            issueLoadReq(1),
            entranceLoadReq(0),
            entranceLoadReq(1)
        ))
    }.elsewhen(normalStoreGrant) {
        selectedValid(0) := true.B
        selectedReq(0) := storeReq
    }

    val dcacheReq = Wire(Vec(DcacheConfig.nPorts, new DcachePpReq))
    dcacheReq := selectedReq

    // Request arbitration already separates cached stores from cached loads.
    // Keep class control out of the wide payload mux at the DCache boundary.
    dcacheReq(0).uop.isSTD := port0StoreValid
    dcacheReq(0).uop.isLD := !port0StoreValid
    dcacheReq(0).uncache := false.B
    dcacheReq(1).uop.isSTD := false.B
    dcacheReq(1).uop.isLD := true.B
    dcacheReq(1).uncache := false.B

    when(selectedValid(0)) {
        assert(!selectedReq(0).uncache,
            "LoadStorePipeline: DCache port 0 request must be cached")
        assert(selectedReq(0).uop.isSTD === port0StoreValid &&
            selectedReq(0).uop.isLD === !port0StoreValid,
            "LoadStorePipeline: DCache port 0 request class must match its grant")
    }
    when(selectedValid(1)) {
        assert(!selectedReq(1).uncache && selectedReq(1).uop.isLD &&
            !selectedReq(1).uop.isSTD,
            "LoadStorePipeline: DCache port 1 request must be a cached load")
    }

    for (port <- 0 until DcacheConfig.nPorts) {
        dcache.io.mainPp(port).req.valid := selectedValid(port)
        dcache.io.mainPp(port).req.bits := dcacheReq(port)
    }

    val channel1LoadFire = selectedValid(1) &&
        dcache.io.mainPp(1).req.ready
    val channel0LoadFire = channel0LoadCanIssue &&
        dcache.io.mainPp(0).req.ready
    assert(channel1LoadFire === dcache.io.mainPp(1).req.fire)
    assert(channel0LoadFire ===
        (dcache.io.mainPp(0).req.fire && !port0StoreValid))
    val entryIssued = VecInit(
        (channel1GrantEntrance0 && channel1LoadFire) ||
            (channel0GrantEntrance0 && channel0LoadFire),
        (channel1GrantEntrance1 && channel1LoadFire) ||
            (channel0GrantEntrance1 && channel0LoadFire)
    )
    loadStateTable.io.entryIssued := entryIssued

    assert(entryIssued(0) ===
        ((channel1GrantEntrance0 && channel1LoadFire) ||
            (channel0GrantEntrance0 && channel0LoadFire)),
        "LoadStorePipeline: entrance 0 issued sideband must match DCache fire")
    assert(entryIssued(1) ===
        ((channel1GrantEntrance1 && channel1LoadFire) ||
            (channel0GrantEntrance1 && channel0LoadFire)),
        "LoadStorePipeline: entrance 1 issued sideband must match DCache fire")

    loadStateTable.io.issueInsts(0).ready :=
        channel1LoadFire && issueLoadCurrent(0)
    loadStateTable.io.issueInsts(1).ready :=
        channel0LoadFire && issueLoadCurrent(1)

    for (port <- 0 until DcacheConfig.nPorts) {
        when(issueLoadValid(port) && !issueLoadCurrent(port)) {
            assert(!loadStateTable.io.issueInsts(port).fire,
                s"LoadStorePipeline: recovery-blocked Load fired on port $port")
        }
    }

    storeQueue.io.dequeue.ready := !atomicActive && !storeRetryBarrier &&
        !storeMshrBlocked && Mux(
            storeQueue.io.dequeue.bits.uncache,
            io.uncache.storeReq.ready,
            storeIssueBuffer.io.enqueue.ready)

    val cachedStoreDequeueFire = storeQueue.io.dequeue.fire &&
        !storeQueue.io.dequeue.bits.uncache
    val dcacheStoreFire = dcache.io.mainPp(0).req.fire && normalStoreGrant
    storeIssueBuffer.io.dequeue.ready := dcacheStoreFire
    assert(cachedStoreDequeueFire === storeIssueBuffer.io.enqueue.fire,
        "LoadStorePipeline: cached Store dequeue must enter StoreIssueBuffer")
    assert(storeIssueBuffer.io.dequeue.fire === dcacheStoreFire,
        "LoadStorePipeline: StoreIssueBuffer dequeue and DCache request must fire together")
    when(storeMshrBlocked) {
        assert(!storeQueue.io.dequeue.fire,
            "LoadStorePipeline: MSHR-blocked Store barrier must stop SQ dequeue")
        assert(!dcacheStoreFire,
            "LoadStorePipeline: MSHR-blocked Store barrier must stop DCache issue")
    }
    val atomicCachedFire =
        atomicCachedIssue && dcache.io.mainPp(0).req.fire
    val atomicCachedRetry =
        atomicState === AtomicState.CachedWait && dcache.io.storeRetry.valid
    val atomicCachedDone =
        atomicState === AtomicState.CachedWait &&
        (dcache.io.storeComplete.valid || mshr.io.storeComplete.valid)
    val atomicUncachedFire =
        atomicUncachedIssue && io.uncache.storeReq.fire
    val atomicUncachedDone =
        atomicState === AtomicState.UncachedWait && io.uncache.writeDone

    when(atomicCachedFire) {
        atomicState := AtomicState.CachedWait
    }
    when(atomicCachedRetry) {
        atomicState := AtomicState.CachedIssue
    }
    when(atomicCachedDone) {
        atomicState := AtomicState.Idle
        io.atomicStoreDone := true.B
    }
    when(atomicUncachedFire) {
        atomicState := AtomicState.UncachedWait
    }
    when(atomicUncachedDone) {
        atomicState := AtomicState.Idle
        io.atomicStoreDone := true.B
    }

    when(atomicCachedRetry) {
        assert(dcache.io.storeRetry.bits.sqindex === atomicTag &&
            !dcache.io.storeRetry.bits.sqindexHigh,
            "LoadStorePipeline: atomic retry lost its private tag")
    }
    when(atomicCachedDone) {
        val completionMask = Mux(
            dcache.io.storeComplete.valid,
            dcache.io.storeComplete.bits.sqMask,
            mshr.io.storeComplete.bits.sqMask
        )
        assert((completionMask & atomicTag).orR,
            "LoadStorePipeline: atomic completion lost its private tag")
        assert(!(dcache.io.storeComplete.valid && mshr.io.storeComplete.valid),
            "LoadStorePipeline: one atomic Store cannot complete twice")
    }

    when(atomicActive) {
        assert(!storeQueue.io.dequeue.fire,
            "LoadStorePipeline: normal Store issued while an atomic Store was active")
    }

    // Uncached responses may return after an arbitrary external delay. Check
    // their complete resident identity on an isolated slow path, then buffer
    // the validated result before it joins the common result lanes. Cached S2
    // responses and MSHR waiters are killed in their owning local pipelines.
    val uncacheResultValid = RegInit(false.B)
    val uncacheResultBits = Reg(new LoadForwardResult)
    val uncacheResultPop = WireDefault(false.B)
    loadStateTable.io.uncacheResultCheck := io.uncache.loadResp.bits.inst
    val uncacheResultCurrent = loadStateTable.io.uncacheResultCurrent
    val uncacheResultSpace = !uncacheResultValid || uncacheResultPop
    // A branch recovery may overlap the response of the older ROB-head
    // uncached Load.  That Load survives recovery, so both a raw response and
    // an already validated buffered response must stall instead of being
    // discarded.  A full architectural flush is the only event that clears
    // the resident token.
    val uncacheResultRecoveryHold = loadResultKill && !io.flush
    io.uncache.loadResp.ready :=
        !loadResultKill && (!uncacheResultCurrent || uncacheResultSpace)
    val uncacheResultFire = io.uncache.loadResp.fire
    val uncacheResultCapture = uncacheResultFire && uncacheResultCurrent
    val uncacheResultStaleDrop = uncacheResultFire && !uncacheResultCurrent
    val uncacheCaptureBits = WireDefault(0.U.asTypeOf(new LoadForwardResult))
    uncacheCaptureBits.result.inst := io.uncache.loadResp.bits.inst
    uncacheCaptureBits.result.data := io.uncache.loadResp.bits.data
    uncacheCaptureBits.result.exception :=
        io.uncache.loadResp.bits.inst.exception

    when(io.flush) {
        uncacheResultValid := false.B
    }.elsewhen(uncacheResultRecoveryHold) {
        // Hold both valid and payload until the recovery window closes.
    }.elsewhen(uncacheResultCapture) {
        uncacheResultValid := true.B
        uncacheResultBits := uncacheCaptureBits
    }.elsewhen(uncacheResultPop) {
        uncacheResultValid := false.B
    }

    when(uncacheResultFire) {
        assert(PopCount(io.uncache.loadResp.bits.inst.ldindex) === 1.U,
            "LoadStorePipeline: uncache response must carry one LDQ index")
    }
    when(uncacheResultValid) {
        assert(PopCount(uncacheResultBits.result.inst.ldindex) === 1.U,
            "LoadStorePipeline: buffered uncache result lost its LDQ identity")
    }
    when(loadResultKill) {
        assert(!io.uncache.loadResp.ready,
            "LoadStorePipeline: uncache response ingress escaped recovery")
        assert(!uncacheResultPop,
            "LoadStorePipeline: buffered uncache result escaped recovery")
        assert(!uncacheResultCapture,
            "LoadStorePipeline: uncache response captured during recovery")
    }
    when(uncacheResultCapture) {
        assert(uncacheResultCurrent && uncacheResultSpace,
            "LoadStorePipeline: uncache capture requires a current identity and space")
    }
    when(uncacheResultPop) {
        assert(uncacheResultValid,
            "LoadStorePipeline: uncache result popped without a resident token")
    }
    when(uncacheResultStaleDrop) {
        assert(!uncacheResultCapture,
            "LoadStorePipeline: stale uncache response cannot enter the result buffer")
    }

    val previousUncacheRecoveryHold = RegNext(uncacheResultRecoveryHold, false.B)
    val previousUncacheResultValid = RegNext(uncacheResultValid, false.B)
    val previousUncacheResultBits = RegNext(uncacheResultBits)
    val previousUncacheCapture = RegNext(uncacheResultCapture, false.B)
    val previousUncacheCaptureBits = RegNext(uncacheCaptureBits)
    val previousUncachePopOnly = RegNext(
        uncacheResultPop && !uncacheResultCapture,
        false.B
    )
    when(previousUncacheRecoveryHold) {
        assert(uncacheResultValid === previousUncacheResultValid,
            "LoadStorePipeline: recovery changed buffered uncache validity")
        when(previousUncacheResultValid) {
            assert(uncacheResultBits.asUInt === previousUncacheResultBits.asUInt,
                "LoadStorePipeline: recovery changed buffered uncache payload")
        }
    }
    when(previousUncacheCapture) {
        assert(uncacheResultValid &&
            uncacheResultBits.asUInt === previousUncacheCaptureBits.asUInt,
            "LoadStorePipeline: uncache capture did not replace the resident token")
    }
    when(previousUncachePopOnly) {
        assert(!uncacheResultValid,
            "LoadStorePipeline: uncache pop did not clear the resident token")
    }

    for (port <- 0 until DcacheConfig.nPorts) {
        val dcacheResp = dcache.io.mainPp(port).resp
        val dcacheIsLoad = dcacheResp.bits.inst.uop.isLD
        val mshrSelected = mshr.io.loadReturn(port).valid && !loadResultKill
        val mshrResult = rebuildMshrLoadResult(mshr.io.loadReturn(port).bits)
        val dcacheResultValid = dcacheResp.valid && dcacheIsLoad
        val cachedResultValid = mshrSelected || dcacheResultValid
        val cachedResultBits = WireDefault(0.U.asTypeOf(new LoadForwardResult))
        cachedResultBits.result.inst := Mux(
            mshrSelected,
            mshrResult.inst,
            dcacheResp.bits.inst
        )
        cachedResultBits.result.data := Mux(
            mshrSelected,
            mshrResult.data,
            dcacheResp.bits.rdata
        )
        cachedResultBits.result.exception := Mux(
            mshrSelected,
            mshrResult.exception,
            dcacheResp.bits.exception
        )
        cachedResultBits.forwarded := !mshrSelected && dcacheResp.bits.forwarded
        cachedResultBits.forwardSqindex := Mux(
            mshrSelected,
            0.U,
            dcacheResp.bits.forwardSqindex
        )
        cachedResultBits.forwardSqindexHigh :=
            !mshrSelected && dcacheResp.bits.forwardSqindexHigh
        cachedResultBits.forwardCommitted :=
            !mshrSelected && dcacheResp.bits.forwardCommitted

        val selectedResultValid = Wire(Bool())
        val selectedResultBits = Wire(new LoadForwardResult)
        val selectedResultFire = Wire(Bool())
        val cachedResultReady = WireDefault(true.B)

        if (port == 0) {
            val cachedInput = 0
            val uncacheInput = 1
            val resultArb = Module(new Arbiter(new LoadForwardResult, 2))
            resultArb.io.in(cachedInput).valid := cachedResultValid
            resultArb.io.in(cachedInput).bits := cachedResultBits
            resultArb.io.in(uncacheInput).valid :=
                uncacheResultValid && !loadResultKill
            resultArb.io.in(uncacheInput).bits := uncacheResultBits
            resultArb.io.out.ready := true.B
            uncacheResultPop := resultArb.io.in(uncacheInput).fire
            cachedResultReady := resultArb.io.in(cachedInput).ready
            selectedResultValid := resultArb.io.out.valid
            selectedResultBits := resultArb.io.out.bits
            selectedResultFire := resultArb.io.out.fire
        } else {
            selectedResultValid := cachedResultValid
            selectedResultBits := cachedResultBits
            selectedResultFire := cachedResultValid
        }

        io.loadResult(port).valid :=
            selectedResultValid && !loadResultKill
        io.loadResult(port).bits := selectedResultBits.result

        headLoadResultNow(port) := io.robHeadLoad.valid &&
            io.robHeadLoad.bits.waiting &&
            selectedResultFire && !loadResultKill &&
            selectedResultBits.result.inst.robPtr.asUInt === io.robHeadLoad.bits.robPtr.asUInt

        loadStateTable.io.complete(port).valid := io.loadResult(port).valid
        loadStateTable.io.complete(port).bits.ldindex := selectedResultBits.result.inst.ldindex
        loadStateTable.io.complete(port).bits.ldindexHigh := selectedResultBits.result.inst.ldindexHigh
        loadStateTable.io.complete(port).bits.robPtr := selectedResultBits.result.inst.robPtr
        loadStateTable.io.complete(port).bits.forwarded := selectedResultBits.forwarded
        loadStateTable.io.complete(port).bits.forwardSqindex := selectedResultBits.forwardSqindex
        loadStateTable.io.complete(port).bits.forwardSqindexHigh :=
            selectedResultBits.forwardSqindexHigh
        loadStateTable.io.complete(port).bits.forwardCommitted :=
            selectedResultBits.forwardCommitted

        forwardResultFire(port) := selectedResultFire &&
            !loadResultKill && selectedResultBits.forwarded
        staleResultFire(port) := (if (port == 0) {
            uncacheResultFire && !uncacheResultCurrent
        } else {
            false.B
        })

        when(dcacheResp.valid && dcacheResp.bits.predictReserved) {
            assert(cachedResultReady,
                s"LoadStorePipeline: predictive result on port $port must not be delayed")
        }

        when(mshr.io.loadReturn(port).valid) {
            assert(!dcache.io.requestAvailable(port),
                s"LoadStorePipeline: direct MSHR return must own request lane $port")
            assert(!dcacheResp.valid,
                s"LoadStorePipeline: direct MSHR return must own result lane $port")
            assert(PopCount(mshrResult.inst.ldindex) === 1.U,
                s"LoadStorePipeline: direct MSHR return on port $port lost its waiter token")
        }
        when(loadResultKill) {
            assert(!io.loadResult(port).valid,
                s"LoadStorePipeline: Load completed during recovery on port $port")
            assert(!loadStateTable.io.complete(port).valid,
                s"LoadStorePipeline: LST completed during recovery on port $port")
            assert(!io.loadPredWake(port).valid,
                s"LoadStorePipeline: predictive wake escaped recovery on port $port")
            when(io.loadPredResolve(port).valid) {
                assert(!io.loadPredResolve(port).bits.success,
                    s"LoadStorePipeline: recovery resolve must cancel port $port")
            }
        }
    }

    if (enablePerfCounters) {
        val perf = io.perf.get

        perf.icacheHit  := false.B
        perf.icacheMiss := false.B
        perf.dcacheLoadHitCount   := dcache.io.perf.loadHitCount
        perf.dcacheLoadMissCount  := dcache.io.perf.loadNormalMissCount
        perf.dcacheMshrFullCount  := dcache.io.perf.loadMshrFullCount
        perf.dcacheStoreHitCount  := dcache.io.perf.storeHitCount
        perf.dcacheStoreMissCount := dcache.io.perf.storeMissCount
        perf.dcacheStoreRetryCount := dcache.io.perf.storeRetryCount
        perf.dcacheStoreMshrFullRetryCount := dcache.io.perf.storeMshrFullRetryCount
        perf.dcacheStoreRefillRetryCount := dcache.io.perf.storeRefillRetryCount
        perf.dcacheStoreEntranceRetryCount := dcache.io.perf.storeEntranceRetryCount
        perf.l2IReadHit := false.B
        perf.l2IReadMiss := false.B
        perf.l2DReadHit := false.B
        perf.l2DReadMiss := false.B
        perf.l2WriteHit := false.B
        perf.l2WriteMiss := false.B
        perf.l2DirtyWriteback := false.B
        perf.l2UncacheRead := false.B
        perf.l2UncacheWrite := false.B
        perf.l2Busy := false.B

        perf.loadIssueCount := PopCount(VecInit((0 until DcacheConfig.nPorts).map { port =>
            dcache.io.mainPp(port).req.fire && selectedReq(port).uop.isLD
        }))
        perf.loadForwardCount := PopCount(forwardResultFire)
        perf.storeDataReplayCount := PopCount(VecInit(dcache.io.loadStoreFail.map { fail =>
            fail.valid && fail.storeData
        }))
        perf.partialOverlapReplayCount := PopCount(VecInit(dcache.io.loadStoreFail.map { fail =>
            fail.valid && fail.partialOverlap
        }))
        perf.waitStoreDataReplayCount := PopCount(VecInit(dcache.io.loadStoreFail.map { fail =>
            fail.valid && fail.waitStoreData
        }))
        perf.staleResultCount := PopCount(staleResultFire)

        perf.headLoadTracked := loadStateTable.io.headPerf.tracked
        perf.headLoadMissing := loadStateTable.io.headPerf.missing
        perf.headLoadReadyIssued := loadStateTable.io.headPerf.readyIssued
        perf.headLoadReadyWait := loadStateTable.io.headPerf.readyWait
        perf.headLoadExecuting := loadStateTable.io.headPerf.executing
        perf.headLoadExecutingAfterStoreReplay :=
            loadStateTable.io.headPerf.executingAfterStoreReplay
        perf.headLoadWaitMshrFull := loadStateTable.io.headPerf.waitMshrFull
        perf.headLoadWaitStoreData := loadStateTable.io.headPerf.waitStoreData
        perf.headLoadComplete := loadStateTable.io.headPerf.complete
        perf.headLoadOther := loadStateTable.io.headPerf.other
        perf.headLoadResultNow := headLoadResultNow.asUInt.orR

        perf.lstOccupancy    := loadStateTable.io.occupancy
        perf.lstFull         := loadStateTable.io.full
        perf.storeBufferFull := false.B
    }
}

object GenerateLoadStorePipeline extends App {
    ChiselStage.emitSystemVerilogFile(
        new LoadStorePipeline(useBlackBoxRam = false),
        args = Array("--target-dir", "generated/load-store-pipeline"),
        firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info")
    )
}
