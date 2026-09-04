package CPUSTC.memory

import chisel3._
import chisel3.util._
import CPUSTC.memory.backend._
import CPUSTC.memory.external._
import CPUSTC.memory.frontend._
import CPUSTC.memory.mmu._
import _root_.circt.stage.ChiselStage
import CPUSTC.perf.MemoryPerfEvents
import CPUSTC.config.ExpCode
import CPUSTC.config.SystemOp
import CPUSTC.backend.{AddressModeTranslator, AddressTranslationState}
import CPUSTC.backend.rob.RobPtr

class MemorySubSystemIO(enablePerfCounters: Boolean = false) extends Bundle {
    val icache = new IcachePpBus

    val backendInst = Vec(LoadQueueConfig.EnqNum, Flipped(Decoupled(new BackendInst)))
    val directCachedLoad = Vec(
        LoadQueueConfig.EnqNum,
        Flipped(Decoupled(new DirectCachedLoad))
    )
    val sysMemCmd = Flipped(Decoupled(new SysMemCmd))
    val sysMemResp = Decoupled(new SysMemResp)
    val addressState = Input(new AddressTranslationState)
    val llCommit = Flipped(Valid(new RobPtr))
    val llbitClear = Input(Bool())
    val llbitValue = Output(Bool())
    val commitStore = Flipped(Valid(Bool()))
    val robHeadLoad = Flipped(Valid(new RobHeadLoadInfo))
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
    val tlbFillDebug = Output(Valid(new TlbFillDebugEvent))
    val loadQueueFull = Output(Bool())
    val uncacheFull = Output(Bool())

    val icacheFlush = Input(Bool())
    val icacheRedirect = Input(Bool())
    val backendFlush = Input(Bool())
    val loadRecovery = Input(Bool())
    val axi = new AXIIO
    val perf = if (enablePerfCounters) {
        Some(Output(new MemoryPerfEvents))
    } else {
        None
    }
}

class MemorySubSystem(
    useBlackBoxRam: Boolean = true,
    enablePerfCounters: Boolean = false,
    memSysConfig: MemSysConfig = MemSysConfig()
) extends Module {
    val io = IO(new MemorySubSystemIO(enablePerfCounters))

    private val enableL2 = memSysConfig.enableL2

    val icache = Module(new ICache(useBlackBoxRam = useBlackBoxRam))
    val tlb = Module(new TLB(nPorts = 4))
    val loadStorePipeline = Module(new LoadStorePipeline(
        useBlackBoxRam = useBlackBoxRam,
        enablePerfCounters = enablePerfCounters
    ))
    val midLayerArbiter = Module(new MidLayerArbiter)
    val externalAXIArbiter = Module(new ExternalAXIArbiter)

    val reservationValid = RegInit(false.B)
    val reservationPaddr = Reg(UInt(30.W))
    val clearReservationForSc = WireDefault(false.B)
    io.llbitValue := reservationValid

    loadStorePipeline.io.llCommit := io.llCommit
    when(io.llbitClear || clearReservationForSc) {
        reservationValid := false.B
    }.elsewhen(loadStorePipeline.io.llCommitPaddr.valid) {
        reservationValid := true.B
        reservationPaddr := loadStorePipeline.io.llCommitPaddr.bits(31, 2)
    }

    when(io.llCommit.valid && !reset.asBool) {
        assert(loadStorePipeline.io.llCommitPaddr.valid,
            "MemorySubSystem: a retiring LL must still have a completed LST entry")
    }

    val sysCmdQueue = withReset(reset.asBool || io.backendFlush) {
        Module(new Queue(
            new SysMemCmd,
            entries = 1,
            pipe = false,
            flow = false
        ))
    }
    val sysRespQueue = withReset(reset.asBool || io.backendFlush) {
        Module(new Queue(
            new SysMemResp,
            entries = 1,
            pipe = false,
            flow = false
        ))
    }

    // A full backend recovery invalidates both sides of the registered slow
    // path.  Gate the recovery cycle as well as resetting the queue valids so
    // a stale command cannot dequeue into the TLB on the flush edge.
    sysCmdQueue.io.enq.valid := io.sysMemCmd.valid && !io.backendFlush
    sysCmdQueue.io.enq.bits := io.sysMemCmd.bits
    io.sysMemCmd.ready := sysCmdQueue.io.enq.ready && !io.backendFlush

    io.sysMemResp.valid := sysRespQueue.io.deq.valid && !io.backendFlush
    io.sysMemResp.bits := sysRespQueue.io.deq.bits
    sysRespQueue.io.deq.ready := io.sysMemResp.ready && !io.backendFlush

    val queuedSysCmd = sysCmdQueue.io.deq
    val isTlbCommand =
        queuedSysCmd.bits.op === SystemOp.TLBSRCH ||
        queuedSysCmd.bits.op === SystemOp.TLBRD ||
        queuedSysCmd.bits.op === SystemOp.TLBWR ||
        queuedSysCmd.bits.op === SystemOp.TLBFILL ||
        queuedSysCmd.bits.op === SystemOp.INVTLB

    val queuedSysCmdLive = queuedSysCmd.valid && !io.backendFlush

    tlb.io.sysCmd.valid := queuedSysCmdLive && isTlbCommand
    tlb.io.sysCmd.bits := queuedSysCmd.bits

    private val tlbTokenWidth = 16
    require((new RobPtr).getWidth <= tlbTokenWidth)

    object LocalSysState extends ChiselEnum {
        val Idle, Drain, DrainConfirm,
            ICacheInvalidateStart, ICacheInvalidateWait,
            ScTlbReq, ScTlbWait, ScStoreIssue, ScStoreWait,
            CacopTlbReq, CacopTlbWait,
            DCacheMaintenanceIssue, DCacheMaintenanceWait,
            L2MaintenanceIssue, L2MaintenanceWait = Value
    }

    val localSysState = RegInit(LocalSysState.Idle)
    val localSysPending = Reg(new SysMemCmd)
    val localSysKilled = RegInit(false.B)
    val barrierHold = RegInit(false.B)
    val localSysRespValid = RegInit(false.B)
    val localSysRespBits = Reg(new SysMemResp)
    val scReservationWasValid = RegInit(false.B)
    val scReservationPaddr = Reg(UInt(30.W))
    val scStorePaddr = Reg(UInt(32.W))
    val scStoreUncache = RegInit(false.B)
    val cacopPaddr = Reg(UInt(32.W))
    val dcacheInvalidateIndex = RegInit(0.U(DcacheConfig.DcacheIndex.W))
    val dcacheInvalidateWay = RegInit(0.U(log2Ceil(DcacheConfig.DcacheWay).W))

    val localSysCmdReady =
        localSysState === LocalSysState.Idle &&
        !localSysRespValid &&
        !barrierHold
    queuedSysCmd.ready := !io.backendFlush && Mux(
        isTlbCommand,
        tlb.io.sysCmd.ready,
        localSysCmdReady
    )

    val localSysCmdFire =
        queuedSysCmdLive && !isTlbCommand && localSysCmdReady
    val dataBarrierDrained =
        loadStorePipeline.io.drainDone && externalAXIArbiter.io.uncacheIdle
    val startICacheInvalidate =
        localSysState === LocalSysState.ICacheInvalidateStart
    val localSysIrrevocable =
        localSysState === LocalSysState.ScStoreWait ||
        localSysState === LocalSysState.DCacheMaintenanceWait ||
        localSysState === LocalSysState.L2MaintenanceWait

    val localComplete = WireDefault(false.B)
    val localCompleteData = WireDefault(0.U(32.W))
    val localCompleteException = WireDefault(false.B)
    val localCompleteCause = WireDefault(0.U(8.W))
    val localCompleteBadvValid = WireDefault(false.B)
    val localCompleteBadv = WireDefault(0.U(32.W))
    val scStoreTrace = WireDefault(0.U.asTypeOf(Valid(new StoreCommitTrace)))

    val serialTlbReq = tlb.io.req(3)
    val serialTlbResp = tlb.io.resp(3)
    val serialAddressTranslator = Module(new AddressModeTranslator)
    serialAddressTranslator.io.vaddr := localSysPending.vaddr
    serialAddressTranslator.io.state := io.addressState
    val cacopTargetsICache = localSysPending.auxOp(2, 0) === 0.U
    val cacopTargetsDCache = localSysPending.auxOp(2, 0) === 1.U
    val cacopTargetsL2 = localSysPending.auxOp(2, 0) === 2.U
    val cacopOperation = localSysPending.auxOp(4, 3)
    val cacopHitOperation = cacopOperation === 2.U
    val invalidateAllL1Caches =
        localSysPending.op === SystemOp.IBAR ||
        localSysPending.op === SystemOp.DBAR
    val cacopHitNeedsTranslation = cacopHitOperation &&
        (cacopTargetsICache || cacopTargetsDCache ||
            (if (enableL2) cacopTargetsL2 else false.B))
    serialTlbReq.valid :=
        (localSysState === LocalSysState.ScTlbReq ||
            localSysState === LocalSysState.CacopTlbReq) &&
            !io.backendFlush
    serialTlbReq.bits.vaddr := localSysPending.vaddr
    serialTlbReq.bits.stall := false.B
    serialTlbReq.bits.access := Mux(
        localSysState === LocalSysState.ScTlbReq,
        TLBAccess.Store,
        TLBAccess.Load
    )
    serialTlbReq.bits.token := localSysPending.robPtr.asUInt.pad(tlbTokenWidth)
    serialTlbResp.ready :=
        localSysState === LocalSysState.ScTlbWait ||
            localSysState === LocalSysState.CacopTlbWait
    tlb.io.cancel(3) := io.backendFlush

    loadStorePipeline.io.atomicStoreReq.valid :=
        localSysState === LocalSysState.ScStoreIssue && !io.backendFlush
    loadStorePipeline.io.atomicStoreReq.bits.paddr := scStorePaddr
    loadStorePipeline.io.atomicStoreReq.bits.data := localSysPending.data
    loadStorePipeline.io.atomicStoreReq.bits.uncache := scStoreUncache

    loadStorePipeline.io.dcacheMaintenanceReq.valid :=
        localSysState === LocalSysState.DCacheMaintenanceIssue && !io.backendFlush
    loadStorePipeline.io.dcacheMaintenanceReq.bits.paddr := Mux(
        invalidateAllL1Caches,
        dcacheInvalidateIndex ## 0.U(DcacheConfig.DcacheOffset.W),
        Mux(cacopHitOperation, cacopPaddr, localSysPending.vaddr)
    )
    loadStorePipeline.io.dcacheMaintenanceReq.bits.indexOnly :=
        invalidateAllL1Caches || !cacopHitOperation
    loadStorePipeline.io.dcacheMaintenanceReq.bits.way := Mux(
        invalidateAllL1Caches,
        dcacheInvalidateWay,
        localSysPending.vaddr(log2Ceil(DcacheConfig.DcacheWay) - 1, 0)
    )
    loadStorePipeline.io.dcacheMaintenanceReq.bits.writeback :=
        invalidateAllL1Caches || cacopOperation =/= 0.U
    loadStorePipeline.io.dcacheMaintenanceReq.bits.invalidate := true.B
    loadStorePipeline.io.dcacheMaintenanceResp.ready :=
        localSysState === LocalSysState.DCacheMaintenanceWait

    val l2MaintenanceReqValid =
        localSysState === LocalSysState.L2MaintenanceIssue && !io.backendFlush
    val l2MaintenanceReqBits = WireDefault(0.U.asTypeOf(new L2MaintenanceRequest))
    l2MaintenanceReqBits.operation := cacopOperation
    l2MaintenanceReqBits.vaddr := localSysPending.vaddr
    l2MaintenanceReqBits.paddr := cacopPaddr
    val l2MaintenanceReqReady = WireDefault(false.B)
    val l2MaintenanceRespValid = WireDefault(false.B)
    val l2MaintenanceRespOperation = WireDefault(0.U(2.W))
    val l2MaintenanceRespReady =
        localSysState === LocalSysState.L2MaintenanceWait

    externalAXIArbiter.io.maintenanceWrite.valid := false.B
    externalAXIArbiter.io.maintenanceWrite.bits := 0.U.asTypeOf(new WritebackRequest)

    when(io.backendFlush && !localSysIrrevocable) {
        localSysState := LocalSysState.Idle
        localSysKilled := false.B
        localSysRespValid := false.B
        barrierHold := false.B
    }.otherwise {
        when(io.backendFlush && localSysIrrevocable) {
            localSysKilled := true.B
            localSysRespValid := false.B
        }

        when(localSysCmdFire) {
            localSysPending := queuedSysCmd.bits
            localSysKilled := false.B
            when(
                queuedSysCmd.bits.op === SystemOp.SC ||
                queuedSysCmd.bits.op === SystemOp.DBAR ||
                queuedSysCmd.bits.op === SystemOp.IBAR ||
                queuedSysCmd.bits.op === SystemOp.CACOP
            ) {
                barrierHold := true.B
                localSysState := LocalSysState.Drain
                when(queuedSysCmd.bits.op === SystemOp.SC) {
                    scReservationWasValid := reservationValid
                    scReservationPaddr := reservationPaddr
                }
            }.otherwise {
                localSysRespValid := true.B
                localSysRespBits := 0.U.asTypeOf(localSysRespBits)
                localSysRespBits.robPtr := queuedSysCmd.bits.robPtr
                localSysRespBits.epoch := queuedSysCmd.bits.epoch
                localSysRespBits.exceptionValid := true.B
                localSysRespBits.exceptionCause := ExpCode.INE
            }
        }

        when(localSysState === LocalSysState.Drain && dataBarrierDrained) {
            localSysState := LocalSysState.DrainConfirm
        }

        when(localSysState === LocalSysState.DrainConfirm) {
            when(!dataBarrierDrained) {
                localSysState := LocalSysState.Drain
            }.otherwise {
                when(localSysPending.op === SystemOp.SC) {
                    when(!scReservationWasValid) {
                        localComplete := true.B
                    }.elsewhen(localSysPending.vaddr(1, 0).orR) {
                        localComplete := true.B
                        localCompleteException := true.B
                        localCompleteCause := ExpCode.ALE
                        localCompleteBadvValid := true.B
                        localCompleteBadv := localSysPending.vaddr
                    }.otherwise {
                        localSysState := LocalSysState.ScTlbReq
                    }
                }.elsewhen(
                    localSysPending.op === SystemOp.IBAR ||
                    localSysPending.op === SystemOp.DBAR
                ) {
                    localSysState := LocalSysState.ICacheInvalidateStart
                }.elsewhen(localSysPending.op === SystemOp.CACOP) {
                    when(cacopHitNeedsTranslation) {
                        localSysState := LocalSysState.CacopTlbReq
                    }.elsewhen(cacopTargetsICache) {
                        localSysState := LocalSysState.ICacheInvalidateStart
                    }.elsewhen(cacopTargetsDCache) {
                        localSysState := LocalSysState.DCacheMaintenanceIssue
                    }.elsewhen(cacopTargetsL2) {
                        if (enableL2) {
                            when(cacopOperation === 3.U) {
                                localComplete := true.B
                            }.otherwise {
                                localSysState := LocalSysState.L2MaintenanceIssue
                            }
                        } else {
                            localComplete := true.B
                        }
                    }.otherwise {
                        localComplete := true.B
                    }
                }.otherwise {
                    localComplete := true.B
                }
            }
        }

        when(localSysState === LocalSysState.ICacheInvalidateStart) {
            localSysState := LocalSysState.ICacheInvalidateWait
        }

        when(
            localSysState === LocalSysState.ICacheInvalidateWait &&
            icache.io.invalidateDone
        ) {
            when(invalidateAllL1Caches) {
                dcacheInvalidateIndex := 0.U
                dcacheInvalidateWay := 0.U
                localSysState := LocalSysState.DCacheMaintenanceIssue
            }.otherwise {
                localComplete := true.B
            }
        }

        when(localSysState === LocalSysState.ScTlbReq && serialTlbReq.fire) {
            localSysState := LocalSysState.ScTlbWait
        }

        when(localSysState === LocalSysState.ScTlbWait && serialTlbResp.fire) {
            assert(serialTlbResp.bits.token ===
                localSysPending.robPtr.asUInt.pad(tlbTokenWidth),
                "MemorySubSystem: SC translation response token mismatch")
            when(serialTlbResp.bits.exception.orR) {
                localComplete := true.B
                localCompleteException := true.B
                localCompleteCause := serialTlbResp.bits.exception
                localCompleteBadvValid := true.B
                localCompleteBadv := localSysPending.vaddr
            }.otherwise {
                // An SC that raises an address or translation exception has no
                // architectural LLBit side effect.  Once translation succeeds,
                // however, both the success and reservation-mismatch outcomes
                // consume the reservation.
                clearReservationForSc := true.B
                when(serialTlbResp.bits.paddr(31, 2) =/= scReservationPaddr) {
                    localComplete := true.B
                }.otherwise {
                    scStorePaddr := serialTlbResp.bits.paddr
                    scStoreUncache := serialTlbResp.bits.uncache
                    localSysState := LocalSysState.ScStoreIssue
                }
            }
        }

        when(localSysState === LocalSysState.CacopTlbReq && serialTlbReq.fire) {
            localSysState := LocalSysState.CacopTlbWait
        }

        when(localSysState === LocalSysState.CacopTlbWait && serialTlbResp.fire) {
            assert(serialTlbResp.bits.token ===
                localSysPending.robPtr.asUInt.pad(tlbTokenWidth),
                "MemorySubSystem: CACOP translation response token mismatch")
            when(serialTlbResp.bits.exception.orR) {
                localComplete := true.B
                localCompleteException := true.B
                localCompleteCause := serialTlbResp.bits.exception
                localCompleteBadvValid := true.B
                localCompleteBadv := localSysPending.vaddr
            }.otherwise {
                cacopPaddr := serialTlbResp.bits.paddr
                when(cacopTargetsICache) {
                    localSysState := LocalSysState.ICacheInvalidateStart
                }.elsewhen(cacopTargetsDCache) {
                    localSysState := LocalSysState.DCacheMaintenanceIssue
                }.otherwise {
                    if (enableL2) {
                        localSysState := LocalSysState.L2MaintenanceIssue
                    } else {
                        localComplete := true.B
                    }
                }
            }
        }

        when(
            localSysState === LocalSysState.DCacheMaintenanceIssue &&
            loadStorePipeline.io.dcacheMaintenanceReq.fire
        ) {
            localSysState := LocalSysState.DCacheMaintenanceWait
        }

        when(
            localSysState === LocalSysState.DCacheMaintenanceWait &&
            loadStorePipeline.io.dcacheMaintenanceResp.fire
        ) {
            when(invalidateAllL1Caches) {
                when(
                    dcacheInvalidateIndex === (DcacheConfig.DcacheSet - 1).U &&
                    dcacheInvalidateWay === (DcacheConfig.DcacheWay - 1).U
                ) {
                    localComplete := true.B
                }.otherwise {
                    when(dcacheInvalidateWay === (DcacheConfig.DcacheWay - 1).U) {
                        dcacheInvalidateWay := 0.U
                        dcacheInvalidateIndex := dcacheInvalidateIndex + 1.U
                    }.otherwise {
                        dcacheInvalidateWay := dcacheInvalidateWay + 1.U
                    }
                    localSysState := LocalSysState.DCacheMaintenanceIssue
                }
            }.otherwise {
                localComplete := true.B
            }
        }

        when(
            localSysState === LocalSysState.L2MaintenanceIssue &&
            l2MaintenanceReqValid && l2MaintenanceReqReady
        ) {
            localSysState := LocalSysState.L2MaintenanceWait
        }

        when(
            localSysState === LocalSysState.L2MaintenanceWait &&
            l2MaintenanceRespValid && l2MaintenanceRespReady
        ) {
            assert(l2MaintenanceRespOperation === cacopOperation,
                "MemorySubSystem: L2 maintenance response operation mismatch")
            localComplete := true.B
        }

        when(
            localSysState === LocalSysState.ScStoreIssue &&
            loadStorePipeline.io.atomicStoreReq.fire
        ) {
            localSysState := LocalSysState.ScStoreWait
        }

        when(
            localSysState === LocalSysState.ScStoreWait &&
            loadStorePipeline.io.atomicStoreDone
        ) {
            localComplete := true.B
            localCompleteData := 1.U
        }

        when(localComplete) {
            when(
                localSysPending.op === SystemOp.SC &&
                localCompleteData === 1.U &&
                !localCompleteException &&
                !localSysKilled &&
                !io.backendFlush
            ) {
                scStoreTrace.valid := true.B
                scStoreTrace.bits.robPtr := localSysPending.robPtr
                scStoreTrace.bits.vaddr := localSysPending.vaddr
                scStoreTrace.bits.paddr := scStorePaddr
                scStoreTrace.bits.data := localSysPending.data
                scStoreTrace.bits.mask := "b1111".U
                scStoreTrace.bits.uncache := scStoreUncache
            }
            when(!localSysKilled && !io.backendFlush) {
                localSysRespValid := true.B
                localSysRespBits := 0.U.asTypeOf(localSysRespBits)
                localSysRespBits.robPtr := localSysPending.robPtr
                localSysRespBits.epoch := localSysPending.epoch
                localSysRespBits.data := localCompleteData
                localSysRespBits.exceptionValid := localCompleteException
                localSysRespBits.exceptionCause := localCompleteCause
                localSysRespBits.badvValid := localCompleteBadvValid
                localSysRespBits.badv := localCompleteBadv
            }.otherwise {
                barrierHold := false.B
            }
            localSysState := LocalSysState.Idle
            localSysKilled := false.B
        }
    }

    val selectLocalSysResp = localSysRespValid
    sysRespQueue.io.enq.valid :=
        (localSysRespValid || tlb.io.sysResp.valid) && !io.backendFlush
    sysRespQueue.io.enq.bits := Mux(
        selectLocalSysResp,
        localSysRespBits,
        tlb.io.sysResp.bits
    )
    tlb.io.sysResp.ready :=
        sysRespQueue.io.enq.ready && !selectLocalSysResp && !io.backendFlush
    io.tlbFillDebug.valid := tlb.io.sysResp.fire && tlb.io.sysResp.bits.tlbFill.valid
    io.tlbFillDebug.bits.robPtr := tlb.io.sysResp.bits.robPtr
    io.tlbFillDebug.bits.index := tlb.io.sysResp.bits.tlbFill.bits

    when(sysRespQueue.io.enq.fire && selectLocalSysResp) {
        localSysRespValid := false.B
    }

    assert(!(localSysRespValid && tlb.io.sysResp.valid))

    icache.io.pp <> io.icache
    // Once a serial memory operation reaches the ROB-head slow path, discard
    // younger fetches and keep the I-side stopped until the completing backend
    // recovery.  This prevents a younger branch from refilling stale code while
    // an older store/CACOP sequence is still being drained.
    val serialFetchHold = barrierHold
    icache.io.flush := io.icacheFlush || serialFetchHold
    icache.io.redirect := io.icacheRedirect && !serialFetchHold
    icache.io.invalidate := startICacheInvalidate
    val effectiveICacheFlush =
        io.icacheFlush || startICacheInvalidate || serialFetchHold
    tlb.io.flush := io.backendFlush
    tlb.io.state := io.addressState

    // Port 0 is the instruction-side client; ports 1 and 2 are the two LSU
    // address lanes below. Port 3 is reserved for the ROB-head SC slow path.
    tlb.io.cancel(0) := effectiveICacheFlush
    tlb.io.req(0) <> icache.io.tlb.req
    icache.io.tlb.resp <> tlb.io.resp(0)

    when(io.icacheRedirect) {
        assert(io.icacheFlush)
    }

    val dTranslationWaiting = RegInit(VecInit.fill(LoadQueueConfig.EnqNum)(false.B))
    val dTranslationInst = Reg(Vec(LoadQueueConfig.EnqNum, new BackendInst))

    // Confirmed cached Loads bypass the D-TLB protocol completely. Their
    // translation/classification fields were registered in RegisterRead.
    for (port <- 0 until LoadQueueConfig.EnqNum) {
        loadStorePipeline.io.directCachedLoad(port).valid :=
            io.directCachedLoad(port).valid
        loadStorePipeline.io.directCachedLoad(port).bits :=
            io.directCachedLoad(port).bits
        io.directCachedLoad(port).ready :=
            loadStorePipeline.io.directCachedLoad(port).ready
    }

    for (port <- 0 until LoadQueueConfig.EnqNum) {
        val client = port + 1
        val source = io.backendInst(port)
        val sink = loadStorePipeline.io.backendInst(port)
        val tlbReq = tlb.io.req(client)
        val tlbResp = tlb.io.resp(client)

        val sourceToken = source.bits.robPtr.asUInt.pad(tlbTokenWidth)
        val sourceIsPageAccess =
            source.valid && source.bits.translationPending
        val cancelWaiting =
            dTranslationWaiting(port) && !source.valid
        val recoveryKillsSourceLoad =
            io.loadPtrCtrl.redirect && source.valid && source.bits.uop.isLD
        val recoveryKillsWaitingLoad =
            io.loadPtrCtrl.redirect && dTranslationWaiting(port) &&
                dTranslationInst(port).uop.isLD

        tlb.io.cancel(client) := io.backendFlush || cancelWaiting ||
            recoveryKillsWaitingLoad
        tlbReq.valid :=
            sourceIsPageAccess &&
            !dTranslationWaiting(port) &&
            !io.backendFlush &&
            !recoveryKillsSourceLoad
        tlbReq.bits.vaddr := source.bits.pc
        tlbReq.bits.stall := false.B
        tlbReq.bits.access := Mux(
            source.bits.uop.isSTA,
            TLBAccess.Store,
            TLBAccess.Load
        )
        tlbReq.bits.token := sourceToken

        val responseTokenMatches =
            tlbResp.bits.token ===
                dTranslationInst(port).robPtr.asUInt.pad(tlbTokenWidth)
        // Each D-side TLB client permits exactly one outstanding request, and
        // TLB port epochs suppress responses from a cancelled request. Keep the
        // token comparison as a protocol assertion instead of placing the ROB
        // pointer comparator on the normal response-valid path.
        val liveTranslationResponse =
            dTranslationWaiting(port) &&
            source.valid &&
            tlbResp.valid &&
            !io.backendFlush &&
            !recoveryKillsWaitingLoad
        val staleTranslationResponse =
            tlbResp.valid && !liveTranslationResponse

        // The Decoupled source is held while translation is pending. Capture
        // its complete context at request time so the response path does not
        // recompare live vaddr/ROB/ASID/store fields before reaching LST/ROB.
        val translated = WireDefault(dTranslationInst(port))
        translated.paddr := tlbResp.bits.paddr
        translated.translationPending := false.B
        translated.uncache := tlbResp.bits.uncache
        translated.exception := tlbResp.bits.exception
        translated.exceptionBadvValid := tlbResp.bits.exception.orR
        translated.exceptionBadv := Mux(
            tlbResp.bits.exception.orR,
            dTranslationInst(port).pc,
            0.U
        )

        val bypassValid =
            source.valid &&
            !source.bits.translationPending &&
            !dTranslationWaiting(port) &&
            !io.backendFlush &&
            !recoveryKillsSourceLoad
        sink.valid := bypassValid || liveTranslationResponse
        sink.bits := Mux(liveTranslationResponse, translated, source.bits)

        source.ready := Mux(
            sourceIsPageAccess || dTranslationWaiting(port),
            liveTranslationResponse && sink.ready,
            sink.ready && !io.backendFlush && !recoveryKillsSourceLoad
        )
        tlbResp.ready := staleTranslationResponse ||
            (liveTranslationResponse && sink.ready)

        when(io.backendFlush || cancelWaiting || recoveryKillsWaitingLoad) {
            dTranslationWaiting(port) := false.B
        }.elsewhen(tlbReq.fire) {
            dTranslationWaiting(port) := true.B
            dTranslationInst(port) := source.bits
        }.elsewhen(tlbResp.fire) {
            dTranslationWaiting(port) := false.B
        }

        when(!reset.asBool) {
            when(tlbReq.fire) {
                assert(source.valid && source.bits.translationPending)
                assert(source.bits.uop.isLD || source.bits.uop.isSTA)
                assert(!source.fire)
            }
            when(liveTranslationResponse && sink.fire) {
                assert(source.fire)
                assert(tlbResp.fire)
                assert(!sink.bits.translationPending)
            }
            when(tlbResp.valid && dTranslationWaiting(port) && source.valid) {
                assert(responseTokenMatches,
                    "D-side TLB response must match the complete ROB pointer token")
            }
            when(dTranslationWaiting(port) && source.valid) {
                assert(source.bits.asUInt === dTranslationInst(port).asUInt,
                    "D-side translation source must remain stable while waiting")
            }
        }
    }

    loadStorePipeline.io.commitStore := io.commitStore
    loadStorePipeline.io.robHeadLoad := io.robHeadLoad
    loadStorePipeline.io.sqHeadOH := io.sqHeadOH
    loadStorePipeline.io.sqHeadHigh := io.sqHeadHigh
    loadStorePipeline.io.sqFlushMask := io.sqFlushMask
    io.sqLiveMask := loadStorePipeline.io.sqLiveMask
    io.sqFreedMask := loadStorePipeline.io.sqFreedMask
    io.sqCommitPtrOH := loadStorePipeline.io.sqCommitPtrOH
    io.sqCommitPtrHigh := loadStorePipeline.io.sqCommitPtrHigh
    io.sqCommittedMask := loadStorePipeline.io.sqCommittedMask
    loadStorePipeline.io.loadPtrCtrl := io.loadPtrCtrl
    loadStorePipeline.io.lsqLive := io.lsqLive
    loadStorePipeline.io.flush := io.backendFlush
    loadStorePipeline.io.loadRecovery := io.loadRecovery
    loadStorePipeline.io.quiesce := barrierHold
    io.loadResult <> loadStorePipeline.io.loadResult
    io.loadPredWake := loadStorePipeline.io.loadPredWake
    io.loadPredResolve := loadStorePipeline.io.loadPredResolve
    io.storeComplete := loadStorePipeline.io.storeComplete
    io.storeException := loadStorePipeline.io.storeException
    io.storeCommitTrace := Mux(
        scStoreTrace.valid,
        scStoreTrace,
        loadStorePipeline.io.storeCommitTrace
    )
    io.loadQueueFull := loadStorePipeline.io.loadQueueFull
    io.uncacheFull := loadStorePipeline.io.uncacheFull

    if (enablePerfCounters) {
        io.perf.get := loadStorePipeline.io.perf.get
        io.perf.get.icacheHit  := icache.io.perf.hit
        io.perf.get.icacheMiss := icache.io.perf.miss
    }

    midLayerArbiter.io.dcache <> loadStorePipeline.io.memory
    midLayerArbiter.io.flush := effectiveICacheFlush
    externalAXIArbiter.io.uncache <> loadStorePipeline.io.uncache
    externalAXIArbiter.io.flush := effectiveICacheFlush

    val icacheMissCacheable = icache.io.missReq.req.bits.cacheable
    midLayerArbiter.io.icache.req.valid :=
        icache.io.missReq.req.valid && icacheMissCacheable
    midLayerArbiter.io.icache.req.bits := icache.io.missReq.req.bits
    externalAXIArbiter.io.icache.req.valid :=
        icache.io.missReq.req.valid && !icacheMissCacheable
    externalAXIArbiter.io.icache.req.bits := icache.io.missReq.req.bits
    icache.io.missReq.req.ready := Mux(
        icacheMissCacheable,
        midLayerArbiter.io.icache.req.ready,
        externalAXIArbiter.io.icache.req.ready
    )

    val midLayerICacheResp = midLayerArbiter.io.icache.resp
    val externalICacheResp = externalAXIArbiter.io.icache.resp
    icache.io.missReq.resp.valid := midLayerICacheResp.valid || externalICacheResp.valid
    icache.io.missReq.resp.bits := Mux(
        midLayerICacheResp.valid,
        midLayerICacheResp.bits,
        externalICacheResp.bits
    )
    midLayerICacheResp.ready := icache.io.missReq.resp.ready
    externalICacheResp.ready := icache.io.missReq.resp.ready

    io.axi <> externalAXIArbiter.io.axi
    if (enableL2) {
        val l2Cache = Module(new UnifiedL2Cache(
            memSysConfig = memSysConfig,
            useBlackBoxRam = useBlackBoxRam
        ))
        l2Cache.io.upstream <> midLayerArbiter.io.l2
        externalAXIArbiter.io.l2 <> l2Cache.io.downstream
        l2Cache.io.uncacheWriteSnoopReq <>
            externalAXIArbiter.io.uncacheWriteSnoopReq
        externalAXIArbiter.io.uncacheWriteSnoopResp <>
            l2Cache.io.uncacheWriteSnoopResp
        l2Cache.io.maintenanceReq.valid := l2MaintenanceReqValid
        l2Cache.io.maintenanceReq.bits := l2MaintenanceReqBits
        l2MaintenanceReqReady := l2Cache.io.maintenanceReq.ready
        l2Cache.io.maintenanceResp.ready := l2MaintenanceRespReady
        l2MaintenanceRespValid := l2Cache.io.maintenanceResp.valid
        l2MaintenanceRespOperation := l2Cache.io.maintenanceResp.bits.operation

        if (enablePerfCounters) {
            io.perf.get.l2IReadHit := l2Cache.io.perf.iReadHit
            io.perf.get.l2IReadMiss := l2Cache.io.perf.iReadMiss
            io.perf.get.l2DReadHit := l2Cache.io.perf.dReadHit
            io.perf.get.l2DReadMiss := l2Cache.io.perf.dReadMiss
            io.perf.get.l2WriteHit := l2Cache.io.perf.writeHit
            io.perf.get.l2WriteMiss := l2Cache.io.perf.writeMiss
            io.perf.get.l2DirtyWriteback := l2Cache.io.perf.dirtyWriteback
            io.perf.get.l2UncacheRead := externalAXIArbiter.io.uncacheReadStart
            io.perf.get.l2UncacheWrite := externalAXIArbiter.io.uncacheWriteStart
            io.perf.get.l2Busy := l2Cache.io.perf.busy
        }
    } else {
        val lineBypass = Module(new LineMemoryBypass)
        lineBypass.io.upstream <> midLayerArbiter.io.l2
        externalAXIArbiter.io.l2 <> lineBypass.io.downstream

        // Preserve the registered request/response protocol when L2 is
        // disabled.  The one-entry queue prevents a same-cycle response from
        // disappearing while ExternalAXIArbiter advances into its wait state.
        val snoopBypass = Module(new Queue(
            new L2UncacheWriteSnoopResponse,
            entries = 1,
            pipe = false,
            flow = false
        ))
        snoopBypass.io.enq.valid :=
            externalAXIArbiter.io.uncacheWriteSnoopReq.valid
        snoopBypass.io.enq.bits.paddr :=
            externalAXIArbiter.io.uncacheWriteSnoopReq.bits.paddr
        externalAXIArbiter.io.uncacheWriteSnoopReq.ready :=
            snoopBypass.io.enq.ready
        externalAXIArbiter.io.uncacheWriteSnoopResp <> snoopBypass.io.deq
    }

    when(!reset.asBool) {
        assert(!(midLayerICacheResp.valid && externalICacheResp.valid),
            "MemorySubSystem: cached and uncached ICache responses must be exclusive")
    }
}

object GenerateMemorySubSystem extends App {
    ChiselStage.emitSystemVerilogFile(
        new MemorySubSystem(useBlackBoxRam = false),
        args = Array("--target-dir", "generated/memory-sub-system"),
        firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info")
    )
}
