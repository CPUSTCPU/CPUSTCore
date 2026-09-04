package CPUSTC.memory.backend

import CPUSTC.memory._
import CPUSTC.memory.MemoryPointerUtils._
import chisel3._
import chisel3.util._
import CPUSTC.backend.rob.RobPtr

class DCacheInst extends Bundle {
    val addr = UInt(32.W)
    val wdata = UInt(32.W)
    val wen = UInt(4.W)
    val uncache = Bool()
    val valid = Bool()
    val sqindex = UInt(StoreQueueConfig.length.W)
    val sqindexHigh = Bool()
}

class SQdispatchCtrlBus extends Bundle {
    val headPtrNext = Input(UInt(StoreQueueConfig.length.W))
    val headPtrNextHigh = Input(Bool())
    val flushMask = Input(UInt(StoreQueueConfig.length.W))
}

class StoreWaitState extends Bundle {
    val dataMissingMask = UInt(StoreQueueConfig.length.W)
    val memoryPendingMask = UInt(StoreQueueConfig.length.W)
    val highMask = UInt(StoreQueueConfig.length.W)
}

class StoreQueueIO extends Bundle {
    val enqueue = Vec(StoreQueueConfig.EnqNum, Flipped(Decoupled(new BackendInst)))
    val flush = Input(Bool())
    val dequeue = Decoupled(new DCacheInst)
    val commit = Flipped(Valid(Bool()))
    val complete = Vec(2, Flipped(Valid(new StoreReadyEvent)))
    val retry = Flipped(Valid(new DcacheStoreRetryBus))
    val dispatch = new SQdispatchCtrlBus
    val lsqLive = Input(new MemoryLsqLiveState)
    val forward = Output(Vec(StoreQueueConfig.length, new StoreForwardSource))
    val waitState = Output(new StoreWaitState)
    val storeReady = Output(Vec(4, Valid(new StoreReadyEvent)))
    val freedMask = Output(Valid(UInt(StoreQueueConfig.length.W)))
    val liveMask = Output(UInt(StoreQueueConfig.length.W))
    val committedMask = Output(UInt(StoreQueueConfig.length.W))
    val commitPtrOH = Output(UInt(StoreQueueConfig.length.W))
    val commitPtrHigh = Output(Bool())
    val dequeuePtrOH = Output(UInt(StoreQueueConfig.length.W))
    val dequeuePtrHigh = Output(Bool())
    val pendingUncacheStore = Output(Bool())
    val normalComplete = Output(Vec(
        StoreQueueConfig.EnqNum,
        Valid(new StoreCompletionToken)
    ))
    val exceptionComplete = Output(Vec(
        LoadQueueConfig.EnqNum,
        Valid(new StoreExceptionEvent)
    ))
    val commitTrace = Output(Valid(new StoreCommitTrace))
}

class STAarrayItem extends Bundle {
    val vaddr = UInt(32.W)
    val addr = UInt(32.W)
    val mask = UInt(4.W)
    val alignedMask = UInt((DcacheConfig.DcacheMaskBits * 2).W)
    val uncache = Bool()
    val valid = Bool()
    val exception = UInt(8.W)
}

class STDarrayItem extends Bundle {
    val data = UInt(32.W)
    val valid = Bool()
}

class StoreIdentity extends Bundle {
    val indexHigh = Bool()
    val robPtr = new RobPtr
}

class StoreQueue extends Module {
    val io = IO(new StoreQueueIO)

    private val length = StoreQueueConfig.length

    val addrArray = RegInit(VecInit.fill(length)(0.U.asTypeOf(new STAarrayItem)))
    val dataArray = RegInit(VecInit.fill(length)(0.U.asTypeOf(new STDarrayItem)))
    val identities = RegInit(VecInit.fill(length)(0.U.asTypeOf(new StoreIdentity)))
    val committed = RegInit(VecInit.fill(length)(false.B))
    val issued = RegInit(VecInit.fill(length)(false.B))
    val completed = RegInit(VecInit.fill(length)(false.B))
    val olderStoreMask = RegInit(VecInit.fill(length)(0.U(length.W)))
    // These are program-order cursors. STA/STD may arrive out of order, so an
    // empty local array must not rebase them to the first arriving Store.
    val commitPtr = RegInit(1.U(length.W))
    val commitPtrHigh = RegInit(false.B)

    def rotateNext(oh: UInt): UInt = Cat(oh(length - 2, 0), oh(length - 1))

    val validMask = VecInit((0 until length).map(index =>
        addrArray(index).valid || dataArray(index).valid)).asUInt
    val residentHighMask = VecInit(identities.map(_.indexHigh)).asUInt
    val readyMask = VecInit((0 until length).map(index =>
        addrArray(index).valid && dataArray(index).valid)).asUInt
    val effectiveFlushMask = io.dispatch.flushMask & ~committed.asUInt

    val allocatedMask = io.lsqLive.stqValidMask & ~effectiveFlushMask
    val completionMask = io.complete.map { complete =>
        Mux(complete.valid, complete.bits.sqMask, 0.U(length.W))
    }.reduce(_ | _)
    // DCache reports only the oldest failed Store. Reconstruct its resident,
    // issued younger-or-equal suffix here so the wide SQ identity masks never
    // cross the DCache/LSP boundary. A same-cycle completion keeps ownership of
    // its entry and therefore is deliberately excluded from retry.
    val retryYoungerOrEqualMask = VecInit((0 until length).map { index =>
        !pointerOlderThanBoundary(
            UIntToOH(index.U, length),
            identities(index).indexHigh,
            io.retry.bits.sqindex,
            io.retry.bits.sqindexHigh
        )
    }).asUInt
    val retryEligibleMask = allocatedMask & issued.asUInt &
        ~completed.asUInt & ~completionMask
    val retryMask = Mux(
        io.retry.valid,
        retryEligibleMask & retryYoungerOrEqualMask,
        0.U(length.W)
    )
    val enqueuePtr = io.lsqLive.stqTailOH

    // Scan physical indices at or after the tail before wrapping to low indices.
    val tailOrAfter = VecInit((0 until length).map { index =>
        enqueuePtr(index, 0).orR
    }).asUInt
    val outstandingMask = allocatedMask & ~completed.asUInt
    val outstandingAtOrAfterTail = outstandingMask & tailOrAfter
    val oldestOutstanding = PriorityEncoderOH(Mux(
        outstandingAtOrAfterTail.orR,
        outstandingAtOrAfterTail,
        outstandingMask
    ))
    val dequeuePtr = Mux(outstandingMask.orR, oldestOutstanding, enqueuePtr)
    val dequeuePtrHigh = Mux(outstandingMask.orR,
        (dequeuePtr & io.lsqLive.stqHighMask).orR, io.lsqLive.stqTailHigh)
    val beforeDequeuePhysical = VecInit((0 until length).map { index =>
        if (index == length - 1) false.B
        else dequeuePtr(length - 1, index + 1).orR
    }).asUInt
    val beforeDequeue = Mux(
        outstandingAtOrAfterTail.orR,
        tailOrAfter & beforeDequeuePhysical,
        tailOrAfter | beforeDequeuePhysical
    )
    val freedMask = allocatedMask & completed.asUInt & beforeDequeue

    io.freedMask.valid := freedMask.orR
    io.freedMask.bits := freedMask
    io.dequeuePtrOH := dequeuePtr
    io.dequeuePtrHigh := dequeuePtrHigh

    // Replay recovery must query resident state rather than depend on a
    // one-cycle STD/completion pulse. These masks are derived only from local
    // registers, keeping flush and completion combinational paths out of LST.
    val dataValidMask = VecInit(dataArray.map(_.valid)).asUInt
    io.waitState.dataMissingMask := validMask & ~dataValidMask
    io.waitState.memoryPendingMask := validMask & ~completed.asUInt
    io.waitState.highMask := residentHighMask & validMask

    val commitAddr = Mux1H(commitPtr, addrArray)
    val commitReady = (readyMask & commitPtr).orR && !(committed.asUInt & commitPtr).orR &&
        !(effectiveFlushMask & commitPtr).orR && !io.flush
    val commitFire = io.commit.valid
    val commitIdentity = Mux1H(commitPtr, identities)
    val commitData = Mux1H(commitPtr, dataArray)
    io.commitTrace.valid := commitFire
    io.commitTrace.bits.robPtr := commitIdentity.robPtr
    io.commitTrace.bits.vaddr := commitAddr.vaddr
    io.commitTrace.bits.paddr := commitAddr.addr
    io.commitTrace.bits.data := commitData.data
    io.commitTrace.bits.mask := commitAddr.mask
    io.commitTrace.bits.uncache := commitAddr.uncache

    val issueCandidates = readyMask & committed.asUInt & ~issued.asUInt & allocatedMask
    val issueAtOrAfterTail = issueCandidates & tailOrAfter
    val issueOH = PriorityEncoderOH(Mux(
        issueAtOrAfterTail.orR,
        issueAtOrAfterTail,
        issueCandidates
    ))
    val issueAddr = Mux1H(issueOH, addrArray)
    val issueData = Mux1H(issueOH, dataArray)
    val issueReady = issueOH.orR && !io.flush
    io.dequeue.valid := issueReady
    io.dequeue.bits.addr := issueAddr.addr
    io.dequeue.bits.wdata := issueData.data
    // Access size/alignment belongs to STA. STD only carries the raw store data
    // and intentionally has a zero mask on the CPU-to-memory interface.
    io.dequeue.bits.wen := issueAddr.mask
    io.dequeue.bits.uncache := issueAddr.uncache
    io.dequeue.bits.valid := issueReady
    io.dequeue.bits.sqindex := issueOH
    io.dequeue.bits.sqindexHigh := Mux1H(issueOH, identities).indexHigh

    val uncacheCompleteMask = Mux(io.dequeue.fire && issueAddr.uncache,
        issueOH, 0.U(length.W))
    val pendingUncacheMask = VecInit((0 until length).map(index =>
        issueCandidates(index) && addrArray(index).uncache)).asUInt
    io.pendingUncacheStore := pendingUncacheMask.orR

    io.storeReady := VecInit.fill(4)(0.U.asTypeOf(Valid(new StoreReadyEvent)))
    io.normalComplete := VecInit.fill(StoreQueueConfig.EnqNum)(
        0.U.asTypeOf(Valid(new StoreCompletionToken))
    )
    io.exceptionComplete := VecInit.fill(LoadQueueConfig.EnqNum)(
        0.U.asTypeOf(Valid(new StoreExceptionEvent))
    )
    val enqueueWrites = WireInit(VecInit.fill(StoreQueueConfig.EnqNum)(false.B))
    val enqueueExceptionCompletes =
        WireInit(VecInit.fill(StoreQueueConfig.EnqNum)(false.B))
    val headGenerationMask = VecInit((0 until length).map(index =>
        io.dispatch.headPtrNext(index, 0).orR)).asUInt

    for (port <- 0 until StoreQueueConfig.EnqNum) {
        val enq = io.enqueue(port)
        val channelMatch = if (port < 2) enq.bits.uop.isSTD else enq.bits.uop.isSTA
        enq.ready := !io.flush && (!enq.valid || channelMatch)
        val expectedHigh = Mux((enq.bits.sqindex & headGenerationMask).orR,
            io.dispatch.headPtrNextHigh, !io.dispatch.headPtrNextHigh)
        val generationMatch = enq.bits.sqindexHigh === expectedHigh
        val acceptedRequest = enq.fire && enq.bits.valid && channelMatch &&
            generationMatch && !(enq.bits.sqindex & effectiveFlushMask).orR
        val architecturalStoreException =
            enq.bits.uop.isSTA && enq.bits.exception.orR

        enqueueWrites(port) := acceptedRequest && !enq.bits.Poisoned &&
            !architecturalStoreException
        enqueueExceptionCompletes(port) :=
            acceptedRequest && architecturalStoreException
    }

    when(commitFire) {
        for (index <- 0 until length) {
            when(commitPtr(index)) {
                committed(index) := true.B
            }
        }
        commitPtr := rotateNext(commitPtr)
        commitPtrHigh := commitPtrHigh ^ commitPtr(length - 1)
    }

    for (index <- 0 until length) {
        val stdMatches = VecInit((0 until 2).map(port =>
            enqueueWrites(port) && io.enqueue(port).bits.sqindex(index))).asUInt
        val staMatches = VecInit((2 until 4).map(port =>
            enqueueWrites(port) && io.enqueue(port).bits.sqindex(index))).asUInt
        val stdWrite = stdMatches.orR
        val staWrite = staMatches.orR
        val stdBits = Mux1H(stdMatches, io.enqueue.take(2).map(_.bits))
        val staBits = Mux1H(staMatches, io.enqueue.drop(2).map(_.bits))
        val newGeneration = Mux(staWrite, staBits.sqindexHigh, stdBits.sqindexHigh)
        val currentHigh = identities(index).indexHigh
        val replacingGeneration = (stdWrite || staWrite) && validMask(index) &&
            currentHigh =/= newGeneration

        when(replacingGeneration) {
            assert(false.B, "StoreQueue: a live entry cannot be overwritten by a new generation")
        }

        when(stdWrite && !replacingGeneration) {
            assert(!dataArray(index).valid,
                "StoreQueue: one generation cannot write STD twice")
            dataArray(index).data := stdBits.operateData
            dataArray(index).valid := true.B
        }
        when(staWrite && !replacingGeneration) {
            assert(!addrArray(index).valid,
                "StoreQueue: one generation cannot write STA twice")
            addrArray(index).vaddr := staBits.pc
            addrArray(index).addr := staBits.paddr
            addrArray(index).mask := staBits.mask
            addrArray(index).alignedMask :=
                (staBits.mask.pad(DcacheConfig.DcacheMaskBits * 2) << staBits.paddr(1, 0))(
                    DcacheConfig.DcacheMaskBits * 2 - 1, 0)
            addrArray(index).uncache := staBits.uncache
            addrArray(index).valid := true.B
            addrArray(index).exception := staBits.exception
        }
        when((stdWrite || staWrite) && !validMask(index) && !replacingGeneration) {
            identities(index).indexHigh := newGeneration
            identities(index).robPtr := Mux(staWrite, staBits.robPtr, stdBits.robPtr)
        }

        when(freedMask(index) || effectiveFlushMask(index)) {
            addrArray(index) := 0.U.asTypeOf(new STAarrayItem)
            dataArray(index) := 0.U.asTypeOf(new STDarrayItem)
            identities(index) := 0.U.asTypeOf(new StoreIdentity)
            committed(index) := false.B
            issued(index) := false.B
            completed(index) := false.B
            olderStoreMask(index) := 0.U
        }.elsewhen(staWrite && !replacingGeneration) {
            olderStoreMask(index) := Mux(validMask(index),
                olderStoreMask(index) & ~freedMask,
                staBits.storeDepMask & ~freedMask)
        }.elsewhen(stdWrite && !replacingGeneration) {
            olderStoreMask(index) := Mux(validMask(index),
                olderStoreMask(index) & ~freedMask,
                stdBits.storeDepMask & ~freedMask)
        }.elsewhen(freedMask.orR) {
            olderStoreMask(index) := olderStoreMask(index) & ~freedMask
        }

        when(!(freedMask(index) || effectiveFlushMask(index))) {
            when(io.dequeue.fire && issueOH(index)) {
                issued(index) := true.B
            }
            when(retryMask(index)) {
                issued(index) := false.B
            }
            when(completionMask(index) || uncacheCompleteMask(index)) {
                completed(index) := true.B
            }
        }

        when(stdWrite && staWrite && !replacingGeneration) {
            assert(stdBits.sqindexHigh === staBits.sqindexHigh,
                "StoreQueue: STA and STD generations must match")
            assert(stdBits.robPtr.asUInt === staBits.robPtr.asUInt,
                "StoreQueue: STA and STD ROB pointers must match")
            assert(stdBits.storeDepMask === staBits.storeDepMask,
                "StoreQueue: STA and STD dependency masks must match")
        }
        when(stdWrite && validMask(index) && !replacingGeneration) {
            assert(identities(index).robPtr.asUInt === stdBits.robPtr.asUInt,
                "StoreQueue: delayed STD must match the resident ROB pointer")
        }
        when(staWrite && validMask(index) && !replacingGeneration) {
            assert(identities(index).robPtr.asUInt === staBits.robPtr.asUInt,
                "StoreQueue: delayed STA must match the resident ROB pointer")
        }
        when(stdWrite && addrArray(index).valid && !replacingGeneration) {
            assert((olderStoreMask(index) & ~stdBits.storeDepMask) === 0.U,
                "StoreQueue: delayed STD must include every live STA dependency")
        }
        when(staWrite && dataArray(index).valid && !replacingGeneration) {
            assert((olderStoreMask(index) & ~staBits.storeDepMask) === 0.U,
                "StoreQueue: delayed STA must include every live STD dependency")
        }
    }

    val stdPorts = io.enqueue.take(2)
    val staPorts = io.enqueue.drop(2)
    for (port <- 0 until 2) {
        val std = stdPorts(port)
        val storedAddr = Mux1H(std.bits.sqindex, addrArray)
        val storedIdentity = Mux1H(std.bits.sqindex, identities)
        val stdCompletes = enqueueWrites(port) && storedAddr.valid &&
            storedIdentity.indexHigh === std.bits.sqindexHigh &&
            storedIdentity.robPtr.asUInt === std.bits.robPtr.asUInt
        // Data readiness is an STD-accept event.  If STA has not arrived yet,
        // neither DCache forwarding nor an LST waiter can match this pulse.
        // Keep architectural Store completion on the stricter STA/ROB match.
        io.storeReady(port).valid := enqueueWrites(port)
        io.storeReady(port).bits.paddr := storedAddr.addr
        io.storeReady(port).bits.sqindex := std.bits.sqindex
        io.storeReady(port).bits.sqindexHigh := std.bits.sqindexHigh
        io.storeReady(port).bits.sqMask := std.bits.sqindex
        io.storeReady(port).bits.sqHighMask :=
            Mux(std.bits.sqindexHigh, std.bits.sqindex, 0.U)
        io.normalComplete(port).valid := stdCompletes
        io.normalComplete(port).bits.robPtr := std.bits.robPtr

        when(enqueueWrites(port) && storedAddr.valid &&
            storedIdentity.indexHigh === std.bits.sqindexHigh) {
            assert(storedIdentity.robPtr.asUInt === std.bits.robPtr.asUInt,
                "StoreQueue: accepted STD cannot alias a resident STA")
        }

        val sta = staPorts(port)
        val sameCycleData = VecInit((0 until 2).map(stdPort =>
            enqueueWrites(stdPort) && (stdPorts(stdPort).bits.sqindex & sta.bits.sqindex).orR &&
                stdPorts(stdPort).bits.sqindexHigh === sta.bits.sqindexHigh &&
                stdPorts(stdPort).bits.robPtr.asUInt === sta.bits.robPtr.asUInt)).asUInt.orR
        val storedData = Mux1H(sta.bits.sqindex, dataArray)
        val staIdentity = Mux1H(sta.bits.sqindex, identities)
        val staCompletes = enqueueWrites(port + 2) &&
            ((storedData.valid && staIdentity.indexHigh === sta.bits.sqindexHigh &&
                staIdentity.robPtr.asUInt === sta.bits.robPtr.asUInt) || sameCycleData)
        val staExceptionCompletes = enqueueExceptionCompletes(port + 2)
        io.storeReady(port + 2).valid := staCompletes
        io.storeReady(port + 2).bits.paddr := sta.bits.paddr
        io.storeReady(port + 2).bits.sqindex := sta.bits.sqindex
        io.storeReady(port + 2).bits.sqindexHigh := sta.bits.sqindexHigh
        io.storeReady(port + 2).bits.sqMask := sta.bits.sqindex
        io.storeReady(port + 2).bits.sqHighMask :=
            Mux(sta.bits.sqindexHigh, sta.bits.sqindex, 0.U)
        io.normalComplete(port + 2).valid := staCompletes
        io.normalComplete(port + 2).bits.robPtr := sta.bits.robPtr

        io.exceptionComplete(port).valid := staExceptionCompletes
        io.exceptionComplete(port).bits.robPtr := sta.bits.robPtr
        io.exceptionComplete(port).bits.sqindex := sta.bits.sqindex
        io.exceptionComplete(port).bits.sqindexHigh := sta.bits.sqindexHigh
        io.exceptionComplete(port).bits.cause := sta.bits.exception
        io.exceptionComplete(port).bits.badvValid := sta.bits.exceptionBadvValid
        io.exceptionComplete(port).bits.badv := Mux(
            sta.bits.exceptionBadvValid,
            sta.bits.exceptionBadv,
            0.U
        )

        when(stdCompletes) {
            assert(!storedAddr.exception.orR)
            assert(!std.bits.exception.orR)
        }
        when(staCompletes) {
            assert(!sta.bits.exception.orR)
        }
        when(staExceptionCompletes) {
            assert(sta.bits.exception.orR)
            assert(!staCompletes)
        }
    }

    assert(
        PopCount(io.normalComplete.map(_.valid)) +
            PopCount(io.exceptionComplete.map(_.valid)) <=
            LoadQueueConfig.EnqNum.U
    )

    for (index <- 0 until length) {
        io.forward(index).valid := validMask(index) && !completed(index) &&
            !effectiveFlushMask(index)
        io.forward(index).addrValid := addrArray(index).valid && !effectiveFlushMask(index)
        io.forward(index).dataValid := dataArray(index).valid && !effectiveFlushMask(index)
        io.forward(index).paddr := addrArray(index).addr
        io.forward(index).data := dataArray(index).data
        io.forward(index).alignedMask := addrArray(index).alignedMask
        io.forward(index).sqindex := (BigInt(1) << index).U(length.W)
        io.forward(index).sqindexHigh := identities(index).indexHigh
        io.forward(index).olderStoreMask := olderStoreMask(index)
        io.forward(index).committed := committed(index)

    }

    io.liveMask := validMask & ~effectiveFlushMask
    io.committedMask := committed.asUInt & validMask
    io.commitPtrOH := commitPtr
    io.commitPtrHigh := commitPtrHigh

    val pastValid = RegNext(!reset.asBool, false.B)
    val previousCommitPtr = RegNext(commitPtr)
    val previousCommitPtrHigh = RegNext(commitPtrHigh)
    val previousCommitFire = RegNext(commitFire, false.B)

    when(!reset.asBool) {
        assert((io.waitState.dataMissingMask &
            (~io.waitState.memoryPendingMask).asUInt) === 0.U,
            "StoreQueue: a completed Store cannot still be missing data")
        when(io.commit.valid) {
            assert(commitReady,
                p"StoreQueue: ROB Store commit must match the complete commit-head entry " +
                    p"commitPtr=${Hexadecimal(commitPtr)} commitHigh=${commitPtrHigh} " +
                    p"ready=${Hexadecimal(readyMask)} valid=${Hexadecimal(validMask)} " +
                    p"committed=${Hexadecimal(committed.asUInt)} " +
                    p"flushMask=${Hexadecimal(effectiveFlushMask)} flush=${io.flush} " +
                    p"head=${Hexadecimal(io.dispatch.headPtrNext)} " +
                    p"headHigh=${io.dispatch.headPtrNextHigh}")
            assert(!commitAddr.exception.orR,
                "StoreQueue: an architectural-exception Store must never commit")
        }
        when(pastValid && !previousCommitFire) {
            assert(commitPtr === previousCommitPtr &&
                commitPtrHigh === previousCommitPtrHigh,
                "StoreQueue: commit pointer changed without a Store commit")
        }
        when(io.flush) {
            assert(!io.commit.valid,
                "StoreQueue: hard flush and Store commit cannot occur together")
        }
        assert(PopCount(io.dispatch.headPtrNext) === 1.U,
            "StoreQueue: allocation head pointer must be one-hot")
        assert(PopCount(enqueuePtr) === 1.U,
            "StoreQueue: allocation tail pointer must be one-hot")
        assert(PopCount(commitPtr) === 1.U, "StoreQueue: commit pointer must be one-hot")
        assert(PopCount(dequeuePtr) === 1.U, "StoreQueue: dequeue pointer must be one-hot")
        assert((committed.asUInt & (~readyMask).asUInt) === 0.U,
            "StoreQueue: committed Store must have both address and data")
        assert((issued.asUInt & (~committed.asUInt).asUInt) === 0.U,
            "StoreQueue: only committed Stores may issue")
        assert((completed.asUInt & (~issued.asUInt).asUInt) === 0.U,
            "StoreQueue: only issued Stores may complete")
        assert((freedMask & (~committed.asUInt).asUInt) === 0.U,
            "StoreQueue: physical release must target committed Stores")
        assert((effectiveFlushMask & committed.asUInt) === 0.U,
            "StoreQueue: recovery must preserve committed Stores")
        assert(!(io.complete(0).valid && io.complete(1).valid &&
            (io.complete(0).bits.sqMask & io.complete(1).bits.sqMask).orR),
            "StoreQueue: completion ports must target different Stores")
        for (port <- io.complete.indices) {
            when(io.complete(port).valid) {
                val complete = io.complete(port).bits
                assert(complete.sqMask.orR,
                    "StoreQueue: completion must target at least one Store")
                assert((complete.sqMask & issued.asUInt) === complete.sqMask,
                    "StoreQueue: completion must target issued Stores")
                assert((complete.sqHighMask & ~complete.sqMask) === 0.U,
                    "StoreQueue: completion high mask must be a subset of its SQ mask")
                assert(((complete.sqHighMask ^ io.lsqLive.stqHighMask) &
                    complete.sqMask) === 0.U,
                    "StoreQueue: completion generations must match")
                assert(PopCount(complete.sqindex) === 1.U &&
                    (complete.sqindex & complete.sqMask).orR,
                    "StoreQueue: representative completion index must be in its SQ mask")
                assert(complete.sqindexHigh ===
                    (complete.sqindex & complete.sqHighMask).orR,
                    "StoreQueue: representative completion generation must match")
            }
        }
        when(io.retry.valid) {
            assert(PopCount(io.retry.bits.sqindex) === 1.U,
                "StoreQueue: retry boundary must be one-hot")
            assert(pointerAlive(
                io.retry.bits.sqindex,
                io.retry.bits.sqindexHigh,
                allocatedMask,
                residentHighMask
            ), "StoreQueue: retry boundary must name a resident Store")
            assert(pointerAlive(
                io.retry.bits.sqindex,
                io.retry.bits.sqindexHigh,
                io.lsqLive.stqValidMask,
                io.lsqLive.stqHighMask
            ), "StoreQueue: retry boundary generation must match allocator state")
            assert((io.retry.bits.sqindex & issued.asUInt).orR,
                "StoreQueue: retry boundary must be issued")
            assert(!(io.retry.bits.sqindex & completed.asUInt).orR,
                "StoreQueue: completed Store cannot be a retry boundary")
            assert(!(io.retry.bits.sqindex & completionMask).orR,
                "StoreQueue: retry boundary cannot complete in the same cycle")
            assert((retryMask & io.retry.bits.sqindex).orR,
                "StoreQueue: retry suffix must include its boundary")
            assert((retryMask & ~allocatedMask) === 0.U,
                "StoreQueue: retry suffix must contain only allocated Stores")
            assert((retryMask & ~issued.asUInt) === 0.U,
                "StoreQueue: retry suffix must contain only issued Stores")
            assert((retryMask & completed.asUInt) === 0.U,
                "StoreQueue: retry suffix must preserve completed Stores")
            assert((retryMask & completionMask) === 0.U,
                "StoreQueue: same-cycle completions must be excluded from retry")
        }
        for (port <- 0 until StoreQueueConfig.EnqNum) {
            when(enqueueWrites(port)) {
                assert(PopCount(io.enqueue(port).bits.sqindex) === 1.U,
                    "StoreQueue: sqindex must be one-hot")
            }
            when(enqueueExceptionCompletes(port)) {
                assert(port.U >= 2.U,
                    "StoreQueue: only STA may report a Store exception")
                assert(!enqueueWrites(port),
                    "StoreQueue: exception Store must not enter the SQ arrays")
                assert(io.enqueue(port).bits.exceptionBadvValid,
                    "StoreQueue: Store address exception must carry BADV")
                assert(!(io.dequeue.fire &&
                    (issueOH & io.enqueue(port).bits.sqindex).orR),
                    "StoreQueue: exception Store must not issue to DCache")
            }
        }
    }
}
