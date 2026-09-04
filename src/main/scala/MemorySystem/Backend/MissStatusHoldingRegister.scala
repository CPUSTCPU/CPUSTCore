package CPUSTC.memory.backend

import chisel3._
import chisel3.util._
import CPUSTC.memory._
import CPUSTC.config.RegisterFile._
import CPUSTC.backend.rob.RobPtr

class MshrReadRequest extends Bundle {
    val paddr = UInt(32.W)
}

class MshrReadResponse extends Bundle {
    val paddr = UInt(32.W)
    val data = UInt(DcacheConfig.DcacheLineBits.W)
    val dirty = Bool()
}

class MshrMemoryIO extends Bundle {
    val readReq = Decoupled(new MshrReadRequest)
    val readResp = Flipped(Decoupled(new MshrReadResponse))
    val writeReq = Decoupled(new WritebackRequest)
    val writeResp = Flipped(Valid(new WritebackResponse))
}

object MshrLoadWaiterConfig {
    val returnCycles = 2
    val slotsPerLine = DcacheConfig.nPorts * returnCycles
    val tokenWidth = log2Ceil(LoadStateTableConfig.length)
    val pageOffsetWidth = 12
    val vaddrVpnWidth = 32 - pageOffsetWidth
}

object MshrLoadFormat {
    val width = 3

    def encode(mask: UInt, signed: Bool): UInt = {
        // Accepted Loads have masks 0001, 0011, or 1111. Encode their size
        // directly; word ignores signedness and therefore has the sole code 4.
        Cat(mask(2), mask(1) && !mask(2), !signed && !mask(2))
    }

    def mask(format: UInt): UInt = {
        Mux(format(2), "b1111".U,
            Mux(format(1), "b0011".U, "b0001".U))
    }

    def signed(format: UInt): Bool = !format(2) && !format(0)
    def legal(format: UInt): Bool = !format(2) || !format(1, 0).orR
}

class MshrLoadMetadata extends Bundle {
    val token = UInt(MshrLoadWaiterConfig.tokenWidth.W)
    val robPtr = new RobPtr
    val pdest = UInt(wpreg.W)
    val ldindexHigh = Bool()
    val format = UInt(MshrLoadFormat.width.W)
    val vaddrVpn = UInt(MshrLoadWaiterConfig.vaddrVpnWidth.W)
}

class MshrLoadWaiter extends Bundle {
    val byteOffset = UInt(DcacheConfig.DcacheOffset.W)
    val metadata = new MshrLoadMetadata
}

class MshrLoadResult extends Bundle {
    val linePaddr = UInt((32 - DcacheConfig.DcacheOffset).W)
    val waiter = new MshrLoadWaiter
    val data = UInt(DcacheConfig.DcacheDataBits.W)
}

class MshrEntry extends Bundle {
    // The baseline keeps all per-line state in one FIFO entry. Refill
    // compaction therefore moves the address, Store overlay, and SQ metadata
    // together.
    val linePaddr = UInt((32 - DcacheConfig.DcacheOffset).W)
    val storeWen = Vec(DcacheConfig.DcacheLineWord,
        UInt(DcacheConfig.DcacheMaskBits.W))
    val storeBuffer = Vec(DcacheConfig.DcacheLineWord,
        UInt(DcacheConfig.DcacheDataBits.W))
    val sqMask = UInt(StoreQueueConfig.length.W)
    val sqHighMask = UInt(StoreQueueConfig.length.W)
    val waiterCount = UInt(log2Ceil(MshrLoadWaiterConfig.slotsPerLine + 1).W)
    val waiters = Vec(
        MshrLoadWaiterConfig.slotsPerLine,
        new MshrLoadWaiter
    )
}

class MissStatusHoldingRegisterIO extends Bundle {
    val req0 = Flipped(Decoupled(new DcacheMshrPort0Req))
    val req1 = Flipped(Decoupled(new DcacheMshrPort1Req))
    val storeAdmissionReady = Output(Bool())
    val victimAvailable = Output(Bool())
    val victimReq = Flipped(Valid(new WritebackRequest))
    val refill = Decoupled(new DcacheMshrRefill)
    val memory = new MshrMemoryIO
    val storeComplete = Output(Valid(new StoreReadyEvent))
    val progress = Output(Bool())
    val loadWaiterFlush = Input(Bool())
    val loadReturn = Output(Vec(
        DcacheConfig.nPorts,
        Valid(new MshrLoadResult)
    ))
    val idle = Output(Bool())
}

class MissStatusHoldingRegister(enableDebug: Boolean = false) extends Module {
    val io = IO(new MissStatusHoldingRegisterIO)

    // Baseline data flow:
    //   1. Compare up to two DCache requests against every live FIFO entry.
    //   2. Merge a Store hit into its existing entry, or append new lines.
    //   3. Issue a memory read only for the FIFO head.
    //   4. Buffer the returned line and overlay all pending Store bytes.
    //   5. Refill DCache, report Store completion, and compact the FIFO.

    // -------------------------------------------------------------------------
    // Geometry and persistent state
    // -------------------------------------------------------------------------
    private val length = MshrConfig.length
    private val waiterSlots = MshrLoadWaiterConfig.slotsPerLine
    private val countWidth = log2Ceil(length + 1)
    private val wordIndexWidth = log2Ceil(DcacheConfig.DcacheLineWord)
    private val byteIndexWidth = log2Ceil(DcacheConfig.DcacheMaskBits)
    require(length >= 2, "MSHR requires at least two entries")
    require(DcacheConfig.nPorts == 2,
        "MSHR two-cycle waiter return requires two DCache result lanes")
    require(MshrLoadWaiterConfig.returnCycles == 2,
        "MSHR waiter return requires two fixed result lanes")

    val entries = RegInit(VecInit.fill(length)(0.U.asTypeOf(new MshrEntry)))
    val count = RegInit(0.U(countWidth.W))
    val readOutstanding = RegInit(false.B)
    val refillValid = RegInit(false.B)
    val refillData = Reg(UInt(DcacheConfig.DcacheLineBits.W))
    val refillLowerDirty = RegInit(false.B)
    val returnValid = RegInit(VecInit.fill(DcacheConfig.nPorts)(false.B))
    val lateReturnPending = RegInit(VecInit.fill(DcacheConfig.nPorts)(false.B))
    val returnLinePaddr = Reg(UInt((32 - DcacheConfig.DcacheOffset).W))
    val returnWaiter = Reg(Vec(DcacheConfig.nPorts, new MshrLoadWaiter))
    val restartReturn = RegNext(io.refill.fire, false.B)

    // -------------------------------------------------------------------------
    // Address, mask, and FIFO lookup helpers
    // -------------------------------------------------------------------------
    def fullLinePaddr(linePaddr: UInt): UInt = {
        Cat(linePaddr, 0.U(DcacheConfig.DcacheOffset.W))
    }

    def byteMaskToBitMask(mask: UInt): UInt = {
        VecInit(mask.asBools.map(bit => Fill(8, bit))).asUInt
    }

    def alignedLineWord(line: UInt, byteOffset: UInt): UInt = {
        val words = line.asTypeOf(Vec(
            DcacheConfig.DcacheLineWord,
            UInt(DcacheConfig.DcacheDataBits.W)
        ))
        val selectedWord = words(byteOffset(
            DcacheConfig.DcacheOffset - 1,
            byteIndexWidth
        ))
        (selectedWord >> (byteOffset(byteIndexWidth - 1, 0) << 3))(
            DcacheConfig.DcacheDataBits - 1,
            0
        )
    }

    def extendLoadData(word: UInt, format: UInt): UInt = {
        val byteData = word(7, 0)
        val halfData = word(15, 0)
        val signed = MshrLoadFormat.signed(format)
        val byteResult = Cat(Fill(24, signed && byteData(7)), byteData)
        val halfResult = Cat(Fill(16, signed && halfData(15)), halfData)

        Mux(format(2), word,
            Mux(format(1), halfResult, byteResult))
    }

    def makeWaiter(byteOffset: UInt, metadata: MshrLoadMetadata): MshrLoadWaiter = {
        val waiter = Wire(new MshrLoadWaiter)
        waiter.byteOffset := byteOffset
        waiter.metadata := metadata
        waiter
    }

    def makeLoadResult(
        linePaddr: UInt,
        waiter: MshrLoadWaiter,
        line: UInt
    ): MshrLoadResult = {
        val result = Wire(new MshrLoadResult)
        result.linePaddr := linePaddr
        result.waiter := waiter
        result.data := extendLoadData(
            alignedLineWord(line, waiter.byteOffset),
            waiter.metadata.format
        )
        result
    }

    def lineHits(linePaddr: UInt) = VecInit((0 until length).map { index =>
        index.U < count && entries(index).linePaddr === linePaddr
    })

    // -------------------------------------------------------------------------
    // Writeback buffer connection and current FIFO-head view
    // -------------------------------------------------------------------------
    val wbb = Module(new WritebackBuffer)
    wbb.io.memoryReq <> io.memory.writeReq
    wbb.io.memoryResp <> io.memory.writeResp
    val victimCapacityCredit = RegInit(true.B)
    victimCapacityCredit := wbb.io.nextHasSpace
    dontTouch(victimCapacityCredit)

    val headValid = count =/= 0.U
    val headEntry = entries(0)
    val headPaddr = fullLinePaddr(headEntry.linePaddr)
    val refillFire = io.refill.fire
    val countAfterRefill = count - refillFire.asUInt
    val responseCycle = io.memory.readResp.fire

    // -------------------------------------------------------------------------
    // Admission and two-port request classification
    // -------------------------------------------------------------------------
    val reqFire0 = io.req0.fire
    val reqFire1 = io.req1.fire
    val hitOH0 = lineHits(io.req0.bits.linePaddr)
    val hitOH1 = lineHits(io.req1.bits.linePaddr)
    val lineHit0 = hitOH0.asUInt.orR
    val lineHit1 = hitOH1.asUInt.orR
    // Preserve the baseline two-line reservation for allocations. Loads may
    // still coalesce into a live line when capacity is reserved, but Stores do
    // not: allowing a Store hit in that state makes the otherwise Load-only
    // tail entry's entire 512-bit Store overlay independently writable.
    val hasTwoFreeEntries = count <= (length - 2).U
    val requestAdmissionOpen = !refillValid && !responseCycle
    val loadAdmissionOpen = requestAdmissionOpen && !io.loadWaiterFlush
    val req1HitWaiterCount = Mux1H(hitOH1, entries.map(_.waiterCount))
    val req1HitHasSlot = req1HitWaiterCount < waiterSlots.U
    val req1HitSlotOH = UIntToOH(req1HitWaiterCount, waiterSlots)
    val req1HitReady = lineHit1 && req1HitHasSlot
    val req1NewReady = !lineHit1 && hasTwoFreeEntries
    io.req1.ready := loadAdmissionOpen &&
        (req1HitReady || req1NewReady)

    // Port 1 has fixed priority for the final waiter slot. This keeps the
    // admission decision identical to the append order below: every Load fire
    // is therefore a proof that one physical waiter slot will be written.
    val req1ClaimsReq0Line = io.req1.valid && loadAdmissionOpen &&
        req1HitHasSlot &&
        (hitOH0.asUInt & hitOH1.asUInt).orR
    val req0HitWaiterCount = Mux1H(hitOH0, entries.map(_.waiterCount))
    val req0HitHasSlot = req0HitWaiterCount < waiterSlots.U
    val req0BaseSlotOH = UIntToOH(req0HitWaiterCount, waiterSlots)
    val req0SlotAfterPort1 =
        (req0BaseSlotOH << 1)(waiterSlots - 1, 0)
    val req0HitSlotOH = Mux(
        req1ClaimsReq0Line,
        req0SlotAfterPort1,
        req0BaseSlotOH
    )
    val req0LoadHitReady = lineHit0 && req0HitHasSlot &&
        !(req1ClaimsReq0Line && req0BaseSlotOH(waiterSlots - 1))
    val req0LoadNewReady = !lineHit0 && hasTwoFreeEntries
    val req0LoadReady = loadAdmissionOpen &&
        (req0LoadHitReady || req0LoadNewReady)
    io.storeAdmissionReady := requestAdmissionOpen && hasTwoFreeEntries
    io.req0.ready := Mux(
        io.req0.bits.store,
        io.storeAdmissionReady,
        req0LoadReady
    )

    val newReq0 = reqFire0 && !lineHit0
    val sameNewLine = newReq0 && reqFire1 && !lineHit1 &&
        io.req0.bits.linePaddr === io.req1.bits.linePaddr
    val newReq1 = reqFire1 && !lineHit1 && !sameNewLine
    val enqCount = newReq0.asUInt +& newReq1.asUInt
    val nextCount = countAfterRefill + enqCount

    // -------------------------------------------------------------------------
    // Capacity notification and global idle state
    // -------------------------------------------------------------------------
    // A blocked request may be waiting on line capacity, a full waiter set, or
    // the response-cycle freeze. Any retired entry is useful progress;
    // wake all such requests and let the normal DCache/MSHR lookup retry them.
    io.progress := refillFire
    io.idle :=
        count === 0.U &&
        !readOutstanding &&
        !refillValid &&
        !returnValid.asUInt.orR &&
        !lateReturnPending.asUInt.orR &&
        wbb.io.empty &&
        !io.victimReq.valid

    // -------------------------------------------------------------------------
    // Store-hit merge preparation and new-entry construction
    // -------------------------------------------------------------------------
    // Only request port 0 may carry a Store. Byte offset/mask are aligned to
    // the selected word before either initializing or merging an entry.
    val storeMerge0 = reqFire0 && io.req0.bits.store && lineHit0
    val storeWordIndex = io.req0.bits.byteOffset(
        DcacheConfig.DcacheOffset - 1, byteIndexWidth)
    val storeByteIndex = io.req0.bits.byteOffset(byteIndexWidth - 1, 0)
    val storeBitShift = storeByteIndex << 3
    val alignedStoreData = (io.req0.bits.storeData << storeBitShift)(
        DcacheConfig.DcacheDataBits - 1, 0)
    val alignedStoreMask = (io.req0.bits.storeMask << storeByteIndex)(
        DcacheConfig.DcacheMaskBits - 1, 0)
    val alignedStoreBitMask = byteMaskToBitMask(alignedStoreMask)

    val firstAllocEntry = WireDefault(0.U.asTypeOf(new MshrEntry))
    firstAllocEntry.linePaddr := Mux(newReq0,
        io.req0.bits.linePaddr, io.req1.bits.linePaddr)
    val firstAllocStore = newReq0 && io.req0.bits.store
    for (word <- 0 until DcacheConfig.DcacheLineWord) {
        val selected = firstAllocStore && storeWordIndex === word.U(wordIndexWidth.W)
        firstAllocEntry.storeWen(word) := Mux(selected, alignedStoreMask, 0.U)
        firstAllocEntry.storeBuffer(word) := Mux(selected, alignedStoreData, 0.U)
    }
    firstAllocEntry.sqMask := Mux(firstAllocStore, io.req0.bits.sqindex, 0.U)
    firstAllocEntry.sqHighMask := Mux(
        firstAllocStore && io.req0.bits.sqindexHigh,
        io.req0.bits.sqindex,
        0.U
    )
    val firstAllocHasPort1 = sameNewLine || (!newReq0 && newReq1)
    val firstAllocHasPort0 = newReq0 && !io.req0.bits.store
    val req0Waiter = makeWaiter(io.req0.bits.byteOffset, io.req0.bits.loadMetadata)
    val req1Waiter = makeWaiter(io.req1.bits.byteOffset, io.req1.bits.loadMetadata)
    firstAllocEntry.waiterCount :=
        firstAllocHasPort1.asUInt +& firstAllocHasPort0.asUInt
    when(firstAllocHasPort1) {
        firstAllocEntry.waiters(0) := req1Waiter
    }
    when(firstAllocHasPort0) {
        when(firstAllocHasPort1) {
            firstAllocEntry.waiters(1) := req0Waiter
        }.otherwise {
            firstAllocEntry.waiters(0) := req0Waiter
        }
    }

    val secondAllocEntry = WireDefault(0.U.asTypeOf(new MshrEntry))
    secondAllocEntry.linePaddr := io.req1.bits.linePaddr
    secondAllocEntry.waiterCount := 1.U
    secondAllocEntry.waiters(0) := req1Waiter

    // -------------------------------------------------------------------------
    // Combined dequeue, compaction, merge, and enqueue next state
    // -------------------------------------------------------------------------
    // baseEntries describes the optional head removal. lineUpdatedEntries then
    // applies a Store merge and appends up to two new cache lines.
    val baseEntries = Wire(Vec(length, new MshrEntry))
    for (index <- 0 until length) {
        if (index < length - 1) {
            baseEntries(index) := Mux(refillFire, entries(index + 1), entries(index))
        } else {
            baseEntries(index) := entries(index)
        }
    }

    val lineUpdatedEntries = WireDefault(baseEntries)
    for (index <- 0 until length) {
        when(storeMerge0 && hitOH0(index)) {
            for (word <- 0 until DcacheConfig.DcacheLineWord) {
                when(storeWordIndex === word.U(wordIndexWidth.W)) {
                    lineUpdatedEntries(index).storeBuffer(word) :=
                        (entries(index).storeBuffer(word) & ~alignedStoreBitMask) |
                            (alignedStoreData & alignedStoreBitMask)
                    lineUpdatedEntries(index).storeWen(word) :=
                        entries(index).storeWen(word) | alignedStoreMask
                }
            }
            lineUpdatedEntries(index).sqMask :=
                entries(index).sqMask | io.req0.bits.sqindex
            lineUpdatedEntries(index).sqHighMask := entries(index).sqHighMask | Mux(
                io.req0.bits.sqindexHigh,
                io.req0.bits.sqindex,
                0.U
            )
        }

        when(newReq0 && index.U === countAfterRefill) {
            lineUpdatedEntries(index) := firstAllocEntry
        }
        when(!newReq0 && newReq1 && index.U === countAfterRefill) {
            lineUpdatedEntries(index) := firstAllocEntry
        }
        when(newReq0 && newReq1 && index.U === countAfterRefill + 1.U) {
            lineUpdatedEntries(index) := secondAllocEntry
        }
    }

    // Existing-line requests already have hitOH from admission. Reuse it here
    // instead of comparing both request addresses against all four post-update
    // entries a second time. New entries initialize their waiter slots above.
    // Admission rejects every request during readResp and rejects an
    // existing-line Load once its fixed waiter set is full. Consequently every
    // accepted hit below must append exactly once.
    val nextEntries = WireDefault(lineUpdatedEntries)
    for (index <- 0 until length) {
        val port1Targets = reqFire1 && hitOH1(index)
        val port0Targets = reqFire0 && !io.req0.bits.store && hitOH0(index)
        val finalWaiterCount = entries(index).waiterCount +
            port1Targets.asUInt + port0Targets.asUInt

        when(port1Targets || port0Targets) {
            nextEntries(index).waiterCount := finalWaiterCount
        }
        for (slot <- 0 until waiterSlots) {
            when(port1Targets && req1HitSlotOH(slot)) {
                nextEntries(index).waiters(slot) := req1Waiter
            }
            when(port0Targets && req0HitSlotOH(slot)) {
                nextEntries(index).waiters(slot) := req0Waiter
            }
        }

        when(io.loadWaiterFlush) {
            nextEntries(index).waiterCount := 0.U
        }

        when(index.U < count && !io.loadWaiterFlush) {
            for (older <- 0 until waiterSlots; younger <- older + 1 until waiterSlots) {
                when(nextEntries(index).waiterCount > younger.U) {
                    assert(nextEntries(index).waiters(older).metadata.token =/=
                        nextEntries(index).waiters(younger).metadata.token,
                        "MSHR: one line cannot contain a duplicate Load token")
                }
            }
        }

        when(port1Targets) {
            assert(req1HitHasSlot,
                "MSHR: port 1 Load fire must allocate one waiter")
        }
        when(port0Targets) {
            assert(req0HitHasSlot && req0HitSlotOH.orR,
                "MSHR: port 0 Load fire must allocate one waiter")
        }
    }

    entries := nextEntries
    count := nextCount

    // -------------------------------------------------------------------------
    // Victim writeback and lower-memory read pipeline
    // -------------------------------------------------------------------------
    io.victimAvailable := victimCapacityCredit
    wbb.io.enqueue(0).valid := io.victimReq.valid
    wbb.io.enqueue(0).bits := io.victimReq.bits
    wbb.io.enqueue(1).valid := false.B
    wbb.io.enqueue(1).bits := 0.U.asTypeOf(new WritebackRequest)

    wbb.io.queryPaddr := headPaddr
    io.memory.readReq.valid := headValid && !readOutstanding && !refillValid &&
        !wbb.io.queryHit && !io.victimReq.valid
    io.memory.readReq.bits.paddr := headPaddr
    io.memory.readResp.ready := readOutstanding && !refillValid

    // Requests are frozen on the response cycle, so the registered head Store
    // overlay is already final. No second 512-bit same-cycle rescue mux is
    // needed here.
    val responseStoreWenBits = headEntry.storeWen.asUInt
    val responseStoreData = Cat(headEntry.storeBuffer.reverse)
    val responseStoreBitMask = byteMaskToBitMask(responseStoreWenBits)
    val responseMergedData =
        (io.memory.readResp.bits.data & ~responseStoreBitMask) |
            (responseStoreData & responseStoreBitMask)

    when(io.memory.readReq.fire) {
        readOutstanding := true.B
    }
    when(io.memory.readResp.fire) {
        assert(headValid, "MSHR: read response without a head entry")
        assert(io.memory.readResp.bits.paddr === headPaddr,
            "MSHR: read response address must match the head entry")
        refillData := responseMergedData
        refillLowerDirty := io.memory.readResp.bits.dirty
        refillValid := true.B
        readOutstanding := false.B
    }

    // -------------------------------------------------------------------------
    // Store overlay, DCache refill, and Store completion
    // -------------------------------------------------------------------------
    // The refill payload is the lower-memory line with the head entry's latest
    // Store bytes applied. The FIFO retires only when DCache accepts it.
    val headStoreWen = headEntry.storeWen.asUInt
    io.refill.valid := refillValid
    io.refill.bits.paddr := headPaddr
    io.refill.bits.dirty := refillLowerDirty || headStoreWen.orR
    io.refill.bits.data := refillData

    // readResp captures identity state, but no Load completes until DCache
    // installs the line. Slots 0/1 use refillFire; slots 2/3 use the following
    // sRestart cycle, so direct returns consume only the two existing stall
    // cycles regardless of refill backpressure.
    for (lane <- 0 until DcacheConfig.nPorts) {
        io.loadReturn(lane).valid := returnValid(lane) &&
            (refillFire || restartReturn) && !io.loadWaiterFlush
        io.loadReturn(lane).bits := makeLoadResult(
            returnLinePaddr,
            returnWaiter(lane),
            refillData
        )
    }

    when(io.loadReturn(0).valid && io.loadReturn(1).valid) {
        assert(io.loadReturn(0).bits.waiter.metadata.token =/=
            io.loadReturn(1).bits.waiter.metadata.token,
            "MSHR: one return phase cannot contain a duplicate Load token")
    }

    when(io.loadWaiterFlush) {
        returnValid := VecInit.fill(DcacheConfig.nPorts)(false.B)
        lateReturnPending := VecInit.fill(DcacheConfig.nPorts)(false.B)
    }.elsewhen(refillFire) {
        // Capture from the old FIFO head on the same edge that compacts it.
        returnValid := lateReturnPending
        lateReturnPending := VecInit.fill(DcacheConfig.nPorts)(false.B)
        for (lane <- 0 until DcacheConfig.nPorts) {
            val secondSlot = lane + DcacheConfig.nPorts
            returnWaiter(lane) := entries(0).waiters(secondSlot)
        }
    }.elsewhen(responseCycle) {
        assert((!returnValid.asUInt.orR && !lateReturnPending.asUInt.orR) ||
            restartReturn,
            "MSHR: a new response must not overlap a pending Load return")
        returnLinePaddr := entries(0).linePaddr
        for (lane <- 0 until DcacheConfig.nPorts) {
            val firstSlot = lane
            val secondSlot = lane + DcacheConfig.nPorts
            returnValid(lane) := entries(0).waiterCount > firstSlot.U
            lateReturnPending(lane) :=
                entries(0).waiterCount > secondSlot.U
            returnWaiter(lane) := entries(0).waiters(firstSlot)
        }
    }.elsewhen(restartReturn) {
        returnValid := VecInit.fill(DcacheConfig.nPorts)(false.B)
    }

    val representativeSqindex = PriorityEncoderOH(headEntry.sqMask)
    io.storeComplete.valid := refillFire && headStoreWen.orR
    io.storeComplete.bits.paddr := headPaddr
    io.storeComplete.bits.sqindex := representativeSqindex
    io.storeComplete.bits.sqindexHigh :=
        (representativeSqindex & headEntry.sqHighMask).orR
    io.storeComplete.bits.sqMask := headEntry.sqMask
    io.storeComplete.bits.sqHighMask := headEntry.sqHighMask

    when(refillFire) {
        refillValid := false.B
        refillLowerDirty := false.B
        readOutstanding := false.B
    }

    // -------------------------------------------------------------------------
    // Optional debug trace
    // -------------------------------------------------------------------------
    if (enableDebug) {
        val debugCycle = RegInit(0.U(64.W))
        debugCycle := debugCycle + 1.U
        val previousCount = RegNext(count, 0.U)
        val previousReadOutstanding = RegNext(readOutstanding, false.B)
        val previousRefillValid = RegNext(refillValid, false.B)
        val stateChanged = previousCount =/= count ||
            previousReadOutstanding =/= readOutstanding ||
            previousRefillValid =/= refillValid
        val heartbeat = count =/= 0.U && debugCycle(7, 0) === 0.U

        when(debugCycle >= 300000.U) {
        when(stateChanged || heartbeat) {
            printf(
                p"[DBG][MSHR][STATE] cycle=${debugCycle} count=${count} " +
                    p"readOutstanding=${readOutstanding} refillValid=${refillValid} " +
                    p"headValid=${headValid} headPaddr=0x${Hexadecimal(headPaddr)} " +
                    p"readReqV=${io.memory.readReq.valid} readReqR=${io.memory.readReq.ready} " +
                    p"readRespV=${io.memory.readResp.valid} readRespR=${io.memory.readResp.ready} " +
                    p"refillV=${io.refill.valid} refillR=${io.refill.ready}\n"
            )
            for (index <- 0 until length) {
                when(index.U < count) {
                    printf(
                        p"[DBG][MSHR][ENTRY] cycle=${debugCycle} entry=${index.U} " +
                            p"paddr=0x${Hexadecimal(fullLinePaddr(entries(index).linePaddr))} " +
                            p"sqMask=0x${Hexadecimal(entries(index).sqMask)} " +
                            p"sqHighMask=0x${Hexadecimal(entries(index).sqHighMask)}\n"
                    )
                }
            }
        }

        when(reqFire0) {
            printf(
                p"[DBG][MSHR][REQ] cycle=${debugCycle} port=0 " +
                    p"paddr=0x${Hexadecimal(fullLinePaddr(io.req0.bits.linePaddr))} " +
                    p"store=${io.req0.bits.store} hit=${lineHit0} new=${newReq0} count=${count}\n"
            )
        }
        when(reqFire1) {
            printf(
                p"[DBG][MSHR][REQ] cycle=${debugCycle} port=1 " +
                    p"paddr=0x${Hexadecimal(fullLinePaddr(io.req1.bits.linePaddr))} " +
                    p"hit=${lineHit1} new=${newReq1} count=${count}\n"
            )
        }
        when(io.memory.readReq.fire) {
            printf(
                p"[DBG][MSHR][READ_REQ] cycle=${debugCycle} " +
                    p"paddr=0x${Hexadecimal(io.memory.readReq.bits.paddr)}\n"
            )
        }
        when(io.memory.readResp.fire) {
            printf(
                p"[DBG][MSHR][READ_RESP] cycle=${debugCycle} " +
                    p"paddr=0x${Hexadecimal(io.memory.readResp.bits.paddr)}\n"
            )
        }
        when(refillFire) {
            printf(
                p"[DBG][MSHR][REFILL] cycle=${debugCycle} " +
                    p"paddr=0x${Hexadecimal(io.refill.bits.paddr)} " +
                    p"dirty=${io.refill.bits.dirty} count=${count}\n"
            )
        }
        when(io.progress) {
            printf(p"[DBG][MSHR][PROGRESS] cycle=${debugCycle} count=${count}\n")
        }
        }
    }

    // -------------------------------------------------------------------------
    // Runtime protocol and state invariants
    // -------------------------------------------------------------------------
    when(!reset.asBool) {
        assert(!readOutstanding || headValid,
            "MSHR: an outstanding read must retain a head entry")
        assert(!refillValid || headValid,
            "MSHR: a buffered refill must retain a head entry")
        assert(!(readOutstanding && refillValid),
            "MSHR: read request and buffered refill phases must be exclusive")
        when(io.victimReq.valid) {
            assert(victimCapacityCredit,
                "MSHR: victim request requires registered WBB capacity")
            assert(wbb.io.enqueue(0).ready,
                "MSHR: victim request must use refill-confirmed WBB capacity")
            assert(wbb.io.enqueue(0).fire,
                "MSHR: a valid victim request must enter the WBB")
        }
        assert(victimCapacityCredit === wbb.io.enqueue(0).ready,
            "MSHR: registered WBB capacity must match physical capacity")
        assert(!(refillFire && (reqFire0 || reqFire1)),
            "MSHR: DCache must block new requests during refill")
        assert(!(responseCycle && (reqFire0 || reqFire1)),
            "MSHR: response-cycle requests must replay after refill progress")
        assert(count +& enqCount <= length.U + refillFire.asUInt,
            "MSHR: enqueue must not overflow the entry array")

        when(reqFire0) {
            when(io.req0.bits.store) {
                assert(PopCount(io.req0.bits.sqindex) === 1.U,
                    "MSHR: each Store request must identify one SQ entry")
                assert(io.req0.bits.storeMask.orR,
                    "MSHR: Store request must write at least one byte")
            }.otherwise {
                assert(!io.req0.bits.storeMask.orR &&
                    !io.req0.bits.sqindex.orR &&
                    !io.req0.bits.sqindexHigh,
                    "MSHR: Load request must not carry Store state")
                assert(io.req0.bits.loadMetadata.token < LoadStateTableConfig.length.U,
                    "MSHR: port 0 Load token must select an LST entry")
                assert(MshrLoadFormat.legal(io.req0.bits.loadMetadata.format),
                    "MSHR: port 0 Load format must be legal")
            }
        }

        when(reqFire1) {
            assert(io.req1.bits.loadMetadata.token < LoadStateTableConfig.length.U,
                "MSHR: port 1 Load token must select an LST entry")
            assert(MshrLoadFormat.legal(io.req1.bits.loadMetadata.format),
                "MSHR: port 1 Load format must be legal")
        }

        for (index <- 0 until length) {
            assert(entries(index).waiterCount <= waiterSlots.U,
                "MSHR: per-line waiter count must not exceed its fixed slots")
        }
        for (lane <- 0 until DcacheConfig.nPorts) {
            when(io.loadReturn(lane).valid) {
                assert(io.loadReturn(lane).bits.waiter.metadata.token <
                    LoadStateTableConfig.length.U,
                    s"MSHR: return lane $lane token must select an LST entry")
                assert(MshrLoadFormat.legal(
                    io.loadReturn(lane).bits.waiter.metadata.format),
                    s"MSHR: return lane $lane format must be legal")
            }
        }

        for (left <- 0 until length; right <- left + 1 until length) {
            assert(!(left.U < count && right.U < count &&
                entries(left).linePaddr === entries(right).linePaddr),
                "MSHR: one cache line must occupy at most one entry")
        }
    }
}
