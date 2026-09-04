package CPUSTC.predict

import chisel3._
import chisel3.util._

import CPUSTC.config.Fetch.nfch

object MiniTageConfig {
    val tableCount = 4
    val entries = 512
    val historyWidth = 64
    val historyLengths = Seq(4, 12, 32, 64)
    val tagWidths = Seq(7, 8, 9, 10)
    val counterWidth = 3
    val usefulWidth = 2
    val useAltOnNaWidth = 4

    val indexWidth = log2Ceil(entries)
    val tableIdWidth = log2Ceil(tableCount + 1)
    val storedTagWidth = tagWidths.max
    // Padding every slot to two 9-bit BRAM lanes permits four independent
    // slot write enables in a single 512 x 72-bit packet-row RAM.
    val storedEntryWidth = 18
    val rowWidth = nfch * storedEntryWidth

    require(nfch == 4)
    require(historyLengths.length == tableCount)
    require(tagWidths.length == tableCount)
    require(historyLengths.max == historyWidth)
    require(isPow2(entries))
    require(storedEntryWidth >=
        storedTagWidth + 1 + counterWidth + usefulWidth)
}

class MiniTageLookupReq extends Bundle {
    val pc = UInt(32.W)
    val history = UInt(MiniTageConfig.historyWidth.W)
}

class MiniTageBaseInfo extends Bundle {
    // This is the existing Agree result, presented one cycle after lookup.
    // PC is repeated only for alignment checking and s3 correction identity.
    val pc = UInt(32.W)
    val baseTaken = UInt(nfch.W)
    val conditionalMask = UInt(nfch.W)
}

class MiniTageSlotMeta extends Bundle {
    // Zero denotes the external Agree base; one through four denote T0-T3.
    val provider = UInt(MiniTageConfig.tableIdWidth.W)
    // The alternate provider identity is not needed at retirement; only its
    // direction is retained beside the PC-local MiniTAGE-vs-fast chooser.
    val chooserCounter = UInt(MiniTageChooserConfig.counterWidth.W)
    val providerCounter = UInt(MiniTageConfig.counterWidth.W)
    val providerUseful = UInt(MiniTageConfig.usefulWidth.W)
    val alternateTaken = Bool()
    val useAlternate = Bool()
    // Allocation needs only the usefulness state of each candidate table.
    val tableUseful = Vec(
        MiniTageConfig.tableCount,
        UInt(MiniTageConfig.usefulWidth.W)
    )
}

class MiniTagePredictionMeta extends Bundle {
    val slots = Vec(nfch, new MiniTageSlotMeta)
}

class MiniTageLookupResp extends Bundle {
    val pc = UInt(32.W)
    val baseTaken = UInt(nfch.W)
    val conditionalMask = UInt(nfch.W)
    val candidateTaken = UInt(nfch.W)
    val providerHitMask = UInt(nfch.W)
    // The module is shadow-only. A later s3 integration can consume this mask
    // to request an early correction without changing the current s2 path.
    val correctionMask = UInt(nfch.W)
    val meta = new MiniTagePredictionMeta
}

class MiniTageTrain extends Bundle {
    // PC/history/base are already FTQ state and deliberately are not copied
    // into MiniTAGE prediction metadata.
    val pc = UInt(32.W)
    val history = UInt(MiniTageConfig.historyWidth.W)
    val baseTaken = UInt(nfch.W)
    val trainMask = UInt(nfch.W)
    val takenMask = UInt(nfch.W)
    val meta = new MiniTagePredictionMeta
}

class MiniTageIO extends Bundle {
    val lookup = Flipped(Valid(new MiniTageLookupReq))
    val base = Flipped(Valid(new MiniTageBaseInfo))
    val resp = Valid(new MiniTageLookupResp)
    val train = Flipped(Valid(new MiniTageTrain))
    val ready = Output(Bool())
    val useAltOnNa = Output(UInt(MiniTageConfig.useAltOnNaWidth.W))
}

private class MiniTagePacketRamBlackBox
    extends BlackBox
    with HasBlackBoxInline {
    import MiniTageConfig._

    val io = IO(new Bundle {
        val clock = Input(Clock())
        val ren = Input(Bool())
        val raddr = Input(UInt(indexWidth.W))
        val rdata = Output(UInt(rowWidth.W))
        val wen = Input(Bool())
        val waddr = Input(UInt(indexWidth.W))
        val wdata = Input(UInt(rowWidth.W))
        val wmask = Input(UInt(nfch.W))
    })

    setInline(
        "MiniTagePacketRamBlackBox.sv",
        s"""
           |module MiniTagePacketRamBlackBox (
           |  input  wire        clock,
           |  input  wire        ren,
           |  input  wire [8:0]  raddr,
           |  output reg  [71:0] rdata,
           |  input  wire        wen,
           |  input  wire [8:0]  waddr,
           |  input  wire [71:0] wdata,
           |  input  wire [3:0]  wmask
           |);
           |  (* ram_style = "block" *) reg [71:0] mem [0:511];
           |
           |  always @(posedge clock) begin
           |    if (ren)
           |      rdata <= mem[raddr];
           |    if (wen) begin
           |      if (wmask[0]) mem[waddr][17:0]  <= wdata[17:0];
           |      if (wmask[1]) mem[waddr][35:18] <= wdata[35:18];
           |      if (wmask[2]) mem[waddr][53:36] <= wdata[53:36];
           |      if (wmask[3]) mem[waddr][71:54] <= wdata[71:54];
           |    end
           |  end
           |endmodule
           |""".stripMargin
    )
}

private class MiniTagePacketRam(useBlackBox: Boolean) extends Module {
    import MiniTageConfig._

    val io = IO(new Bundle {
        val ren = Input(Bool())
        val raddr = Input(UInt(indexWidth.W))
        val rdata = Output(Vec(nfch, UInt(storedEntryWidth.W)))
        val wen = Input(Bool())
        val waddr = Input(UInt(indexWidth.W))
        val wdata = Input(Vec(nfch, UInt(storedEntryWidth.W)))
        val wmask = Input(UInt(nfch.W))
    })

    val rawRead = Wire(Vec(nfch, UInt(storedEntryWidth.W)))
    if (useBlackBox) {
        val ram = Module(new MiniTagePacketRamBlackBox)
        ram.io.clock := clock
        ram.io.ren := io.ren
        ram.io.raddr := io.raddr
        ram.io.wen := io.wen
        ram.io.waddr := io.waddr
        ram.io.wdata := io.wdata.asUInt
        ram.io.wmask := io.wmask
        rawRead := ram.io.rdata.asTypeOf(rawRead)
    } else {
        val ram = SyncReadMem(
            entries,
            Vec(nfch, UInt(storedEntryWidth.W)),
            SyncReadMem.ReadFirst
        )
        rawRead := ram.read(io.raddr, io.ren)
        when(io.wen) {
            ram.write(io.waddr, io.wdata, io.wmask.asBools)
        }
    }

    // SyncReadMem read-during-write behavior is target-dependent. Register the
    // complete read/write command before comparing addresses so the bypass
    // cannot feed the current lookup address back into its own capture logic.
    val readValidReg = RegNext(io.ren, false.B)
    val readAddressReg = RegNext(io.raddr)
    val writeValidReg = RegNext(io.wen, false.B)
    val writeAddressReg = RegNext(io.waddr)
    val forwardedMask = RegNext(io.wmask)
    val forwardedData = RegNext(io.wdata)
    val collisionReg = readValidReg && writeValidReg &&
        readAddressReg === writeAddressReg
    for (slot <- 0 until nfch) {
        io.rdata(slot) := Mux(
            collisionReg && forwardedMask(slot),
            forwardedData(slot),
            rawRead(slot)
        )
    }
}

class MiniTage(useBlackBoxRam: Boolean = false) extends Module {
    import MiniTageConfig._

    val io = IO(new MiniTageIO)

    private def balancedXor(values: Seq[UInt]): UInt = {
        require(values.nonEmpty)
        if (values.length == 1) {
            values.head
        } else {
            val (left, right) = values.splitAt((values.length + 1) / 2)
            balancedXor(left) ^ balancedXor(right)
        }
    }

    private def fold(value: UInt, inputWidth: Int, outputWidth: Int): UInt = {
        val chunks = (0 until inputWidth by outputWidth).map { low =>
            val high = math.min(low + outputWidth - 1, inputWidth - 1)
            val chunk = value(high, low)
            if (high - low + 1 == outputWidth) chunk else chunk.pad(outputWidth)
        }
        balancedXor(chunks)
    }

    private def rotateLeft(value: UInt, width: Int): UInt =
        if (width == 1) value else Cat(value(width - 2, 0), value(width - 1))

    private def tableIndex(pc: UInt, history: UInt, table: Int): UInt = {
        val blockPc = pc(31, 4)
        val historyLength = historyLengths(table)
        fold(blockPc, 28, indexWidth) ^
            fold(history(historyLength - 1, 0), historyLength, indexWidth)
    }

    private def tableTag(pc: UInt, history: UInt, table: Int): UInt = {
        val blockPc = pc(31, 4)
        val historyLength = historyLengths(table)
        val tagWidth = tagWidths(table)
        val historySlice = history(historyLength - 1, 0)
        val primary = fold(historySlice, historyLength, tagWidth)
        val rotatedHistory = if (historyLength == 1) {
            historySlice
        } else {
            Cat(historySlice(historyLength - 2, 0), historySlice(historyLength - 1))
        }
        fold(blockPc, 28, tagWidth) ^ primary ^
            rotateLeft(fold(rotatedHistory, historyLength, tagWidth), tagWidth)
    }

    private def packEntry(
        valid: Bool,
        tag: UInt,
        counter: UInt,
        useful: UInt
    ): UInt = {
        val payload = Cat(
            useful(usefulWidth - 1, 0),
            counter(counterWidth - 1, 0),
            valid,
            tag.pad(storedTagWidth)
        )
        payload.pad(storedEntryWidth)
    }

    private def entryTag(entry: UInt): UInt = entry(storedTagWidth - 1, 0)
    private def entryValid(entry: UInt): Bool = entry(storedTagWidth)
    private def entryCounter(entry: UInt): UInt =
        entry(storedTagWidth + counterWidth, storedTagWidth + 1)
    private def entryUseful(entry: UInt): UInt = entry(
        storedTagWidth + counterWidth + usefulWidth,
        storedTagWidth + counterWidth + 1
    )

    private def updateCounter(old: UInt, taken: Bool, width: Int): UInt = {
        val max = ((1 << width) - 1).U(width.W)
        Mux(
            taken,
            Mux(old === max, old, old + 1.U),
            Mux(old === 0.U, old, old - 1.U)
        )
    }

    private val lookupTables = Seq.fill(tableCount)(
        Module(new MiniTagePacketRam(useBlackBoxRam))
    )
    private val trainTables = Seq.fill(tableCount)(
        Module(new MiniTagePacketRam(useBlackBoxRam))
    )

    val clearActive = RegInit(true.B)
    val clearIndex = RegInit(0.U(indexWidth.W))
    when(clearActive) {
        when(clearIndex === (entries - 1).U) {
            clearActive := false.B
        }.otherwise {
            clearIndex := clearIndex + 1.U
        }
    }
    io.ready := !clearActive

    val lookupAccept = io.lookup.valid && io.ready
    val lookupIndices = VecInit((0 until tableCount).map { table =>
        tableIndex(io.lookup.bits.pc, io.lookup.bits.history, table)
    })
    val lookupTagsNow = VecInit((0 until tableCount).map { table =>
        tableTag(io.lookup.bits.pc, io.lookup.bits.history, table).pad(
            storedTagWidth
        )
    })
    val s1Valid = RegNext(lookupAccept, false.B)
    val s1Pc = RegEnable(io.lookup.bits.pc, lookupAccept)
    val s1Tags = RegEnable(lookupTagsNow, lookupAccept)

    val trainReadIndices = VecInit((0 until tableCount).map { table =>
        tableIndex(io.train.bits.pc, io.train.bits.history, table)
    })
    val trainTagsNow = VecInit((0 until tableCount).map { table =>
        tableTag(io.train.bits.pc, io.train.bits.history, table).pad(
            storedTagWidth
        )
    })
    val trainAccept = io.train.valid && io.ready
    val trainValid = RegNext(trainAccept, false.B)
    val train = RegEnable(io.train.bits, trainAccept)
    val trainIndices = RegEnable(trainReadIndices, trainAccept)
    val trainTags = RegEnable(trainTagsNow, trainAccept)

    val tableWriteMask = Wire(Vec(tableCount, UInt(nfch.W)))
    val tableWriteData = Wire(
        Vec(tableCount, Vec(nfch, UInt(storedEntryWidth.W)))
    )

    def metaPrediction(slot: Int): Bool = {
        val meta = train.meta.slots(slot)
        val providerTaken = meta.providerCounter(counterWidth - 1)
        Mux(
            meta.provider === 0.U,
            train.baseTaken(slot),
            Mux(meta.useAlternate, meta.alternateTaken, providerTaken)
        )
    }

    val mispredicted = VecInit((0 until nfch).map { slot =>
        trainValid && train.trainMask(slot) &&
            metaPrediction(slot) =/= train.takenMask(slot)
    })

    val allocCandidates = Wire(Vec(nfch, UInt(tableCount.W)))
    for (slot <- 0 until nfch) {
        val provider = train.meta.slots(slot).provider
        val candidates = VecInit((0 until tableCount).map { table =>
            val currentEntry = trainTables(table).io.rdata(slot)
            (table + 1).U > provider && entryUseful(currentEntry) === 0.U
        })
        allocCandidates(slot) := candidates.asUInt
    }
    val canAllocate = VecInit((0 until nfch).map { slot =>
        mispredicted(slot) && allocCandidates(slot).orR
    })
    val allocateSlotOH = PriorityEncoderOH(canAllocate.asUInt)
    val allocateTableOH = Wire(UInt(tableCount.W))
    allocateTableOH := 0.U
    for (slot <- 0 until nfch) {
        when(allocateSlotOH(slot)) {
            allocateTableOH := PriorityEncoderOH(allocCandidates(slot))
        }
    }
    val pressureCandidates = VecInit((0 until nfch).map { slot =>
        val hasLongerTable = train.meta.slots(slot).provider < tableCount.U
        mispredicted(slot) && hasLongerTable
    })
    val ageSlotOH = Mux(
        canAllocate.asUInt.orR,
        0.U(nfch.W),
        PriorityEncoderOH(pressureCandidates.asUInt)
    )

    // The duplicated training RAM supplies the current physical row. Prediction
    // metadata chooses the logical provider, but never supplies mutable state.
    val providerUpdateValid = Wire(Vec(tableCount, Vec(nfch, Bool())))
    val providerUpdateData = Wire(
        Vec(tableCount, Vec(nfch, UInt(storedEntryWidth.W)))
    )
    val allocationValid = Wire(Vec(tableCount, Vec(nfch, Bool())))
    val allocationData = Wire(
        Vec(tableCount, Vec(nfch, UInt(storedEntryWidth.W)))
    )
    val usefulAgeValid = Wire(Vec(tableCount, Vec(nfch, Bool())))
    val usefulAgeData = Wire(
        Vec(tableCount, Vec(nfch, UInt(storedEntryWidth.W)))
    )
    for (table <- 0 until tableCount) {
        for (slot <- 0 until nfch) {
            val meta = train.meta.slots(slot)
            val currentEntry = trainTables(table).io.rdata(slot)
            val updatesProvider = trainValid && train.trainMask(slot) &&
                meta.provider === (table + 1).U
            val currentTagMatches = entryValid(currentEntry) &&
                entryTag(currentEntry)(tagWidths(table) - 1, 0) ===
                    trainTags(table)(tagWidths(table) - 1, 0)
            val updateAllowed = updatesProvider && currentTagMatches
            val oldCounter = entryCounter(currentEntry)
            val oldUseful = entryUseful(currentEntry)
            // Usefulness evaluates the direction that actually made the old
            // prediction, while the saturating state update starts from the
            // current table contents read above.
            val predictedProviderTaken =
                meta.providerCounter(counterWidth - 1)
            val alternateTaken = meta.alternateTaken
            val usefulChanges = predictedProviderTaken =/= alternateTaken
            val providerCorrect =
                predictedProviderTaken === train.takenMask(slot)
            val nextUseful = Mux(
                usefulChanges,
                updateCounter(oldUseful, providerCorrect, usefulWidth),
                oldUseful
            )
            val nextCounter = updateCounter(
                oldCounter,
                train.takenMask(slot),
                counterWidth
            )

            providerUpdateValid(table)(slot) := updateAllowed
            providerUpdateData(table)(slot) := packEntry(
                true.B,
                entryTag(currentEntry),
                nextCounter,
                nextUseful
            )

            val allocatesHere = trainValid && allocateSlotOH(slot) &&
                allocateTableOH(table)
            allocationValid(table)(slot) := allocatesHere
            allocationData(table)(slot) := packEntry(
                true.B,
                trainTags(table),
                Mux(train.takenMask(slot), 4.U, 3.U),
                0.U
            )

            // When every longer candidate is useful, pressure from a miss
            // lazily ages those current entries. Repeated pressure therefore
            // reaches zero and cannot permanently freeze allocation.
            val agesHere = trainValid && ageSlotOH(slot) &&
                (table + 1).U > meta.provider &&
                entryUseful(currentEntry) =/= 0.U
            usefulAgeValid(table)(slot) := agesHere
            usefulAgeData(table)(slot) := packEntry(
                entryValid(currentEntry),
                entryTag(currentEntry),
                entryCounter(currentEntry),
                entryUseful(currentEntry) - 1.U
            )
        }
        tableWriteMask(table) := VecInit((0 until nfch).map { slot =>
            providerUpdateValid(table)(slot) ||
                allocationValid(table)(slot) || usefulAgeValid(table)(slot)
        }).asUInt
        for (slot <- 0 until nfch) {
            tableWriteData(table)(slot) := Mux(
                allocationValid(table)(slot),
                allocationData(table)(slot),
                Mux(
                    usefulAgeValid(table)(slot),
                    usefulAgeData(table)(slot),
                    providerUpdateData(table)(slot)
                )
            )
        }
    }

    val useAltOnNaReg = RegInit((1 << (useAltOnNaWidth - 1)).U(
        useAltOnNaWidth.W
    ))
    val useAltTrain = VecInit((0 until nfch).map { slot =>
        val meta = train.meta.slots(slot)
        val weakProvider = meta.providerCounter === 3.U ||
            meta.providerCounter === 4.U
        trainValid && train.trainMask(slot) &&
            meta.provider =/= 0.U && meta.providerUseful === 0.U &&
            weakProvider &&
            (meta.providerCounter(counterWidth - 1) =/= meta.alternateTaken)
    })
    val useAltTrainOH = PriorityEncoderOH(useAltTrain.asUInt)
    when(useAltTrain.asUInt.orR) {
        val actualTaken = Mux1H(useAltTrainOH, train.takenMask.asBools)
        val alternateTaken = Mux1H(
            useAltTrainOH,
            train.meta.slots.map(_.alternateTaken)
        )
        useAltOnNaReg := updateCounter(
            useAltOnNaReg,
            actualTaken === alternateTaken,
            useAltOnNaWidth
        )
    }
    io.useAltOnNa := useAltOnNaReg

    for (table <- 0 until tableCount) {
        val writeMask = tableWriteMask(table)
        val writeValid = writeMask.orR
        val writeAddress = Mux(clearActive, clearIndex, trainIndices(table))
        val writeData = Mux(
            clearActive,
            0.U.asTypeOf(tableWriteData(table)),
            tableWriteData(table)
        )
        val effectiveWriteMask = Mux(
            clearActive,
            Fill(nfch, 1.U(1.W)),
            writeMask
        )

        val lookupRam = lookupTables(table)
        lookupRam.io.ren := true.B
        lookupRam.io.raddr := lookupIndices(table)
        lookupRam.io.wen := clearActive || writeValid
        lookupRam.io.waddr := writeAddress
        lookupRam.io.wdata := writeData
        lookupRam.io.wmask := effectiveWriteMask

        val trainRam = trainTables(table)
        trainRam.io.ren := true.B
        trainRam.io.raddr := trainReadIndices(table)
        trainRam.io.wen := clearActive || writeValid
        trainRam.io.waddr := writeAddress
        trainRam.io.wdata := writeData
        trainRam.io.wmask := effectiveWriteMask
    }

    when(!reset.asBool && trainValid) {
        for (table <- 0 until tableCount) {
            for (slot <- 0 until nfch) {
                when(providerUpdateValid(table)(slot)) {
                    assert(entryValid(trainTables(table).io.rdata(slot)))
                    assert(
                        entryTag(trainTables(table).io.rdata(slot))(
                            tagWidths(table) - 1,
                            0
                        ) === trainTags(table)(tagWidths(table) - 1, 0)
                    )
                }
            }
        }
    }

    // RAM output is stage 2 relative to the existing fast predictor request.
    // Provider/alternate selection is registered once more, making this a
    // shadow s3 result and keeping all tag compares off the s2 fast path.
    val selectionValid = s1Valid && io.base.valid

    val selectedResp = Wire(new MiniTageLookupResp)
    selectedResp := 0.U.asTypeOf(selectedResp)
    selectedResp.pc := io.base.bits.pc
    selectedResp.baseTaken := io.base.bits.baseTaken
    selectedResp.conditionalMask := io.base.bits.conditionalMask

    val candidateTaken = Wire(Vec(nfch, Bool()))
    val providerHit = Wire(Vec(nfch, Bool()))
    for (slot <- 0 until nfch) {
        val entriesByTable = VecInit((0 until tableCount).map { table =>
            lookupTables(table).io.rdata(slot)
        })
        val hits = VecInit((0 until tableCount).map { table =>
            entryValid(entriesByTable(table)) &&
                entryTag(entriesByTable(table))(tagWidths(table) - 1, 0) ===
                    s1Tags(table)(tagWidths(table) - 1, 0)
        })
        val hitMask = hits.asUInt
        val providerOH = Reverse(PriorityEncoderOH(Reverse(hitMask)))
        val alternateHitMask = hitMask & ~providerOH
        val alternateOH = Reverse(
            PriorityEncoderOH(Reverse(alternateHitMask))
        )
        val provider = Mux(
            providerOH.orR,
            OHToUInt(providerOH).pad(tableIdWidth) + 1.U(tableIdWidth.W),
            0.U(tableIdWidth.W)
        )
        val countersByTable = (0 until tableCount).map { table =>
            entryCounter(entriesByTable(table))
        }
        val usefulByTable = (0 until tableCount).map { table =>
            entryUseful(entriesByTable(table))
        }
        val takenByTable = (0 until tableCount).map { table =>
            entryCounter(entriesByTable(table))(counterWidth - 1)
        }
        val baseCounter = Mux(
            io.base.bits.baseTaken(slot),
            4.U(counterWidth.W),
            3.U(counterWidth.W)
        )
        val providerCounter = Mux(
            providerOH.orR,
            Mux1H(providerOH, countersByTable),
            baseCounter
        )
        val providerUseful = Mux(
            providerOH.orR,
            Mux1H(providerOH, usefulByTable),
            0.U(usefulWidth.W)
        )
        val alternateTaken = Mux(
            alternateOH.orR,
            Mux1H(alternateOH, takenByTable),
            io.base.bits.baseTaken(slot)
        )
        val weakProvider = providerCounter === 3.U || providerCounter === 4.U
        val useAlternate = provider =/= 0.U && providerUseful === 0.U &&
            weakProvider && useAltOnNaReg(useAltOnNaWidth - 1)
        val providerTaken = providerCounter(counterWidth - 1)

        candidateTaken(slot) := Mux(
            provider === 0.U,
            io.base.bits.baseTaken(slot),
            Mux(useAlternate, alternateTaken, providerTaken)
        )
        providerHit(slot) := provider =/= 0.U
        selectedResp.meta.slots(slot).provider := provider
        // BPU fills this field with the aligned chooser lookup in s3.
        selectedResp.meta.slots(slot).chooserCounter := 0.U
        selectedResp.meta.slots(slot).providerCounter := providerCounter
        selectedResp.meta.slots(slot).providerUseful := providerUseful
        selectedResp.meta.slots(slot).alternateTaken := alternateTaken
        selectedResp.meta.slots(slot).useAlternate := useAlternate
        for (table <- 0 until tableCount) {
            selectedResp.meta.slots(slot).tableUseful(table) :=
                entryUseful(entriesByTable(table))
        }
    }
    selectedResp.candidateTaken := candidateTaken.asUInt
    selectedResp.providerHitMask := providerHit.asUInt
    selectedResp.correctionMask :=
        (candidateTaken.asUInt ^ io.base.bits.baseTaken) &
            io.base.bits.conditionalMask

    val respValidReg = RegNext(selectionValid, false.B)
    val respBitsReg = RegEnable(selectedResp, selectionValid)
    io.resp.valid := respValidReg
    io.resp.bits := respBitsReg

    when(!reset.asBool) {
        when(io.lookup.valid && io.ready) {
            assert(io.lookup.bits.pc(3, 0) === 0.U)
        }
        when(io.base.valid) {
            assert(s1Valid)
            assert(io.base.bits.pc === s1Pc)
        }
        when(io.train.valid && io.ready) {
            assert(io.train.bits.pc(3, 0) === 0.U)
            assert((io.train.bits.trainMask & ~Fill(nfch, 1.U(1.W))) === 0.U)
            assert(PopCount(allocateSlotOH) <= 1.U)
            assert(PopCount(allocateTableOH) <= 1.U)
        }
    }
}
