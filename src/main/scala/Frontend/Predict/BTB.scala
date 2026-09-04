package CPUSTC.predict

import chisel3._
import chisel3.util._
import CPUSTC.config.Predict.BTBMini._
import CPUSTC.config.Fetch._
import CPUSTC.config.JumpOp._
import CPUSTC.config.Predict.Agree.chooserWidth
import CPUSTC.config.Predict.GShare.historyLength
import CPUSTC.config.MuxOH

class BTBMiniEntry extends Bundle {
    val imm      = UInt(30.W) // jump target >> 2, the low 2 bits are always 0
    val predType = UInt(2.W)
    def apply(imm: UInt, predType: UInt): BTBMiniEntry = {
        val entry = Wire(new BTBMiniEntry)
        entry.imm      := imm
        entry.predType := predType
        entry
    }
}

class BtbPredictionMeta extends Bundle {
    val writeWay    = UInt(log2Ceil(way).W)
    val localCtr    = UInt(3.W)
    val predictorCtr = UInt(3.W)
    val bias        = Bool()
    val chooseAgree = UInt(chooserWidth.W)
}

class BtbPacketTrain extends Bundle {
    val basePc    = UInt(32.W)
    val history   = UInt(historyLength.W)
    val trainMask = UInt(nfch.W)
    val takenMask = UInt(nfch.W)
    val predHit   = UInt(nfch.W)
    val writeWay = Vec(nfch, UInt(log2Ceil(way).W))
    val oldCtr   = Vec(nfch, UInt(3.W))
    val oldPredictorCtr = Vec(nfch, UInt(3.W))
    val bias     = Vec(nfch, Bool())
    val oldChooseAgree = Vec(nfch, UInt(chooserWidth.W))
    val target   = Vec(nfch, UInt(30.W))
    val predType = Vec(nfch, UInt(2.W))

    def isConditional(slot: Int): Bool = if (useRamBtb) {
        writeWay(slot)(0)
    } else {
        predType(slot) === BR
    }
}

class BTBMiniTagEntry extends Bundle {
    val tag      = UInt(tagWidth.W)
    val valid    = UInt(nfch.W) // per-bank valid bits

    def apply(tag: UInt, valid: UInt): BTBMiniTagEntry = {
        val entry = Wire(new BTBMiniTagEntry)
        entry.tag    := tag
        entry.valid  := valid
        entry
    }
}

class BTBMiniBPUIO extends Bundle {
    val pc            = Input(UInt(32.W))
    val jumpTgt       = Output(Vec(nfch, UInt(32.W)))
    val predType      = Output(Vec(nfch, UInt(2.W)))
    val rValid        = Output(Vec(nfch, Bool()))
    val jumpCandidate = Output(Vec(nfch, Bool()))
    val rawPredType   = Output(Vec(nfch, UInt(2.W)))
    val rawBias       = Output(Vec(nfch, Bool()))
    val rawLocalCtrMsb = Output(Vec(nfch, Bool()))
    val historyHit    = Output(Vec(nfch, Bool()))
    val rawIsConditional = Output(Vec(nfch, Bool()))
    val meta          = Output(Vec(nfch, new BtbPredictionMeta))
    val initDone      = Output(Bool())

    val updatepc       = Input(UInt(32.W))
    val updatejumpTgt  = Input(UInt(32.W))
    val updatepredType = Input(UInt(2.W))
    val updatejumpEn   = Input(Bool())
    val packetTrain    = Flipped(Valid(new BtbPacketTrain))
}

class BTBMiniIO extends Bundle {
    val bpu = new BTBMiniBPUIO
    val ras = Flipped(new RASBTBMiniIO)
}

abstract class BTBMiniBase extends Module {
    val io = IO(new BTBMiniIO)
}

class BTBMini(useBlackBoxRam: Boolean = false) extends BTBMiniBase {
    private val historyTagWidth = 12
    private val tagHighWidth = tagWidth - historyTagWidth
    private val entryWidth = tagHighWidth + historyTagWidth + 30 + 2 + 3 + 1 + 1 + 1
    private val entryDepth = way * sizePerBank
    private val entryIndexWidth = log2Ceil(entryDepth)
    private val blockPcWidth = 32 - (bankWidth + 2)

    require(nfch == (1 << bankWidth))
    require(way == 2)
    require(isPow2(sizePerBank))
    require(isPow2(entryDepth))
    require(addrWidth == bankWidth + log2Ceil(sizePerBank))
    require(tagHighWidth > 0 && tagHighWidth <= historyTagWidth)

    // Fold every block-PC bit into the physical RAM index. This preserves the
    // original five-bit mapping at 32 entries and scales with larger tables.
    private def fetchIndex(pc: UInt): UInt = {
        val blockPc = pc(31, bankWidth + 2)
        val chunks = (0 until blockPcWidth by entryIndexWidth).map { low =>
            val high = math.min(low + entryIndexWidth - 1, blockPcWidth - 1)
            val chunk = blockPc(high, low)
            if (high - low + 1 == entryIndexWidth) {
                chunk
            } else {
                Cat(0.U((entryIndexWidth - (high - low + 1)).W), chunk)
            }
        }
        chunks.reduce(_ ^ _)
    }
    private def fetchTag(pc: UInt): UInt = pc(31, addrWidth + 2)
    private def tagHigh(tag: UInt): UInt =
        tag(tagWidth - 1, historyTagWidth)
    private def historyTag(tag: UInt): UInt =
        tag(historyTagWidth - 1, 0) ^ tagHigh(tag).pad(historyTagWidth)
    val banks = Seq.tabulate(nfch) { _ =>
        Module(new BpuSdpRam(entryWidth, entryDepth, useBlackBoxRam))
    }

    // Keep validity beside the payload in BRAM. All banks share a one-port
    // startup scrub, so prediction remains disabled until every row is known.
    val clearActive = RegInit(true.B)
    val clearIndex = RegInit(0.U(entryIndexWidth.W))
    val lookupReady = RegNext(!clearActive, false.B)
    io.bpu.initDone := !clearActive
    when(clearActive) {
        when(clearIndex === (entryDepth - 1).U) {
            clearActive := false.B
        }.otherwise {
            clearIndex := clearIndex + 1.U
        }
    }

    val readIndex = fetchIndex(io.bpu.pc)
    val readTag = fetchTag(io.bpu.pc)
    val readTagHighReg = RegNext(tagHigh(readTag))
    val readHistoryTagReg = RegNext(historyTag(readTag))
    val readWayReg = RegNext(readIndex(entryIndexWidth - 1))

    val train = io.bpu.packetTrain
    val trainIndex = fetchIndex(train.bits.basePc)
    val trainTag = fetchTag(train.bits.basePc)
    val trainTagHigh = tagHigh(trainTag)
    val trainHistoryTag = historyTag(trainTag)

    def nextCounter(old: UInt, taken: Bool, hit: Bool): UInt = {
        val incremented = old + (old =/= 7.U)
        val decremented = old - (old =/= 0.U)
        Mux(hit, Mux(taken, incremented, decremented), Mux(taken, 5.U, 2.U))
    }

    val readEntries = Wire(Vec(nfch, new RamBtbEntry))
    val readValids = Wire(Vec(nfch, Bool()))

    for (slot <- 0 until nfch) {
        val ram = banks(slot)
        val writeEntry = Wire(new RamBtbEntry)
        writeEntry.valid    := !clearActive
        writeEntry.tagHigh  := trainTagHigh
        writeEntry.historyTag := trainHistoryTag
        writeEntry.target   := train.bits.target(slot)
        writeEntry.predType := train.bits.predType(slot)
        // The RAM BTB does not consume replacement-way metadata. FTQ reuses
        // that existing training bit to carry the decoded conditional mask.
        writeEntry.isConditional := train.bits.isConditional(slot)
        writeEntry.bias := Mux(
            train.bits.predHit(slot),
            train.bits.bias(slot),
            train.bits.takenMask(slot)
        )
        writeEntry.localCtr := nextCounter(
            train.bits.oldCtr(slot),
            train.bits.takenMask(slot),
            train.bits.predHit(slot)
        )

        val trainThisBank = train.valid && train.bits.trainMask(slot)
        ram.io.ren   := true.B
        ram.io.raddr := readIndex
        ram.io.wen   := clearActive || trainThisBank
        ram.io.waddr := Mux(clearActive, clearIndex, trainIndex)
        ram.io.wdata := writeEntry.asUInt

        readEntries(slot) := ram.io.rdata.asTypeOf(new RamBtbEntry)
        readValids(slot) := lookupReady && readEntries(slot).valid

        val historyHit = readValids(slot) &&
            readEntries(slot).historyTag === readHistoryTagReg
        // With equal high bits, equal folded tags imply equal low bits, so this
        // reconstructs the original full-tag comparison without storing both.
        val hit = historyHit &&
            readEntries(slot).tagHigh === readTagHighReg
        io.bpu.rValid(slot) := hit
        io.bpu.historyHit(slot) := historyHit
        io.bpu.rawPredType(slot) := readEntries(slot).predType
        io.bpu.rawBias(slot) := readEntries(slot).bias
        io.bpu.rawLocalCtrMsb(slot) := readEntries(slot).localCtr(2)
        io.bpu.rawIsConditional(slot) := readEntries(slot).isConditional
        io.bpu.predType(slot) := Mux(hit, readEntries(slot).predType, 0.U)
        io.bpu.jumpTgt(slot) := readEntries(slot).target ## 0.U(2.W)
        io.bpu.meta(slot).writeWay := readWayReg
        io.bpu.meta(slot).localCtr := Mux(hit, readEntries(slot).localCtr, 3.U)
        io.bpu.meta(slot).predictorCtr := Mux(hit, readEntries(slot).localCtr, 3.U)
        io.bpu.meta(slot).bias := Mux(hit, readEntries(slot).bias, false.B)
        io.bpu.meta(slot).chooseAgree := 0.U

        io.bpu.jumpCandidate(slot) := hit && readEntries(slot).localCtr(2)
    }

    io.ras.predType := io.bpu.predType

    when(!reset.asBool) {
        assert(io.bpu.pc(bankWidth + 1, 2) === 0.U,
            "BTB prediction requests must be fetch-block aligned")
        when(train.valid) {
            assert(train.bits.basePc(bankWidth + 1, 2) === 0.U)
            for (slot <- 0 until nfch) {
                when(train.bits.trainMask(slot)) {
                    assert(train.bits.predType(slot) =/= 0.U)
                }
            }
        }
    }
}

class RamBtbEntry extends Bundle {
    val valid    = Bool()
    val tagHigh  = UInt((tagWidth - 12).W)
    val historyTag = UInt(12.W)
    val target   = UInt(30.W)
    val predType = UInt(2.W)
    val isConditional = Bool()
    val bias     = Bool()
    val localCtr = UInt(3.W)
}

class LegacyBTBMini extends BTBMiniBase {
    io.bpu.initDone := true.B

    /* memory units */
    // btb: "way" banks of async-read reg ram, each line holds nfch entries, 1R1W
    val btb = VecInit.fill(way)(
        Module(new CPUSTC.utils.AsyncRegRam(Vec(nfch, new BTBMiniEntry), sizePerBank, 1, 1)).io
    )
    // btbTag: "way" banks of async-read reg ram, 2R (fetch / commit) 1W
    val btbTag = VecInit.fill(way)(
        Module(new CPUSTC.utils.AsyncRegRam(new BTBMiniTagEntry, sizePerBank, 1, 2)).io
    )
    // pht: way x sizePerBank x nfch 3-bit saturating counters, init 3;
    //      the MSB serves as the jumpCandidate hint
    val pht = RegInit(VecInit.fill(way)(
        VecInit.fill(sizePerBank)(VecInit.fill(nfch)(3.U(3.W)))
    ))

    /* address split: | tag | idx | bank | of (pc >> 2) */
    def bank(rIdx: UInt) = rIdx(bankWidth-1, 0)
    def idx(rIdx: UInt)  = rIdx(addrWidth-1, bankWidth)
    def tag(rIdx: UInt)  = rIdx(totalWidth-1, addrWidth)

    /* replacement: free-running counter covering ALL ways */
    val rand = RegInit(0.U(log2Ceil(way).W))
    rand := rand + 1.U

    /* write path (commit side, 2-stage) */
    // stage 1: read the tag ram and judge whether the target is already in the BTB
    val cmtRAddr = io.bpu.updatepc >> 2
    val cmtRIdx  = idx(cmtRAddr)
    btbTag.foreach{ b => b.raddr(1) := cmtRIdx }
    // bypass the in-flight write of the same line to avoid a read-write race
    val cmtRTag   = btbTag.map{ b => Mux(b.wen(0).orR && b.waddr(0) === cmtRIdx, b.wdata(0).tag,   b.rdata(1).tag)  }
    val cmtRValid = VecInit(btbTag.map{ b => Mux(b.wen(0).orR && b.waddr(0) === cmtRIdx, b.wdata(0).valid, b.rdata(1).valid)})
    val cmtRHit   = VecInit.tabulate(way){ i =>
        cmtRValid(i).orR && tag(cmtRAddr) === cmtRTag(i)
    }
    // register the inputs and the hit information towards stage 2
    val cmtJumpTgt  = ShiftRegister(io.bpu.updatejumpTgt >> 2, 1, 0.U, true.B)
    val cmtPC       = ShiftRegister(io.bpu.updatepc >> 2, 1, 0.U, true.B)
    val cmtPredType = ShiftRegister(io.bpu.updatepredType, 1, 0.U, true.B)
    val cmtJumpEn   = ShiftRegister(io.bpu.updatejumpEn, 1, false.B, true.B)
    val cmtWHit     = ShiftRegister(cmtRHit, 1, VecInit.fill(way)(false.B), true.B)
    val cmtWValid   = ShiftRegister(cmtRValid, 1, VecInit.fill(way)(0.U(nfch.W)), true.B)

    // stage 2: write btb / btbTag; hit -> the hit way, miss -> way "rand"
    val cmtAnyHit = cmtWHit.reduce(_ || _)
    val cmtWImm   = cmtJumpTgt
    btb.zipWithIndex.foreach{ case (b, i) =>
        b.waddr(0) := idx(cmtPC)
        b.wdata(0) := VecInit.fill(nfch)((new BTBMiniEntry)(cmtWImm, cmtPredType))
        b.wen(0)   := Mux(cmtPredType === 0.U, 0.U,
            Mux(Mux(cmtAnyHit, cmtWHit(i), rand === i.U), UIntToOH(bank(cmtPC)), 0.U))
    }
    btbTag.zipWithIndex.foreach{ case (b, i) =>
        b.waddr(0) := idx(cmtPC)
        // hit: merge the new bank into the hit way's valid bits;
        // miss: rebuild the line with only the new bank valid
        b.wdata(0) := (new BTBMiniTagEntry)(tag(cmtPC), Mux1H(PriorityEncoderOH(cmtWHit), cmtWValid) | UIntToOH(bank(cmtPC)))
        b.wen(0)   := Mux(cmtPredType === 0.U, false.B, Mux(cmtAnyHit, cmtWHit(i), rand === i.U))
    }

    /* pht update */
    pht.zipWithIndex.foreach{ case (p, i) =>
        when(cmtPredType =/= 0.U){
            val pitem = p(idx(cmtPC))(bank(cmtPC))
            when(cmtWHit(i)){
                // hit: saturating increment / decrement
                pitem := Mux(cmtJumpEn, pitem + (pitem =/= 7.U), pitem - (pitem =/= 0.U))
            }.elsewhen(!cmtAnyHit && rand === i.U){
                // Miss allocation starts weakly in the observed direction.
                pitem := Mux(cmtJumpEn, 5.U, 2.U)
            }
        }
    }

    /* read path (fetch side) */
    val rIdx = io.bpu.pc >> 2
    btb.foreach   { _.raddr(0) := idx(rIdx) }
    btbTag.foreach{ _.raddr(0) := idx(rIdx) }
    val rHit   = VecInit(btbTag.map{ btag => btag.rdata(0).valid.orR && tag(rIdx) === btag.rdata(0).tag })
    val rData  = MuxOH(rHit, btb.map(_.rdata(0)))
    val rValid = MuxOH(rHit, VecInit(btbTag.map{ btag => btag.rdata(0).valid }))

    // shift-align: when the packet head is not aligned to a full line (bank != 0),
    // shift all per-bank outputs right by "bank" so that index i matches slot i
    io.bpu.rValid.zipWithIndex.foreach{ case (r, i) =>
        r := (rValid >> bank(rIdx))(i)
    }
    io.bpu.predType.zipWithIndex.foreach{ case (r, i) =>
        r := Mux(io.bpu.rValid(i),
            (VecInit(rData.map{_.predType}).asUInt >> (2*i)).asTypeOf(Vec(nfch, UInt(2.W)))(bank(rIdx)), 0.U)
    }
    io.bpu.jumpTgt.zipWithIndex.foreach{ case (r, i) =>
        val imm = (VecInit(rData.map{_.imm}).asUInt >> (30*i)).asTypeOf(Vec(nfch, UInt(30.W)))(bank(rIdx))
        // RET takes the target from the RAS; the value is don't-care when invalid
        // (predType is forced to 0 there, so downstream never consumes it)
        r := Mux(io.bpu.predType(i) === RET, io.ras.returnOffset, imm ## 0.U(2.W))
    }

    // Forward predType to the RAS.
    io.ras.predType := io.bpu.predType

    // jumpCandidate: MSB of the hit way's pht counters, qualified by rValid
    val phtBits = Mux1H(rHit, pht.map{ p => VecInit(p(idx(rIdx)).map{_(2)}).asUInt })
    io.bpu.jumpCandidate := ((phtBits >> bank(rIdx)) & io.bpu.rValid.asUInt).asBools

    val selectedPht = Mux1H(rHit, pht.map(p => p(idx(rIdx))))
    io.bpu.meta.zipWithIndex.foreach { case (meta, i) =>
        val ctr =
            (selectedPht.asUInt >> (3 * i)).asTypeOf(Vec(nfch, UInt(3.W)))(bank(rIdx))
        meta.writeWay := Mux(rHit.asUInt.orR, OHToUInt(rHit), rand)
        meta.localCtr := Mux(io.bpu.rValid(i), ctr, 3.U)
        meta.predictorCtr := Mux(io.bpu.rValid(i), ctr, 3.U)
        meta.bias := Mux(io.bpu.rValid(i), ctr(2), false.B)
        meta.chooseAgree := 0.U
    }
    io.bpu.rawPredType := io.bpu.predType
    io.bpu.rawBias := VecInit(io.bpu.meta.map(_.bias))
    io.bpu.rawLocalCtrMsb := io.bpu.jumpCandidate
    io.bpu.historyHit := io.bpu.rValid
    io.bpu.rawIsConditional := VecInit(io.bpu.predType.map(_ === BR))
}
