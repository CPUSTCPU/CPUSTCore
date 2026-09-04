package CPUSTC.predict

import chisel3._
import chisel3.util._

import CPUSTC.config.Fetch.nfch

object MiniTageChooserConfig {
    val entries = 1024
    val counterWidth = 2
    val initialCounter = 2

    val indexWidth = log2Ceil(entries)
    val rowWidth = nfch * counterWidth

    require(nfch == 4)
    require(isPow2(entries))
    require(initialCounter >= 0 && initialCounter < (1 << counterWidth))
}

class MiniTageChooserLookup extends Bundle {
    val packetPc = UInt(32.W)
}

class MiniTageChooserLookupResp extends Bundle {
    val packetPc = UInt(32.W)
    val counters = Vec(
        nfch,
        UInt(MiniTageChooserConfig.counterWidth.W)
    )
}

class MiniTageChooserTrain extends Bundle {
    val packetPc = UInt(32.W)
    val slot = UInt(log2Ceil(nfch).W)
    val miniCorrect = Bool()
}

class MiniTageChooserIO extends Bundle {
    val lookup = Flipped(Valid(new MiniTageChooserLookup))
    val resp = Valid(new MiniTageChooserLookupResp)
    val train = Flipped(Valid(new MiniTageChooserTrain))
    val ready = Output(Bool())
}

private class MiniTageChooserRam(useBlackBoxRam: Boolean) extends Module {
    import MiniTageChooserConfig._

    val io = IO(new Bundle {
        val ren = Input(Bool())
        val raddr = Input(UInt(indexWidth.W))
        val rdata = Output(UInt(rowWidth.W))
        val wen = Input(Bool())
        val waddr = Input(UInt(indexWidth.W))
        val wdata = Input(UInt(rowWidth.W))
    })

    val ram = Module(new BpuSdpRam(rowWidth, entries, useBlackBoxRam))
    ram.io.ren := io.ren
    ram.io.raddr := io.raddr
    ram.io.wen := io.wen
    ram.io.waddr := io.waddr
    ram.io.wdata := io.wdata

    // Compare only registered command identities. The current lookup address
    // never controls a payload-register CE or reset path.
    val readValidReg = RegNext(io.ren, false.B)
    val readAddressReg = RegNext(io.raddr)
    val writeValidReg = RegNext(io.wen, false.B)
    val writeAddressReg = RegNext(io.waddr)
    val forwardedData = RegNext(io.wdata)
    val collision = readValidReg && writeValidReg &&
        readAddressReg === writeAddressReg
    io.rdata := Mux(collision, forwardedData, ram.io.rdata)
}

/**
  * PC-only confidence chooser between MiniTAGE and the existing fast predictor.
  *
  * The lookup and training copies are kept identical so prediction and one
  * retirement update can proceed every cycle. Training always reads the
  * authoritative physical row before updating one slot; prediction metadata is
  * never used as mutable state.
  */
class MiniTageChooser(useBlackBoxRam: Boolean = false) extends Module {
    import MiniTageChooserConfig._

    val io = IO(new MiniTageChooserIO)

    private val packetPcLow = 4
    private val packetPcWidth = 32 - packetPcLow

    private def index(pc: UInt): UInt = {
        val packetPc = pc(31, packetPcLow)
        val chunks = (0 until packetPcWidth by indexWidth).map { low =>
            val high = math.min(low + indexWidth - 1, packetPcWidth - 1)
            val chunk = packetPc(high, low)
            if (high - low + 1 == indexWidth) chunk else chunk.pad(indexWidth)
        }
        chunks.reduce(_ ^ _)
    }

    private def nextCounter(old: UInt, increment: Bool): UInt = {
        val maximum = ((1 << counterWidth) - 1).U(counterWidth.W)
        Mux(
            increment,
            Mux(old === maximum, old, old + 1.U),
            Mux(old === 0.U, old, old - 1.U)
        )
    }

    private def rowCounters(row: UInt): Vec[UInt] =
        row.asTypeOf(Vec(nfch, UInt(counterWidth.W)))

    private val lookupRam = Module(new MiniTageChooserRam(useBlackBoxRam))
    private val trainRam = Module(new MiniTageChooserRam(useBlackBoxRam))

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

    val initialRow = VecInit.fill(nfch)(
        initialCounter.U(counterWidth.W)
    ).asUInt

    val lookupIndex = index(io.lookup.bits.packetPc)
    val lookupAccepted = io.lookup.valid && !clearActive
    lookupRam.io.ren := lookupAccepted
    lookupRam.io.raddr := lookupIndex

    val lookupRespValid = RegNext(lookupAccepted, false.B)
    val lookupRespPc = RegEnable(io.lookup.bits.packetPc, lookupAccepted)

    val trainIndex = index(io.train.bits.packetPc)
    val trainAccepted = io.train.valid && !clearActive
    trainRam.io.ren := trainAccepted
    trainRam.io.raddr := trainIndex

    val trainRequestValid = RegNext(trainAccepted, false.B)
    val trainRequestIndex = RegEnable(trainIndex, trainAccepted)
    val trainRequestSlot = RegEnable(io.train.bits.slot, trainAccepted)
    val trainRequestMiniCorrect = RegEnable(
        io.train.bits.miniCorrect,
        trainAccepted
    )

    val trainBaseCounters = rowCounters(trainRam.io.rdata)
    val trainNextCounters = WireDefault(trainBaseCounters)
    when(trainRequestValid) {
        trainNextCounters(trainRequestSlot) := nextCounter(
            trainBaseCounters(trainRequestSlot),
            trainRequestMiniCorrect
        )
    }
    val trainNextRow = trainNextCounters.asUInt
    val trainWrite = trainRequestValid

    val tableWriteEnable = clearActive || trainWrite
    val tableWriteAddress = Mux(
        clearActive,
        clearIndex,
        trainRequestIndex
    )
    val tableWriteData = Mux(clearActive, initialRow, trainNextRow)

    lookupRam.io.wen := tableWriteEnable
    lookupRam.io.waddr := tableWriteAddress
    lookupRam.io.wdata := tableWriteData
    trainRam.io.wen := tableWriteEnable
    trainRam.io.waddr := tableWriteAddress
    trainRam.io.wdata := tableWriteData

    io.resp.valid := lookupRespValid
    io.resp.bits.packetPc := lookupRespPc
    io.resp.bits.counters := rowCounters(lookupRam.io.rdata)

    when(!reset.asBool) {
        when(io.lookup.valid && io.ready) {
            assert(io.lookup.bits.packetPc(3, 0) === 0.U)
        }
        when(io.train.valid && io.ready) {
            assert(io.train.bits.packetPc(3, 0) === 0.U)
            assert(io.train.bits.slot < nfch.U)
        }
    }
}
