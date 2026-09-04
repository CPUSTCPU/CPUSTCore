package CPUSTC.predict

import chisel3._
import chisel3.util._
import CPUSTC.config.Fetch._
import CPUSTC.config.JumpOp._
import CPUSTC.config.Predict.BIM.{counterWidth, entries}

class BIMIO extends Bundle {
    val pc = Input(UInt(32.W))
    val counters = Output(Vec(nfch, UInt(counterWidth.W)))
    val ready = Output(Bool())
    val packetTrain = Flipped(Valid(new BtbPacketTrain))
}

class BIM(useBlackBoxRam: Boolean = false) extends Module {
    val io = IO(new BIMIO)

    private val indexWidth = log2Ceil(entries)
    private val dataWidth = nfch * counterWidth
    private val blockPcLow = log2Ceil(nfch) + 2
    private val blockPcWidth = 32 - blockPcLow

    require(isPow2(entries))

    private def index(pc: UInt): UInt = {
        val blockPc = pc(31, blockPcLow)
        val chunks = (0 until blockPcWidth by indexWidth).map { low =>
            val high = math.min(low + indexWidth - 1, blockPcWidth - 1)
            val chunk = blockPc(high, low)
            if (high - low + 1 == indexWidth) {
                chunk
            } else {
                Cat(0.U((indexWidth - (high - low + 1)).W), chunk)
            }
        }
        chunks.reduce(_ ^ _)
    }

    private def nextCounter(old: UInt, taken: Bool): UInt = {
        val incremented = old + (old =/= ((1 << counterWidth) - 1).U)
        val decremented = old - (old =/= 0.U)
        Mux(taken, incremented, decremented)
    }

    val ram = Module(new BpuSdpRam(dataWidth, entries, useBlackBoxRam))
    val resetActive = RegInit(true.B)
    val resetIndex = RegInit(0.U(indexWidth.W))
    val train = io.packetTrain
    val trainIndex = index(train.bits.basePc)

    val trainData = VecInit((0 until nfch).map { slot =>
        val old = train.bits.oldPredictorCtr(slot)(counterWidth - 1, 0)
        Mux(
            train.bits.trainMask(slot) && train.bits.isConditional(slot),
            nextCounter(old, train.bits.takenMask(slot)),
            old
        )
    }).asUInt
    val initialData = VecInit.fill(nfch)(((1 << (counterWidth - 1)) - 1).U(counterWidth.W)).asUInt

    ram.io.ren := true.B
    ram.io.raddr := index(io.pc)
    ram.io.wen := resetActive || train.valid
    ram.io.waddr := Mux(resetActive, resetIndex, trainIndex)
    ram.io.wdata := Mux(resetActive, initialData, trainData)

    val writeMatchesRead = ram.io.wen && ram.io.waddr === ram.io.raddr
    val writeMatchesReadReg = RegNext(writeMatchesRead, false.B)
    val forwardedWriteData = RegEnable(ram.io.wdata, writeMatchesRead)
    val readData = Mux(writeMatchesReadReg, forwardedWriteData, ram.io.rdata)

    when(resetActive) {
        when(resetIndex === (entries - 1).U) {
            resetActive := false.B
        }.otherwise {
            resetIndex := resetIndex + 1.U
        }
    }

    io.ready := !resetActive
    for (slot <- 0 until nfch) {
        io.counters(slot) := readData(counterWidth * (slot + 1) - 1, counterWidth * slot)
    }
}
