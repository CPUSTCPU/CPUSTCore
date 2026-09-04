package CPUSTC.predict

import chisel3._
import chisel3.util._

import CPUSTC.config.Fetch._
import CPUSTC.config.JumpOp._
import CPUSTC.config.Predict.Agree.{chooserInitial, chooserWidth, counterWidth, entries}
import CPUSTC.config.Predict.GShare.historyLength

class AgreeIO extends Bundle {
    val pc = Input(UInt(32.W))
    val history = Input(UInt(historyLength.W))
    val counters = Output(Vec(nfch, UInt(counterWidth.W)))
    val chooseAgree = Output(Vec(nfch, UInt(chooserWidth.W)))
    val ready = Output(Bool())
    val packetTrain = Flipped(Valid(new BtbPacketTrain))
}

class Agree(useBlackBoxRam: Boolean = false) extends Module {
    val io = IO(new AgreeIO)

    private val indexWidth = log2Ceil(entries)
    private val slotWidth = counterWidth + chooserWidth
    private val dataWidth = nfch * slotWidth
    private val blockPcLow = log2Ceil(nfch) + 2
    private val blockPcWidth = 32 - blockPcLow

    require(isPow2(entries))
    require(historyLength > 1)

    private def foldPc(pc: UInt): UInt = {
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

    private def foldHistory(history: UInt): UInt = {
        VecInit((0 until indexWidth).map { bit =>
            val extra11 = if (bit == 8) history(11) else false.B
            history(bit) ^ extra11
        }).asUInt
    }

    private def index(pc: UInt, history: UInt): UInt =
        foldPc(pc) ^ foldHistory(history)

    private def nextCounter(old: UInt, increment: Bool, width: Int): UInt = {
        val maxValue = ((1 << width) - 1).U(width.W)
        val incremented = Mux(old === maxValue, old, old + 1.U)
        val decremented = Mux(old === 0.U, old, old - 1.U)
        Mux(increment, incremented, decremented)
    }

    private def packSlot(counter: UInt, chooseAgree: UInt): UInt =
        Cat(chooseAgree, counter)

    val ram = Module(new AgreeWriteFirstRam(dataWidth, entries, useBlackBoxRam))
    val resetActive = RegInit(true.B)
    val resetIndex = RegInit(0.U(indexWidth.W))
    val train = io.packetTrain
    val trainIndex = index(train.bits.basePc, train.bits.history)

    val trainData = VecInit((0 until nfch).map { slot =>
        val oldCounter = train.bits.oldPredictorCtr(slot)(counterWidth - 1, 0)
        val trainThisSlot =
            train.bits.trainMask(slot) && train.bits.isConditional(slot)
        val agrees = !train.bits.predHit(slot) ||
            (train.bits.takenMask(slot) === train.bits.bias(slot))
        val nextCounterValue = Mux(
            trainThisSlot,
            nextCounter(oldCounter, agrees, counterWidth),
            oldCounter
        )
        val localDirection = train.bits.oldCtr(slot)(
            train.bits.oldCtr(slot).getWidth - 1
        )
        val agreeDirection =
            oldCounter(counterWidth - 1) === train.bits.bias(slot)
        val predictionsDisagree =
            trainThisSlot && train.bits.predHit(slot) &&
                localDirection =/= agreeDirection
        val oldChooser = train.bits.oldChooseAgree(slot)
        val nextChooseAgree = Mux(
            predictionsDisagree,
            nextCounter(
                oldChooser,
                agreeDirection === train.bits.takenMask(slot),
                chooserWidth
            ),
            oldChooser
        )
        packSlot(nextCounterValue, nextChooseAgree)
    }).asUInt
    val initialData = VecInit.fill(nfch)(
        packSlot(
            (1 << (counterWidth - 1)).U(counterWidth.W),
            chooserInitial.U(chooserWidth.W)
        )
    ).asUInt

    ram.io.raddr := index(io.pc, io.history)
    ram.io.wen := resetActive || train.valid
    ram.io.waddr := Mux(resetActive, resetIndex, trainIndex)
    ram.io.wdata := Mux(resetActive, initialData, trainData)

    val readData = ram.io.rdata

    when(resetActive) {
        when(resetIndex === (entries - 1).U) {
            resetActive := false.B
        }.otherwise {
            resetIndex := resetIndex + 1.U
        }
    }

    io.ready := !resetActive
    for (slot <- 0 until nfch) {
        val slotLow = slot * slotWidth
        io.counters(slot) := readData(
            slotLow + counterWidth - 1,
            slotLow
        )
        io.chooseAgree(slot) := readData(
            slotLow + counterWidth + chooserWidth - 1,
            slotLow + counterWidth
        )
    }
}
