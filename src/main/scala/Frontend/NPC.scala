package CPUSTC.frontend

import chisel3._
import chisel3.util._
import CPUSTC.utils.BLevelPAdder32
import CPUSTC.config.Fetch._
import CPUSTC.config.Predict._

class NPCCommitIO extends Bundle {
    val flush      = Input(Bool())
    val jumpEn     = Input(Bool())
    val jumpTgt    = Input(UInt(32.W))
}
class NPCPreDecodeIO extends Bundle {
    val flush      = Input(Bool())
    val pc         = Input(UInt(32.W))
    val jumpOffset = Input(UInt(32.W))
}
class NPCFetchQueueIO extends Bundle {
    val ready      = Input(Bool())
}
class NPCFetchIO extends Bundle {
    val pc         = Input(UInt(32.W))
    val npc        = Output(UInt(32.W))
    val validMask  = Output(Vec(nfch, Bool()))
    val currentMask = Output(Vec(nfch, Bool()))
    val sequentialPc = Output(UInt(32.W))
}
class NPCICacheIO extends Bundle {
    val miss       = Input(Bool())
}
class NPCPredictIO extends Bundle {
    val flush      = Input(Bool())
    val pc         = Input(UInt(32.W))
    val jumpOffset = Input(UInt(32.W))
    val predType   = Input(UInt(2.W))
}

class NPCIO extends Bundle {
    val cmt = new NPCCommitIO
    val pd  = new NPCPreDecodeIO
    val fq  = new NPCFetchQueueIO
    val pr  = new NPCPredictIO
    val pc  = new NPCFetchIO
    val ic  = new NPCICacheIO
}
class NPC extends Module {
    val io             = IO(new NPCIO)
    val pc             = WireDefault(io.pc.pc)

    private val fetchBytes = nfch * 4
    private val fetchOffsetBits = log2Ceil(fetchBytes)
    private val fullFetchMask = ((BigInt(1) << nfch) - 1).U(nfch.W)
    private val maskReg = RegInit(fullFetchMask)

    private def alignFetchBlock(addr: UInt): UInt = {
        addr(31, fetchOffsetBits) ## 0.U(fetchOffsetBits.W)
    }

    private def genRedirectMask(addr: UInt): UInt = {
        val wordOffset = addr(fetchOffsetBits - 1, 2)
        MuxLookup(wordOffset, fullFetchMask)(
            (0 until nfch).map { i =>
                i.U -> ((((BigInt(1) << nfch) - 1) << i) & ((BigInt(1) << nfch) - 1)).U(nfch.W)
            }
        )
    }

    val commitTarget = BLevelPAdder32(
        io.cmt.jumpTgt,
        Mux(io.cmt.jumpEn, 0.U, 4.U),
        0.U
    ).io.res
    val predecodeTarget = BLevelPAdder32(io.pd.pc, io.pd.jumpOffset, 0.U).io.res
    val sequentialTarget = BLevelPAdder32(pc, fetchBytes.U, 0.U).io.res

    val nextPc = WireDefault(pc)
    val nextMask = Wire(UInt(nfch.W))
    nextMask := maskReg

    when(io.cmt.flush){
        assert(commitTarget(1, 0) === 0.U, "NPC commit target must be word aligned")
        nextPc := alignFetchBlock(commitTarget)
        nextMask := genRedirectMask(commitTarget)
    }.elsewhen(io.pd.flush){
        assert(predecodeTarget(1, 0) === 0.U, "NPC predecode target must be word aligned")
        nextPc := alignFetchBlock(predecodeTarget)
        nextMask := genRedirectMask(predecodeTarget)
    }.elsewhen(io.pr.flush){
        assert(io.pr.jumpOffset(1, 0) === 0.U, "NPC predictor target must be word aligned")
        nextPc := alignFetchBlock(io.pr.jumpOffset)
        nextMask := genRedirectMask(io.pr.jumpOffset)
    }.elsewhen(io.fq.ready){
        when(io.ic.miss){
            nextPc := pc
            nextMask := maskReg
        }.otherwise{
            nextPc := alignFetchBlock(sequentialTarget)
            nextMask := fullFetchMask
        }
    }.otherwise{
        nextPc := pc
        nextMask := maskReg
    }

    maskReg := nextMask
    io.pc.npc := nextPc
    io.pc.validMask := nextMask.asBools
    io.pc.currentMask := maskReg.asBools
    io.pc.sequentialPc := alignFetchBlock(sequentialTarget)
}
