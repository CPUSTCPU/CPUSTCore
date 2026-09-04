package CPUSTC.backend.execute.fu

import chisel3._
import chisel3.util._

import CPUSTC.config.Branch._
import CPUSTC.config.LookupTreeDefault
import CPUSTC.config.RegisterFile.dataWidth

object PredictedTargetCompare {
    /** JIRL target equality can be moved across the modulo-2^XLEN add:
      * predicted == src1 + imm iff predicted - imm == src1.
      */
    def encode(predictedTarget: UInt, imm: UInt, isJirl: Bool): UInt =
        Mux(isJirl, predictedTarget - imm, predictedTarget)

    def matches(
        encodedPredictedTarget: UInt,
        actualTarget: UInt,
        src1: UInt,
        isJirl: Bool
    ): Bool = Mux(
        isJirl,
        encodedPredictedTarget === src1,
        encodedPredictedTarget === actualTarget
    )

    /** Select a JIRL target match after comparing every possible operand in
      * parallel. This is equivalent to forwarding the 32-bit operand first
      * and comparing afterward when at most one forwarding hit is valid, but
      * it keeps the forwarding tag match out of the wide data/comparator cone.
      */
    def matchesForwardedBase(
        encodedPredictedTarget: UInt,
        residentSrc1: UInt,
        forwardHits: Seq[Bool],
        forwardData: Seq[UInt]
    ): Bool = {
        require(forwardHits.nonEmpty)
        require(forwardHits.length == forwardData.length)

        val forwardedMatches = forwardData.map(
            _ === encodedPredictedTarget
        )
        Mux(
            VecInit(forwardHits).asUInt.orR,
            Mux1H(forwardHits, forwardedMatches),
            residentSrc1 === encodedPredictedTarget
        )
    }
}

class BranchUnitIO extends Bundle {
    val brType = Input(UInt(BR_SZ.W))

    val pc   = Input(UInt(dataWidth.W))
    val src1 = Input(UInt(dataWidth.W))
    val imm  = Input(UInt(dataWidth.W))

    val cmp = Input(new ALUCompare)

    val actualTaken  = Output(Bool())
    val branchTarget = Output(UInt(dataWidth.W))
    val actualNextPc = Output(UInt(dataWidth.W))
}

class BranchUnit extends Module {
    val io = IO(new BranchUnitIO)

    require(dataWidth == 32)

    val actualTaken = LookupTreeDefault(
        io.brType,
        false.B,
        Seq(
            BR_EQ  -> io.cmp.eq,
            BR_NE  -> !io.cmp.eq,
            BR_LT  -> io.cmp.lt,
            BR_GE  -> !io.cmp.lt,
            BR_LTU -> io.cmp.ltu,
            BR_GEU -> !io.cmp.ltu,
            BR_J   -> true.B,
            BR_JR  -> true.B
        )
    )

    val targetBase = Mux(io.brType === BR_JR, io.src1, io.pc)
    val takenTarget = targetBase + io.imm
    val fallThrough = io.pc + 4.U

    io.actualTaken := actualTaken
    io.branchTarget := takenTarget
    io.actualNextPc := Mux(actualTaken, takenTarget, fallThrough)
}
