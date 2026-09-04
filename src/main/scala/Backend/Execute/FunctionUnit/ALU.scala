package CPUSTC.backend.execute.fu

import chisel3._
import chisel3.util._

import CPUSTC.config.AluOp._
import CPUSTC.config.Execute.FU_OP_SZ
import CPUSTC.config.LookupTreeDefault
import CPUSTC.config.RegisterFile.dataWidth

class ALUCompare extends Bundle {
    val eq  = Bool()
    val lt  = Bool()
    val ltu = Bool()
}

class ALUIO extends Bundle {
    val fn  = Input(UInt(FU_OP_SZ.W))
    val op1 = Input(UInt(dataWidth.W))
    val op2 = Input(UInt(dataWidth.W))

    val result = Output(UInt(dataWidth.W))
    val cmp    = Output(new ALUCompare)
}

class ALU extends Module {
    val io = IO(new ALUIO)

    require(isPow2(dataWidth))

    val isSub  = io.fn === SUB
    val isSlt  = io.fn === SLT
    val isSltu = io.fn === SLTU
    val doSub  = isSub || isSlt || isSltu

    // ADD, SUB, SLT and SLTU share one carry chain. Branch operations are
    // decoded to a subtraction-class ALU operation before reaching this unit.
    val adderOp2 = Mux(doSub, ~io.op2, io.op2)
    val addSubExt =
        Cat(0.U(1.W), io.op1) +
        Cat(0.U(1.W), adderOp2) +
        doSub

    val addSubResult = addSubExt(dataWidth - 1, 0)
    val carryOut     = addSubExt(dataWidth)

    val eq = !(io.op1 ^ io.op2).orR
    val ltu = !carryOut
    val signDiff = io.op1(dataWidth - 1) ^ io.op2(dataWidth - 1)
    val lt = Mux(signDiff, io.op1(dataWidth - 1), addSubResult(dataWidth - 1))

    io.cmp.eq  := eq
    io.cmp.lt  := lt
    io.cmp.ltu := ltu

    val shamtWidth = log2Ceil(dataWidth)
    val shamt      = io.op2(shamtWidth - 1, 0)
    val isLeft     = io.fn === SLL
    val isArith    = io.fn === SRA

    def reverseBits(x: UInt): UInt = {
        Cat((0 until x.getWidth).map(x(_)))
    }

    val shiftInput = Mux(isLeft, reverseBits(io.op1), io.op1)

    // Reversing the input and output lets SLL share the right barrel shifter.
    val shiftedRight = (
        Cat(isArith && shiftInput(dataWidth - 1), shiftInput).asSInt >> shamt
    ).asUInt
    val rightResult = shiftedRight(dataWidth - 1, 0)
    val shiftResult = Mux(isLeft, reverseBits(rightResult), rightResult)

    val sltResult  = Cat(0.U((dataWidth - 1).W), lt)
    val sltuResult = Cat(0.U((dataWidth - 1).W), ltu)

    io.result := LookupTreeDefault(io.fn, 0.U(dataWidth.W), Seq(
        ADD  -> addSubResult,
        SUB  -> addSubResult,
        SLT  -> sltResult,
        SLTU -> sltuResult,
        AND  -> (io.op1 & io.op2),
        OR   -> (io.op1 | io.op2),
        XOR  -> (io.op1 ^ io.op2),
        NOR  -> (~(io.op1 | io.op2)).asUInt,
        ANDN -> (io.op1 & (~io.op2).asUInt),
        ORN  -> (io.op1 | (~io.op2).asUInt),
        SLL  -> shiftResult,
        SRL  -> shiftResult,
        SRA  -> shiftResult
    ))
}
