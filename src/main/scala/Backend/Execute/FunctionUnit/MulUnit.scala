package CPUSTC.backend.execute.fu

import chisel3._
import chisel3.util._
import chisel3.util.HasBlackBoxInline

import CPUSTC.config.Execute.FU_OP_SZ
import CPUSTC.config.MulOp._
import CPUSTC.config.RegisterFile.dataWidth

private class SignedMul17Reg extends BlackBox with HasBlackBoxInline {
    val io = IO(new Bundle {
        val clock   = Input(Clock())
        val enable  = Input(Bool())
        val lhs     = Input(SInt(17.W))
        val rhs     = Input(SInt(17.W))
        val product = Output(SInt(34.W))
    })

    setInline(
        "SignedMul17Reg.sv",
        """
          |module SignedMul17Reg (
          |  input  wire               clock,
          |  input  wire               enable,
          |  input  wire signed [16:0] lhs,
          |  input  wire signed [16:0] rhs,
          |  output wire signed [33:0] product
          |);
          |  (* use_dsp = "yes" *) reg signed [33:0] product_reg;
          |
          |  always @(posedge clock) begin
          |    if (enable)
          |      product_reg <= lhs * rhs;
          |  end
          |
          |  assign product = product_reg;
          |endmodule
          |""".stripMargin
    )
}

class MulUnitIO extends Bundle {
    val load   = Input(Bool())
    val fn     = Input(UInt(FU_OP_SZ.W))
    val src1   = Input(UInt(dataWidth.W))
    val src2   = Input(UInt(dataWidth.W))
    val result = Output(UInt(dataWidth.W))
}

class MulUnit extends Module {
    require(dataWidth == 32)

    val io = IO(new MulUnitIO)

    val signedHigh = io.fn === MULH

    // Encode signedness in the high chunks so MULH does not need a 32-bit
    // absolute-value carry chain in front of the DSP input registers.
    val src1Lo = Cat(0.U(1.W), io.src1(15, 0)).asSInt
    val src1Hi = Cat(signedHigh && io.src1(31), io.src1(31, 16)).asSInt
    val src2Lo = Cat(0.U(1.W), io.src2(15, 0)).asSInt
    val src2Hi = Cat(signedHigh && io.src2(31), io.src2(31, 16)).asSInt

    def mul17Reg(lhs: SInt, rhs: SInt): SInt = {
        val mul = Module(new SignedMul17Reg)
        mul.io.clock  := clock
        mul.io.enable := io.load
        mul.io.lhs    := lhs
        mul.io.rhs    := rhs
        mul.io.product
    }

    val pp00 = mul17Reg(src1Lo, src2Lo)
    val pp01 = mul17Reg(src1Lo, src2Hi)
    val pp10 = mul17Reg(src1Hi, src2Lo)
    val pp11 = mul17Reg(src1Hi, src2Hi)

    val selectLow    = RegEnable(io.fn === MUL, io.load)

    def signExtend64(value: SInt): UInt = {
        val width = value.getWidth
        require(width <= 64)
        Cat(Fill(64 - width, value.asUInt(width - 1)), value.asUInt)
    }

    val term0 = signExtend64(pp00)
    val term1 = (signExtend64(pp01) << 16)(63, 0)
    val term2 = (signExtend64(pp10) << 16)(63, 0)
    val term3 = (signExtend64(pp11) << 32)(63, 0)

    def csa64(a: UInt, b: UInt, c: UInt): (UInt, UInt) = {
        val sum = a ^ b ^ c
        val carryBits = (a & b) | (a & c) | (b & c)
        val carry = Cat(carryBits(62, 0), 0.U(1.W))
        (sum, carry)
    }

    val (sum0, carry0) = csa64(term0, term1, term2)
    val (sum1, carry1) = csa64(sum0, carry0, term3)
    val product = sum1 + carry1
    io.result := Mux(selectLow, product(31, 0), product(63, 32))

    when(io.load) {
        assert(io.fn === MUL || io.fn === MULH || io.fn === MULHU)
    }
}
