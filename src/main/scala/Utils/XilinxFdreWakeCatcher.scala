package CPUSTC.utils

import chisel3._
import chisel3.util.HasBlackBoxInline

private[CPUSTC] class XilinxFdreWakeCatcher
    extends BlackBox
    with HasBlackBoxInline {

    val io = IO(new Bundle {
        val clock  = Input(Clock())
        val enable = Input(Bool())
        val data   = Input(Bool())
        val reset = Input(Bool())
        val out    = Output(Bool())
    })

    setInline(
        "XilinxFdreWakeCatcher.sv",
        """
          |module XilinxFdreWakeCatcher (
          |  input  wire clock,
          |  input  wire enable,
          |  input  wire data,
          |  input  wire reset,
          |  output wire out
          |);
          |`ifdef SYNTHESIS
          |  FDRE #(
          |    .INIT(1'b0),
          |    .IS_C_INVERTED(1'b0),
          |    .IS_D_INVERTED(1'b0),
          |    .IS_R_INVERTED(1'b0)
          |  ) wake_catcher_fdre (
          |    .Q(out),
          |    .C(clock),
          |    .CE(enable),
          |    .D(data),
          |    .R(reset)
          |  );
          |`else
          |  reg out_reg = 1'b0;
          |
          |  always @(posedge clock) begin
          |    if (reset)
          |      out_reg <= 1'b0;
          |    else if (enable)
          |      out_reg <= data;
          |  end
          |
          |  assign out = out_reg;
          |`endif
          |endmodule
          |""".stripMargin
    )
}

/** Shared, explicit inversion for the catchers' synchronous reset control. */
private[CPUSTC] class XilinxLut1Inverter
    extends BlackBox
    with HasBlackBoxInline {

    val io = IO(new Bundle {
        val in = Input(Bool())
        val out = Output(Bool())
    })

    setInline(
        "XilinxLut1Inverter.sv",
        """
          |module XilinxLut1Inverter (
          |  input  wire in,
          |  output wire out
          |);
          |`ifdef SYNTHESIS
          |  (* DONT_TOUCH = "yes" *)
          |  LUT1 #(
          |    .INIT(2'b01)
          |  ) wake_reset_inverter_lut (
          |    .I0(in),
          |    .O(out)
          |  );
          |`else
          |  assign out = ~in;
          |`endif
          |endmodule
          |""".stripMargin
    )
}
