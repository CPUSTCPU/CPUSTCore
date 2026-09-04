package CPUSTC.decode

import chisel3._
import chisel3.util._

import CPUSTC.config._
import CPUSTC.config.AluOp._
import CPUSTC.config.Branch._
import CPUSTC.config.CntOp._
import CPUSTC.config.CsrOp
import CPUSTC.config.DivOp._
import CPUSTC.config.EXEOp._
import CPUSTC.config.Execute._
import CPUSTC.config.MulOp._
import CPUSTC.config.OPSource._
import CPUSTC.config.SystemOp
import CPUSTC.config.Consts._
import CPUSTC.config.ImplicitCast.uintToBitPat

class FuDecodeCtrl extends Bundle {
    val fuOp   = UInt(FU_OP_SZ.W)
    val op1Sel = UInt(OP1_SZ.W)
    val op2Sel = UInt(OP2_SZ.W)
    val brType = UInt(BR_SZ.W)
}

class FuCtrlSigs extends Bundle {
    val recognized = Bool()
    val ctrl       = new FuDecodeCtrl

    def decode(
        uop: UInt,
        table: Iterable[(BitPat, List[BitPat])]
    ): FuCtrlSigs = {
        val decoded = DecodeTool(uop, AluFuDecode.default, table)
        val signals = Seq(
            recognized,
            ctrl.fuOp,
            ctrl.op1Sel,
            ctrl.op2Sel,
            ctrl.brType
        )

        signals.zip(decoded).foreach { case (signal, value) =>
            signal := value
        }

        this
    }
}

class FuDecoderIO extends Bundle {
    val uop = Input(UInt(OP_SZ.W))
    val out = Output(new FuCtrlSigs)
}

trait FuDecodeConstants {
    val default: List[BitPat] = List[BitPat](
        // recognized | fuOp | op1Sel | op2Sel | brType
        N, AluOp.ADD, OP1_ZERO, OP2_ZERO, BR_N
    )

    protected def row(
        uop: UInt,
        fuOp: UInt,
        op1Sel: UInt,
        op2Sel: UInt,
        brType: UInt
    ): (BitPat, List[BitPat]) = {
        BitPat(uop) -> List(
            Y,
            BitPat(fuOp),
            BitPat(op1Sel),
            BitPat(op2Sel),
            BitPat(brType)
        )
    }

    val table: Array[(BitPat, List[BitPat])]
}

object AluFuDecode extends FuDecodeConstants {
    val table: Array[(BitPat, List[BitPat])] =
        Array[(BitPat, List[BitPat])](
        row(opADD,      AluOp.ADD,  OP1_RS1,  OP2_RS2, BR_N),
        row(opSUB,      AluOp.SUB,  OP1_RS1,  OP2_RS2, BR_N),
        row(opSLT,      AluOp.SLT,  OP1_RS1,  OP2_RS2, BR_N),
        row(opSLTU,     AluOp.SLTU, OP1_RS1,  OP2_RS2, BR_N),
        row(opNOR,      AluOp.NOR,  OP1_RS1,  OP2_RS2, BR_N),
        row(opAND,      AluOp.AND,  OP1_RS1,  OP2_RS2, BR_N),
        row(opOR,       AluOp.OR,   OP1_RS1,  OP2_RS2, BR_N),
        row(opXOR,      AluOp.XOR,  OP1_RS1,  OP2_RS2, BR_N),
        row(opANDN,     AluOp.ANDN, OP1_RS1,  OP2_RS2, BR_N),
        row(opORN,      AluOp.ORN,  OP1_RS1,  OP2_RS2, BR_N),

        row(opADDIW,    AluOp.ADD,  OP1_RS1,  OP2_IMM, BR_N),
        row(opSLTI,     AluOp.SLT,  OP1_RS1,  OP2_IMM, BR_N),
        row(opSLTUI,    AluOp.SLTU, OP1_RS1,  OP2_IMM, BR_N),
        row(opANDI,     AluOp.AND,  OP1_RS1,  OP2_IMM, BR_N),
        row(opORI,      AluOp.OR,   OP1_RS1,  OP2_IMM, BR_N),
        row(opXORI,     AluOp.XOR,  OP1_RS1,  OP2_IMM, BR_N),

        row(opLU12IW,   AluOp.ADD,  OP1_ZERO, OP2_IMM, BR_N),

        row(opSLLIW,    AluOp.SLL,  OP1_RS1,  OP2_IMM, BR_N),
        row(opSRLIW,    AluOp.SRL,  OP1_RS1,  OP2_IMM, BR_N),
        row(opSRAIW,    AluOp.SRA,  OP1_RS1,  OP2_IMM, BR_N),
        row(opSLLW,     AluOp.SLL,  OP1_RS1,  OP2_RS2, BR_N),
        row(opSRLW,     AluOp.SRL,  OP1_RS1,  OP2_RS2, BR_N),
        row(opSRAW,     AluOp.SRA,  OP1_RS1,  OP2_RS2, BR_N)
    )
}

object JumpFuDecode extends FuDecodeConstants {
    val table: Array[(BitPat, List[BitPat])] =
        Array[(BitPat, List[BitPat])](
        row(opBEQ,  AluOp.SUB,  OP1_RS1, OP2_RS2, BR_EQ),
        row(opBNE,  AluOp.SUB,  OP1_RS1, OP2_RS2, BR_NE),
        row(opBLT,  AluOp.SLT,  OP1_RS1, OP2_RS2, BR_LT),
        row(opBGE,  AluOp.SLT,  OP1_RS1, OP2_RS2, BR_GE),
        row(opBLTU, AluOp.SLTU, OP1_RS1, OP2_RS2, BR_LTU),
        row(opBGEU, AluOp.SLTU, OP1_RS1, OP2_RS2, BR_GEU),

        row(opBL,   AluOp.ADD,  OP1_PC,  OP2_NTPC, BR_J),
        row(opJIRL, AluOp.ADD,  OP1_PC,  OP2_NTPC, BR_JR),

        row(opPCADDI,    AluOp.ADD, OP1_PC, OP2_IMM, BR_N),
        row(opPCADDU12I, AluOp.ADD, OP1_PC, OP2_IMM, BR_N)
    )
}

object MulFuDecode extends FuDecodeConstants {
    val table: Array[(BitPat, List[BitPat])] =
        Array[(BitPat, List[BitPat])](
        row(opMULW,   MulOp.MUL,   OP1_RS1, OP2_RS2, BR_N),
        row(opMULHW,  MulOp.MULH,  OP1_RS1, OP2_RS2, BR_N),
        row(opMULHWU, MulOp.MULHU, OP1_RS1, OP2_RS2, BR_N)
    )
}

object DivFuDecode extends FuDecodeConstants {
    val table: Array[(BitPat, List[BitPat])] =
        Array[(BitPat, List[BitPat])](
        row(opDIVW,  DivOp.DIV,  OP1_RS1, OP2_RS2, BR_N),
        row(opMODW,  DivOp.MOD,  OP1_RS1, OP2_RS2, BR_N),
        row(opDIVWU, DivOp.DIVU, OP1_RS1, OP2_RS2, BR_N),
        row(opMODWU, DivOp.MODU, OP1_RS1, OP2_RS2, BR_N)
    )
}

object CntFuDecode extends FuDecodeConstants {
    val table: Array[(BitPat, List[BitPat])] =
        Array[(BitPat, List[BitPat])](
        row(opRDCNTVLW, LO, OP1_ZERO, OP2_ZERO, BR_N),
        row(opRDCNTVHW, HI, OP1_ZERO, OP2_ZERO, BR_N)
    )
}

object CsrFuDecode extends FuDecodeConstants {
    val table: Array[(BitPat, List[BitPat])] =
        Array[(BitPat, List[BitPat])](
        row(opCSRRD,    CsrOp.READ,  OP1_ZERO, OP2_ZERO, BR_N),
        row(opCSRWR,    CsrOp.WRITE, OP1_RS1,  OP2_ZERO, BR_N),
        row(opCSRXCHG,  CsrOp.XCHG,  OP1_RS1,  OP2_RS2,  BR_N),
        row(opRDCNTIDW, CsrOp.CNTID, OP1_ZERO, OP2_ZERO, BR_N),
        row(opERTN,     CsrOp.ERTN,  OP1_ZERO, OP2_ZERO, BR_N),
        row(opIDLE,     CsrOp.IDLE,  OP1_ZERO, OP2_ZERO, BR_N)
    )
}

object SystemFuDecode extends FuDecodeConstants {
    val table: Array[(BitPat, List[BitPat])] =
        Array[(BitPat, List[BitPat])](
        row(opSC,      SystemOp.SC,      OP1_RS1,  OP2_RS2,  BR_N),
        row(opTLBSRCH, SystemOp.TLBSRCH, OP1_ZERO, OP2_ZERO, BR_N),
        row(opTLBRD,   SystemOp.TLBRD,   OP1_ZERO, OP2_ZERO, BR_N),
        row(opTLBWR,   SystemOp.TLBWR,   OP1_ZERO, OP2_ZERO, BR_N),
        row(opTLBFILL, SystemOp.TLBFILL, OP1_ZERO, OP2_ZERO, BR_N),
        row(opINVTLB,  SystemOp.INVTLB,  OP1_RS1,  OP2_RS2,  BR_N),
        row(opCACOP,   SystemOp.CACOP,   OP1_RS1,  OP2_IMM,  BR_N),
        row(opDBAR,    SystemOp.DBAR,    OP1_ZERO, OP2_ZERO, BR_N),
        row(opIBAR,    SystemOp.IBAR,    OP1_ZERO, OP2_ZERO, BR_N)
    )
}

class FuDecoder(params: IntPortParams) extends Module {
    val io = IO(new FuDecoderIO)

    private val emptyTable = Array.empty[(BitPat, List[BitPat])]

    private val table: Array[(BitPat, List[BitPat])] =
        AluFuDecode.table ++
        (if (params.jmp) JumpFuDecode.table else emptyTable) ++
        (if (params.mul) MulFuDecode.table else emptyTable) ++
        (if (params.div) DivFuDecode.table else emptyTable) ++
        (if (params.csr) CsrFuDecode.table else emptyTable) ++
        (if (params.system) SystemFuDecode.table else emptyTable) ++
        (if (params.cnt) CntFuDecode.table else emptyTable)

    val signals = Wire(new FuCtrlSigs)
    signals.decode(io.uop, table)

    io.out := signals

    when(params.csr.B && io.uop === opCPUCFG) {
        io.out.recognized := true.B
        io.out.ctrl.fuOp   := CsrOp.CPUCFG
        io.out.ctrl.op1Sel := OP1_RS1
        io.out.ctrl.op2Sel := OP2_ZERO
        io.out.ctrl.brType := BR_N
    }
}
