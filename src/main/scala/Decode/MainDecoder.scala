package CPUSTC.decode

import chisel3._
import chisel3.util._

import CPUSTC.config._
import CPUSTC.config.Consts._
import CPUSTC.config.CtrlFlowInstr._
import CPUSTC.config.Decode._
import CPUSTC.config.EXEOp._
import CPUSTC.config.Fetch._
import CPUSTC.config.FunctionUnit._
import CPUSTC.config.Imm._
import CPUSTC.config.ImplicitCast._
import CPUSTC.config.IssueQueue._
import CPUSTC.config.RegisterFile._
import CPUSTC.config.Memory._
import CPUSTC.config.ExpCode
import CPUSTC.frontend.FtqPtr
import CPUSTC.isa.Instructions._

class DecodeMeta extends Bundle {
    val pc        = UInt(32.W)
    val instr     = UInt(32.W)
    val ftqPtr    = new FtqPtr
    val ftqOffset = UInt(log2Ceil(nfch).W)
    val ftqLast   = Bool()
}

class DecodeRegInfo extends Bundle {
    val lsrc1      = UInt(wlreg.W)
    val lsrc2      = UInt(wlreg.W)
    val ldest      = UInt(wlreg.W)

    val lsrc1Valid = Bool()
    val lsrc2Valid = Bool()
    val ldestValid = Bool()
    
    val rfWen      = Bool()
}

class DecodeCtrlInfo extends Bundle {
  val legal  = Bool()
  val uop    = UInt(OP_SZ.W)
  val iqType = UInt(IQT_SZ.W)
  val fuType = UInt(FUC_SZ.W)

  val immSel = UInt(immSZ.W)
  val imm    = UInt(32.W)

  val exceptionValid = Bool()
  val exceptionCause = UInt(8.W)
  val exceptionBadvValid = Bool()
  val exceptionBadv      = UInt(32.W)
}

class DecodeMemInfo extends Bundle {
    val isLoad  = Bool()
    val isStore = Bool()
    val memType = UInt(MEM_TYPE_SZ.W)
    val memSigned = Bool()
}

class DecodeBrInfo extends Bundle {
    val isBr    = Bool()
    val isBl    = Bool()
    val isJirl  = Bool()
    val isCall  = Bool()
    val isRet   = Bool()
    val cfiType = UInt(CFI_SZ.W)
}

class DecodeOut extends Bundle {
    val meta = new DecodeMeta
    val reg  = new DecodeRegInfo
    val ctrl = new DecodeCtrlInfo
    val mem  = new DecodeMemInfo
    val br   = new DecodeBrInfo
}

class MainDecoderIO extends Bundle {
    val in  = Input(new DecodeMeta)
    val out = Output(new DecodeOut)
}

trait MainDecodeTable {

    def decode_default: List[BitPat] =
    //
    //     is val inst?                                      
    //       |  op                              rfWen          
    //       |   |   iqType                       | isLoad
    //       |   |     |    fuType                |  | isStore
    //       |   |     |      |   ldestValid      |  |  | isBr 
    //       |   |     |      |     |    lsrc1Valid  |  |  |  immSel
    //       |   |     |      |     |    |        |  |  |  |    | 
    //       |   |     |      |     |    | lsrc2Valid|  |  |    | 
    //       |   |     |      |     |    |    |   |  |  |  |    |
        List(N, opX, IQT_X, FU_X,   N,   N,   N,  N, N, N, N, immX)

    val table: Array[(BitPat, List[BitPat])]
}

object DecodeTable extends MainDecodeTable {
    val table: Array[(BitPat, List[BitPat])] = Array(
        //             legal uop          iqType   fuType   ldv r1v r2v rfWen ld st br immSel
        ADDW      -> List(Y,    opADD,       IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),
        SUBW      -> List(Y,    opSUB,       IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),
        SLT       -> List(Y,    opSLT,       IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),
        SLTU      -> List(Y,    opSLTU,      IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),
        NOR       -> List(Y,    opNOR,       IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),
        AND       -> List(Y,    opAND,       IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),
        OR        -> List(Y,    opOR,        IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),
        XOR       -> List(Y,    opXOR,       IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),
        ANDN      -> List(Y,    opANDN,      IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),
        ORN       -> List(Y,    opORN,       IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),

        ADDIW     -> List(Y,    opADDIW,     IQT_INT, FU_ALU, Y,  Y,  N,  Y,    N, N, N, immS12),
        SLTI      -> List(Y,    opSLTI,      IQT_INT, FU_ALU, Y,  Y,  N,  Y,    N, N, N, immS12),
        SLTUI     -> List(Y,    opSLTUI,     IQT_INT, FU_ALU, Y,  Y,  N,  Y,    N, N, N, immS12),
        ANDI      -> List(Y,    opANDI,      IQT_INT, FU_ALU, Y,  Y,  N,  Y,    N, N, N, immU12),
        ORI       -> List(Y,    opORI,       IQT_INT, FU_ALU, Y,  Y,  N,  Y,    N, N, N, immU12),
        XORI      -> List(Y,    opXORI,      IQT_INT, FU_ALU, Y,  Y,  N,  Y,    N, N, N, immU12),

        LU12IW    -> List(Y,    opLU12IW,    IQT_INT, FU_ALU, Y,  N,  N,  Y,    N, N, N, immU20),
        PCADDI    -> List(Y,    opPCADDI,    IQT_INT, FU_JMP, Y,  N,  N,  Y,    N, N, N, immS20),
        PCADDU12I -> List(Y,    opPCADDU12I, IQT_INT, FU_JMP, Y,  N,  N,  Y,    N, N, N, immU20),

        SLLIW     -> List(Y,    opSLLIW,     IQT_INT, FU_ALU, Y,  Y,  N,  Y,    N, N, N, immU5),
        SRLIW     -> List(Y,    opSRLIW,     IQT_INT, FU_ALU, Y,  Y,  N,  Y,    N, N, N, immU5),
        SRAIW     -> List(Y,    opSRAIW,     IQT_INT, FU_ALU, Y,  Y,  N,  Y,    N, N, N, immU5),
        SLLW      -> List(Y,    opSLLW,      IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),
        SRLW      -> List(Y,    opSRLW,      IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),
        SRAW      -> List(Y,    opSRAW,      IQT_INT, FU_ALU, Y,  Y,  Y,  Y,    N, N, N, immX),

        MULW      -> List(Y,    opMULW,      IQT_INT, FU_MUL, Y,  Y,  Y,  Y,    N, N, N, immX),
        MULHW     -> List(Y,    opMULHW,     IQT_INT, FU_MUL, Y,  Y,  Y,  Y,    N, N, N, immX),
        MULHWU    -> List(Y,    opMULHWU,    IQT_INT, FU_MUL, Y,  Y,  Y,  Y,    N, N, N, immX),
        DIVW      -> List(Y,    opDIVW,      IQT_INT, FU_DIV, Y,  Y,  Y,  Y,    N, N, N, immX),
        MODW      -> List(Y,    opMODW,      IQT_INT, FU_DIV, Y,  Y,  Y,  Y,    N, N, N, immX),
        DIVWU     -> List(Y,    opDIVWU,     IQT_INT, FU_DIV, Y,  Y,  Y,  Y,    N, N, N, immX),
        MODWU     -> List(Y,    opMODWU,     IQT_INT, FU_DIV, Y,  Y,  Y,  Y,    N, N, N, immX),

        RDTIMELW  -> List(Y,    opRDTIMELW,  IQT_INT, FU_CNT, Y,  N,  N,  Y,    N, N, N, immX),
        RDCNTVHW  -> List(Y,    opRDCNTVHW,  IQT_INT, FU_CNT, Y,  N,  N,  Y,    N, N, N, immX),

        CSRRD     -> List(Y,    opCSRRD,     IQT_INT, FU_CSR, Y,  N,  N,  Y,    N, N, N, immCSR),
        CSRWR     -> List(Y,    opCSRWR,     IQT_INT, FU_CSR, Y,  Y,  N,  Y,    N, N, N, immCSR),
        CSRXCHG1  -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG2  -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG3  -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG4  -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG5  -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG6  -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG7  -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG8  -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG9  -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG10 -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG11 -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG12 -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG13 -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG14 -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),
        CSRXCHG15 -> List(Y,    opCSRXCHG,   IQT_INT, FU_CSR, Y,  Y,  Y,  Y,    N, N, N, immCSR),

        ERTN      -> List(Y,    opERTN,      IQT_INT, FU_CSR, N,  N,  N,  N,    N, N, N, immX),
        IDLE      -> List(Y,    opIDLE,      IQT_INT, FU_CSR, N,  N,  N,  N,    N, N, N, immX),
        CPUCFG    -> List(Y,    opCPUCFG,    IQT_INT, FU_CSR, Y,  Y,  N,  Y,    N, N, N, immX),
        TLBSRCH   -> List(Y,    opTLBSRCH,   IQT_INT, FU_SYS, N,  N,  N,  N,    N, N, N, immX),
        TLBRD     -> List(Y,    opTLBRD,     IQT_INT, FU_SYS, N,  N,  N,  N,    N, N, N, immX),
        TLBWR     -> List(Y,    opTLBWR,     IQT_INT, FU_SYS, N,  N,  N,  N,    N, N, N, immX),
        TLBFILL   -> List(Y,    opTLBFILL,   IQT_INT, FU_SYS, N,  N,  N,  N,    N, N, N, immX),
        INVTLB0   -> List(Y,    opINVTLB,    IQT_INT, FU_SYS, N,  Y,  Y,  N,    N, N, N, immCID),
        INVTLB1   -> List(Y,    opINVTLB,    IQT_INT, FU_SYS, N,  Y,  Y,  N,    N, N, N, immCID),
        INVTLB2   -> List(Y,    opINVTLB,    IQT_INT, FU_SYS, N,  Y,  Y,  N,    N, N, N, immCID),
        INVTLB3   -> List(Y,    opINVTLB,    IQT_INT, FU_SYS, N,  Y,  Y,  N,    N, N, N, immCID),
        INVTLB4   -> List(Y,    opINVTLB,    IQT_INT, FU_SYS, N,  Y,  Y,  N,    N, N, N, immCID),
        INVTLB5   -> List(Y,    opINVTLB,    IQT_INT, FU_SYS, N,  Y,  Y,  N,    N, N, N, immCID),
        INVTLB6   -> List(Y,    opINVTLB,    IQT_INT, FU_SYS, N,  Y,  Y,  N,    N, N, N, immCID),
        CACOP     -> List(Y,    opCACOP,     IQT_INT, FU_SYS, N,  Y,  N,  N,    N, N, N, immS12),
        PRELD     -> List(Y,    opADD,       IQT_INT, FU_ALU, N,  N,  N,  N,    N, N, N, immX),
        DBAR      -> List(Y,    opDBAR,      IQT_INT, FU_SYS, N,  N,  N,  N,    N, N, N, immX),
        IBAR      -> List(Y,    opIBAR,      IQT_INT, FU_SYS, N,  N,  N,  N,    N, N, N, immX),
        SYSCALL   -> List(Y,    opSYSCALL,   IQT_X,   FU_X,   N,  N,  N,  N,    N, N, N, immX),
        BREAK     -> List(Y,    opBREAK,     IQT_X,   FU_X,   N,  N,  N,  N,    N, N, N, immX),

        LLW       -> List(Y,    opLL,        IQT_MEM, FU_MEM, Y,  Y,  N,  Y,    Y, N, N, immS14),
        SCW       -> List(Y,    opSC,        IQT_INT, FU_SYS, Y,  Y,  Y,  Y,    N, N, N, immS14),
        LDB       -> List(Y,    opLD,        IQT_MEM, FU_MEM, Y,  Y,  N,  Y,    Y, N, N, immS12),
        LDH       -> List(Y,    opLD,        IQT_MEM, FU_MEM, Y,  Y,  N,  Y,    Y, N, N, immS12),
        LDW       -> List(Y,    opLD,        IQT_MEM, FU_MEM, Y,  Y,  N,  Y,    Y, N, N, immS12),
        LDBU      -> List(Y,    opLD,        IQT_MEM, FU_MEM, Y,  Y,  N,  Y,    Y, N, N, immS12),
        LDHU      -> List(Y,    opLD,        IQT_MEM, FU_MEM, Y,  Y,  N,  Y,    Y, N, N, immS12),

        STB       -> List(Y,    opST,        IQT_MEM, FU_MEM, N,  Y,  Y,  N,    N, Y, N, immS12),
        STH       -> List(Y,    opST,        IQT_MEM, FU_MEM, N,  Y,  Y,  N,    N, Y, N, immS12),
        STW       -> List(Y,    opST,        IQT_MEM, FU_MEM, N,  Y,  Y,  N,    N, Y, N, immS12),

        JIRL      -> List(Y,    opJIRL,      IQT_INT, FU_JMP, Y,  Y,  N,  Y,    N, N, N, immS16),
        B         -> List(Y,    opBL,        IQT_INT, FU_JMP, N,  N,  N,  N,    N, N, N, immS26),
        BL        -> List(Y,    opBL,        IQT_INT, FU_JMP, Y,  N,  N,  Y,    N, N, N, immS26),
        BEQ       -> List(Y,    opBEQ,       IQT_INT, FU_JMP, N,  Y,  Y,  N,    N, N, Y, immS16),
        BNE       -> List(Y,    opBNE,       IQT_INT, FU_JMP, N,  Y,  Y,  N,    N, N, Y, immS16),
        BLT       -> List(Y,    opBLT,       IQT_INT, FU_JMP, N,  Y,  Y,  N,    N, N, Y, immS16),
        BGE       -> List(Y,    opBGE,       IQT_INT, FU_JMP, N,  Y,  Y,  N,    N, N, Y, immS16),
        BLTU      -> List(Y,    opBLTU,      IQT_INT, FU_JMP, N,  Y,  Y,  N,    N, N, Y, immS16),
        BGEU      -> List(Y,    opBGEU,      IQT_INT, FU_JMP, N,  Y,  Y,  N,    N, N, Y, immS16),

        NEMU_TRAP -> List(Y,    opADD,       IQT_INT, FU_ALU, N,  N,  N,  N,    N, N, N, immX)
    )
}

class MainDecoder extends Module {
    val io = IO(new MainDecoderIO)

    val instr = io.in.instr
    val rd    = instr(4, 0)
    val rj    = instr(9, 5)
    val rk    = instr(14, 10)

    io.out := 0.U.asTypeOf(new DecodeOut)
    io.out.meta := io.in

    val decoded = DecodeTool(instr, DecodeTable.decode_default, DecodeTable.table)
    
    val rawLegal  = decoded(0)(0)
    val rawUop    = decoded(1)
    val rawFuType = decoded(3)

    val isRdTimeLowGroup = rawLegal && rawUop === opRDTIMELW
    val isCntId = isRdTimeLowGroup && rd === 0.U
    val isCntLow = isRdTimeLowGroup && rd =/= 0.U && rj === 0.U
    val cntEncodingLegal = !isRdTimeLowGroup || isCntId || isCntLow

    val legal = rawLegal && cntEncodingLegal
    val uop = Mux(
        isCntId,
        opRDCNTIDW,
        Mux(isCntLow, opRDCNTVLW, rawUop)
    )
    val fuType = Mux(isCntId, FU_CSR, rawFuType)

    val isLoad  = legal && decoded(8)(0)
    val isStore = legal && decoded(9)(0)
    val isConBr  = legal && decoded(10)(0)
    val isJirl   = legal && uop === opJIRL
    val isDirect = legal && uop === opBL

    val isDirectCall = isDirect && instr(26)
    val isJirlCall   = isJirl && rd === 1.U
    val isRet        = isJirl && rd === 0.U && rj === 1.U && instr(25, 10) === 0.U

    io.out.reg.ldest := Mux(
        isCntId,
        rj,
        Mux(uop === opBL && decoded(4)(0), 1.U, rd)
    )
    val isCsrWrite = uop === opCSRWR || uop === opCSRXCHG
    val isCsrXchg  = uop === opCSRXCHG
    val isSc       = uop === opSC

    io.out.reg.lsrc1 := Mux(isCsrWrite, rd, rj)
    io.out.reg.lsrc2 := Mux(
        isCsrXchg,
        rj,
        Mux(isStore || isConBr || isSc, rd, rk)
    )

    io.out.ctrl.immSel := decoded(11)
    io.out.ctrl.imm    := ImmGen(instr, io.out.ctrl.immSel)

    io.out.ctrl.legal  := legal
    io.out.ctrl.uop    := uop
    io.out.ctrl.iqType := decoded(2)
    io.out.ctrl.fuType := fuType
    io.out.ctrl.exceptionValid := !legal || uop === opSYSCALL || uop === opBREAK
    io.out.ctrl.exceptionCause := Mux(
        !legal,
        ExpCode.INE,
        Mux(uop === opSYSCALL, ExpCode.SYS,
            Mux(uop === opBREAK, ExpCode.BRK, 0.U))
    )
    io.out.ctrl.exceptionBadvValid := false.B
    io.out.ctrl.exceptionBadv      := 0.U

    val rawLdestValid = decoded(4)(0)
    val rawLsrc1Valid = decoded(5)(0)
    val rawLsrc2Valid = decoded(6)(0)
    val rawRfWen      = decoded(7)(0)

    io.out.reg.lsrc1Valid := rawLsrc1Valid
    io.out.reg.lsrc2Valid := rawLsrc2Valid
    io.out.reg.ldestValid := legal && rawLdestValid && io.out.reg.ldest =/= 0.U
    io.out.reg.rfWen      := legal && rawRfWen      && io.out.reg.ldest =/= 0.U

    io.out.mem.isLoad  := isLoad
    io.out.mem.isStore := isStore

    val isAtomicWord = uop === opLL || uop === opSC
    io.out.mem.memType   := Mux(
        isAtomicWord,
        MEM_WORD,
        Mux(io.out.mem.isLoad || io.out.mem.isStore, instr(23, 22), 0.U)
    )
    io.out.mem.memSigned := io.out.mem.isLoad && !instr(25)

    io.out.br.isBr   := isConBr
    io.out.br.isBl   := isDirect
    io.out.br.isJirl := isJirl
    io.out.br.isCall := isDirectCall || isJirlCall
    io.out.br.isRet  := isRet

    io.out.br.cfiType := Mux(isConBr, CFI_BR,
                        Mux(isDirect, CFI_BL,
                        Mux(isJirl,   CFI_JIRL,
                                    CFI_X)))
}
