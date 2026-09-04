package CPUSTC.config

import chisel3._
import chisel3.util._

object RegisterFile{
    val nlreg = 32
    val wlreg = log2Ceil(nlreg)
    val npreg = 64
    val wpreg = log2Ceil(npreg)

    val dataWidth = 32
    val nRead     = 8
    val nWrite    = WritebackConfig.nDataWb

    // Set false to elaborate the original flip-flop-based multiported RF.
    val useLvtPhysicalRegisterFile = true
}

object Fetch{
    val nfch = 4
    val nftq = 16
    val nib = 12
    val icacheLatency = 3
}

object Predict {
        object BTBMini {
            val way         = 2
            val sizePerBank = 64
            val bankWidth   = 2         // log2(nfch)
            val addrWidth   = bankWidth + log2Ceil(sizePerBank)
            val totalWidth  = 30        // width of pc >> 2
            val tagWidth    = totalWidth - addrWidth
            val useRamBtb   = true
        }
        object BIM {
            val entries = 512
            val counterWidth = 3
            val enabled = false
        }
        object GShare {
            val entries = 1024
            val counterWidth = 3
            val historyLength = 12
            val enabled = false
        }
        object Agree {
            val entries = 2048
            val counterWidth = 2
            val chooserWidth = 2
            val chooserInitial = 1
            val enabled = true
        }
        object RAS {
            val size = 8
        }
    }
object Decode{
    val ndcd = 3
    val wdecode = log2Ceil(ndcd)
}

object Commit{
    import Decode._
    val ncmt = 3
    assert(ncmt <= ndcd, "ncmt must be less than or equal to ndcd")
    val nrob = 33
    assert(nrob % ndcd == 0, "nrob must be divisible by ndcd")
    val nrobQ = nrob / ndcd
    val wrob = log2Ceil(nrob)
    val wrobQ = log2Ceil(nrobQ)
    val nbdb = 12
    val nbdbQ = nbdb / ndcd
    val wbdb = log2Ceil(nbdb)
    val wbdbQ = log2Ceil(nbdbQ)
}

object WritebackConfig {
    val nIntWb         = Issue.intNissue
    val nLoadWb        = Issue.memNissue
    val nDataWb        = nIntWb + nLoadWb
    val nFastIntWb     = 2
    val nStoreComplete = Issue.memNissue
    val nRobComplete   = nDataWb + nStoreComplete

    require(nFastIntWb <= nIntWb)
}

object Consts {
    val X = BitPat("b?")
    val Y = BitPat("b1")
    val N = BitPat("b0")

    val BUBBLE = 0.U(32.W)

    val CSR_CRMD      = "h000".U(14.W)
    val CSR_PRMD      = "h001".U(14.W)
    val CSR_EUEN      = "h002".U(14.W)
    val CSR_ECFG      = "h004".U(14.W)
    val CSR_ESTAT     = "h005".U(14.W)
    val CSR_ERA       = "h006".U(14.W)
    val CSR_BADV      = "h007".U(14.W)
    val CSR_EENTRY    = "h00c".U(14.W)
    val CSR_TLBIDX    = "h010".U(14.W)
    val CSR_TLBEHI    = "h011".U(14.W)
    val CSR_TLBELO0   = "h012".U(14.W)
    val CSR_TLBELO1   = "h013".U(14.W)
    val CSR_ASID      = "h018".U(14.W)
    val CSR_PGDL      = "h019".U(14.W)
    val CSR_PGDH      = "h01a".U(14.W)
    val CSR_PGD       = "h01b".U(14.W)
    val CSR_CPUID     = "h020".U(14.W)
    val CSR_SAVE0     = "h030".U(14.W)
    val CSR_SAVE1     = "h031".U(14.W)
    val CSR_SAVE2     = "h032".U(14.W)
    val CSR_SAVE3     = "h033".U(14.W)
    val CSR_TID       = "h040".U(14.W)
    val CSR_TCFG      = "h041".U(14.W)
    val CSR_TVAL      = "h042".U(14.W)
    val CSR_TICLR     = "h044".U(14.W)
    val CSR_LLBCTL    = "h060".U(14.W)
    val CSR_TLBRENTRY = "h088".U(14.W)
    val CSR_CTAG      = "h098".U(14.W)
    val CSR_DMW0      = "h180".U(14.W)
    val CSR_DMW1      = "h181".U(14.W)
}

object ExpCode {
    val INT  = "h00".U(6.W)
    val PIL  = "h01".U(6.W)
    val PIS  = "h02".U(6.W)
    val PIF  = "h03".U(6.W)
    val PME  = "h04".U(6.W)
    val PNR  = "h05".U(6.W)
    val PNX  = "h06".U(6.W)
    val PPI  = "h07".U(6.W)
    val ADEF = "h08".U(6.W)
    val ALE  = "h09".U(6.W)
    val BCE  = "h0a".U(6.W)
    val SYS  = "h0b".U(6.W)
    val BRK  = "h0c".U(6.W)
    val INE  = "h0d".U(6.W)
    val IPE  = "h0e".U(6.W)
    val FPD  = "h0f".U(6.W)
    val FPE  = "h12".U(6.W)
    val TLBR = "h3f".U(6.W)
}

object CtrlFlowInstr {
    val CFI_SZ = 2
    val CFI_X    = 0.U(CFI_SZ.W)
    val CFI_BR   = 1.U(CFI_SZ.W)
    val CFI_BL   = 2.U(CFI_SZ.W)
    val CFI_JIRL = 3.U(CFI_SZ.W)
}

object RenameConfig {
    val maxBrCount = 8
    val wBrTag = log2Ceil(maxBrCount)
    val nwkp = WritebackConfig.nDataWb
}

object Issue{
    val niq          = 2
    val nis          = 5

    val intNiq       = 9
    val intNissue    = 3

    val memNiq       = 6
    val memNissue    = 2
}

object IssueQueue {
    val IQT_SZ  = 1
    val IQT_X   = BitPat.dontCare(IQT_SZ)
    val IQT_INT = 0.U(IQT_SZ.W)
    val IQT_MEM = 1.U(IQT_SZ.W)
}

object MemIssueOp {
    val MEMQ_SZ = 2
    val MEM_X   = 0.U(MEMQ_SZ.W)
    val MEM_LD  = 1.U(MEMQ_SZ.W)
    val MEM_STA = 2.U(MEMQ_SZ.W)
    val MEM_STD = 3.U(MEMQ_SZ.W)
}

object LoadStoreQueue {
    val nldq = 10
    val nstq = 8
    val wldq = log2Ceil(nldq)
    val wstq = log2Ceil(nstq)
}

// Memory-system capacities live here with the rest of the core parameters.
// MemoryConfig.scala exposes compatibility views for existing memory modules.
object MemorySystemConfig {
    object ICache {
        val fetchWidth = Fetch.nfch
        val sets = 64
        val ways = 4
        val lineWords = 16
        val dataBits = RegisterFile.dataWidth
    }

    object DCache {
        val ports = Issue.memNissue
        val sets = 64
        val ways = 2
        val lineWords = 16
        val dataBits = RegisterFile.dataWidth
    }

    object StoreQueue {
        val entries = LoadStoreQueue.nstq
        val enqueueWidth = 4
    }

    object LoadQueue {
        val entries = LoadStoreQueue.nldq
        val enqueueWidth = Issue.memNissue
    }

    object LoadStateTable {
        val entries = LoadStoreQueue.nldq
    }

    object L2Cache {
        val enabled = true
        val sets = 256
        val ways = 4
        val lineBytes = 64
    }

    object Mshr {
        val entries = 4
    }

    object WritebackBuffer {
        val entries = 2
    }

    object AXI {
        val dataBits = RegisterFile.dataWidth
        val idBits = 4
        val icacheMshrBypassLimit = 2
    }
}

object FunctionUnit {
    val FUC_SZ = 8
    val FU_X = BitPat.dontCare(FUC_SZ)
    val FU_ALU = 1.U(FUC_SZ.W)
    val FU_JMP = 2.U(FUC_SZ.W)
    val FU_MEM = 4.U(FUC_SZ.W)
    val FU_MUL = 8.U(FUC_SZ.W)
    val FU_DIV = 16.U(FUC_SZ.W)
    val FU_CSR = 32.U(FUC_SZ.W)
    val FU_CNT = 64.U(FUC_SZ.W)
    val FU_SYS = 128.U(FUC_SZ.W)
}

object Imm {
    val immSZ  = 4
    val immX   = BitPat("b????")
    val immU5  = 0.U(immSZ.W)  // Cat(Fill(27,0.U),inst(14,10))
    val immU12 = 1.U(immSZ.W) // Cat(Fill(20,0.U),inst(21,10))
    val immS12 = 2.U(immSZ.W) //Cat(Fill(20,inst(21)),inst(21,10))
    val immS14 = 3.U(immSZ.W) // Cat(Fill(16,inst(23)),inst(23,10),Fill(2,0.U))
    val immS16 = 4.U(immSZ.W) // Cat(Fill(14,inst(25)),inst(25,10),Fill(2,0.U))
    val immU20 = 5.U(immSZ.W) //Cat(inst(24,5),Fill(12,0.U))
    val immS20 = 6.U(immSZ.W) //Cat(Fill(10,inst(24)),inst(24,5),Fill(2,0.U))
    val immS26 = 7.U(immSZ.W) // Cat(Fill(4,inst(9)),inst(9,0),inst(25,10),Fill(2,0.U))
    val immCSR = 8.U(immSZ.W)
    val immCID = 9.U(immSZ.W)
}

object Branch {
    val BR_SZ  = 4
    val BR_N   = 0.U(4.W) // Next
    val BR_NE  = 1.U(4.W) // Branch on NotEqual
    val BR_EQ  = 2.U(4.W) // Branch on Equal
    val BR_GE  = 3.U(4.W) // Branch on Greater/Equal
    val BR_GEU = 4.U(4.W) // Branch on Greater/Equal Unsigned
    val BR_LT  = 5.U(4.W) // Branch on Less Than
    val BR_LTU = 6.U(4.W) // Branch on Less Than Unsigned
    val BR_J   = 7.U(4.W) // Jump
    val BR_JR  = 8.U(4.W) // Jump Register
}

object PC {
    val PC_PLUS4 = 0.U(2.W)
    val PC_BRJMP = 1.U(2.W)
    val PC_JIRL  = 2.U(2.W)
}

object OPSource {
    val OP1_SZ = 2
    val OP2_SZ = 3

    val OP1_X    = BitPat("b??")
    val OP1_RS1  = 0.U(2.W)
    val OP1_ZERO = 1.U(2.W)
    val OP1_PC   = 2.U(2.W)

    val OP2_X    = BitPat("b???")
    val OP2_RS2  = 0.U(3.W)
    val OP2_IMM  = 1.U(3.W)
    val OP2_ZERO = 2.U(3.W)
    val OP2_NTPC = 3.U(3.W)
}
object JumpOp{
    val NOP     = 0x0.U(2.W)
    val BR      = 0x1.U(2.W)
    val CALL    = 0x2.U(2.W)
    val RET     = 0x3.U(2.W)
}
object EXEOp {
    val OP_SZ = 7
    val opX = BitPat.dontCare(OP_SZ)

    val opNOP = 0.U(OP_SZ.W)
    val opADD = 1.U(OP_SZ.W)
    val opSUB = 2.U(OP_SZ.W)
    val opSLT = 3.U(OP_SZ.W)
    val opSLTU = 4.U(OP_SZ.W)
    val opNOR = 5.U(OP_SZ.W)
    val opAND = 6.U(OP_SZ.W)
    val opOR = 7.U(OP_SZ.W)
    val opXOR = 8.U(OP_SZ.W)
    val opLU12IW = 9.U(OP_SZ.W)
    val opADDIW = 10.U(OP_SZ.W)
    val opSLTI = 11.U(OP_SZ.W)
    val opSLTUI = 12.U(OP_SZ.W)
    val opPCADDI = 13.U(OP_SZ.W)
    val opPCADDU12I = 14.U(OP_SZ.W)
    val opANDN = 15.U(OP_SZ.W)
    val opORN = 16.U(OP_SZ.W)
    val opANDI = 17.U(OP_SZ.W)
    val opORI = 18.U(OP_SZ.W)
    val opXORI = 19.U(OP_SZ.W)
    val opMULW = 20.U(OP_SZ.W)
    val opMULHW = 21.U(OP_SZ.W)
    val opMULHWU = 22.U(OP_SZ.W)
    val opDIVW = 23.U(OP_SZ.W)
    val opMODW = 24.U(OP_SZ.W)
    val opDIVWU = 25.U(OP_SZ.W)
    val opMODWU = 26.U(OP_SZ.W)
    val opSLLIW = 27.U(OP_SZ.W)
    val opSRLIW = 28.U(OP_SZ.W)
    val opSRAIW = 29.U(OP_SZ.W)
    val opSLLW = 30.U(OP_SZ.W)
    val opSRLW = 31.U(OP_SZ.W)
    val opSRAW = 32.U(OP_SZ.W)
    val opJIRL = 33.U(OP_SZ.W)
    val opBL = 34.U(OP_SZ.W)
    val opBEQ = 36.U(OP_SZ.W)
    val opBNE = 37.U(OP_SZ.W)
    val opBLT = 38.U(OP_SZ.W)
    val opBGE = 39.U(OP_SZ.W)
    val opBLTU = 40.U(OP_SZ.W)
    val opBGEU = 41.U(OP_SZ.W)
    val opLD = 42.U(OP_SZ.W)
    val opST = 43.U(OP_SZ.W)
    val opRDTIMELW = 44.U(OP_SZ.W)
    val opRDCNTIDW = 45.U(OP_SZ.W)
    val opRDCNTVLW = 46.U(OP_SZ.W)
    val opRDCNTVHW = 47.U(OP_SZ.W)
    val opCSRRD = 48.U(OP_SZ.W)
    val opCSRWR = 49.U(OP_SZ.W)
    val opCSRXCHG = 50.U(OP_SZ.W)
    val opSYSCALL = 51.U(OP_SZ.W)
    val opBREAK = 52.U(OP_SZ.W)
    val opERTN = 53.U(OP_SZ.W)
    val opIDLE = 54.U(OP_SZ.W)
    val opCPUCFG = 55.U(OP_SZ.W)
    val opLL = 56.U(OP_SZ.W)
    val opSC = 57.U(OP_SZ.W)
    val opTLBSRCH = 58.U(OP_SZ.W)
    val opTLBRD = 59.U(OP_SZ.W)
    val opTLBWR = 60.U(OP_SZ.W)
    val opTLBFILL = 61.U(OP_SZ.W)
    val opINVTLB = 62.U(OP_SZ.W)
    val opCACOP = 63.U(OP_SZ.W)
    val opDBAR = 64.U(OP_SZ.W)
    val opIBAR = 65.U(OP_SZ.W)
}

object Execute {
    val FU_OP_SZ = 4

    case class IntPortParams(
        jmp: Boolean = false,
        csr: Boolean = false,
        system: Boolean = false,
        cnt: Boolean = false,
        mul: Boolean = false,
        div: Boolean = false
    )

    val intPorts = Seq(
        IntPortParams(jmp = true),             // P0: ALU + JMP
        IntPortParams(csr = true, system = true, cnt = true), // P1: ALU + CSR + SYS + CNT
        IntPortParams(mul = true, div = true)  // P2: ALU + MUL + DIV
    )
}

object AluOp {
    import Execute._

    val ADD  = 0.U(FU_OP_SZ.W)
    val SUB  = 1.U(FU_OP_SZ.W)
    val SLT  = 2.U(FU_OP_SZ.W)
    val SLTU = 3.U(FU_OP_SZ.W)
    val AND  = 4.U(FU_OP_SZ.W)
    val OR   = 5.U(FU_OP_SZ.W)
    val XOR  = 6.U(FU_OP_SZ.W)
    val NOR  = 7.U(FU_OP_SZ.W)
    val ANDN = 8.U(FU_OP_SZ.W)
    val ORN  = 9.U(FU_OP_SZ.W)
    val SLL  = 10.U(FU_OP_SZ.W)
    val SRL  = 11.U(FU_OP_SZ.W)
    val SRA  = 12.U(FU_OP_SZ.W)
}

object MulOp {
    import Execute._

    val MUL   = 0.U(FU_OP_SZ.W)
    val MULH  = 1.U(FU_OP_SZ.W)
    val MULHU = 2.U(FU_OP_SZ.W)
}

object DivOp {
    import Execute._

    val DIV  = 0.U(FU_OP_SZ.W)
    val MOD  = 1.U(FU_OP_SZ.W)
    val DIVU = 2.U(FU_OP_SZ.W)
    val MODU = 3.U(FU_OP_SZ.W)
}

object CntOp {
    import Execute._

    val LO = 0.U(FU_OP_SZ.W)
    val HI = 1.U(FU_OP_SZ.W)
}

object CsrOp {
    import Execute._

    val READ  = 0.U(FU_OP_SZ.W)
    val WRITE = 1.U(FU_OP_SZ.W)
    val XCHG  = 2.U(FU_OP_SZ.W)
    val CNTID = 3.U(FU_OP_SZ.W)
    val ERTN  = 4.U(FU_OP_SZ.W)
    val IDLE  = 5.U(FU_OP_SZ.W)
    val CPUCFG = 6.U(FU_OP_SZ.W)
}

object SystemOp {
    import Execute._

    val LL = 0.U(FU_OP_SZ.W)
    val SC = 1.U(FU_OP_SZ.W)
    val TLBSRCH = 2.U(FU_OP_SZ.W)
    val TLBRD = 3.U(FU_OP_SZ.W)
    val TLBWR = 4.U(FU_OP_SZ.W)
    val TLBFILL = 5.U(FU_OP_SZ.W)
    val INVTLB = 6.U(FU_OP_SZ.W)
    val CACOP = 7.U(FU_OP_SZ.W)
    val DBAR = 8.U(FU_OP_SZ.W)
    val IBAR = 9.U(FU_OP_SZ.W)
}

object Memory {
    val MEM_TYPE_SZ = 2
    val dataBytes   = RegisterFile.dataWidth / 8

    val MEM_BYTE = 0.U(MEM_TYPE_SZ.W)
    val MEM_HALF = 1.U(MEM_TYPE_SZ.W)
    val MEM_WORD = 2.U(MEM_TYPE_SZ.W)
}

object MemoryException {
    val EXC_NONE = 0.U(8.W)
    val EXC_PIL  = 1.U(8.W)
    val EXC_PIS  = 2.U(8.W)
    val EXC_PIF  = 3.U(8.W)
    val EXC_PME  = 4.U(8.W)
    val EXC_PPI  = 7.U(8.W)
    val EXC_ADEF = 8.U(8.W)
    // ADE uses ECODE 0x08. Bits 7:6 carry the architectural ESUBCODE while
    // the exception traverses the existing 8-bit writeback/ROB payload.
    val EXC_ADEM = 0x48.U(8.W)
    val EXC_ALE  = 9.U(8.W)
    val EXC_TLBR = 0x3f.U(8.W)
}
