package CPUSTC.backend.execute

import chisel3._
import chisel3.util._

import CPUSTC.config.Fetch._
import CPUSTC.config.FunctionUnit._
import CPUSTC.config.RegisterFile._
import CPUSTC.decode.FuDecodeCtrl
import CPUSTC.backend.dispatch.{DispatchUop, LdqPtr, StqPtr}
import CPUSTC.backend.rob.RobPtr
import CPUSTC.backend.{AddressFastMap, AddressTranslationState, CSRDebugState, CSRExceptionInfo}
import CPUSTC.config.MemIssueOp._
import CPUSTC.config.CtrlFlowInstr._
import CPUSTC.config.LoadStoreQueue._
import CPUSTC.config.Memory._
import CPUSTC.config.RenameConfig._
import CPUSTC.config.WritebackConfig._
import CPUSTC.backend.branch.BranchUpdate
import CPUSTC.frontend.FtqPtr
import CPUSTC.memory.{SysMemCmd, SysMemResp}

class ExecuteFuInfo extends Bundle {
    val fuType = UInt(FUC_SZ.W)
}

class CounterDebugEvent extends Bundle {
    val robPtr = new RobPtr
    val value = UInt(64.W)
}

class IntExecuteMeta extends Bundle {
    val pc        = UInt(dataWidth.W)
    val ftqPtr    = new FtqPtr
    val ftqOffset = UInt(log2Ceil(nfch).W)

    val predTaken       = Bool()
    val predTargetValid = Bool()
    val predTarget      = UInt(dataWidth.W)
    val sysAux          = UInt(5.W)
}

class IntExecuteBrInfo extends Bundle {
    val cfiType = UInt(CFI_SZ.W)
    val isCall  = Bool()
    val isRet   = Bool()
}

class IntExecuteRegInfo extends Bundle {
    val psrc1 = UInt(wpreg.W)
    val psrc2 = UInt(wpreg.W)
    val pdest = UInt(wpreg.W)

    val lsrc1Valid = Bool()
    val lsrc2Valid = Bool()
    val ldestValid = Bool()
    val rfWen      = Bool()
}

class IntExecuteSpecInfo extends Bundle {
    val brMask = UInt(maxBrCount.W)
    val brTag  = Valid(UInt(wBrTag.W))
}

class IntExecuteUop extends Bundle {
    val meta = new IntExecuteMeta
    val ctrl = new ExecuteFuInfo
    val br   = new IntExecuteBrInfo
    val reg  = new IntExecuteRegInfo
    val spec = new IntExecuteSpecInfo

    val robPtr = new RobPtr
    val imm    = UInt(dataWidth.W)

    def connectFrom(in: DispatchUop): Unit = {
        meta.pc              := in.meta.pc
        meta.ftqPtr          := in.meta.ftqPtr
        meta.ftqOffset       := in.meta.ftqOffset
        meta.predTaken       := false.B
        meta.predTargetValid := false.B
        meta.predTarget      := 0.U
        meta.sysAux          := in.meta.instr(4, 0)

        ctrl.fuType := in.ctrl.fuType

        br.cfiType := in.br.cfiType
        br.isCall  := in.br.isCall
        br.isRet   := in.br.isRet

        reg.psrc1      := in.reg.psrc1
        reg.psrc2      := in.reg.psrc2
        reg.pdest      := in.reg.pdest
        reg.lsrc1Valid := in.reg.lsrc1Valid
        reg.lsrc2Valid := in.reg.lsrc2Valid
        reg.ldestValid := in.reg.ldestValid
        reg.rfWen      := in.reg.rfWen

        spec.brMask := in.spec.brMask
        spec.brTag  := in.spec.brTag

        robPtr := in.robPtr
        imm    := in.imm
    }
}

class MemExecuteInfo extends Bundle {
    val isLoad    = Bool()
    val isStore   = Bool()
    val memType   = UInt(MEM_TYPE_SZ.W)
    val memSigned = Bool()
}

class MemExecuteRegInfo extends Bundle {
    val psrc1 = UInt(wpreg.W)
    val psrc2 = UInt(wpreg.W)
    val pdest = UInt(wpreg.W)

    val lsrc1Valid = Bool()
    val lsrc2Valid = Bool()
    val rfWen      = Bool()
}

class MemExecuteSpecInfo extends Bundle {
    val brMask = UInt(maxBrCount.W)
}

class MemExecuteUop extends Bundle {
    val ctrl = new ExecuteFuInfo
    val mem  = new MemExecuteInfo
    val reg  = new MemExecuteRegInfo
    val spec = new MemExecuteSpecInfo

    val robPtr = new RobPtr
    val ldqIdx = new LdqPtr
    val stqIdx = new StqPtr

    val stOrderMask = UInt(nstq.W)
    val imm         = UInt(dataWidth.W)

    def connectFrom(in: DispatchUop): Unit = {
        ctrl.fuType := in.ctrl.fuType

        mem.isLoad    := in.mem.isLoad
        mem.isStore   := in.mem.isStore
        mem.memType   := in.mem.memType
        mem.memSigned := in.mem.memSigned

        reg.psrc1      := in.reg.psrc1
        reg.psrc2      := in.reg.psrc2
        reg.pdest      := in.reg.pdest
        reg.lsrc1Valid := in.reg.lsrc1Valid
        reg.lsrc2Valid := in.reg.lsrc2Valid
        reg.rfWen      := in.reg.rfWen

        spec.brMask := in.spec.brMask

        robPtr := in.robPtr
        ldqIdx := in.ldqIdx
        stqIdx := in.stqIdx

        stOrderMask := in.stOrderMask
        imm         := in.imm
    }
}

class IntExecuteInput extends Bundle {
    val uop      = new IntExecuteUop
    val ctrl     = new FuDecodeCtrl
    val src1Data = UInt(dataWidth.W)
    val src2Data = UInt(dataWidth.W)
}

class MemExecuteInput extends Bundle {
    val uop         = new MemExecuteUop
    val memOp       = UInt(MEMQ_SZ.W)
    val operandData = UInt(dataWidth.W)

    // Registered beside the memory PRF read. Only confirmed cached Loads may
    // consume these fields through the direct Load entrance.
    val preVaddr              = UInt(dataWidth.W)
    val preFastPseg           = UInt(3.W)
    val preSizeMask           = UInt(dataBytes.W)
    val preMisaligned         = Bool()
    val preTranslationResolved = Bool()
    val preFastCacheable      = Bool()
    val addrSpeculative       = Bool()
}

class ExecuteResult extends Bundle {
    val robPtr = new RobPtr

    val pdest = UInt(wpreg.W)
    val rfWen = Bool()
    val data  = UInt(dataWidth.W)

    val brMask = UInt(maxBrCount.W)

    val branchResolved = Bool()
    val exceptionValid = Bool()
    val exceptionCause = UInt(8.W)
    val exceptionBadvValid = Bool()
    val exceptionBadv = UInt(dataWidth.W)
}

class BranchResolve extends Bundle {
    val robPtr    = new RobPtr

    val ftqPtr    = new FtqPtr
    val ftqOffset = UInt(log2Ceil(nfch).W)

    val pc           = UInt(dataWidth.W)
    val actualTaken  = Bool()
    val branchTarget = UInt(dataWidth.W)
    val actualNextPc = UInt(dataWidth.W)

    val predTaken  = Bool()
    val mispredict = Bool()
    val directionWrong = Bool()
    val targetWrong    = Bool()

    val cfiType = UInt(CFI_SZ.W)
    val isCall  = Bool()
    val isRet   = Bool()

    val brMask = UInt(maxBrCount.W)
    val brTag  = Valid(UInt(wBrTag.W))
}

class ExecutePortStatus extends Bundle {
    val supportedFuMask = UInt(FUC_SZ.W)

    // Structural input capacity. Branch recovery suppresses valid/fire at the
    // producer and must not feed back through this mask into issue selection.
    val readyFuMask = UInt(FUC_SZ.W)
}

class ExecuteForward extends Bundle {
    val pdest = UInt(wpreg.W)
    val data  = UInt(dataWidth.W)
}

class MemExecutePortIO extends Bundle {
    val flush = Input(Bool())
    val branchUpdate = Flipped(Valid(new BranchUpdate))
    val fastAddressMap = Input(new AddressFastMap)

    val in = Flipped(Decoupled(new MemExecuteInput))
    val fastForward = Input(Vec(nFastIntWb, Valid(new ExecuteForward)))
    val backendInst = Decoupled(new CPUSTC.memory.BackendInst)
    val directCachedLoad = Decoupled(new CPUSTC.memory.DirectCachedLoad)
    val staDependencyReleaseEarly = Output(Valid(new StqPtr))
    val staDependencyRelease = Output(Valid(new StqPtr))
}

class IntExecutePortIO extends Bundle {
    val flush = Input(Bool())
    val counterValue = Input(UInt(64.W))

    val in     = Flipped(Decoupled(new IntExecuteInput))
    val fastForward = Input(Vec(nFastIntWb, Valid(new ExecuteForward)))
    val earlyForward = Output(Valid(new ExecuteForward))
    val operandForward = Output(Valid(new ExecuteForward))
    val robRawValid = Output(Bool())
    val recoveryIndependentReady = Output(Bool())
    val rawWakeup = Output(Valid(UInt(wpreg.W)))
    val counterDebug = Output(Valid(new CounterDebugEvent))
    val result = Decoupled(new ExecuteResult)

    val branchResolve = Output(Valid(new BranchResolve))
    val branchUpdate = Flipped(Valid(new BranchUpdate))

    val status = Output(new ExecutePortStatus)

    val csrCommit = Input(Valid(new RobPtr))
    val csrCommitDone = Output(Bool())
    val sysHeadReq = Output(Valid(new RobPtr))
    val sysHeadGrant = Input(Valid(new RobPtr))
    val sysMemCmd = Decoupled(new SysMemCmd)
    val sysMemResp = Flipped(Decoupled(new SysMemResp))
    val csrException = Flipped(Valid(new CSRExceptionInfo))
    val hardwareInterrupt = Input(UInt(8.W))
    val csrInterruptPending = Output(Bool())
    val csrArchRedirect = Output(Valid(UInt(dataWidth.W)))
    val csrIdle = Output(Bool())
    val csrIdleResumePc = Output(UInt(dataWidth.W))
    val csrBusy = Output(Bool())
    val csrResultFire = Output(Bool())
    val csrAddressState = Output(new AddressTranslationState)
    val csrFastAddressMap = Output(new AddressFastMap)
    val csrDebugState = Output(new CSRDebugState)
    val csrDebugErtn = Output(Bool())
    val csrDebugInterrupt = Output(UInt(11.W))
    val llbitValue = Input(Bool())
    val llbitClear = Output(Bool())
}

class StoreComplete extends Bundle {
    val robPtr = new RobPtr
    val stqIdx = new StqPtr

    val exceptionValid = Bool()
    val exceptionCause = UInt(8.W)
    val exceptionBadvValid = Bool()
    val exceptionBadv = UInt(dataWidth.W)
}
