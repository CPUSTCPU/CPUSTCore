package CPUSTC.backend

import chisel3._
import chisel3.util._

import CPUSTC.config.Commit._
import CPUSTC.config.Decode._
import CPUSTC.config.Fetch._
import CPUSTC.config.Issue._
import CPUSTC.config.RegisterFile._
import CPUSTC.config.WritebackConfig._
import CPUSTC.config.LoadStoreQueue._
import CPUSTC.backend.control.PipelineRedirect
import CPUSTC.decode.DecodeOut
import CPUSTC.backend.dispatch.{LsqLiveState, StqPtr}
import CPUSTC.memory.{
    LoadDebugEvent,
    LoadResult,
    StoreCompletionToken,
    StoreExceptionEvent,
    SysMemCmd,
    SysMemResp
}
import CPUSTC.frontend.{FtqPredictionRead, FtqPtr, FtqRetire}
import CPUSTC.backend.rob.RobCommitEntry
import CPUSTC.backend.execute.CounterDebugEvent
import CPUSTC.perf.BackendPerfEvents

class BackendIO(
    enableCommitDebug: Boolean = false,
    enablePerfCounters: Boolean = false
) extends Bundle {
    val hardRedirect = Flipped(Valid(UInt(dataWidth.W)))
    val hardwareInterrupt = Input(UInt(8.W))
    val counterValue = Input(UInt(64.W))

    // Decode -> Rename
    val decode = Vec(ndcd, Flipped(Decoupled(new DecodeOut)))

    // Memory接口
    val memRequest = Vec(
        memNissue,
        Decoupled(new CPUSTC.memory.BackendInst)
    )
    val directCachedLoad = Vec(
        memNissue,
        Decoupled(new CPUSTC.memory.DirectCachedLoad)
    )
    val loadResult = Vec(
        nLoadWb,
        Flipped(Valid(new LoadResult))
    )
    val loadPredWake = Input(Vec(
        memNissue,
        Valid(new CPUSTC.memory.LoadPredictInfo)
    ))
    val loadPredResolve = Input(Vec(
        memNissue,
        Valid(new CPUSTC.memory.LoadPredictResolve)
    ))
    val storeComplete = Input(Vec(
        CPUSTC.memory.StoreQueueConfig.EnqNum,
        Valid(new StoreCompletionToken)
    ))
    val storeException = Input(Vec(
        CPUSTC.memory.LoadQueueConfig.EnqNum,
        Valid(new StoreExceptionEvent)
    ))
    val stqFreed = Flipped(Valid(UInt(nstq.W)))
    val stqCommitPtr = Input(new StqPtr)
    val stqCommittedMask = Input(UInt(nstq.W))
    // The memory side uses the ROB identity only for performance attribution;
    // uncache issue still depends solely on valid.
    val robHeadLoad = Output(Valid(new CPUSTC.memory.RobHeadLoadInfo))

    val sysMemCmd  = Decoupled(new SysMemCmd)
    val sysMemResp = Flipped(Decoupled(new SysMemResp))
    val addressState = Output(new AddressTranslationState)
    val llbitValue = Input(Bool())
    val llbitClear = Output(Bool())
    val llCommit = Output(Valid(new CPUSTC.backend.rob.RobPtr))

    // Registered, irrevocable ROB retirement broadcast.
    val commit = Output(Vec(ncmt, Valid(new RobCommitEntry)))
    val storeCommit = Output(Valid(Bool()))

    val commitData = if (enableCommitDebug) {
        Some(Output(Vec(ncmt, UInt(dataWidth.W))))
    } else {
        None
    }

    val ftqRetire = Output(Valid(new FtqRetire))
    val ftqPredictionReadReq = Output(Valid(new FtqPtr))
    val ftqPredictionReadResp = Input(Valid(new FtqPredictionRead))
    val stqHeadCurrent = Output(new CPUSTC.backend.dispatch.StqPtr)
    val ldqFlushMask = Output(UInt(nldq.W))
    val stqFlushMask = Output(UInt(nstq.W))

    val redirect = Output(Valid(new PipelineRedirect))
    val fullFlush = Output(Bool())
    val memoryStateFlush = Output(Bool())
    val idle = Output(Bool())
    val branchRecoveryFlush = Output(Bool())
    val exceptionTrace = Output(Valid(new CSRExceptionInfo))
    val csrDebugState = Output(new CSRDebugState)
    val csrDebugErtn = Output(Bool())
    val csrDebugInterrupt = Output(UInt(11.W))
    val counterDebug = Output(Vec(intNissue, Valid(new CounterDebugEvent)))
    val loadDebug = Output(Vec(nLoadWb, Valid(new LoadDebugEvent)))
    val lsqLive = Output(new LsqLiveState)

    val perf = if (enablePerfCounters) {
        Some(Output(new BackendPerfEvents))
    } else {
        None
    }
}
