package CPUSTC.backend.writeback

import chisel3._
import chisel3.util._

import CPUSTC.backend.branch.BranchUpdate
import CPUSTC.config.LoadStoreQueue._
import CPUSTC.config.WritebackConfig._
import CPUSTC.backend.dispatch.LsqLiveState
import CPUSTC.backend.execute.ExecuteResult
import CPUSTC.backend.issue.IssueWakeup
import CPUSTC.backend.regfile.{OperandBypass, RfWriteReq}
import CPUSTC.backend.rename.RenameWakeupInfo
import CPUSTC.backend.rob.{RobPtr, RobWriteback}
import CPUSTC.memory.LoadDebugEvent

class WritebackIO extends Bundle {
    val flush        = Input(Bool())
    val branchUpdate = Flipped(Valid(new BranchUpdate))
    val loadRecovery = Input(Bool())
    val lsqLive      = Input(new LsqLiveState)

    val intResult = Vec(
        nIntWb,
        Flipped(Decoupled(new ExecuteResult))
    )

    // ROB-only completion validity before branch-recovery filtering.  This is
    // intentionally separate from the two-port fast-forward configuration:
    // every integer port may complete a now-unreachable younger ROB slot.
    val fastIntRawValid = Input(Vec(nIntWb, Bool()))

    val loadResult = Vec(
        nLoadWb,
        Flipped(Valid(new CPUSTC.memory.LoadResult))
    )

    val loadDebug = Output(
        Vec(nLoadWb, Valid(new LoadDebugEvent))
    )

    val storeComplete = Input(
        Vec(nStoreComplete, Valid(new RobPtr))
    )

    val rfWrite = Output(
        Vec(nDataWb, Valid(new RfWriteReq))
    )

    val bypass = Output(
        Vec(nDataWb, Valid(new OperandBypass))
    )

    val issueWakeup = Output(
        Vec(nDataWb, Valid(new IssueWakeup))
    )

    val rawLoadWakeup = Output(
        Vec(nLoadWb, Valid(new IssueWakeup))
    )

    val renameWakeup = Output(
        Vec(nDataWb, Valid(new RenameWakeupInfo))
    )

    val robComplete = Output(
        Vec(nRobComplete, Valid(new RobWriteback))
    )
}
