package CPUSTC.backend.regfile

import chisel3._
import chisel3.util._

import CPUSTC.config.RegisterFile._
import CPUSTC.config.Issue._
import CPUSTC.config.FunctionUnit._
import CPUSTC.config.MemIssueOp._
import CPUSTC.config.RenameConfig._
import CPUSTC.config.WritebackConfig._
import CPUSTC.backend.dispatch.DispatchUop
import CPUSTC.backend.issue.IssueOut
import CPUSTC.decode.FuDecodeCtrl
import CPUSTC.backend.execute.{IntExecuteInput, MemExecuteInput}
import CPUSTC.backend.branch.BranchUpdate
import CPUSTC.frontend.FtqPredictionRead
import CPUSTC.backend.AddressFastMap

class RfReadReq extends Bundle {
    val en = Bool()
    val addr = UInt(wpreg.W)
    val speculative = Bool()
}

class RfWriteReq extends Bundle {
    val addr = UInt(wpreg.W)
    val data = UInt(dataWidth.W)
}

class PhysicalRegisterFileIO extends Bundle {
    val readReq  = Input(Vec(nRead, new RfReadReq))
    val readData = Output(Vec(nRead, UInt(dataWidth.W)))

    val write = Input(Vec(nWrite, Valid(new RfWriteReq)))

}

class OperandBypass extends Bundle {
    val pdest = UInt(wpreg.W)
    val data  = UInt(dataWidth.W)
}

class RegisterReadIO extends Bundle {
    val flush = Input(Bool())
    val fastAddressMap = Input(new AddressFastMap)

    val intIssue = Vec(intNissue, Flipped(Decoupled(new IssueOut)))
    val memIssue = Vec(memNissue, Flipped(Decoupled(new IssueOut)))
    val ftqPredictionReadResp = Input(Valid(new FtqPredictionRead))

    val rfReadReq  = Output(Vec(nRead, new RfReadReq))
    val rfReadData = Input(Vec(nRead, UInt(dataWidth.W)))

    val bypass = Input(Vec(nwkp, Valid(new OperandBypass)))

    val intExecute = Vec(intNissue, Decoupled(new IntExecuteInput))
    val memExecute = Vec(memNissue, Decoupled(new MemExecuteInput))

    val intBufferedFuMask = Output(Vec(intNissue, UInt(FUC_SZ.W)))
    val branchUpdate = Flipped(Valid(new BranchUpdate))
}
