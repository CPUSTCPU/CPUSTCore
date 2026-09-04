package CPUSTC.backend.rob

import chisel3._
import chisel3.util._

import CPUSTC.config.RegisterFile._
import CPUSTC.config.Fetch._
import CPUSTC.config.Decode._
import CPUSTC.config.Commit._
import CPUSTC.config.WritebackConfig._
import CPUSTC.config.MemoryException._
import CPUSTC.config.EXEOp._
import CPUSTC.config.FunctionUnit._
import CPUSTC.backend.rename.RenameCommitInfo
import CPUSTC.backend.branch.BranchUpdate
import CPUSTC.frontend.FtqPtr

object RobConfig {
    val epochBits = 4
}

class RobPtr extends Bundle {
    val qidx   = UInt(wdecode.W)
    val offset = UInt(wrobQ.W)
    val high   = Bool()
    val epoch  = UInt(RobConfig.epochBits.W)
}

class RobEntry extends Bundle {
    val valid    = Bool()
    val complete = Bool()
    val ptrHigh  = Bool()
    val epoch    = UInt(RobConfig.epochBits.W)

    val exceptionValid = Bool()
    val exceptionCause = UInt(8.W)
    val exceptionBadvValid = Bool()
    val exceptionBadv      = UInt(dataWidth.W)
    val data           = UInt(dataWidth.W)

    val pc        = UInt(32.W)
    val instr     = UInt(32.W)
    val ftqPtr    = new FtqPtr
    val ftqOffset = UInt(log2Ceil(nfch).W)
    val ftqLast   = Bool()

    val uop    = UInt(OP_SZ.W)
    val fuType = UInt(FUC_SZ.W)

    val ldest      = UInt(wlreg.W)
    val pdest      = UInt(wpreg.W)
    val pprd       = UInt(wpreg.W)
    val ldestValid = Bool()
    val rfWen      = Bool()

    val isLoad  = Bool()
    val isStore = Bool()
    val isBr    = Bool()
    val isBl    = Bool()
    val isJirl  = Bool()
    val branchResolved = Bool()

    def enqueue(data: Data): Unit = {
        val x = data.asInstanceOf[RobEntry]

        this := x
        this.valid          := true.B
        this.complete       := x.complete
        this.exceptionValid := x.exceptionValid
        this.exceptionCause := Mux(x.exceptionValid, x.exceptionCause, EXC_NONE)
        this.exceptionBadvValid := x.exceptionValid && x.exceptionBadvValid
        this.exceptionBadv := Mux(
            x.exceptionValid && x.exceptionBadvValid,
            x.exceptionBadv,
            0.U
        )
    }

    def write(data: Data): Unit = {
        val x = data.asInstanceOf[RobEntry]

        this.complete       := x.complete
        this.exceptionValid := x.exceptionValid
        this.exceptionCause := x.exceptionCause
        this.exceptionBadvValid := x.exceptionValid && x.exceptionBadvValid
        this.exceptionBadv := Mux(
            x.exceptionValid && x.exceptionBadvValid,
            x.exceptionBadv,
            0.U
        )
        this.data           := x.data
        when(x.branchResolved) {
            this.branchResolved := true.B
        }
    }
}

class RobEnqIO extends Bundle {
    val canAccept = Output(Bool())
    val req       = Input(Vec(ndcd, Valid(new RobEntry)))
    val resp      = Output(Vec(ndcd, Valid(new RobPtr)))
}

class RobWriteback extends Bundle {
    val robPtr = new RobPtr

    // Marks a clean integer completion whose validity is independent of
    // branch-recovery filtering.  Exceptional results use the staged path.
    val fastEligible = Bool()

    val exceptionValid = Bool()
    val exceptionCause = UInt(8.W)
    val exceptionBadvValid = Bool()
    val exceptionBadv = UInt(dataWidth.W)
    val data           = UInt(dataWidth.W)

    val branchResolved = Bool()
}

class RobExceptionWriteback extends Bundle {
    val robPtr = new RobPtr
    val cause = UInt(8.W)
    val badvValid = Bool()
    val badv = UInt(dataWidth.W)
}

class RobCommitEntry extends Bundle {
    val robPtr    = new RobPtr
    val pc        = UInt(32.W)
    val instr     = UInt(32.W)
    val data      = UInt(dataWidth.W)

    val ftqPtr  = new FtqPtr
    val ftqOffset = UInt(log2Ceil(nfch).W)
    val ftqLast = Bool()

    val ldest      = UInt(wlreg.W)
    val pdest      = UInt(wpreg.W)
    val pprd       = UInt(wpreg.W)
    val ldestValid = Bool()
    val rfWen      = Bool()

    val isLoad  = Bool()
    val isStore = Bool()
    val isBr    = Bool()
    val isBl    = Bool()
    val isJirl  = Bool()
    val uncache = Bool()
    val uop = UInt(OP_SZ.W)
    val fuType = UInt(FUC_SZ.W)
    val commitBoundary = Bool()
}

class RobTrap extends Bundle {
    val robPtr = new RobPtr
    val pc = UInt(dataWidth.W)
    val instr = UInt(dataWidth.W)
    val exceptionCause = UInt(8.W)
    val exceptionBadvValid = Bool()
    val exceptionBadv      = UInt(dataWidth.W)
}

class RobP0HandoffStatus extends Bundle {
    // Clean load result reaches an incomplete ROB head through the P0 path.
    val event = Bool()
    val commitBlocked = Bool()
    val noNext = Bool()
    val nextReady = Bool()
    val nextWaitInt = Bool()
    val nextWaitLoad = Bool()
    val nextWaitStore = Bool()
    val nextWaitBranch = Bool()
    val nextWaitOther = Bool()

    // P0 retires the completed head on the following cycle.
    val retireOne = Bool()
    val retireWide = Bool()
    val retireDeferred = Bool()
}

class RobStatus extends Bundle {
    val headValid    = Bool()
    val headComplete = Bool()
    val headIsLoad   = Bool()
    val headIsStore  = Bool()
    val headIsBranch = Bool()
    val headRobPtr = new RobPtr
    val headPc = UInt(dataWidth.W)
    val p0Handoff = new RobP0HandoffStatus
}

class RobIO extends Bundle {
    val flush = Input(Bool())
    val commitBlock = Input(Bool())
    val enq = new RobEnqIO
    val wb  = Input(Vec(nRobComplete, Valid(new RobWriteback)))
    val storeException = Input(Vec(
        nStoreComplete,
        Valid(new RobExceptionWriteback)
    ))
    val cmt = Output(Vec(ncmt, Valid(new RobCommitEntry)))
    val rnmCmt = Output(Vec(ncmt, Valid(new RenameCommitInfo)))
    val retireMask = Output(UInt(ncmt.W))
    val loadCommit = Output(Vec(ncmt, Bool()))
    val storeCommit = Output(Valid(Bool()))
    val boundaryCommit = Output(Bool())
    val branchUpdate = Flipped(Valid(new BranchUpdate))

    val empty = Output(Bool())
    val full  = Output(Bool())
    val occupancy = Output(UInt(log2Ceil(nrob + 1).W))
    val status = Output(new RobStatus)
    val trap = Output(Valid(new RobTrap))
}
