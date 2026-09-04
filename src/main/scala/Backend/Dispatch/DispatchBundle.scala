package CPUSTC.backend.dispatch

import chisel3._
import chisel3.util._

import CPUSTC.config.Decode._
import CPUSTC.config.Issue._
import CPUSTC.config.IssueQueue._
import CPUSTC.config.LoadStoreQueue._
import CPUSTC.config.RegisterFile._
import CPUSTC.decode._
import CPUSTC.backend.rename._
import CPUSTC.backend.rob._
import CPUSTC.config.RenameConfig._

class StqPtr extends Bundle {
    val oh    = UInt(nstq.W)
    val flag  = Bool()
}

class LdqPtr extends Bundle {
    val oh    = UInt(nldq.W)
    val flag  = Bool()
}

class DispatchUop extends Bundle {
    val meta = new DecodeMeta
    val ctrl = new DecodeCtrlInfo
    val mem  = new DecodeMemInfo
    val br   = new DecodeBrInfo
    val reg  = new RenameRegInfo
    val spec = new RenameSpecInfo

    val robPtr = new RobPtr
    val ldqIdx = new LdqPtr
    val stqIdx = new StqPtr

    val stDepMask = UInt(nstq.W)
    val stOrderMask = UInt(nstq.W)

    val imm = UInt(32.W)
}

class LsqAllocReq extends Bundle {
    val isLoad  = Bool()
    val isStore = Bool()
}

class LsqAllocResp extends Bundle {
    val ldqIdx    = new LdqPtr
    val stqIdx    = new StqPtr
    val stDepMask = UInt(nstq.W)
    val stOrderMask = UInt(nstq.W)
}

class LsqDispatchIO extends Bundle {
    val req        = Input(Vec(ndcd, Valid(new LsqAllocReq)))
    val doAllocate = Input(Bool())

    val canAccept  = Output(Bool())
    val resp       = Output(Vec(ndcd, Valid(new LsqAllocResp)))

    val brSnapshotReqs = Input(Vec(ndcd, Valid(UInt(wBrTag.W))))
}

class IssueDispatchIO(enqWidth: Int) extends Bundle {
    val reqCountOH = Input(UInt((enqWidth + 1).W))
    val canAccept  = Output(Bool())
    val enq        = Input(Vec(enqWidth, Valid(new DispatchUop)))
    val src1IntProducerOH = Input(Vec(enqWidth, UInt(intNiq.W)))
    val src2IntProducerOH = Input(Vec(enqWidth, UInt(intNiq.W)))
}

class DispatchIO extends Bundle {
    val flush = Input(Bool())
    val branchMispredict = Input(Bool())

    val in = Vec(ndcd, Flipped(Decoupled(new RenameOut)))

    val rob = Flipped(new RobEnqIO)
    val lsq = Flipped(new LsqDispatchIO)

    val intIq = Flipped(new IssueDispatchIO(ndcd))
    val memIq = Flipped(new IssueDispatchIO(ndcd))

    val intResidentProducers = Input(Vec(
        intNiq,
        Valid(UInt(wpreg.W))
    ))
    val intAllocOH = Input(Vec(ndcd, UInt(intNiq.W)))

    val fire = Output(Vec(ndcd, Bool()))
}

object LdqPtr {
    def init: LdqPtr = {
        val ptr = Wire(new LdqPtr)
        ptr.oh := 1.U(nldq.W)
        ptr.flag := false.B
        ptr
    }
}

object StqPtr {
    def init: StqPtr = {
        val ptr = Wire(new StqPtr)
        ptr.oh := 1.U(nstq.W)
        ptr.flag := false.B
        ptr
    }
}

class LsqLiveState extends Bundle {
    val ldqHead = new LdqPtr
    val ldqTail = new LdqPtr
    val stqHead = new StqPtr
    val stqTail = new StqPtr

    val ldqValidMask = UInt(nldq.W)
    val stqValidMask = UInt(nstq.W)

    val ldqHighMask = UInt(nldq.W)
    val stqHighMask = UInt(nstq.W)
}
