package CPUSTC.backend.rename

import chisel3._
import chisel3.util._

import CPUSTC.config.RegisterFile._
import CPUSTC.config.RenameConfig._
import CPUSTC.config.Decode._
import CPUSTC.config.Commit._
import CPUSTC.config.WritebackConfig._
import CPUSTC.decode._
import CPUSTC.backend.branch.BranchUpdate


class RenameRegInfo extends Bundle {
    val lsrc1 = UInt(wlreg.W)
    val lsrc2 = UInt(wlreg.W)
    val ldest = UInt(wlreg.W)

    val psrc1 = UInt(wpreg.W)
    val psrc2 = UInt(wpreg.W)
    val pdest = UInt(wpreg.W)

    val pprd  = UInt(wpreg.W)

    val lsrc1Valid = Bool()
    val lsrc2Valid = Bool()
    val ldestValid = Bool()
    val rfWen      = Bool()

    val psrc1Ready = Bool()
    val psrc2Ready = Bool()
}

class RenameSpecInfo extends Bundle {
    val brMask = UInt(maxBrCount.W)
    val brTag  = Valid(UInt(wBrTag.W))
}

class RenameOut extends Bundle {
    val meta = new DecodeMeta
    val ctrl = new DecodeCtrlInfo
    val mem  = new DecodeMemInfo
    val br   = new DecodeBrInfo

    val reg  = new RenameRegInfo
    val spec = new RenameSpecInfo
}

class RenameCommitInfo extends Bundle {
    val ldest      = UInt(wlreg.W)
    val pdest      = UInt(wpreg.W)
    val pprd       = UInt(wpreg.W)

    val ldestValid = Bool()
    val rfWen      = Bool()
}

class RenameWakeupInfo extends Bundle {
    val pdest = UInt(wpreg.W)
}

class RenameStatus extends Bundle {
    val outputBlocked = Bool()
    val freeBlocked   = Bool()
    val tagBlocked    = Bool()
}

class RenameIO extends Bundle {
    val flush = Input(Bool())

    val in  = Vec(ndcd, Flipped(Decoupled(new DecodeOut)))
    val out = Vec(ndcd, Decoupled(new RenameOut))

    val commit = Input(Vec(ncmt, Valid(new RenameCommitInfo)))
    val wakeup = Input(Vec(nwkp, Valid(new RenameWakeupInfo)))
    val earlyWakeup = Input(Vec(nFastIntWb, Valid(new RenameWakeupInfo)))

    val branchUpdate = Flipped(Valid(new BranchUpdate))

    val status = Output(new RenameStatus)
}

class RenameMapReadReq extends Bundle {
    val lsrc1 = UInt(wlreg.W)
    val lsrc2 = UInt(wlreg.W)
    val ldest = UInt(wlreg.W)
}

class RenameMapReadResp extends Bundle {
    val psrc1 = UInt(wpreg.W)
    val psrc2 = UInt(wpreg.W)
    val pprd  = UInt(wpreg.W)
}

class RenameMapWriteReq extends Bundle {
    val ldest = UInt(wlreg.W)
    val pdest = UInt(wpreg.W)
}
