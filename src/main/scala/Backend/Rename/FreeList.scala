package CPUSTC.backend.rename

import chisel3._
import chisel3.util._

import CPUSTC.config.Commit._
import CPUSTC.config.Decode._
import CPUSTC.config.RegisterFile._
import CPUSTC.config.RenameConfig._
import CPUSTC.backend.branch.BranchUpdate

class FreeListIO extends Bundle {
    val flush = Input(Bool())

    val allocReqs  = Input(Vec(ndcd, Bool()))
    val allocResps = Output(Vec(ndcd, Valid(UInt(wpreg.W))))
    val canAllocate = Output(Bool())
    val doAllocate  = Input(Bool())

    val freeCount = Output(UInt(log2Ceil(npreg + 1).W))

    val commit = Input(Vec(ncmt, Valid(new RenameCommitInfo)))

    val brSnapshotReqs = Input(Vec(ndcd, Valid(UInt(wBrTag.W))))
    val branchUpdate = Flipped(Valid(new BranchUpdate))
}

class FreeList extends Module {
    val io = IO(new FreeListIO)

    val queue = Module(new PregFreeQueue)
    queue.io <> io
}
