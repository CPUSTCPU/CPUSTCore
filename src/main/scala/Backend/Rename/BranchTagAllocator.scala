package CPUSTC.backend.rename

import chisel3._
import chisel3.util._

import CPUSTC.config.Decode._
import CPUSTC.config.RenameConfig._
import CPUSTC.backend.branch.BranchUpdate

class BranchTagAllocatorIO extends Bundle {
    val hardFlush = Input(Bool())

    val req = Input(Vec(ndcd, Bool()))
    val doAllocate = Input(Bool())

    val branchUpdate = Flipped(Valid(new BranchUpdate))

    val canAllocate = Output(Bool())

    val brMask = Output(Vec(ndcd, UInt(maxBrCount.W)))

    val brTag = Output(Vec(ndcd, Valid(UInt(wBrTag.W))))

    val activeMask = Output(UInt(maxBrCount.W))
}

class BranchTagAllocator extends Module {
    val io = IO(new BranchTagAllocatorIO)

    val activeMaskReg = RegInit(0.U(maxBrCount.W))

    io.activeMask := activeMaskReg

    val resolveMask = Mux(io.branchUpdate.valid, io.branchUpdate.bits.resolveMask, 0.U(maxBrCount.W))

    val mispredict  = io.branchUpdate.valid && io.branchUpdate.bits.mispredictMask.orR

    val resolvedActiveMask = activeMaskReg & (~resolveMask).asUInt

    val freeMask = (~activeMaskReg)(maxBrCount - 1, 0)

    val rawTagOH = Wire(Vec(ndcd, UInt(maxBrCount.W)))

    var remaining = freeMask

    for (i <- 0 until ndcd) {
        val selected = PriorityEncoderOH(remaining)

        rawTagOH(i) := Mux(io.req(i), selected, 0.U)

        remaining = remaining & (~rawTagOH(i)).asUInt
    }

    val laneEnough = VecInit(
        (0 until ndcd).map { i => 
            !io.req(i) || rawTagOH(i).orR
        }
    )

    io.canAllocate := laneEnough.asUInt.andR && !io.hardFlush

    val allocTagOH = Wire(Vec(ndcd, UInt(maxBrCount.W)))

    for (i <- 0 until ndcd) {
        allocTagOH(i) := Mux(
            io.req(i) && io.canAllocate,
            rawTagOH(i),
            0.U
        )

        io.brTag(i).valid := io.req(i) && io.canAllocate

        io.brTag(i).bits := OHToUInt(rawTagOH(i))
    }

    val makeCursor = Wire(Vec(ndcd + 1, UInt(maxBrCount.W)))

    makeCursor(0) := resolvedActiveMask

    for (i <- 0 until ndcd) {
        io.brMask(i) := makeCursor(i)

        makeCursor(i + 1) := makeCursor(i) | allocTagOH(i)
    }

    val allocMask = Mux(io.doAllocate && io.canAllocate, allocTagOH.reduce(_ | _), 0.U(maxBrCount.W))

    when(io.hardFlush) {
        activeMaskReg := 0.U
    }.elsewhen(mispredict) {
        activeMaskReg := io.branchUpdate.bits.recoverMask
    }.otherwise {
        activeMaskReg := resolvedActiveMask | allocMask
    }

    assert(!(io.doAllocate && !io.canAllocate))

    for (i <- 0 until ndcd) {
        assert(PopCount(rawTagOH(i)) <= 1.U)

        when(!io.req(i)) {
            assert(rawTagOH(i) === 0.U)
        }

        when(io.brTag(i).valid) {
            val tagOH = UIntToOH(io.brTag(i).bits, maxBrCount)
            assert((io.brMask(i) & tagOH) === 0.U)
        }
    }

    for (i <- 0 until ndcd; j <- i + 1 until ndcd) {
        assert((rawTagOH(i) & rawTagOH(j)) === 0.U)
    }

    when(io.doAllocate && io.canAllocate) {
        assert((allocMask & activeMaskReg) === 0.U)
    }

    when(io.branchUpdate.valid && !io.hardFlush) {
        assert(PopCount(io.branchUpdate.bits.resolveMask) === 1.U)
        assert((io.branchUpdate.bits.resolveMask & activeMaskReg).orR)
        assert(
            (io.branchUpdate.bits.mispredictMask &
                (~io.branchUpdate.bits.resolveMask).asUInt) === 0.U
        )
    }

    when(mispredict && !io.hardFlush) {
        assert(
            io.branchUpdate.bits.mispredictMask ===
                io.branchUpdate.bits.resolveMask
        )
        assert(
            (io.branchUpdate.bits.recoverMask &
                io.branchUpdate.bits.resolveMask) === 0.U
        )
        assert(
            (io.branchUpdate.bits.recoverMask &
                (~activeMaskReg).asUInt) === 0.U
        )
    }
}
