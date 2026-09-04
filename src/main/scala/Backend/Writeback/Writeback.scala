package CPUSTC.backend.writeback

import chisel3._
import chisel3.util._

import CPUSTC.backend.branch.BranchMask
import CPUSTC.config.LoadStoreQueue._
import CPUSTC.config.MemIssueOp._
import CPUSTC.config.MemoryException._
import CPUSTC.config.RegisterFile._
import CPUSTC.config.RenameConfig._
import CPUSTC.config.WritebackConfig._
import CPUSTC.backend.execute.ExecuteResult

class Writeback extends Module {
    val io = IO(new WritebackIO)

    require(nIntWb + nLoadWb == nDataWb)
    require(nDataWb + nStoreComplete == nRobComplete)
    require(nDataWb == nWrite)

    io.rfWrite      := 0.U.asTypeOf(io.rfWrite)
    io.bypass       := 0.U.asTypeOf(io.bypass)
    io.issueWakeup  := 0.U.asTypeOf(io.issueWakeup)
    io.renameWakeup := 0.U.asTypeOf(io.renameWakeup)
    io.robComplete  := 0.U.asTypeOf(io.robComplete)

    for (p <- 0 until nIntWb) {
        io.intResult(p).ready := true.B
    }

    val mispredictMask = Mux(
        io.branchUpdate.valid,
        io.branchUpdate.bits.mispredictMask,
        0.U(maxBrCount.W)
    )

    val dataWb = Wire(Vec(nDataWb, Valid(new ExecuteResult)))
    dataWb := 0.U.asTypeOf(dataWb)

    // A DCache response otherwise fans out directly into every replicated
    // LVT PRF bank.  Keep completion and wakeup in the raw response cycle,
    // but relay the architectural data write through this local boundary.
    // RegisterRead consumes the matching registered bypass in the same cycle
    // as the delayed PRF write, so a load-use pair does not gain a bubble.
    val delayedLoadWriteValid = RegInit(
        VecInit(Seq.fill(nLoadWb)(false.B))
    )
    val delayedLoadWritePdest = Reg(Vec(nLoadWb, UInt(wpreg.W)))
    val delayedLoadWriteData = Reg(Vec(nLoadWb, UInt(dataWidth.W)))

    def ptrAlive(
        ptrOH: UInt,
        ptrFlag: Bool,
        validMask: UInt,
        highMask: UInt
    ): Bool = {
        require(ptrOH.getWidth == validMask.getWidth)
        require(ptrOH.getWidth == highMask.getWidth)

        val generationMask = Mux(
            ptrFlag,
            highMask,
            (~highMask).asUInt
        )

        (ptrOH & validMask & generationMask).orR
    }

    for (p <- 0 until nIntWb) {
        val in = io.intResult(p)

        val killed = BranchMask.isKilled(
            in.bits.brMask,
            mispredictMask
        )

        val accepted =
            in.fire &&
            !io.flush &&
            !killed

        dataWb(p).valid := accepted
        dataWb(p).bits  := in.bits
    }

    for (l <- 0 until nLoadWb) {
        val in   = io.loadResult(l)
        val inst = in.bits.inst
        val port = nIntWb + l

        val alive = ptrAlive(
            inst.ldindex,
            inst.ldindexHigh,
            io.lsqLive.ldqValidMask,
            io.lsqLive.ldqHighMask
        )
        // CPUSTC.memory exports only results whose owning source has already
        // validated the complete Load identity. Keep the local liveness check
        // as a proof assertion, not as a second wide gate on wakeup/completion.
        val accepted =
            in.valid &&
            inst.uop.isLD &&
            !io.loadRecovery &&
            !io.flush

        val requestException = inst.exception =/= EXC_NONE
        val responseException = in.bits.exception =/= EXC_NONE
        val exceptionValid = requestException || responseException

        io.loadDebug(l).valid := accepted && !exceptionValid
        io.loadDebug(l).bits.robPtr := inst.robPtr
        io.loadDebug(l).bits.vaddr := inst.pc
        io.loadDebug(l).bits.paddr := inst.paddr
        io.loadDebug(l).bits.mask := inst.mask
        io.loadDebug(l).bits.signed := inst.signed

        val rawAccepted = accepted

        io.rawLoadWakeup(l).valid :=
            rawAccepted &&
            inst.rfWen &&
            !exceptionValid &&
            inst.pdest =/= 0.U
        io.rawLoadWakeup(l).bits.pdest := inst.pdest
        io.rawLoadWakeup(l).bits.fast := false.B

        val exceptionCause = Mux(
            requestException,
            inst.exception,
            Mux(responseException, in.bits.exception, EXC_NONE)
        )

        dataWb(port).valid               := accepted
        dataWb(port).bits.robPtr         := inst.robPtr
        dataWb(port).bits.pdest          := inst.pdest
        dataWb(port).bits.rfWen          := inst.rfWen
        dataWb(port).bits.data           := in.bits.data
        dataWb(port).bits.brMask         := 0.U
        dataWb(port).bits.exceptionValid := exceptionValid
        dataWb(port).bits.exceptionCause := exceptionCause
        dataWb(port).bits.exceptionBadvValid :=
            requestException && inst.exceptionBadvValid
        dataWb(port).bits.exceptionBadv := Mux(
            requestException && inst.exceptionBadvValid,
            inst.exceptionBadv,
            0.U
        )

        when(in.valid) {
            assert(inst.uop.isLD)
            assert(PopCount(inst.ldindex) === 1.U)
            assert(PopCount(inst.sqindex) === 1.U)
            assert(alive,
                s"Writeback: MemorySystem exported stale Load result on port $l")
        }

        assert(
            io.rawLoadWakeup(l).valid ===
                (accepted &&
                    inst.rfWen &&
                    !exceptionValid &&
                    inst.pdest =/= 0.U)
        )
        when(io.loadRecovery) {
            assert(!dataWb(port).valid,
                s"Writeback: Load completed during recovery on port $l")
            assert(!io.rawLoadWakeup(l).valid,
                s"Writeback: Load wakeup escaped recovery on port $l")
        }
    }

    for (l <- 0 until nLoadWb) {
        val port = nIntWb + l
        val event = dataWb(port)
        val writesData =
            event.valid &&
            event.bits.rfWen &&
            !event.bits.exceptionValid &&
            event.bits.pdest =/= 0.U

        when(io.flush) {
            delayedLoadWriteValid(l) := false.B
        }.otherwise {
            delayedLoadWriteValid(l) := writesData
            when(writesData) {
                delayedLoadWritePdest(l) := event.bits.pdest
                delayedLoadWriteData(l) := event.bits.data
            }
        }
    }

    for (i <- 0 until nDataWb) {
        val event = dataWb(i)
        val fastRobValid = if (i < nIntWb) {
            io.fastIntRawValid(i) && !io.flush
        } else {
            false.B
        }

        val rawWritesData =
            event.valid &&
            event.bits.rfWen &&
            !event.bits.exceptionValid &&
            event.bits.pdest =/= 0.U

        io.issueWakeup(i).valid      := rawWritesData
        io.issueWakeup(i).bits.pdest := event.bits.pdest

        if (i < nIntWb) {
            io.renameWakeup(i).valid      := rawWritesData
            io.renameWakeup(i).bits.pdest := event.bits.pdest

            io.rfWrite(i).valid     := rawWritesData
            io.rfWrite(i).bits.addr := event.bits.pdest
            io.rfWrite(i).bits.data := event.bits.data

            io.bypass(i).valid      := rawWritesData
            io.bypass(i).bits.pdest := event.bits.pdest
            io.bypass(i).bits.data  := event.bits.data
        } else {
            val l = i - nIntWb
            val delayedWritesData =
                delayedLoadWriteValid(l) &&
                !io.flush

            // Rename does not need the Load wakeup until the cycle after the
            // raw response: resident/newly dispatched IQ consumers already
            // see issueWakeup above. Reuse the registered PRF relay here so
            // DCache response qualification cannot drive the global BusyTable
            // update cone in the response cycle.
            io.renameWakeup(i).valid      := delayedWritesData
            io.renameWakeup(i).bits.pdest := delayedLoadWritePdest(l)

            io.rfWrite(i).valid     := delayedWritesData
            io.rfWrite(i).bits.addr := delayedLoadWritePdest(l)
            io.rfWrite(i).bits.data := delayedLoadWriteData(l)

            io.bypass(i).valid      := delayedWritesData
            io.bypass(i).bits.pdest := delayedLoadWritePdest(l)
            io.bypass(i).bits.data  := delayedLoadWriteData(l)
        }

        io.robComplete(i).valid       := event.valid || fastRobValid
        io.robComplete(i).bits.robPtr := event.bits.robPtr
        io.robComplete(i).bits.fastEligible := (
            if (i < nIntWb) fastRobValid else false.B
        )
        io.robComplete(i).bits.data   := event.bits.data
        io.robComplete(i).bits.branchResolved := event.bits.branchResolved

        io.robComplete(i).bits.exceptionValid := event.bits.exceptionValid
        io.robComplete(i).bits.exceptionCause := event.bits.exceptionCause
        io.robComplete(i).bits.exceptionBadvValid :=
            event.bits.exceptionValid && event.bits.exceptionBadvValid
        io.robComplete(i).bits.exceptionBadv := Mux(
            event.bits.exceptionValid && event.bits.exceptionBadvValid,
            event.bits.exceptionBadv,
            0.U
        )

        if (i < nIntWb) {
            when(fastRobValid) {
                assert(!event.bits.exceptionValid)
                assert(event.bits.exceptionCause === EXC_NONE)
                assert(!event.bits.exceptionBadvValid)
            }
        }

        when(event.valid && event.bits.rfWen) {
            assert(event.bits.pdest =/= 0.U)
        }

        when(event.valid && !event.bits.exceptionValid) {
            assert(event.bits.exceptionCause === EXC_NONE)
            assert(!event.bits.exceptionBadvValid)
        }

        when(rawWritesData) {
            assert(io.issueWakeup(i).valid)
            if (i < nIntWb) {
                assert(io.renameWakeup(i).valid)
            }
        }

        if (i >= nIntWb) {
            val l = i - nIntWb
            when(delayedLoadWriteValid(l) && !io.flush) {
                assert(io.renameWakeup(i).valid)
                assert(io.renameWakeup(i).bits.pdest === delayedLoadWritePdest(l))
            }
        }

        when(io.rfWrite(i).valid) {
            assert(io.bypass(i).valid)
            assert(io.rfWrite(i).bits.addr === io.bypass(i).bits.pdest)
            assert(io.rfWrite(i).bits.data === io.bypass(i).bits.data)
        }
    }

    for (s <- 0 until nStoreComplete) {
        val in   = io.storeComplete(s)
        val port = nDataWb + s

        // StoreQueue has already checked the physical SQ generation before
        // emitting this clean completion token. A same-cycle recovered token
        // is harmless: the ROB validates the complete pointer generation.
        // Keep recovery and hard-flush controls off this narrow normal path.
        io.robComplete(port).valid := in.valid

        io.robComplete(port).bits.robPtr := in.bits
        io.robComplete(port).bits.fastEligible := false.B
        io.robComplete(port).bits.data := 0.U
        io.robComplete(port).bits.branchResolved := false.B
        io.robComplete(port).bits.exceptionValid := false.B
        io.robComplete(port).bits.exceptionCause := EXC_NONE
        io.robComplete(port).bits.exceptionBadvValid := false.B
        io.robComplete(port).bits.exceptionBadv := 0.U
    }

    for (i <- 0 until nDataWb; j <- i + 1 until nDataWb) {
        when(io.rfWrite(i).valid && io.rfWrite(j).valid) {
            assert(io.rfWrite(i).bits.addr =/= io.rfWrite(j).bits.addr)
        }
    }
}
