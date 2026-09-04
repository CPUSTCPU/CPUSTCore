package CPUSTC.decode

import chisel3._
import chisel3.util._

import CPUSTC.config.Decode._
import CPUSTC.frontend.IBufferEntry

class DecodeIO extends Bundle {
    val flush = Input(Bool())

    val in       = Vec(ndcd, Flipped(Decoupled(new IBufferEntry)))
    val outReady = Input(Vec(ndcd, Bool()))
    val out      = Vec(ndcd, Valid(new DecodeOut))
}

class DecodePacket extends Bundle {
    val validMask = UInt(ndcd.W)
    val bits      = Vec(ndcd, new DecodeOut)
}

class Decode extends Module {
    val io = IO(new DecodeIO)

    val decoded = Wire(Vec(ndcd, new DecodeOut))
    val storedDecoded = Wire(Vec(ndcd, new DecodeOut))

    for (i <- 0 until ndcd) {
        val dec = Module(new MainDecoder)

        dec.io.in.pc        := io.in(i).bits.pc
        dec.io.in.instr     := io.in(i).bits.instr
        dec.io.in.ftqPtr    := io.in(i).bits.ftqPtr
        dec.io.in.ftqOffset := io.in(i).bits.ftqOffset
        dec.io.in.ftqLast   := io.in(i).bits.ftqLast

        decoded(i) := dec.io.out
        when(io.in(i).bits.exception.valid) {
            decoded(i).ctrl.exceptionValid := true.B
            decoded(i).ctrl.exceptionCause := io.in(i).bits.exception.cause
            decoded(i).ctrl.exceptionBadvValid :=
                io.in(i).bits.exception.badvValid
            decoded(i).ctrl.exceptionBadv := io.in(i).bits.exception.badv
        }
    }

    // A fetch exception's BADV is the instruction PC. Keep only the valid bit
    // in the packet FIFO and reconstruct the 32-bit value at its output.
    // This removes six duplicated BADV registers from the two-entry FIFO.
    storedDecoded := decoded
    for (i <- 0 until ndcd) {
        storedDecoded(i).ctrl.exceptionBadv := 0.U
    }

    // A two-entry, non-flow-through packet FIFO makes Decode a real pipeline
    // boundary. Its enqueue readiness depends only on registered occupancy,
    // so downstream backpressure cannot propagate into the IBuffer in one cycle.
    val entries = Reg(Vec(2, new DecodePacket))
    val head    = RegInit(false.B)
    val tail    = RegInit(false.B)
    val count   = RegInit(0.U(2.W))

    val inputValidMask = VecInit(io.in.map(_.valid)).asUInt
    val enqValid = inputValidMask.orR
    val enqReady = count =/= 2.U && !io.flush
    val enqFire  = enqValid && enqReady

    val headPacket = entries(head)
    val deqValid = count =/= 0.U
    val deqReady = (0 until ndcd).map { i =>
        !headPacket.validMask(i) || io.outReady(i)
    }.reduce(_ && _)
    val deqFire = deqValid && deqReady && !io.flush

    for (i <- 0 until ndcd) {
        io.in(i).ready := enqReady
        // Rename has a cycle-aligned local flush copy and refuses this packet
        // while recovery is active. Keep the registered packet visible here;
        // the FIFO state is still cleared below, but the global Decode flush no
        // longer enters Rename's allocation and snapshot data cones.
        io.out(i).valid := deqValid && headPacket.validMask(i)
        io.out(i).bits  := headPacket.bits(i)
        io.out(i).bits.ctrl.exceptionBadv := headPacket.bits(i).meta.pc
    }

    when(io.flush) {
        head  := false.B
        tail  := false.B
        count := 0.U
    }.otherwise {
        when(enqFire) {
            entries(tail).validMask := inputValidMask
            entries(tail).bits      := storedDecoded
            tail := ~tail
        }

        when(deqFire) {
            head := ~head
        }

        switch(Cat(enqFire, deqFire)) {
            is("b10".U) { count := count + 1.U }
            is("b01".U) { count := count - 1.U }
        }
    }

    for (i <- 1 until ndcd) {
        when(io.in(i).valid) {
            assert(io.in(i - 1).valid, "Decode expects prefix-valid input lanes")
        }
        when(io.out(i).valid) {
            assert(io.out(i - 1).valid, "Decode preserves prefix-valid output lanes")
        }
    }

    for (i <- 0 until ndcd) {
        when(io.in(i).valid && decoded(i).ctrl.exceptionBadvValid) {
            assert(decoded(i).ctrl.exceptionBadv === decoded(i).meta.pc,
                "Decode FIFO reconstructs a valid exception BADV from its PC")
        }
    }

    assert(count <= 2.U)
    when(io.flush) {
        assert(!enqFire)
        assert(!deqFire)
    }
}
