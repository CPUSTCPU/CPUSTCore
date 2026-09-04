package CPUSTC.memory.backend

import chisel3._
import chisel3.util._

class StoreIssueBufferIO extends Bundle {
    val enqueue = Flipped(Decoupled(new DcachePpReq))
    val dequeue = Decoupled(new DcachePpReq)
    val clear = Input(Bool())
}

/** One-entry elastic boundary between StoreQueue dequeue and DCache channel 0. */
class StoreIssueBuffer extends Module {
    val io = IO(new StoreIssueBufferIO)

    val valid = RegInit(false.B)
    val bits = RegInit(0.U.asTypeOf(new DcachePpReq))

    val dequeueFire = valid && io.dequeue.ready
    io.enqueue.ready := !valid || dequeueFire
    io.dequeue.valid := valid
    io.dequeue.bits := bits

    when(io.clear) {
        valid := false.B
    }.elsewhen(io.enqueue.fire) {
        bits := io.enqueue.bits
        valid := true.B
    }.elsewhen(dequeueFire) {
        valid := false.B
    }

    when(io.clear) {
        assert(!io.enqueue.fire && !io.dequeue.fire,
            "StoreIssueBuffer: retry clear must not overlap a transfer")
    }
}
