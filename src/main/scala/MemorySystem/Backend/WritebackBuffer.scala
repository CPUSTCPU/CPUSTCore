package CPUSTC.memory.backend

import chisel3._
import chisel3.util._
import CPUSTC.memory._

class WritebackRequest extends Bundle {
    val paddr = UInt(32.W)
    val data = UInt(DcacheConfig.DcacheLineBits.W)
}

class WritebackResponse extends Bundle {
    val paddr = UInt(32.W)
}

class WritebackBufferIO extends Bundle {
    val enqueue = Vec(2, Flipped(Decoupled(new WritebackRequest)))
    val memoryReq = Decoupled(new WritebackRequest)
    val memoryResp = Flipped(Valid(new WritebackResponse))
    val queryPaddr = Input(UInt(32.W))
    val queryHit = Output(Bool())
    val empty = Output(Bool())
    val full = Output(Bool())
    val nextHasSpace = Output(Bool())
}

class WritebackBuffer extends Module {
    val io = IO(new WritebackBufferIO)

    private val length = WritebackBufferConfig.length
    private val countWidth = log2Ceil(length + 1)

    val entries = RegInit(VecInit.fill(length)(0.U.asTypeOf(new WritebackRequest)))
    val count = RegInit(0.U(countWidth.W))
    val requestSent = RegInit(false.B)

    val headValid = count =/= 0.U
    val responseFire = io.memoryResp.valid && headValid && requestSent &&
        io.memoryResp.bits.paddr === entries(0).paddr
    val countAfterResponse = count - responseFire.asUInt
    val availableBeforeResponse = length.U - count

    io.enqueue(0).ready := availableBeforeResponse >= 1.U
    io.enqueue(1).ready := Mux(
        io.enqueue(0).valid,
        availableBeforeResponse >= 2.U,
        availableBeforeResponse >= 1.U
    )

    val enqFire0 = io.enqueue(0).fire
    val enqFire1 = io.enqueue(1).fire
    val enqCount = enqFire0.asUInt +& enqFire1.asUInt
    val baseEntries = Wire(Vec(length, new WritebackRequest))
    for (index <- 0 until length) {
        if (index < length - 1) {
            baseEntries(index) := Mux(responseFire, entries(index + 1), entries(index))
        } else {
            baseEntries(index) := entries(index)
        }
    }

    val nextEntries = WireDefault(baseEntries)
    for (index <- 0 until length) {
        when(enqFire0 && index.U === countAfterResponse) {
            nextEntries(index) := io.enqueue(0).bits
        }
        when(enqFire1 && index.U === countAfterResponse + enqFire0.asUInt) {
            nextEntries(index) := io.enqueue(1).bits
        }
    }

    val nextCount = countAfterResponse + enqCount
    entries := nextEntries
    count := nextCount

    io.memoryReq.valid := headValid && !requestSent
    io.memoryReq.bits := entries(0)

    when(responseFire) {
        requestSent := false.B
    }.elsewhen(io.memoryReq.fire) {
        requestSent := true.B
    }

    io.queryHit := (0 until length).map { index =>
        index.U < count && entries(index).paddr === io.queryPaddr
    }.reduce(_ || _)
    io.empty := count === 0.U
    io.full := count === length.U
    io.nextHasSpace := nextCount < length.U

    when(io.memoryResp.valid && requestSent) {
        assert(headValid, "WritebackBuffer: response without an active entry")
        assert(io.memoryResp.bits.paddr === entries(0).paddr,
            "WritebackBuffer: response address must match the head entry")
    }
}
