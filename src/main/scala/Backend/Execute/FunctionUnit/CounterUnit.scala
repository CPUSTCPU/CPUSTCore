package CPUSTC.backend.execute.fu

import chisel3._

import CPUSTC.config.CntOp._
import CPUSTC.config.Execute._
import CPUSTC.config.RegisterFile._

class StableCounter(initialValue: BigInt = 0) extends Module {
    val io = IO(new Bundle {
        val value = Output(UInt(64.W))
    })

    val counter = RegInit(initialValue.U(64.W))
    counter := counter +% 1.U

    io.value := counter
}

class CounterUnitIO extends Bundle {
    val valid        = Input(Bool())
    val fn           = Input(UInt(FU_OP_SZ.W))
    val counterValue = Input(UInt(64.W))
    val result       = Output(UInt(dataWidth.W))
}

class CounterUnit extends Module {
    val io = IO(new CounterUnitIO)

    io.result := Mux(
        io.fn === HI,
        io.counterValue(63, 32),
        io.counterValue(31, 0)
    )

    when(io.valid) {
        assert(io.fn === LO || io.fn === HI)
    }
}
