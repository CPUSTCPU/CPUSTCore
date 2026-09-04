package CPUSTC.backend.execute.fu

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

import CPUSTC.memory.MemSysConfig

class CpuConfigHarness(memSysConfig: MemSysConfig) extends Module {
  val io = IO(new Bundle {
    val index = Input(UInt(32.W))
    val value = Output(UInt(32.W))
  })

  private val cpuConfig = new CpuConfig(memSysConfig)
  io.value := cpuConfig.read(io.index)
}

class CpuConfigSpec extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "CpuConfig"

  it should "advertise paging through CPUCFG1" in {
    test(new CpuConfigHarness(MemSysConfig())) { dut =>
      dut.io.index.poke(1.U)
      dut.io.value.expect("h0001f1f4".U)
    }
  }
}
