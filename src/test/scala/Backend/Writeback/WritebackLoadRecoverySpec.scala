package CPUSTC.backend.writeback

import chisel3._
import chiseltest._
import CPUSTC.config.LoadStoreQueue._
import CPUSTC.config.RegisterFile._
import CPUSTC.config.WritebackConfig._
import org.scalatest.flatspec.AnyFlatSpec

class WritebackLoadRecoveryHarness extends Module {
  val io = IO(new Bundle {
    val loadValid = Input(Bool())
    val loadRecovery = Input(Bool())
    val rawWake = Output(Bool())
    val robComplete = Output(Bool())
    val delayedRfWrite = Output(Bool())
    val delayedRfAddr = Output(UInt(wpreg.W))
    val delayedRfData = Output(UInt(32.W))
  })

  val writeback = Module(new Writeback)
  writeback.io.flush := false.B
  writeback.io.branchUpdate := 0.U.asTypeOf(writeback.io.branchUpdate)
  writeback.io.loadRecovery := io.loadRecovery
  writeback.io.lsqLive := 0.U.asTypeOf(writeback.io.lsqLive)
  writeback.io.lsqLive.ldqValidMask := 1.U
  writeback.io.lsqLive.ldqHighMask := 0.U
  writeback.io.intResult.foreach { result =>
    result.valid := false.B
    result.bits := 0.U.asTypeOf(result.bits)
  }
  writeback.io.fastIntRawValid.foreach(_ := false.B)
  writeback.io.loadResult := 0.U.asTypeOf(writeback.io.loadResult)
  writeback.io.loadResult(0).valid := io.loadValid
  writeback.io.loadResult(0).bits.inst.uop.isLD := true.B
  writeback.io.loadResult(0).bits.inst.ldindex := 1.U
  writeback.io.loadResult(0).bits.inst.sqindex := 1.U
  writeback.io.loadResult(0).bits.inst.pdest := 3.U
  writeback.io.loadResult(0).bits.inst.rfWen := true.B
  writeback.io.loadResult(0).bits.data := "h12345678".U
  writeback.io.storeComplete := 0.U.asTypeOf(writeback.io.storeComplete)

  val loadPort = nIntWb
  io.rawWake := writeback.io.rawLoadWakeup(0).valid
  io.robComplete := writeback.io.robComplete(loadPort).valid
  io.delayedRfWrite := writeback.io.rfWrite(loadPort).valid
  io.delayedRfAddr := writeback.io.rfWrite(loadPort).bits.addr
  io.delayedRfData := writeback.io.rfWrite(loadPort).bits.data
}

class WritebackLoadRecoverySpec extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "Writeback Load recovery"

  it should "block a recovery-cycle completion but preserve an accepted delayed write" in {
    test(new WritebackLoadRecoveryHarness) { dut =>
      dut.io.loadRecovery.poke(false.B)
      dut.io.loadValid.poke(true.B)
      dut.io.rawWake.expect(true.B)
      dut.io.robComplete.expect(true.B)
      dut.io.delayedRfWrite.expect(false.B)
      dut.clock.step()

      dut.io.loadRecovery.poke(true.B)
      dut.io.loadValid.poke(true.B)
      dut.io.rawWake.expect(false.B)
      dut.io.robComplete.expect(false.B)
      dut.io.delayedRfWrite.expect(true.B)
      dut.io.delayedRfAddr.expect(3.U)
      dut.io.delayedRfData.expect("h12345678".U)
      dut.clock.step()

      dut.io.loadRecovery.poke(false.B)
      dut.io.loadValid.poke(false.B)
      dut.io.delayedRfWrite.expect(false.B)
    }
  }
}
