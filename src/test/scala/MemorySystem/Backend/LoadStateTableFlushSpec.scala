package CPUSTC.memory.backend

import chisel3._
import chisel3.util.Fill
import chiseltest._
import CPUSTC.memory._
import org.scalatest.flatspec.AnyFlatSpec

class LoadStateTableFlushHarness extends Module {
  val io = IO(new Bundle {
    val allocate = Input(Bool())
    val redirect = Input(Bool())
    val flushMask = Input(UInt(LoadStateTableConfig.length.W))
    val issueReady = Input(Bool())

    val issueValid = Output(Bool())
    val issueFire = Output(Bool())
    val issueLdindex = Output(UInt(LoadStateTableConfig.length.W))
    val occupied = Output(Bool())
  })

  private val slot = (BigInt(1) << 6).U(LoadStateTableConfig.length.W)
  val table = Module(new LoadStateTable)

  table.io.loadStoreBack := 0.U.asTypeOf(table.io.loadStoreBack)
  table.io.loadMshrBack := 0.U.asTypeOf(table.io.loadMshrBack)
  table.io.complete := 0.U.asTypeOf(table.io.complete)
  table.io.uncacheResultCheck := 0.U.asTypeOf(table.io.uncacheResultCheck)
  table.io.interReplay := 0.U.asTypeOf(table.io.interReplay)
  table.io.storeWaitState := 0.U.asTypeOf(table.io.storeWaitState)
  table.io.storeFreedMask := 0.U
  table.io.robHeadLoad := 0.U.asTypeOf(table.io.robHeadLoad)
  table.io.llCommit := 0.U.asTypeOf(table.io.llCommit)
  table.io.pendingUncacheStore := false.B
  table.io.sqHeadOH := 1.U
  table.io.sqHeadHigh := false.B

  table.io.ptrCtrl := 0.U.asTypeOf(table.io.ptrCtrl)
  table.io.ptrCtrl.nextHeadPtr := 1.U
  table.io.ptrCtrl.nextHeadSuffixMask := Fill(LoadStateTableConfig.length, true.B)
  table.io.ptrCtrl.nextTailPtr := 1.U
  table.io.ptrCtrl.flushMask := io.flushMask
  table.io.ptrCtrl.redirect := io.redirect

  table.io.lsqLive := 0.U.asTypeOf(table.io.lsqLive)
  table.io.lsqLive.ldqValidMask := slot
  table.io.lsqLive.ldqHighMask := 0.U
  table.io.lsqLive.stqTailOH := 1.U

  table.io.entry := 0.U.asTypeOf(table.io.entry)
  table.io.entry(0).valid := io.allocate
  table.io.entry(0).uop.isLD := true.B
  table.io.entry(0).pc := "ha08abdc4".U
  table.io.entry(0).paddr := "h008abdc4".U
  table.io.entry(0).ldindex := slot
  table.io.entry(0).ldindexHigh := false.B
  table.io.entry(0).robPtr.qidx := 1.U
  table.io.entry(0).robPtr.offset := 7.U
  table.io.entry(0).robPtr.high := false.B
  table.io.entry(0).robPtr.epoch := "he".U
  table.io.entry(0).pdest := "h16".U
  table.io.entry(0).rfWen := true.B

  table.io.entryIssued := 0.U.asTypeOf(table.io.entryIssued)
  table.io.entryIssued(0) := io.allocate
  table.io.issueInsts(0).ready := io.issueReady
  table.io.issueInsts(1).ready := false.B
  table.io.uncacheReq.ready := false.B

  io.issueValid := table.io.issueInsts(0).valid
  io.issueFire := table.io.issueInsts(0).fire
  io.issueLdindex := table.io.issueInsts(0).bits.ldindex
  io.occupied := table.io.occupancy.orR
}

class LoadStateTableFlushSpec extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "LoadStateTable replay flush gating"

  private val slot6 = (BigInt(1) << 6).U

  private def init(dut: LoadStateTableFlushHarness): Unit = {
    dut.io.allocate.poke(false.B)
    dut.io.redirect.poke(false.B)
    dut.io.flushMask.poke(0.U)
    dut.io.issueReady.poke(true.B)
    dut.reset.poke(true.B)
    dut.clock.step(2)
    dut.reset.poke(false.B)
    dut.clock.step()
  }

  private def allocateExecutingLoad(dut: LoadStateTableFlushHarness): Unit = {
    dut.io.allocate.poke(true.B)
    dut.clock.step()
    dut.io.allocate.poke(false.B)
    dut.io.occupied.expect(true.B)
    dut.io.issueValid.expect(false.B)
  }

  it should "not replay an entry in the arriving flush mask" in {
    test(new LoadStateTableFlushHarness) { dut =>
      init(dut)
      allocateExecutingLoad(dut)

      dut.io.redirect.poke(true.B)
      dut.clock.step()

      dut.io.redirect.poke(false.B)
      dut.io.flushMask.poke(slot6)
      dut.io.issueValid.expect(false.B)
      dut.io.issueFire.expect(false.B)
      dut.clock.step()

      dut.io.flushMask.poke(0.U)
      dut.io.occupied.expect(false.B)
      dut.io.issueValid.expect(false.B)
    }
  }

  it should "still replay an entry outside a partial flush mask" in {
    test(new LoadStateTableFlushHarness) { dut =>
      init(dut)
      allocateExecutingLoad(dut)

      dut.io.redirect.poke(true.B)
      dut.clock.step()

      dut.io.redirect.poke(false.B)
      dut.io.flushMask.poke((BigInt(1) << 5).U)
      dut.io.issueValid.expect(true.B)
      dut.io.issueFire.expect(true.B)
      dut.io.issueLdindex.expect(slot6)
    }
  }
}
