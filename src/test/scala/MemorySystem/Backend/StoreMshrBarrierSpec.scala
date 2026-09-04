package CPUSTC.memory.backend

import chisel3._
import chiseltest._
import CPUSTC.memory._
import org.scalatest.flatspec.AnyFlatSpec

class StoreMshrBarrierHarnessIO extends Bundle {
  val retry = Input(Bool())
  val retrySqindex = Input(UInt(StoreQueueConfig.length.W))
  val retrySqindexHigh = Input(Bool())
  val mshrProgress = Input(Bool())

  val sqValid = Input(Bool())
  val sqindex = Input(UInt(StoreQueueConfig.length.W))
  val sqindexHigh = Input(Bool())
  val dcacheReady = Input(Bool())

  val sqReady = Output(Bool())
  val sqFire = Output(Bool())
  val sibValid = Output(Bool())
  val sibEnqueueFire = Output(Bool())
  val sibDequeueFire = Output(Bool())
  val dcacheValid = Output(Bool())
  val dcacheFire = Output(Bool())
  val blocked = Output(Bool())
  val delayedProgress = Output(Bool())

  val orderedRetryValid = Output(Bool())
  val orderedRetrySqindex = Output(UInt(StoreQueueConfig.length.W))
  val orderedRetrySqindexHigh = Output(Bool())
}

/** Test-only extraction of the Store retry barrier in LoadStorePipeline.
  *
  * It instantiates the production StoreIssueBuffer and preserves the exact
  * retry-clear, delayed-progress priority, SQ admission, and DCache issue gates.
  */
class StoreMshrBarrierHarness extends Module {
  val io = IO(new StoreMshrBarrierHarnessIO)

  val storeIssueBuffer = Module(new StoreIssueBuffer)
  val delayedMshrProgress = RegNext(io.mshrProgress, false.B)
  val storeMshrBlocked = RegInit(false.B)
  when(delayedMshrProgress) {
    storeMshrBlocked := false.B
  }.elsewhen(io.retry) {
    storeMshrBlocked := true.B
  }

  io.orderedRetryValid := io.retry
  io.orderedRetrySqindex := io.retrySqindex
  io.orderedRetrySqindexHigh := io.retrySqindexHigh
  storeIssueBuffer.io.clear := io.retry

  val storeReq = WireDefault(0.U.asTypeOf(new DcachePpReq))
  storeReq.uop.isSTD := true.B
  storeReq.valid := io.sqValid
  storeReq.sqindex := io.sqindex
  storeReq.sqindexHigh := io.sqindexHigh

  val storeAdmissionAllowed = !io.retry && !storeMshrBlocked
  storeIssueBuffer.io.enqueue.valid := storeAdmissionAllowed && io.sqValid
  storeIssueBuffer.io.enqueue.bits := storeReq
  io.sqReady := storeAdmissionAllowed && storeIssueBuffer.io.enqueue.ready
  io.sqFire := io.sqValid && io.sqReady

  // Mirror the production grant exactly. The blocked term is functionally
  // redundant once the SIB is empty, but retaining it proved better for the
  // rebuilt FPGA mapping and keeps this harness valid as RTL-equivalent evidence.
  io.dcacheValid := !io.retry && !storeMshrBlocked &&
    storeIssueBuffer.io.dequeue.valid
  io.dcacheFire := io.dcacheValid && io.dcacheReady
  storeIssueBuffer.io.dequeue.ready := io.dcacheFire

  io.sibValid := storeIssueBuffer.io.dequeue.valid
  io.sibEnqueueFire := storeIssueBuffer.io.enqueue.fire
  io.sibDequeueFire := storeIssueBuffer.io.dequeue.fire
  io.blocked := storeMshrBlocked
  io.delayedProgress := delayedMshrProgress

  assert(io.sqFire === io.sibEnqueueFire,
    "Store barrier harness: SQ fire must equal SIB enqueue fire")
  assert(io.sibDequeueFire === io.dcacheFire,
    "Store barrier harness: SIB dequeue must equal DCache fire")
  when(storeMshrBlocked) {
    assert(!storeIssueBuffer.io.dequeue.valid,
      "Store barrier harness: blocked state must imply an empty SIB")
    assert(!io.sqFire && !io.sibEnqueueFire && !io.sibDequeueFire &&
      !io.dcacheFire,
      "Store barrier harness: a blocked Store path must not transfer")
  }
}

class StoreMshrBarrierSpec extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "LoadStorePipeline Store MSHR retry barrier"

  private def init(dut: StoreMshrBarrierHarness): Unit = {
    dut.io.retry.poke(false.B)
    dut.io.retrySqindex.poke(0.U)
    dut.io.retrySqindexHigh.poke(false.B)
    dut.io.mshrProgress.poke(false.B)
    dut.io.sqValid.poke(false.B)
    dut.io.sqindex.poke(0.U)
    dut.io.sqindexHigh.poke(false.B)
    dut.io.dcacheReady.poke(false.B)
    dut.reset.poke(true.B)
    dut.clock.step(2)
    dut.reset.poke(false.B)
    dut.clock.step()
  }

  private def expectNoTransfer(dut: StoreMshrBarrierHarness): Unit = {
    dut.io.sqReady.expect(false.B)
    dut.io.sqFire.expect(false.B)
    dut.io.sibEnqueueFire.expect(false.B)
    dut.io.sibDequeueFire.expect(false.B)
    dut.io.dcacheValid.expect(false.B)
    dut.io.dcacheFire.expect(false.B)
  }

  it should "clear the SIB and stop SQ and DCache until delayed progress" in {
    test(new StoreMshrBarrierHarness) { dut =>
      init(dut)
      dut.io.sqValid.poke(true.B)
      dut.io.sqindex.poke("b000100".U)
      dut.io.sqindexHigh.poke(true.B)

      dut.io.sqReady.expect(true.B)
      dut.io.sqFire.expect(true.B)
      dut.io.sibEnqueueFire.expect(true.B)
      dut.clock.step()
      dut.io.sibValid.expect(true.B)

      dut.io.retry.poke(true.B)
      dut.io.retrySqindex.poke("b000001".U)
      dut.io.retrySqindexHigh.poke(false.B)
      dut.io.dcacheReady.poke(true.B)
      expectNoTransfer(dut)
      dut.io.sibValid.expect(true.B)
      dut.io.orderedRetryValid.expect(true.B)
      dut.io.orderedRetrySqindex.expect("b000001".U)
      dut.io.orderedRetrySqindexHigh.expect(false.B)
      dut.clock.step()

      dut.io.retry.poke(false.B)
      dut.io.retrySqindex.poke(0.U)
      dut.io.sibValid.expect(false.B)
      dut.io.blocked.expect(true.B)
      expectNoTransfer(dut)
      dut.clock.step(2)
      dut.io.blocked.expect(true.B)
      expectNoTransfer(dut)

      dut.io.mshrProgress.poke(true.B)
      dut.io.delayedProgress.expect(false.B)
      expectNoTransfer(dut)
      dut.clock.step()
      dut.io.mshrProgress.poke(false.B)

      dut.io.delayedProgress.expect(true.B)
      dut.io.blocked.expect(true.B)
      expectNoTransfer(dut)
      dut.clock.step()

      dut.io.delayedProgress.expect(false.B)
      dut.io.blocked.expect(false.B)
      dut.io.sqReady.expect(true.B)
      dut.io.sqFire.expect(true.B)
      dut.io.sibEnqueueFire.expect(true.B)
      dut.io.dcacheFire.expect(false.B)
      dut.clock.step()

      dut.io.sqValid.poke(false.B)
      dut.io.sibValid.expect(true.B)
      dut.io.dcacheValid.expect(true.B)
      dut.io.dcacheFire.expect(true.B)
      dut.io.sibDequeueFire.expect(true.B)
      dut.clock.step()
      dut.io.sibValid.expect(false.B)
    }
  }

  it should "let delayed progress clear a simultaneous retry" in {
    test(new StoreMshrBarrierHarness) { dut =>
      init(dut)
      dut.io.sqValid.poke(true.B)
      dut.io.sqindex.poke("b000010".U)
      dut.io.dcacheReady.poke(true.B)

      dut.io.retry.poke(true.B)
      expectNoTransfer(dut)
      dut.clock.step()
      dut.io.retry.poke(false.B)
      dut.io.blocked.expect(true.B)
      expectNoTransfer(dut)

      dut.io.mshrProgress.poke(true.B)
      dut.clock.step()
      dut.io.mshrProgress.poke(false.B)
      dut.io.retry.poke(true.B)
      dut.io.delayedProgress.expect(true.B)
      dut.io.blocked.expect(true.B)
      expectNoTransfer(dut)
      dut.clock.step()

      dut.io.retry.poke(false.B)
      dut.io.delayedProgress.expect(false.B)
      dut.io.blocked.expect(false.B)
      dut.io.sqReady.expect(true.B)
      dut.io.sqFire.expect(true.B)
      dut.io.sibEnqueueFire.expect(true.B)
    }
  }
}
