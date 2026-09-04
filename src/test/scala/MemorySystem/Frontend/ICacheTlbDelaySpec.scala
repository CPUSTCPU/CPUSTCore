package CPUSTC.memory.frontend

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

import CPUSTC.memory.IcacheConfig

class ICacheTlbDelaySpec extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "ICache"

  it should "keep a current hit paired with its data while the next TLB response is delayed" in {
    test(new ICache(useBlackBoxRam = false)) { dut =>
      val fullMask = (1 << IcacheConfig.nfetch) - 1
      val lineMask = IcacheConfig.IcacheLineBytes - 1
      val aPc = BigInt("00001000", 16)
      val bPc = BigInt("00001040", 16)
      val aFetch = BigInt("11112222333344445555666677778888", 16)
      val bFetch = BigInt("aaaabbbbccccddddeeeeffff00001234", 16)
      val aLine = aFetch
      val bLine = bFetch

      def driveTlbBits(pc: BigInt): Unit = {
        dut.io.tlb.resp.bits.paddr.poke(pc.U)
        dut.io.tlb.resp.bits.uncache.poke(false.B)
        dut.io.tlb.resp.bits.exception.poke(0.U)
        dut.io.tlb.resp.bits.token.poke((pc & 0xffff).U)
        dut.io.tlb.resp.bits.deferredFetchCheck.poke(false.B)
        dut.io.tlb.resp.bits.fetchPageValid.poke(true.B)
        dut.io.tlb.resp.bits.fetchPagePlv.poke(0.U)
        dut.io.tlb.resp.bits.fetchRequestPlv.poke(0.U)
      }

      def waitUntil(signal: Bool, clue: String, limit: Int = 32): Unit = {
        var cycles = 0
        while (!signal.peek().litToBoolean && cycles < limit) {
          dut.clock.step()
          cycles += 1
        }
        assert(signal.peek().litToBoolean, s"timeout waiting for $clue")
      }

      def acceptFetch(pc: BigInt): Unit = {
        dut.io.pp.req.bits.pc.poke(pc.U)
        dut.io.pp.req.bits.mask.poke(fullMask.U)
        dut.io.pp.req.valid.poke(true.B)
        waitUntil(dut.io.pp.req.ready, f"fetch request 0x$pc%x ready")
        dut.io.tlb.req.valid.expect(true.B)
        dut.io.tlb.req.bits.vaddr.expect(pc.U)
        dut.clock.step()
        dut.io.pp.req.valid.poke(false.B)
      }

      def returnTranslation(pc: BigInt): Unit = {
        driveTlbBits(pc)
        dut.io.tlb.resp.valid.poke(true.B)
        waitUntil(dut.io.tlb.resp.ready, f"TLB response 0x$pc%x ready")
        dut.clock.step()
        dut.io.tlb.resp.valid.poke(false.B)
      }

      def fillLine(pc: BigInt, line: BigInt, expectedFetch: BigInt): Unit = {
        acceptFetch(pc)
        returnTranslation(pc)

        waitUntil(dut.io.missReq.req.valid, f"miss request 0x$pc%x")
        dut.io.missReq.req.bits.paddr.expect((pc & ~BigInt(lineMask)).U)
        dut.io.missReq.req.bits.cacheable.expect(true.B)
        dut.clock.step()

        dut.io.missReq.resp.bits.refillLine.poke(line.U)
        dut.io.missReq.resp.valid.poke(true.B)
        dut.io.missReq.resp.ready.expect(true.B)
        dut.clock.step()
        dut.io.missReq.resp.valid.poke(false.B)

        waitUntil(dut.io.pp.resp.valid, f"refill response 0x$pc%x")
        dut.io.pp.resp.bits.pc.expect(pc.U)
        dut.io.pp.resp.bits.instrs.expect(expectedFetch.U)
        dut.clock.step()
        dut.clock.step()
      }

      dut.io.pp.req.valid.poke(false.B)
      dut.io.pp.req.bits.pc.poke(0.U)
      dut.io.pp.req.bits.mask.poke(0.U)
      dut.io.pp.resp.ready.poke(true.B)
      dut.io.tlb.req.ready.poke(true.B)
      dut.io.tlb.resp.valid.poke(false.B)
      driveTlbBits(0)
      dut.io.missReq.req.ready.poke(true.B)
      dut.io.missReq.resp.valid.poke(false.B)
      dut.io.missReq.resp.bits.refillLine.poke(0.U)
      dut.io.flush.poke(false.B)
      dut.io.redirect.poke(false.B)
      dut.io.invalidate.poke(false.B)

      dut.reset.poke(true.B)
      dut.clock.step(2)
      dut.reset.poke(false.B)
      dut.clock.step()

      fillLine(aPc, aLine, aFetch)
      fillLine(bPc, bLine, bFetch)

      // Put A into c1s1, then accept B on the same edge that A's translation
      // advances. B remains in c1s1 while the already translated A hit waits
      // in c1s2.
      acceptFetch(aPc)
      driveTlbBits(aPc)
      dut.io.tlb.resp.valid.poke(true.B)
      dut.io.pp.req.bits.pc.poke(bPc.U)
      dut.io.pp.req.bits.mask.poke(fullMask.U)
      dut.io.pp.req.valid.poke(true.B)
      dut.io.tlb.resp.ready.expect(true.B)
      dut.io.pp.req.ready.expect(true.B)
      dut.clock.step()
      dut.io.tlb.resp.valid.poke(false.B)
      dut.io.pp.req.valid.poke(false.B)

      // Keep invalid response payload deterministic. The old implementation
      // repeatedly selected B's RAM address here even though B had no valid
      // translation response, replacing the data associated with A.
      driveTlbBits(bPc)
      dut.clock.step(3)
      dut.io.pp.resp.valid.expect(false.B)

      dut.io.tlb.resp.valid.poke(true.B)
      dut.io.tlb.resp.ready.expect(true.B)
      dut.clock.step()
      dut.io.tlb.resp.valid.poke(false.B)

      dut.io.pp.resp.valid.expect(true.B)
      dut.io.pp.resp.bits.pc.expect(aPc.U)
      dut.io.pp.resp.bits.instrs.expect(aFetch.U)
    }
  }
}
