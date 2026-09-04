package CPUSTC.memory.backend

import chisel3._
import chiseltest._
import CPUSTC.memory._
import org.scalatest.flatspec.AnyFlatSpec

class MissStatusHoldingRegisterTokenSpec
    extends AnyFlatSpec
    with ChiselScalatestTester {
  behavior of "MissStatusHoldingRegister compact Load waiters"

  private val lineBytes = DcacheConfig.DcacheLineBytes
  private val lineMask = (BigInt(1) << DcacheConfig.DcacheLineBits) - 1

  private def clearMetadata(metadata: MshrLoadMetadata): Unit = {
    metadata.token.poke(0.U)
    metadata.robPtr.qidx.poke(0.U)
    metadata.robPtr.offset.poke(0.U)
    metadata.robPtr.high.poke(false.B)
    metadata.robPtr.epoch.poke(0.U)
    metadata.pdest.poke(0.U)
    metadata.ldindexHigh.poke(false.B)
    metadata.format.poke(4.U)
    metadata.vaddrVpn.poke(0.U)
  }

  private def driveMetadata(metadata: MshrLoadMetadata, token: Int): Unit = {
    clearMetadata(metadata)
    metadata.token.poke(token.U)
    metadata.robPtr.qidx.poke((token & 0x3).U)
    metadata.robPtr.offset.poke((token + 1).U)
    metadata.robPtr.epoch.poke((token + 2).U)
    metadata.pdest.poke((token + 1).U)
    metadata.vaddrVpn.poke((0x80000 + token).U)
  }

  private def clearReq0(req: DcacheMshrPort0Req): Unit = {
    req.linePaddr.poke(0.U)
    req.byteOffset.poke(0.U)
    clearMetadata(req.loadMetadata)
    req.store.poke(false.B)
    req.storeData.poke(0.U)
    req.storeMask.poke(0.U)
    req.sqindex.poke(0.U)
    req.sqindexHigh.poke(false.B)
  }

  private def clearReq1(req: DcacheMshrPort1Req): Unit = {
    req.linePaddr.poke(0.U)
    req.byteOffset.poke(0.U)
    clearMetadata(req.loadMetadata)
  }

  private def init(dut: MissStatusHoldingRegister): Unit = {
    dut.io.req0.valid.poke(false.B)
    dut.io.req1.valid.poke(false.B)
    clearReq0(dut.io.req0.bits)
    clearReq1(dut.io.req1.bits)
    dut.io.victimReq.valid.poke(false.B)
    dut.io.victimReq.bits.paddr.poke(0.U)
    dut.io.victimReq.bits.data.poke(0.U)
    dut.io.refill.ready.poke(false.B)
    dut.io.loadWaiterFlush.poke(false.B)

    dut.io.memory.readReq.ready.poke(false.B)
    dut.io.memory.readResp.valid.poke(false.B)
    dut.io.memory.readResp.bits.paddr.poke(0.U)
    dut.io.memory.readResp.bits.data.poke(0.U)
    dut.io.memory.readResp.bits.dirty.poke(false.B)
    dut.io.memory.writeReq.ready.poke(true.B)
    dut.io.memory.writeResp.valid.poke(false.B)
    dut.io.memory.writeResp.bits.paddr.poke(0.U)

    dut.reset.poke(true.B)
    dut.clock.step(2)
    dut.reset.poke(false.B)
    dut.clock.step()
  }

  private def driveLoad0(
      req: DcacheMshrPort0Req,
      lineBase: BigInt,
      token: Int,
      byteOffset: Int = 0
  ): Unit = {
    clearReq0(req)
    req.linePaddr.poke((lineBase >> DcacheConfig.DcacheOffset).U)
    req.byteOffset.poke(byteOffset.U)
    driveMetadata(req.loadMetadata, token)
  }

  private def pulseLoad(
      dut: MissStatusHoldingRegister,
      lineBase: BigInt,
      token: Int,
      byteOffset: Int = 0
  ): Unit = {
    driveLoad0(dut.io.req0.bits, lineBase, token, byteOffset)
    dut.io.req0.valid.poke(true.B)
    dut.io.req0.ready.expect(true.B)
    dut.clock.step()
    dut.io.req0.valid.poke(false.B)
    clearReq0(dut.io.req0.bits)
  }

  private def driveLoad1(
      req: DcacheMshrPort1Req,
      lineBase: BigInt,
      token: Int,
      byteOffset: Int
  ): Unit = {
    clearReq1(req)
    req.linePaddr.poke((lineBase >> DcacheConfig.DcacheOffset).U)
    req.byteOffset.poke(byteOffset.U)
    driveMetadata(req.loadMetadata, token)
  }

  private def pulseStore(
      dut: MissStatusHoldingRegister,
      lineBase: BigInt,
      byteOffset: Int,
      data: BigInt,
      mask: Int,
      sqSlot: Int,
      sqHigh: Boolean = false
  ): Unit = {
    val req = dut.io.req0.bits
    clearReq0(req)
    req.linePaddr.poke((lineBase >> DcacheConfig.DcacheOffset).U)
    req.byteOffset.poke(byteOffset.U)
    req.store.poke(true.B)
    req.storeData.poke(data.U)
    req.storeMask.poke(mask.U)
    req.sqindex.poke((BigInt(1) << sqSlot).U)
    req.sqindexHigh.poke(sqHigh.B)
    dut.io.req0.valid.poke(true.B)
    dut.io.req0.ready.expect(true.B)
    dut.clock.step()
    dut.io.req0.valid.poke(false.B)
    clearReq0(req)
  }

  private def startReadResponse(
      dut: MissStatusHoldingRegister,
      lineBase: BigInt,
      data: BigInt,
      dirty: Boolean = false
  ): Unit = {
    dut.io.memory.readReq.valid.expect(true.B)
    dut.io.memory.readReq.bits.paddr.expect(lineBase.U)
    dut.io.memory.readReq.ready.poke(true.B)
    dut.clock.step()
    dut.io.memory.readReq.ready.poke(false.B)

    dut.io.memory.readResp.valid.poke(true.B)
    dut.io.memory.readResp.bits.paddr.poke(lineBase.U)
    dut.io.memory.readResp.bits.data.poke(data.U)
    dut.io.memory.readResp.bits.dirty.poke(dirty.B)
    dut.io.memory.readResp.ready.expect(true.B)
  }

  private def finishReadResponse(
      dut: MissStatusHoldingRegister,
      lineBase: BigInt
  ): Unit = {
    dut.clock.step()
    dut.io.memory.readResp.valid.poke(false.B)
    dut.io.refill.valid.expect(true.B)
    dut.io.refill.bits.paddr.expect(lineBase.U)
    expectNoTokens(dut)
  }

  private def expectTokens(
      dut: MissStatusHoldingRegister,
      expected: Seq[Option[(Int, BigInt)]]
  ): Unit = {
    require(expected.length == DcacheConfig.nPorts)
    expected.zipWithIndex.foreach { case (item, lane) =>
      dut.io.loadReturn(lane).valid.expect(item.nonEmpty.B)
      item.foreach { case (token, word) =>
        dut.io.loadReturn(lane).bits.waiter.metadata.token.expect(token.U)
        dut.io.loadReturn(lane).bits.data.expect(word.U)
      }
    }
  }

  private def expectNoTokens(dut: MissStatusHoldingRegister): Unit = {
    dut.io.loadReturn.foreach(_.valid.expect(false.B))
  }

  private def mergeStoreBytes(
      line: BigInt,
      byteOffset: Int,
      data: BigInt,
      mask: Int
  ): BigInt = {
    (0 until DcacheConfig.DcacheMaskBits).foldLeft(line) { (merged, byte) =>
      if ((mask & (1 << byte)) == 0) merged
      else {
        val shift = (byteOffset + byte) * 8
        val byteMask = BigInt(0xff) << shift
        (merged & ~byteMask) | (((data >> (byte * 8)) & 0xff) << shift)
      }
    } & lineMask
  }

  private def alignedWord(line: BigInt, byteOffset: Int): BigInt = {
    val wordBase = byteOffset & ~0x3
    val selectedWord =
      (line >> (wordBase * 8)) & BigInt("ffffffff", 16)
    (selectedWord >> ((byteOffset & 0x3) * 8)) & BigInt("ffffffff", 16)
  }

  private def fillTokens(
      dut: MissStatusHoldingRegister,
      lineBase: BigInt,
      waiters: Seq[(Int, Int)]
  ): Unit = {
    waiters.foreach { case (token, byteOffset) =>
      pulseLoad(dut, lineBase, token, byteOffset)
    }
  }

  private def expectRefillWait(
      dut: MissStatusHoldingRegister,
      cycles: Int
  ): Unit = {
    require(cycles >= 0)
    for (_ <- 0 until cycles) {
      dut.io.refill.valid.expect(true.B)
      dut.io.refill.ready.expect(false.B)
      dut.io.progress.expect(false.B)
      expectNoTokens(dut)
      dut.clock.step()
    }
  }

  private def beginRefillFire(
      dut: MissStatusHoldingRegister,
      expected: Seq[Option[(Int, BigInt)]]
  ): Unit = {
    dut.io.refill.valid.expect(true.B)
    dut.io.refill.ready.poke(true.B)
    dut.io.progress.expect(true.B)
    expectTokens(dut, expected)
  }

  private def stepToRestart(
      dut: MissStatusHoldingRegister,
      expected: Seq[Option[(Int, BigInt)]]
  ): Unit = {
    dut.clock.step()
    dut.io.refill.ready.poke(false.B)
    dut.io.progress.expect(false.B)
    dut.io.refill.valid.expect(false.B)
    expectTokens(dut, expected)
  }

  private def finishRestart(dut: MissStatusHoldingRegister): Unit = {
    dut.clock.step()
    expectNoTokens(dut)
  }

  it should "wait for refillFire, then return slots 0/1 at F and 2/3 at S" in {
    test(new MissStatusHoldingRegister) { dut =>
      init(dut)
      val base = BigInt("90000000", 16)
      val line = (0 until lineBytes).foldLeft(BigInt(0)) { (value, byte) =>
        value | (BigInt(byte) << (byte * 8))
      }
      val waiters = Seq((0, 0), (1, 5), (2, 10), (3, 15))
      fillTokens(dut, base, waiters)

      startReadResponse(dut, base, line)
      dut.io.refill.valid.expect(false.B)
      expectNoTokens(dut)

      finishReadResponse(dut, base)
      dut.io.refill.bits.data.expect(line.U)
      expectRefillWait(dut, cycles = 3)
      beginRefillFire(dut, Seq(
        Some((0, alignedWord(line, 0))),
        Some((1, alignedWord(line, 5)))
      ))
      stepToRestart(dut, Seq(
        Some((2, alignedWord(line, 10))),
        Some((3, alignedWord(line, 15)))
      ))
      dut.io.idle.expect(false.B)
      finishRestart(dut)
      dut.io.idle.expect(true.B)
    }
  }

  it should "reject a fifth same-line waiter" in {
    test(new MissStatusHoldingRegister) { dut =>
      init(dut)
      val base = BigInt("91000000", 16)
      val line = BigInt("8877665544332211", 16)
      val waiters = Seq((0, 0), (1, 4), (2, 8), (3, 12))
      fillTokens(dut, base, waiters)

      driveLoad0(dut.io.req0.bits, base, token = 4, byteOffset = 16)
      dut.io.req0.valid.poke(true.B)
      dut.io.req0.ready.expect(false.B)
      dut.io.req0.valid.poke(false.B)
      clearReq0(dut.io.req0.bits)

      startReadResponse(dut, base, line)
      expectNoTokens(dut)
      finishReadResponse(dut, base)
      beginRefillFire(dut, Seq(
        Some((0, alignedWord(line, 0))),
        Some((1, alignedWord(line, 4)))
      ))
      stepToRestart(dut, Seq(
        Some((2, alignedWord(line, 8))),
        Some((3, alignedWord(line, 12)))
      ))
      finishRestart(dut)
      dut.io.memory.readReq.valid.expect(false.B)
      dut.io.idle.expect(true.B)
    }
  }

  it should "give port 1 the final waiter slot when both ports target it" in {
    test(new MissStatusHoldingRegister) { dut =>
      init(dut)
      val base = BigInt("91400000", 16)
      val line = BigInt("887766554433221100ffeeddccbbaa99", 16)
      fillTokens(dut, base, Seq((0, 0), (1, 4), (2, 8)))

      driveLoad0(dut.io.req0.bits, base, token = 3, byteOffset = 12)
      driveLoad1(dut.io.req1.bits, base, token = 4, byteOffset = 16)
      dut.io.req0.valid.poke(true.B)
      dut.io.req1.valid.poke(true.B)
      dut.io.req1.ready.expect(true.B)
      dut.io.req0.ready.expect(false.B)
      dut.clock.step()
      dut.io.req0.valid.poke(false.B)
      dut.io.req1.valid.poke(false.B)
      clearReq0(dut.io.req0.bits)
      clearReq1(dut.io.req1.bits)

      startReadResponse(dut, base, line)
      expectNoTokens(dut)
      finishReadResponse(dut, base)
      beginRefillFire(dut, Seq(
        Some((0, alignedWord(line, 0))),
        Some((1, alignedWord(line, 4)))
      ))
      stepToRestart(dut, Seq(
        Some((2, alignedWord(line, 8))),
        Some((4, alignedWord(line, 16)))
      ))
      finishRestart(dut)
    }
  }

  it should "coalesce Loads but backpressure Stores when line capacity is reserved" in {
    test(new MissStatusHoldingRegister) { dut =>
      init(dut)
      val bases = Seq.tabulate(MshrConfig.length) { index =>
        BigInt("94000000", 16) + index * lineBytes
      }

      pulseLoad(dut, bases(0), token = 0)
      pulseLoad(dut, bases(1), token = 1)

      driveLoad0(dut.io.req0.bits, bases(2), token = 2)
      driveLoad1(dut.io.req1.bits, bases(3), token = 3, byteOffset = 4)
      dut.io.req0.valid.poke(true.B)
      dut.io.req1.valid.poke(true.B)
      dut.io.req0.ready.expect(true.B)
      dut.io.req1.ready.expect(true.B)
      dut.clock.step()
      dut.io.req0.valid.poke(false.B)
      dut.io.req1.valid.poke(false.B)

      driveLoad0(dut.io.req0.bits, bases(3), token = 4, byteOffset = 8)
      dut.io.req0.valid.poke(true.B)
      dut.io.req0.ready.expect(true.B)
      dut.clock.step()
      dut.io.req0.valid.poke(false.B)

      driveLoad1(dut.io.req1.bits, bases(3), token = 5, byteOffset = 12)
      dut.io.req1.valid.poke(true.B)
      dut.io.req1.ready.expect(true.B)
      dut.clock.step()
      dut.io.req1.valid.poke(false.B)

      val store = dut.io.req0.bits
      clearReq0(store)
      store.linePaddr.poke(
        (bases(3) >> DcacheConfig.DcacheOffset).U
      )
      store.store.poke(true.B)
      store.storeData.poke("hdeadbeef".U)
      store.storeMask.poke("b1111".U)
      store.sqindex.poke("b000001".U)
      dut.io.req0.valid.poke(true.B)
      dut.io.req0.ready.expect(false.B)
    }
  }

  it should "preserve reachable allocation and Store-merge boundaries from empty to full" in {
    test(new MissStatusHoldingRegister) { dut =>
      init(dut)
      val baseA = BigInt("95000000", 16)
      val baseB = baseA + lineBytes
      val baseC = baseB + lineBytes
      val baseD = baseC + lineBytes
      val lineA = BigInt("8877665544332211", 16)
      val lineB = BigInt("0123456789abcdef", 16) << 128
      val lineC = BigInt("10203040", 16)
      val lineD = BigInt("50607080", 16) << 32
      val storeData = BigInt("deadbeef", 16)
      val storeOffset = 8
      val mergedB = mergeStoreBytes(lineB, storeOffset, storeData, 0xf)

      // count 0 -> 1: both ports allocate one new line. Port 1 owns slot 0
      // by construction and port 0 owns slot 1.
      driveLoad0(dut.io.req0.bits, baseA, token = 0, byteOffset = 0)
      driveLoad1(dut.io.req1.bits, baseA, token = 1, byteOffset = 4)
      dut.io.req0.valid.poke(true.B)
      dut.io.req1.valid.poke(true.B)
      dut.io.req0.ready.expect(true.B)
      dut.io.req1.ready.expect(true.B)
      dut.clock.step()
      dut.io.req0.valid.poke(false.B)
      dut.io.req1.valid.poke(false.B)
      clearReq0(dut.io.req0.bits)
      clearReq1(dut.io.req1.bits)

      // count 1 -> 2, then merge a Store into the highest reachable Store
      // target (entry 1).
      pulseLoad(dut, baseB, token = 2, byteOffset = storeOffset)
      pulseStore(
        dut,
        baseB,
        byteOffset = storeOffset,
        data = storeData,
        mask = 0xf,
        sqSlot = 3,
        sqHigh = true
      )

      // count 2 -> 4: the first allocation must land in entry 2 and the
      // second in entry 3.
      driveLoad0(dut.io.req0.bits, baseC, token = 3, byteOffset = 0)
      driveLoad1(dut.io.req1.bits, baseD, token = 4, byteOffset = 4)
      dut.io.req0.valid.poke(true.B)
      dut.io.req1.valid.poke(true.B)
      dut.io.req0.ready.expect(true.B)
      dut.io.req1.ready.expect(true.B)
      dut.clock.step()
      dut.io.req0.valid.poke(false.B)
      dut.io.req1.valid.poke(false.B)
      clearReq0(dut.io.req0.bits)
      clearReq1(dut.io.req1.bits)

      startReadResponse(dut, baseA, lineA)
      finishReadResponse(dut, baseA)

      // Even an existing-line Load is refused while this refill retires.
      driveLoad0(dut.io.req0.bits, baseC, token = 5, byteOffset = 0)
      dut.io.req0.valid.poke(true.B)
      dut.io.req0.ready.expect(false.B)
      beginRefillFire(dut, Seq(
        Some((1, alignedWord(lineA, 4))),
        Some((0, alignedWord(lineA, 0)))
      ))
      stepToRestart(dut, Seq(None, None))
      dut.io.req0.valid.poke(false.B)
      clearReq0(dut.io.req0.bits)

      startReadResponse(dut, baseB, lineB)
      finishReadResponse(dut, baseB)
      dut.io.refill.bits.data.expect(mergedB.U)
      dut.io.refill.bits.dirty.expect(true.B)
      beginRefillFire(dut, Seq(
        Some((2, alignedWord(mergedB, storeOffset))),
        None
      ))
      dut.io.storeComplete.valid.expect(true.B)
      dut.io.storeComplete.bits.sqMask.expect("b001000".U)
      dut.io.storeComplete.bits.sqHighMask.expect("b001000".U)
      stepToRestart(dut, Seq(None, None))
      dut.io.storeComplete.valid.expect(false.B)

      startReadResponse(dut, baseC, lineC)
      finishReadResponse(dut, baseC)
      dut.io.refill.bits.data.expect(lineC.U)
      beginRefillFire(dut, Seq(
        Some((3, alignedWord(lineC, 0))),
        None
      ))
      stepToRestart(dut, Seq(None, None))

      startReadResponse(dut, baseD, lineD)
      finishReadResponse(dut, baseD)
      dut.io.refill.bits.data.expect(lineD.U)
      beginRefillFire(dut, Seq(
        Some((4, alignedWord(lineD, 4))),
        None
      ))
      stepToRestart(dut, Seq(None, None))
      finishRestart(dut)
      dut.io.idle.expect(true.B)
    }
  }

  it should "merge a Store while port 1 appends a waiter to the same line" in {
    test(new MissStatusHoldingRegister) { dut =>
      init(dut)
      val base = BigInt("91600000", 16)
      val lower = BigInt("0123456789abcdef", 16) << 64
      val storeData = BigInt("deadbeef", 16)
      val storeOffset = 8
      val expected = mergeStoreBytes(lower, storeOffset, storeData, 0xf)
      pulseLoad(dut, base, token = 0, byteOffset = storeOffset)

      clearReq0(dut.io.req0.bits)
      dut.io.req0.bits.linePaddr.poke(
        (base >> DcacheConfig.DcacheOffset).U
      )
      dut.io.req0.bits.byteOffset.poke(storeOffset.U)
      dut.io.req0.bits.store.poke(true.B)
      dut.io.req0.bits.storeData.poke(storeData.U)
      dut.io.req0.bits.storeMask.poke("b1111".U)
      dut.io.req0.bits.sqindex.poke("b000010".U)
      driveLoad1(dut.io.req1.bits, base, token = 1, byteOffset = storeOffset)
      dut.io.req0.valid.poke(true.B)
      dut.io.req1.valid.poke(true.B)
      dut.io.req0.ready.expect(true.B)
      dut.io.req1.ready.expect(true.B)
      dut.clock.step()
      dut.io.req0.valid.poke(false.B)
      dut.io.req1.valid.poke(false.B)
      clearReq0(dut.io.req0.bits)
      clearReq1(dut.io.req1.bits)

      startReadResponse(dut, base, lower)
      expectNoTokens(dut)
      finishReadResponse(dut, base)
      dut.io.refill.bits.data.expect(expected.U)
      dut.io.refill.bits.dirty.expect(true.B)
      beginRefillFire(dut, Seq(
        Some((0, alignedWord(expected, storeOffset))),
        Some((1, alignedWord(expected, storeOffset)))
      ))
      dut.io.storeComplete.valid.expect(true.B)
      dut.io.storeComplete.bits.sqMask.expect("b000010".U)
      stepToRestart(dut, Seq(None, None))
      dut.io.storeComplete.valid.expect(false.B)
      finishRestart(dut)
      dut.io.idle.expect(true.B)
    }
  }

  it should "backpressure both request ports while readResp fires" in {
    test(new MissStatusHoldingRegister) { dut =>
      init(dut)
      val base = BigInt("91800000", 16)
      val lower = BigInt("0123456789abcdef", 16) << 64
      val storeData = BigInt("deadbeef", 16)
      val storeOffset = 8
      pulseLoad(dut, base, token = 0, byteOffset = storeOffset)
      startReadResponse(dut, base, lower)

      clearReq0(dut.io.req0.bits)
      dut.io.req0.bits.linePaddr.poke(
        (base >> DcacheConfig.DcacheOffset).U
      )
      dut.io.req0.bits.byteOffset.poke(storeOffset.U)
      dut.io.req0.bits.store.poke(true.B)
      dut.io.req0.bits.storeData.poke(storeData.U)
      dut.io.req0.bits.storeMask.poke("b1111".U)
      dut.io.req0.bits.sqindex.poke("b000010".U)
      dut.io.req0.valid.poke(true.B)
      dut.io.req0.ready.expect(false.B)
      driveLoad1(
        dut.io.req1.bits,
        base + lineBytes,
        token = 1,
        byteOffset = storeOffset
      )
      dut.io.req1.valid.poke(true.B)
      dut.io.req1.ready.expect(false.B)

      expectNoTokens(dut)
      finishReadResponse(dut, base)

      dut.io.req0.valid.poke(false.B)
      dut.io.req1.valid.poke(false.B)
      clearReq0(dut.io.req0.bits)
      clearReq1(dut.io.req1.bits)
      beginRefillFire(dut, Seq(
        Some((0, alignedWord(lower, storeOffset))),
        None
      ))
      dut.io.refill.bits.data.expect(lower.U)
      dut.io.refill.bits.dirty.expect(false.B)
      dut.io.storeComplete.valid.expect(false.B)
      stepToRestart(dut, Seq(None, None))
      finishRestart(dut)
      dut.io.idle.expect(true.B)
    }
  }

  it should "suppress response-cycle tokens on flush without dropping Store refill work" in {
    test(new MissStatusHoldingRegister) { dut =>
      init(dut)
      val base = BigInt("92000000", 16)
      val lower = BigInt("11223344", 16) << 64
      val storeData = BigInt("deadbeef", 16)
      val expected = mergeStoreBytes(lower, 8, storeData, 0xf)

      pulseStore(dut, base, 8, storeData, 0xf, sqSlot = 2, sqHigh = true)
      fillTokens(dut, base, Seq((0, 8), (1, 9), (2, 10), (3, 11)))
      startReadResponse(dut, base, lower)
      dut.io.loadWaiterFlush.poke(true.B)
      expectNoTokens(dut)
      finishReadResponse(dut, base)
      dut.io.loadWaiterFlush.poke(false.B)

      dut.io.refill.bits.data.expect(expected.U)
      dut.io.refill.bits.dirty.expect(true.B)
      expectRefillWait(dut, cycles = 2)
      beginRefillFire(dut, Seq(None, None))
      dut.io.storeComplete.valid.expect(true.B)
      dut.io.storeComplete.bits.paddr.expect(base.U)
      dut.io.storeComplete.bits.sqMask.expect("b000100".U)
      dut.io.storeComplete.bits.sqHighMask.expect("b000100".U)
      stepToRestart(dut, Seq(None, None))
      dut.io.storeComplete.valid.expect(false.B)
      finishRestart(dut)
      dut.io.idle.expect(true.B)
    }
  }

  it should "clear captured waiters when flush arrives while refill waits" in {
    test(new MissStatusHoldingRegister) { dut =>
      init(dut)
      val base = BigInt("93000000", 16)
      val line = BigInt("ffeeddccbbaa99887766554433221100", 16)
      fillTokens(dut, base, Seq((0, 0), (1, 4), (2, 8), (3, 12)))
      startReadResponse(dut, base, line)
      expectNoTokens(dut)
      finishReadResponse(dut, base)
      expectRefillWait(dut, cycles = 2)

      dut.io.loadWaiterFlush.poke(true.B)
      expectNoTokens(dut)
      dut.clock.step()
      dut.io.loadWaiterFlush.poke(false.B)
      expectRefillWait(dut, cycles = 2)
      dut.io.refill.bits.data.expect(line.U)
      beginRefillFire(dut, Seq(None, None))
      stepToRestart(dut, Seq(None, None))
      finishRestart(dut)
      dut.io.idle.expect(true.B)
    }
  }

  it should "suppress both phases when flush coincides with refillFire" in {
    test(new MissStatusHoldingRegister) { dut =>
      init(dut)
      val base = BigInt("93200000", 16)
      val line = BigInt("00112233445566778899aabbccddeeff", 16)
      fillTokens(dut, base, Seq((0, 0), (1, 4), (2, 8), (3, 12)))
      startReadResponse(dut, base, line)
      finishReadResponse(dut, base)
      expectRefillWait(dut, cycles = 1)

      dut.io.loadWaiterFlush.poke(true.B)
      beginRefillFire(dut, Seq(None, None))
      dut.clock.step()
      dut.io.refill.ready.poke(false.B)
      dut.io.loadWaiterFlush.poke(false.B)
      dut.io.progress.expect(false.B)
      dut.io.refill.valid.expect(false.B)
      expectNoTokens(dut)
      finishRestart(dut)
      dut.io.idle.expect(true.B)
    }
  }

  it should "return slots 0/1 at F but suppress slots 2/3 on an S flush" in {
    test(new MissStatusHoldingRegister) { dut =>
      init(dut)
      val base = BigInt("93400000", 16)
      val line = BigInt("0123456789abcdeffedcba9876543210", 16)
      fillTokens(dut, base, Seq((0, 0), (1, 4), (2, 8), (3, 12)))
      startReadResponse(dut, base, line)
      finishReadResponse(dut, base)
      beginRefillFire(dut, Seq(
        Some((0, alignedWord(line, 0))),
        Some((1, alignedWord(line, 4)))
      ))
      dut.clock.step()
      dut.io.refill.ready.poke(false.B)
      dut.io.progress.expect(false.B)
      dut.io.refill.valid.expect(false.B)

      dut.io.loadWaiterFlush.poke(true.B)
      expectNoTokens(dut)
      dut.clock.step()
      dut.io.loadWaiterFlush.poke(false.B)
      expectNoTokens(dut)
      dut.io.idle.expect(true.B)
    }
  }
}
