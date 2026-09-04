package CPUSTC.predict

import chisel3._
import chiseltest._
import circt.stage.ChiselStage
import org.scalatest.flatspec.AnyFlatSpec

class LoopPredictorSpec extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "LoopPredictor"

  private case class EntrySnapshot(
      valid: Boolean,
      packetTag: BigInt,
      slot: BigInt,
      loopDirection: Boolean,
      numIter: BigInt,
      currentIter: BigInt,
      confidence: BigInt,
      age: BigInt
  )

  private case class MetaSnapshot(
      valid: Boolean,
      hit: Boolean,
      index: BigInt,
      packetTag: BigInt,
      entry: EntrySnapshot
  )

  private case class PredictionSnapshot(
      meta: MetaSnapshot,
      shadowValid: Boolean,
      shadowTaken: Boolean,
      overrideValid: Boolean,
      overrideTaken: Boolean,
      slot: BigInt
  )

  private def bool(value: Bool): Boolean = value.peek().litToBoolean

  private def captureEntry(entry: LoopPredictorEntry): EntrySnapshot =
    EntrySnapshot(
      bool(entry.valid),
      entry.packetTag.peek().litValue,
      entry.slot.peek().litValue,
      bool(entry.loopDirection),
      entry.numIter.peek().litValue,
      entry.currentIter.peek().litValue,
      entry.confidence.peek().litValue,
      entry.age.peek().litValue
    )

  private def captureMeta(meta: LoopPredictorMeta): MetaSnapshot =
    MetaSnapshot(
      bool(meta.valid),
      bool(meta.hit),
      meta.index.peek().litValue,
      meta.packetTag.peek().litValue,
      captureEntry(meta.entry)
    )

  private def pokeEntry(
      entry: LoopPredictorEntry,
      snapshot: EntrySnapshot
  ): Unit = {
    entry.valid.poke(snapshot.valid.B)
    entry.packetTag.poke(snapshot.packetTag.U)
    entry.slot.poke(snapshot.slot.U)
    entry.loopDirection.poke(snapshot.loopDirection.B)
    entry.numIter.poke(snapshot.numIter.U)
    entry.currentIter.poke(snapshot.currentIter.U)
    entry.confidence.poke(snapshot.confidence.U)
    entry.age.poke(snapshot.age.U)
  }

  private def pokeMeta(meta: LoopPredictorMeta, snapshot: MetaSnapshot): Unit = {
    meta.valid.poke(snapshot.valid.B)
    meta.hit.poke(snapshot.hit.B)
    meta.index.poke(snapshot.index.U)
    meta.packetTag.poke(snapshot.packetTag.U)
    pokeEntry(meta.entry, snapshot.entry)
  }

  private val zeroEntry = EntrySnapshot(
    valid = false,
    packetTag = 0,
    slot = 0,
    loopDirection = false,
    numIter = 0,
    currentIter = 0,
    confidence = 0,
    age = 0
  )
  private val zeroMeta = MetaSnapshot(
    valid = false,
    hit = false,
    index = 0,
    packetTag = 0,
    entry = zeroEntry
  )

  private def initialize(dut: LoopPredictor): Unit = {
    dut.io.lookup.valid.poke(false.B)
    dut.io.lookup.bits.packetPc.poke(0.U)
    dut.io.lookup.bits.overrideSafe.poke(false.B)
    dut.io.flush.poke(false.B)
    dut.io.commit.valid.poke(false.B)
    dut.io.commit.bits.pc.poke(0.U)
    dut.io.commit.bits.taken.poke(false.B)
    dut.io.commit.bits.allocate.poke(false.B)
    pokeMeta(dut.io.commit.bits.meta, zeroMeta)

    dut.reset.poke(true.B)
    dut.clock.step(2)
    dut.reset.poke(false.B)

    var cycles = 0
    while (!bool(dut.io.ready) && cycles < 132) {
      dut.clock.step()
      cycles += 1
    }
    assert(bool(dut.io.ready), "startup RAM scrub did not finish")
  }

  private def lookup(
      dut: LoopPredictor,
      packetPc: BigInt,
      overrideSafe: Boolean = false
  ): PredictionSnapshot = {
    dut.io.lookup.valid.poke(true.B)
    dut.io.lookup.bits.packetPc.poke(packetPc.U)
    dut.io.lookup.bits.overrideSafe.poke(overrideSafe.B)
    dut.clock.step()
    dut.io.lookup.valid.poke(false.B)

    val meta = captureMeta(dut.io.meta)
    PredictionSnapshot(
      meta,
      bool(dut.io.shadow.valid),
      bool(dut.io.shadow.bits.taken),
      bool(dut.io.overridePrediction.valid),
      bool(dut.io.overridePrediction.bits.taken),
      dut.io.shadow.bits.slot.peek().litValue
    )
  }

  private def commit(
      dut: LoopPredictor,
      pc: BigInt,
      taken: Boolean,
      allocate: Boolean,
      meta: MetaSnapshot
  ): Unit = {
    dut.io.commit.valid.poke(true.B)
    dut.io.commit.bits.pc.poke(pc.U)
    dut.io.commit.bits.taken.poke(taken.B)
    dut.io.commit.bits.allocate.poke(allocate.B)
    pokeMeta(dut.io.commit.bits.meta, meta)
    dut.clock.step()
    dut.io.commit.valid.poke(false.B)
  }

  private def trainOutcome(
      dut: LoopPredictor,
      packetPc: BigInt,
      branchPc: BigInt,
      taken: Boolean,
      allocate: Boolean = false
  ): PredictionSnapshot = {
    val prediction = lookup(dut, packetPc)
    commit(dut, branchPc, taken, allocate, prediction.meta)
    prediction
  }

  private def trainCountOneLoop(
      dut: LoopPredictor,
      packetPc: BigInt,
      branchPc: BigInt
  ): Unit = {
    // The allocation event is the observed loop exit. Four complete T,F
    // sequences then learn numIter=1 and saturate 2-bit confidence.
    trainOutcome(
      dut,
      packetPc,
      branchPc,
      taken = false,
      allocate = true
    )
    for (_ <- 0 until 4) {
      trainOutcome(dut, packetPc, branchPc, taken = true)
      trainOutcome(dut, packetPc, branchPc, taken = false)
    }
  }

  it should "learn a stable loop and gate override with registered safety" in {
    test(new LoopPredictor(useBlackBoxRam = false)) { dut =>
      initialize(dut)
      val packetPc = BigInt("80001000", 16)
      val branchPc = packetPc + 8
      trainCountOneLoop(dut, packetPc, branchPc)

      val shadowOnly = lookup(dut, packetPc, overrideSafe = false)
      assert(shadowOnly.meta.hit)
      assert(shadowOnly.shadowValid)
      assert(shadowOnly.shadowTaken)
      assert(!shadowOnly.overrideValid)
      assert(shadowOnly.slot == 2)
      assert(shadowOnly.meta.entry.numIter == 1)
      assert(shadowOnly.meta.entry.currentIter == 0)
      assert(shadowOnly.meta.entry.confidence == 3)

      val safe = lookup(dut, packetPc, overrideSafe = true)
      assert(safe.overrideValid)
      assert(safe.overrideTaken)
    }
  }

  it should "expose commit lag in shadow mode without allowing an unsafe override" in {
    test(new LoopPredictor(useBlackBoxRam = false)) { dut =>
      initialize(dut)
      val packetPc = BigInt("80002000", 16)
      val branchPc = packetPc + 4
      trainCountOneLoop(dut, packetPc, branchPc)

      val first = lookup(dut, packetPc, overrideSafe = true)
      val younger = lookup(dut, packetPc, overrideSafe = false)

      // For a one-iteration loop the younger branch should be the exit, but
      // no commit occurred between these lookups. Both therefore see the old
      // committed currentIter=0 and produce the same shadow direction.
      assert(first.shadowValid && first.shadowTaken)
      assert(first.overrideValid)
      assert(younger.shadowValid && younger.shadowTaken)
      assert(!younger.overrideValid)
      assert(first.meta.entry.currentIter == 0)
      assert(younger.meta.entry.currentIter == 0)

      commit(dut, branchPc, taken = true, allocate = false, first.meta)
      val afterCommit = lookup(dut, packetPc, overrideSafe = true)
      assert(afterCommit.shadowValid)
      assert(!afterCommit.shadowTaken)
      assert(afterCommit.overrideValid)
      assert(!afterCommit.overrideTaken)
      assert(afterCommit.meta.entry.currentIter == 1)
    }
  }

  it should "use all 128 rows without confusing colliding full tags" in {
    test(new LoopPredictor(useBlackBoxRam = false)) { dut =>
      initialize(dut)

      for (row <- 0 until LoopPredictorConfig.entries) {
        val packetPc = BigInt(row) << LoopPredictorConfig.packetOffsetWidth
        val branchPc = packetPc + ((row & 3) << 2)
        val miss = lookup(dut, packetPc)
        assert(!miss.meta.hit)
        commit(dut, branchPc, taken = false, allocate = true, miss.meta)
      }

      for (row <- 0 until LoopPredictorConfig.entries) {
        val packetPc = BigInt(row) << LoopPredictorConfig.packetOffsetWidth
        val hit = lookup(dut, packetPc)
        assert(hit.meta.hit, s"row $row was not retained")
        assert(hit.meta.entry.slot == (row & 3))
      }

      val pcA = BigInt("00010000", 16)
      // Toggling packet-tag bits 0 and 7 preserves their folded index.
      val pcB = pcA ^ BigInt("00000810", 16)
      val indexA = lookup(dut, pcA).meta.index
      val collision = lookup(dut, pcB)
      assert(collision.meta.index == indexA)
      assert(!collision.meta.hit)

      commit(dut, pcB, taken = false, allocate = true, collision.meta)
      assert(lookup(dut, pcB).meta.hit)
      assert(!lookup(dut, pcA).meta.hit)
    }
  }

  it should "use current training state after five intervening row writes" in {
    test(new LoopPredictor(useBlackBoxRam = false)) { dut =>
      initialize(dut)
      val packetPc = BigInt(10) << LoopPredictorConfig.packetOffsetWidth
      val branchPc = packetPc

      val initialMiss = lookup(dut, packetPc)
      commit(
        dut,
        branchPc,
        taken = false,
        allocate = true,
        initialMiss.meta
      )

      // Both dynamic continuations were fetched from currentIter=0. The first
      // retires now; the second keeps this deliberately stale FTQ metadata.
      val staleMeta = lookup(dut, packetPc).meta
      commit(dut, branchPc, taken = true, allocate = false, staleMeta)

      for (row <- 20 until 25) {
        val otherPacket = BigInt(row) << LoopPredictorConfig.packetOffsetWidth
        val otherMiss = lookup(dut, otherPacket)
        commit(
          dut,
          otherPacket,
          taken = false,
          allocate = true,
          otherMiss.meta
        )
      }

      // A finite four-entry update FIFO would have forgotten the first write
      // and would incorrectly write currentIter=1 again from staleMeta.
      commit(dut, branchPc, taken = true, allocate = false, staleMeta)
      val updated = lookup(dut, packetPc)
      assert(updated.meta.hit)
      assert(updated.meta.entry.currentIter == 2)
    }
  }

  it should "forward consecutive same-row commits at one update per cycle" in {
    test(new LoopPredictor(useBlackBoxRam = false)) { dut =>
      initialize(dut)
      val packetPc = BigInt(40) << LoopPredictorConfig.packetOffsetWidth
      val branchPc = packetPc + 8

      val miss = lookup(dut, packetPc)
      commit(dut, branchPc, taken = false, allocate = true, miss.meta)
      val staleMeta = lookup(dut, packetPc).meta

      dut.io.commit.valid.poke(true.B)
      dut.io.commit.bits.pc.poke(branchPc.U)
      dut.io.commit.bits.taken.poke(true.B)
      dut.io.commit.bits.allocate.poke(false.B)
      pokeMeta(dut.io.commit.bits.meta, staleMeta)
      dut.clock.step(3)
      dut.io.commit.valid.poke(false.B)

      // Drain the last request in the two-stage training pipeline.
      dut.clock.step()
      val updated = lookup(dut, packetPc)
      assert(updated.meta.hit)
      assert(updated.meta.entry.currentIter == 3)
    }
  }

  it should "drop a late commit after a colliding tag replaces its row" in {
    test(new LoopPredictor(useBlackBoxRam = false)) { dut =>
      initialize(dut)
      val packetA = BigInt("000002a0", 16)
      val branchA = packetA
      val packetB = packetA ^ BigInt("00000810", 16)
      val branchB = packetB + 4

      val missA = lookup(dut, packetA)
      commit(dut, branchA, taken = false, allocate = true, missA.meta)
      val lateA = lookup(dut, packetA).meta
      assert(lateA.hit)

      val collisionB = lookup(dut, packetB)
      assert(collisionB.meta.index == lateA.index)
      assert(!collisionB.meta.hit)
      commit(dut, branchB, taken = false, allocate = true, collisionB.meta)
      assert(lookup(dut, packetB).meta.hit)

      // Evict any finite recent-write shortcut before the old A completion.
      for (row <- 70 until 75) {
        val otherPacket = BigInt(row) << LoopPredictorConfig.packetOffsetWidth
        val otherMiss = lookup(dut, otherPacket)
        commit(
          dut,
          otherPacket,
          taken = false,
          allocate = true,
          otherMiss.meta
        )
      }

      commit(dut, branchA, taken = true, allocate = false, lateA)
      assert(lookup(dut, packetB).meta.hit)
      assert(!lookup(dut, packetA).meta.hit)
    }
  }

  it should "saturate the committed iteration count without wrapping" in {
    test(new LoopPredictor(useBlackBoxRam = false)) { dut =>
      initialize(dut)
      val packetPc = BigInt("80003000", 16)
      val branchPc = packetPc + 12

      val miss = lookup(dut, packetPc)
      commit(dut, branchPc, taken = false, allocate = true, miss.meta)

      for (_ <- 0 until 270) {
        trainOutcome(dut, packetPc, branchPc, taken = true)
      }
      val saturated = lookup(dut, packetPc)
      assert(saturated.meta.hit)
      assert(saturated.meta.entry.currentIter == 255)

      commit(
        dut,
        branchPc,
        taken = false,
        allocate = false,
        saturated.meta
      )
      val learned = lookup(dut, packetPc)
      assert(learned.meta.entry.currentIter == 0)
      assert(learned.meta.entry.numIter == 255)
    }
  }

  it should "discard a pending lookup response on flush" in {
    test(new LoopPredictor(useBlackBoxRam = false)) { dut =>
      initialize(dut)
      dut.io.lookup.valid.poke(true.B)
      dut.io.lookup.bits.packetPc.poke("h80004000".U)
      dut.io.lookup.bits.overrideSafe.poke(false.B)
      dut.io.flush.poke(true.B)
      dut.clock.step()
      dut.io.lookup.valid.poke(false.B)
      dut.io.flush.poke(false.B)

      dut.io.meta.valid.expect(false.B)
      dut.io.shadow.valid.expect(false.B)
      dut.io.overridePrediction.valid.expect(false.B)
    }
  }

  it should "elaborate with the optional block-RAM blackbox" in {
    val systemVerilog = ChiselStage.emitSystemVerilog(
      new LoopPredictor(useBlackBoxRam = true)
    )
    assert(systemVerilog.contains("BpuSdpRamBlackBox"))
  }
}
