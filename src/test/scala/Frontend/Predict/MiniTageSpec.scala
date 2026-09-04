package CPUSTC.predict

import chisel3._
import chiseltest._
import circt.stage.ChiselStage
import org.scalatest.flatspec.AnyFlatSpec

import CPUSTC.config.Fetch.nfch

class MiniTageSpec extends AnyFlatSpec with ChiselScalatestTester {
    behavior of "MiniTage"

    private case class SlotMetaSnapshot(
        provider: BigInt,
        chooserCounter: BigInt,
        providerCounter: BigInt,
        providerUseful: BigInt,
        alternateTaken: Boolean,
        useAlternate: Boolean,
        tableUseful: Seq[BigInt]
    )

    private case class PredictionSnapshot(
        pc: BigInt,
        baseTaken: BigInt,
        conditionalMask: BigInt,
        candidateTaken: BigInt,
        providerHitMask: BigInt,
        correctionMask: BigInt,
        slots: Seq[SlotMetaSnapshot]
    )

    private def initialize(dut: MiniTage): Unit = {
        dut.io.lookup.valid.poke(false.B)
        dut.io.lookup.bits.pc.poke(0.U)
        dut.io.lookup.bits.history.poke(0.U)
        dut.io.base.valid.poke(false.B)
        dut.io.base.bits.pc.poke(0.U)
        dut.io.base.bits.baseTaken.poke(0.U)
        dut.io.base.bits.conditionalMask.poke(0.U)
        dut.io.train.valid.poke(false.B)
        dut.io.train.bits.pc.poke(0.U)
        dut.io.train.bits.history.poke(0.U)
        dut.io.train.bits.baseTaken.poke(0.U)
        dut.io.train.bits.trainMask.poke(0.U)
        dut.io.train.bits.takenMask.poke(0.U)
        for (slot <- 0 until nfch) {
            val meta = dut.io.train.bits.meta.slots(slot)
            meta.provider.poke(0.U)
            meta.chooserCounter.poke(0.U)
            meta.providerCounter.poke(0.U)
            meta.providerUseful.poke(0.U)
            meta.alternateTaken.poke(false.B)
            meta.useAlternate.poke(false.B)
            for (table <- 0 until MiniTageConfig.tableCount) {
                meta.tableUseful(table).poke(0.U)
            }
        }

        dut.reset.poke(true.B)
        dut.clock.step(2)
        dut.reset.poke(false.B)
        var cycles = 0
        while (!dut.io.ready.peek().litToBoolean && cycles < 520) {
            dut.clock.step()
            cycles += 1
        }
        assert(dut.io.ready.peek().litToBoolean, "MiniTAGE startup scrub timed out")
    }

    private def lookup(
        dut: MiniTage,
        pc: BigInt,
        history: BigInt,
        baseTaken: BigInt,
        conditionalMask: BigInt = 0xf
    ): PredictionSnapshot = {
        dut.io.lookup.bits.pc.poke(pc.U)
        dut.io.lookup.bits.history.poke(history.U)
        dut.io.lookup.valid.poke(true.B)
        dut.clock.step()
        dut.io.lookup.valid.poke(false.B)
        dut.io.base.bits.pc.poke(pc.U)
        dut.io.base.bits.baseTaken.poke(baseTaken.U)
        dut.io.base.bits.conditionalMask.poke(conditionalMask.U)
        dut.io.base.valid.poke(true.B)
        dut.clock.step()
        dut.io.base.valid.poke(false.B)

        var cycles = 0
        while (!dut.io.resp.valid.peek().litToBoolean && cycles < 5) {
            dut.clock.step()
            cycles += 1
        }
        assert(dut.io.resp.valid.peek().litToBoolean, "MiniTAGE response timed out")
        val result = PredictionSnapshot(
            dut.io.resp.bits.pc.peek().litValue,
            dut.io.resp.bits.baseTaken.peek().litValue,
            dut.io.resp.bits.conditionalMask.peek().litValue,
            dut.io.resp.bits.candidateTaken.peek().litValue,
            dut.io.resp.bits.providerHitMask.peek().litValue,
            dut.io.resp.bits.correctionMask.peek().litValue,
            (0 until nfch).map { slot =>
                val meta = dut.io.resp.bits.meta.slots(slot)
                SlotMetaSnapshot(
                    meta.provider.peek().litValue,
                    meta.chooserCounter.peek().litValue,
                    meta.providerCounter.peek().litValue,
                    meta.providerUseful.peek().litValue,
                    meta.alternateTaken.peek().litToBoolean,
                    meta.useAlternate.peek().litToBoolean,
                    (0 until MiniTageConfig.tableCount).map { table =>
                        meta.tableUseful(table).peek().litValue
                    }
                )
            }
        )
        dut.clock.step()
        result
    }

    private def driveTrain(
        dut: MiniTage,
        prediction: PredictionSnapshot,
        history: BigInt,
        trainMask: BigInt,
        takenMask: BigInt
    ): Unit = {
        dut.io.train.bits.pc.poke(prediction.pc.U)
        dut.io.train.bits.history.poke(history.U)
        dut.io.train.bits.baseTaken.poke(prediction.baseTaken.U)
        dut.io.train.bits.trainMask.poke(trainMask.U)
        dut.io.train.bits.takenMask.poke(takenMask.U)
        for (slot <- 0 until nfch) {
            val source = prediction.slots(slot)
            val sink = dut.io.train.bits.meta.slots(slot)
            sink.provider.poke(source.provider.U)
            sink.chooserCounter.poke(source.chooserCounter.U)
            sink.providerCounter.poke(source.providerCounter.U)
            sink.providerUseful.poke(source.providerUseful.U)
            sink.alternateTaken.poke(source.alternateTaken.B)
            sink.useAlternate.poke(source.useAlternate.B)
            for (table <- 0 until MiniTageConfig.tableCount) {
                sink.tableUseful(table).poke(source.tableUseful(table).U)
            }
        }
        dut.io.train.valid.poke(true.B)
    }

    private def train(
        dut: MiniTage,
        prediction: PredictionSnapshot,
        history: BigInt,
        trainMask: BigInt,
        takenMask: BigInt
    ): Unit = {
        driveTrain(dut, prediction, history, trainMask, takenMask)
        dut.clock.step()
        dut.io.train.valid.poke(false.B)
        // The duplicated training RAM reads at t0 and broadcasts its current
        // row update to both copies at t1.
        dut.clock.step()
    }

    it should "use Agree as base, allocate at most one entry, and select the longest provider" in {
        test(new MiniTage(useBlackBoxRam = false)) { dut =>
            initialize(dut)
            val pc = BigInt("1000", 16)
            val history = BigInt("89abcdef", 16)

            val cold = lookup(dut, pc, history, baseTaken = 0)
            assert(cold.providerHitMask == 0)
            assert(cold.candidateTaken == 0)
            assert(cold.correctionMask == 0)

            // Two misses retire together, but the packet allocates only the
            // first eligible slot.
            train(dut, cold, history, trainMask = 0x3, takenMask = 0x3)
            val afterFirstAlloc = lookup(dut, pc, history, baseTaken = 0)
            assert(afterFirstAlloc.slots(0).provider == 1)
            assert(afterFirstAlloc.slots(1).provider == 0)
            assert(afterFirstAlloc.slots(0).providerCounter == 4)
            assert(afterFirstAlloc.slots(0).useAlternate)
            assert((afterFirstAlloc.candidateTaken & 1) == 0)

            // A weak new T0 initially trusts Agree. Once T0 proves better,
            // useAltOnNa crosses toward provider and the miss allocates T1.
            train(dut, afterFirstAlloc, history, trainMask = 0x1, takenMask = 0x1)
            assert(dut.io.useAltOnNa.peek().litValue == 7)
            val withT1 = lookup(dut, pc, history, baseTaken = 0)
            assert(withT1.slots(0).provider == 2)
            assert(withT1.slots(0).chooserCounter == 0)
            assert(withT1.slots(0).providerCounter == 4)
            assert(!withT1.slots(0).useAlternate)
            assert((withT1.candidateTaken & 1) == 1)

            // Make T1 wrong once. The next longer table is allocated and must
            // become the provider even though both shorter tags still hit.
            train(dut, withT1, history, trainMask = 0x1, takenMask = 0x0)
            val withT2 = lookup(dut, pc, history, baseTaken = 0)
            assert(withT2.slots(0).provider == 3)
            assert(withT2.slots(0).chooserCounter == 0)
            assert(withT2.slots(0).providerCounter == 3)
            assert((withT2.providerHitMask & 1) == 1)
        }
    }

    it should "align adjacent s1 lookups with their live s2 Agree bases" in {
        test(new MiniTage(useBlackBoxRam = false)) { dut =>
            initialize(dut)
            val pcA = BigInt("3000", 16)
            val pcB = BigInt("3040", 16)

            // s1(A)
            dut.io.lookup.valid.poke(true.B)
            dut.io.lookup.bits.pc.poke(pcA.U)
            dut.io.lookup.bits.history.poke(BigInt("11111111", 16).U)
            dut.clock.step()

            // s1(B) overlaps s2 Agree(A).
            dut.io.lookup.bits.pc.poke(pcB.U)
            dut.io.lookup.bits.history.poke(BigInt("22222222", 16).U)
            dut.io.base.valid.poke(true.B)
            dut.io.base.bits.pc.poke(pcA.U)
            dut.io.base.bits.baseTaken.poke("ha".U)
            dut.io.base.bits.conditionalMask.poke("h5".U)
            dut.clock.step()
            dut.io.resp.valid.expect(true.B)
            dut.io.resp.bits.pc.expect(pcA.U)
            dut.io.resp.bits.baseTaken.expect("ha".U)
            dut.io.resp.bits.conditionalMask.expect("h5".U)
            dut.io.resp.bits.candidateTaken.expect("ha".U)
            dut.io.resp.bits.correctionMask.expect(0.U)

            // s2 Agree(B); its deliberately different base/mask must not be
            // paired with A's RAM row or response identity.
            dut.io.lookup.valid.poke(false.B)
            dut.io.base.bits.pc.poke(pcB.U)
            dut.io.base.bits.baseTaken.poke("h3".U)
            dut.io.base.bits.conditionalMask.poke("hc".U)
            dut.clock.step()
            dut.io.resp.valid.expect(true.B)
            dut.io.resp.bits.pc.expect(pcB.U)
            dut.io.resp.bits.baseTaken.expect("h3".U)
            dut.io.resp.bits.conditionalMask.expect("hc".U)
            dut.io.resp.bits.candidateTaken.expect("h3".U)
            dut.io.resp.bits.correctionMask.expect(0.U)
            dut.io.base.valid.poke(false.B)
        }
    }

    it should "read current state after stale gaps and forward a t1 write into lookup" in {
        test(new MiniTage(useBlackBoxRam = false)) { dut =>
            initialize(dut)
            val pc = BigInt("2400", 16)
            val history = BigInt("12345678", 16)

            val cold = lookup(dut, pc, history, baseTaken = 0)
            train(dut, cold, history, trainMask = 0x1, takenMask = 0x1)
            val weakT0 = lookup(dut, pc, history, baseTaken = 0)
            train(dut, weakT0, history, trainMask = 0x1, takenMask = 0x1)
            val weakT1 = lookup(dut, pc, history, baseTaken = 0)
            train(dut, weakT1, history, trainMask = 0x1, takenMask = 0x0)
            val weakT2 = lookup(dut, pc, history, baseTaken = 0)
            assert(weakT2.slots(0).provider == 3)
            assert(weakT2.slots(0).providerCounter == 3)

            // Allocate the longest table so repeated mispredictions cannot
            // legitimately move the observed provider to another table.
            train(dut, weakT2, history, trainMask = 0x1, takenMask = 0x1)
            val weakT3 = lookup(dut, pc, history, baseTaken = 0)
            assert(weakT3.slots(0).provider == 4)
            assert(weakT3.slots(0).providerCounter == 4)

            // Two back-to-back requests deliberately carry the same stale
            // counter=4 metadata. The train RAM write-forward must sustain one
            // update per cycle and produce 4->5->6, not two copies of 4->5.
            driveTrain(dut, weakT3, history, trainMask = 0x1, takenMask = 0x1)
            dut.clock.step()
            dut.clock.step()
            dut.io.train.valid.poke(false.B)
            dut.clock.step()
            val afterBackToBack = lookup(dut, pc, history, baseTaken = 0)
            assert(afterBackToBack.slots(0).provider == 4)
            assert(afterBackToBack.slots(0).providerCounter == 6)

            // Write another row and leave idle cycles. A one-cycle or one-row
            // metadata bypass cannot recover A after this sequence.
            val otherPc = BigInt("4800", 16)
            val otherHistory = BigInt("deadbeef", 16)
            val otherCold = lookup(dut, otherPc, otherHistory, baseTaken = 0)
            train(dut, otherCold, otherHistory, trainMask = 0x1, takenMask = 0x1)
            dut.clock.step(3)

            // Reuse the old counter=4 snapshot. At t0 the training copy reads
            // A's current row; at t1 its 6->7 write collides with this lookup.
            driveTrain(dut, weakT3, history, trainMask = 0x1, takenMask = 0x1)
            dut.clock.step()
            dut.io.train.valid.poke(false.B)
            dut.io.lookup.bits.pc.poke(pc.U)
            dut.io.lookup.bits.history.poke(history.U)
            dut.io.lookup.valid.poke(true.B)
            dut.clock.step()
            dut.io.lookup.valid.poke(false.B)
            dut.io.base.bits.pc.poke(pc.U)
            dut.io.base.bits.baseTaken.poke(0.U)
            dut.io.base.bits.conditionalMask.poke(1.U)
            dut.io.base.valid.poke(true.B)
            dut.clock.step()
            dut.io.base.valid.poke(false.B)
            var cycles = 0
            while (!dut.io.resp.valid.peek().litToBoolean && cycles < 5) {
                dut.clock.step()
                cycles += 1
            }
            assert(dut.io.resp.valid.peek().litToBoolean)
            dut.io.resp.bits.meta.slots(0).provider.expect(4.U)
            dut.io.resp.bits.meta.slots(0).providerCounter.expect(7.U)
            assert((dut.io.resp.bits.candidateTaken.peek().litValue & 1) == 1)
            assert((dut.io.resp.bits.correctionMask.peek().litValue & 1) == 1)
        }
    }

    it should "never let stale provider metadata resurrect a replaced tag" in {
        test(new MiniTage(useBlackBoxRam = false)) { dut =>
            initialize(dut)
            val pcA = BigInt("5000", 16)
            val historyA = BigInt("2468ace0", 16)
            // Toggling PC bit 4 and history bit 0 preserves all three folded
            // indices but changes their secondary history tag hash.
            val pcB = pcA ^ BigInt("10", 16)
            val historyB = historyA ^ 1

            val coldA = lookup(dut, pcA, historyA, baseTaken = 0)
            train(dut, coldA, historyA, trainMask = 1, takenMask = 1)
            val staleA = lookup(dut, pcA, historyA, baseTaken = 0)
            assert(staleA.slots(0).provider == 1)
            assert(staleA.slots(0).providerCounter == 4)

            val coldB = lookup(dut, pcB, historyB, baseTaken = 0)
            assert(coldB.slots(0).provider == 0)
            train(dut, coldB, historyB, trainMask = 1, takenMask = 1)
            val liveB = lookup(dut, pcB, historyB, baseTaken = 0)
            assert(liveB.slots(0).provider == 1)
            assert(liveB.slots(0).providerCounter == 4)

            // A's old prediction was not-taken because useAltOnNa selected
            // Agree. Training it as correct must be discarded on tag mismatch.
            train(dut, staleA, historyA, trainMask = 1, takenMask = 0)
            val preservedB = lookup(dut, pcB, historyB, baseTaken = 0)
            assert(preservedB.slots(0).provider == 1)
            assert(preservedB.slots(0).providerCounter == 4)
            val evictedA = lookup(dut, pcA, historyA, baseTaken = 0)
            assert(evictedA.slots(0).provider == 0)
        }
    }

    it should "age saturated useful entries under pressure and eventually allocate" in {
        test(new MiniTage(useBlackBoxRam = false)) { dut =>
            initialize(dut)
            val pc = BigInt("6000", 16)
            val history = BigInt("13579bdf", 16)

            val cold = lookup(dut, pc, history, baseTaken = 0)
            train(dut, cold, history, trainMask = 1, takenMask = 1)
            val t0 = lookup(dut, pc, history, baseTaken = 0)
            train(dut, t0, history, trainMask = 1, takenMask = 1)
            val t1 = lookup(dut, pc, history, baseTaken = 0)
            train(dut, t1, history, trainMask = 1, takenMask = 0)
            val t2 = lookup(dut, pc, history, baseTaken = 0)
            assert(t2.slots(0).provider == 3)

            // Change only history beyond H4 so T0 remains provider. Its taken
            // direction beats the not-taken Agree base until useful saturates.
            val t0History = history ^ (BigInt(1) << 4)
            for (expectedUseful <- Seq(1, 2)) {
                val pred = lookup(dut, pc, t0History, baseTaken = 0)
                assert(pred.slots(0).provider == 1)
                assert(pred.slots(0).providerUseful == expectedUseful)
                train(dut, pred, t0History, trainMask = 1, takenMask = 1)
            }

            // Change only history beyond H12 so T1 remains provider. T1 is
            // not-taken while its T0 alternate is taken.
            val t1History = history ^ (BigInt(1) << 12)
            for (expectedUseful <- Seq(0, 1, 2)) {
                val pred = lookup(dut, pc, t1History, baseTaken = 0)
                assert(pred.slots(0).provider == 2)
                assert(pred.slots(0).providerUseful == expectedUseful)
                train(dut, pred, t1History, trainMask = 1, takenMask = 0)
            }

            // Flip T2 taken and allocate T3, then restore T2 to not-taken so
            // T3 can prove useful against it.
            train(dut, t2, history, trainMask = 1, takenMask = 1)
            train(dut, t2, history, trainMask = 1, takenMask = 0)
            val t3 = lookup(dut, pc, history, baseTaken = 0)
            assert(t3.slots(0).provider == 4)
            assert(t3.slots(0).providerCounter == 4)
            for (expectedUseful <- Seq(0, 1, 2)) {
                val pred = lookup(dut, pc, history, baseTaken = 0)
                assert(pred.slots(0).provider == 4)
                assert(pred.slots(0).providerUseful == expectedUseful)
                train(dut, pred, history, trainMask = 1, takenMask = 1)
            }

            // Exercise T2 through retained metadata while T3 remains the
            // longest provider. Its direction differs from the not-taken T1
            // alternate, so current-row usefulness must still accumulate.
            val t2Taken = t2.copy(slots = t2.slots.updated(
                0,
                t2.slots(0).copy(providerCounter = 4)
            ))
            for (_ <- 0 until 3) {
                train(dut, t2Taken, history, trainMask = 1, takenMask = 1)
            }
            val saturated = lookup(dut, pc, history, baseTaken = 0)
            assert(saturated.slots(0).tableUseful == Seq(3, 3, 3, 3))

            val aliasPc = pc ^ BigInt("10", 16)
            val aliasHistory = history ^ 1
            for (expectedUseful <- Seq(3, 2, 1)) {
                val blocked = lookup(dut, aliasPc, aliasHistory, baseTaken = 0)
                assert(blocked.slots(0).provider == 0)
                assert(blocked.slots(0).tableUseful ==
                    Seq.fill(MiniTageConfig.tableCount)(expectedUseful))
                train(dut, blocked, aliasHistory, trainMask = 1, takenMask = 1)
            }
            val aged = lookup(dut, aliasPc, aliasHistory, baseTaken = 0)
            assert(aged.slots(0).provider == 0)
            assert(aged.slots(0).tableUseful ==
                Seq.fill(MiniTageConfig.tableCount)(BigInt(0)))
            train(dut, aged, aliasHistory, trainMask = 1, takenMask = 1)
            val allocated = lookup(dut, aliasPc, aliasHistory, baseTaken = 0)
            assert(allocated.slots(0).provider == 1)
        }
    }

    it should "ignore training while the startup scrub owns the write ports" in {
        test(new MiniTage(useBlackBoxRam = false)) { dut =>
            dut.io.lookup.valid.poke(false.B)
            dut.io.lookup.bits.pc.poke(0.U)
            dut.io.lookup.bits.history.poke(0.U)
            dut.io.base.valid.poke(false.B)
            dut.io.base.bits.pc.poke(0.U)
            dut.io.base.bits.baseTaken.poke(0.U)
            dut.io.base.bits.conditionalMask.poke(0.U)
            dut.io.train.bits.pc.poke(0.U)
            dut.io.train.bits.history.poke(0.U)
            dut.io.train.bits.baseTaken.poke(0.U)
            dut.io.train.bits.trainMask.poke(0.U)
            dut.io.train.bits.takenMask.poke(0.U)
            for (slot <- 0 until nfch) {
                val meta = dut.io.train.bits.meta.slots(slot)
                meta.provider.poke(0.U)
                meta.chooserCounter.poke(0.U)
                meta.providerCounter.poke(0.U)
                meta.providerUseful.poke(0.U)
                meta.alternateTaken.poke(false.B)
                meta.useAlternate.poke(false.B)
                for (table <- 0 until MiniTageConfig.tableCount) {
                    meta.tableUseful(table).poke(0.U)
                }
            }
            dut.io.train.valid.poke(false.B)
            dut.reset.poke(true.B)
            dut.clock.step(2)
            dut.reset.poke(false.B)
            dut.clock.step(500)
            dut.io.ready.expect(false.B)

            dut.io.train.bits.pc.poke("h7000".U)
            dut.io.train.bits.history.poke("h12345678".U)
            dut.io.train.bits.baseTaken.poke(0.U)
            dut.io.train.bits.trainMask.poke(1.U)
            dut.io.train.bits.takenMask.poke(1.U)
            dut.io.train.valid.poke(true.B)
            dut.clock.step()
            dut.io.train.valid.poke(false.B)
            while (!dut.io.ready.peek().litToBoolean) dut.clock.step()

            val cold = lookup(
                dut,
                BigInt("7000", 16),
                BigInt("12345678", 16),
                baseTaken = 0
            )
            assert(cold.providerHitMask == 0)
        }
    }

    it should "elaborate duplicated packed block-RAM tables" in {
        val systemVerilog = ChiselStage.emitSystemVerilog(
            new MiniTage(useBlackBoxRam = true)
        )
        assert(systemVerilog.contains("MiniTagePacketRamBlackBox"))
        assert(systemVerilog.contains("input  wire [71:0] wdata"))
    }
}
