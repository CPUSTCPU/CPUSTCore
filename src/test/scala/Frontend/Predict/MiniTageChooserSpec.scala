package CPUSTC.predict

import chisel3._
import chiseltest._
import circt.stage.ChiselStage
import org.scalatest.flatspec.AnyFlatSpec

import CPUSTC.config.Fetch.nfch

class MiniTageChooserSpec extends AnyFlatSpec with ChiselScalatestTester {
    behavior of "MiniTageChooser"

    private def initialize(dut: MiniTageChooser): Unit = {
        dut.clock.setTimeout(0)
        dut.io.lookup.valid.poke(false.B)
        dut.io.lookup.bits.packetPc.poke(0.U)
        dut.io.train.valid.poke(false.B)
        dut.io.train.bits.packetPc.poke(0.U)
        dut.io.train.bits.slot.poke(0.U)
        dut.io.train.bits.miniCorrect.poke(false.B)

        dut.reset.poke(true.B)
        dut.clock.step(2)
        dut.reset.poke(false.B)

        var cycles = 0
        while (!dut.io.ready.peek().litToBoolean && cycles < 1030) {
            dut.clock.step()
            cycles += 1
        }
        assert(dut.io.ready.peek().litToBoolean, "chooser scrub timed out")
    }

    private def lookup(
        dut: MiniTageChooser,
        packetPc: BigInt
    ): Seq[BigInt] = {
        dut.io.lookup.bits.packetPc.poke(packetPc.U)
        dut.io.lookup.valid.poke(true.B)
        dut.clock.step()
        dut.io.lookup.valid.poke(false.B)
        dut.io.resp.valid.expect(true.B)
        dut.io.resp.bits.packetPc.expect(packetPc.U)
        val counters = (0 until nfch).map { slot =>
            dut.io.resp.bits.counters(slot).peek().litValue
        }
        dut.clock.step()
        counters
    }

    private def train(
        dut: MiniTageChooser,
        packetPc: BigInt,
        slot: Int,
        miniCorrect: Boolean
    ): Unit = {
        dut.io.train.bits.packetPc.poke(packetPc.U)
        dut.io.train.bits.slot.poke(slot.U)
        dut.io.train.bits.miniCorrect.poke(miniCorrect.B)
        dut.io.train.valid.poke(true.B)
        dut.clock.step()
        dut.io.train.valid.poke(false.B)
        dut.clock.step()
    }

    it should "scrub every slot to weakly prefer MiniTAGE" in {
        test(new MiniTageChooser(useBlackBoxRam = false)) { dut =>
            initialize(dut)
            assert(lookup(dut, BigInt("1000", 16)) == Seq.fill(nfch)(2))
            assert(lookup(dut, BigInt("81234000", 16)) == Seq.fill(nfch)(2))
        }
    }

    it should "saturate one slot while preserving the rest of its row" in {
        test(new MiniTageChooser(useBlackBoxRam = false)) { dut =>
            initialize(dut)
            val pc = BigInt("2400", 16)

            train(dut, pc, slot = 2, miniCorrect = true)
            train(dut, pc, slot = 2, miniCorrect = true)
            assert(lookup(dut, pc) == Seq(2, 2, 3, 2))

            for (_ <- 0 until 4) {
                train(dut, pc, slot = 2, miniCorrect = false)
            }
            assert(lookup(dut, pc) == Seq(2, 2, 0, 2))
        }
    }

    it should "forward back-to-back same-row training without losing updates" in {
        test(new MiniTageChooser(useBlackBoxRam = false)) { dut =>
            initialize(dut)
            val pc = BigInt("3600", 16)

            dut.io.train.bits.packetPc.poke(pc.U)
            dut.io.train.bits.slot.poke(0.U)
            dut.io.train.bits.miniCorrect.poke(false.B)
            dut.io.train.valid.poke(true.B)
            dut.clock.step()

            dut.io.train.bits.slot.poke(1.U)
            dut.io.train.bits.miniCorrect.poke(true.B)
            dut.clock.step()

            dut.io.train.bits.slot.poke(2.U)
            dut.io.train.bits.miniCorrect.poke(false.B)
            dut.clock.step()
            dut.io.train.valid.poke(false.B)
            dut.clock.step()

            assert(lookup(dut, pc) == Seq(1, 3, 1, 2))
        }
    }

    it should "forward a simultaneous same-row write into the lookup response" in {
        test(new MiniTageChooser(useBlackBoxRam = false)) { dut =>
            initialize(dut)
            val pc = BigInt("4800", 16)

            // Launch training read.
            dut.io.train.bits.packetPc.poke(pc.U)
            dut.io.train.bits.slot.poke(3.U)
            dut.io.train.bits.miniCorrect.poke(false.B)
            dut.io.train.valid.poke(true.B)
            dut.clock.step()
            dut.io.train.valid.poke(false.B)

            // The pending RMW writes while this lookup reads the same row.
            dut.io.lookup.bits.packetPc.poke(pc.U)
            dut.io.lookup.valid.poke(true.B)
            dut.clock.step()
            dut.io.lookup.valid.poke(false.B)

            dut.io.resp.valid.expect(true.B)
            assert((0 until nfch).map { slot =>
                dut.io.resp.bits.counters(slot).peek().litValue
            } == Seq(2, 2, 2, 1))
        }
    }

    it should "ignore training while startup scrub owns both write ports" in {
        test(new MiniTageChooser(useBlackBoxRam = false)) { dut =>
            dut.clock.setTimeout(0)
            dut.io.lookup.valid.poke(false.B)
            dut.io.lookup.bits.packetPc.poke(0.U)
            dut.io.train.bits.packetPc.poke("h5000".U)
            dut.io.train.bits.slot.poke(0.U)
            dut.io.train.bits.miniCorrect.poke(false.B)
            dut.io.train.valid.poke(false.B)

            dut.reset.poke(true.B)
            dut.clock.step(2)
            dut.reset.poke(false.B)
            dut.clock.step(900)
            dut.io.ready.expect(false.B)

            dut.io.train.valid.poke(true.B)
            dut.clock.step()
            dut.io.train.valid.poke(false.B)
            while (!dut.io.ready.peek().litToBoolean) dut.clock.step()

            assert(lookup(dut, BigInt("5000", 16)) == Seq.fill(nfch)(2))
        }
    }

    it should "elaborate duplicated 1024-row block RAMs" in {
        val systemVerilog = ChiselStage.emitSystemVerilog(
            new MiniTageChooser(useBlackBoxRam = true)
        )
        assert(systemVerilog.contains("BpuSdpRamBlackBox"))
        assert(systemVerilog.contains("RAM_DEPTH(1024)"))
        assert(systemVerilog.contains("RAM_WIDTH(8)"))
    }
}
