package CPUSTC.frontend

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

import CPUSTC.config.Fetch.{icacheLatency, nfch}
import CPUSTC.config.JumpOp.BR
import CPUSTC.predict.{MiniTageChooserConfig, MiniTageConfig}

class InstructionFetchUnitMiniTageSpec
    extends AnyFlatSpec
    with ChiselScalatestTester {

    behavior of "InstructionFetchUnit MiniTAGE repair"

    private val packetPc = BigInt("1000", 16)
    private val branchSlot = 1
    private val branchBit = BigInt(1) << branchSlot
    private val lookupId = 7

    // Forward BEQ with a four-word displacement. Its BTFNT fallback is NT.
    private val branchInstr = BigInt("58000000", 16) | (BigInt(4) << 10)
    private val branchTarget = packetPc + branchSlot * 4 + 4 * 4
    private val sequentialTarget = packetPc + nfch * 4
    private val packetInstrs = branchInstr << (branchSlot * 32)

    private def clearICacheResponse(dut: InstructionFetchUnit): Unit = {
        dut.io.icache.resp.bits.pc.poke(0.U)
        dut.io.icache.resp.bits.instrs.poke(0.U)
        dut.io.icache.resp.bits.exceptions.foreach(_.poke(0.U))
        dut.io.icache.resp.bits.normal.poke(false.B)
        dut.io.icache.resp.bits.mask.poke(0.U)
    }

    private def clearBasePrediction(dut: InstructionFetchUnit): Unit = {
        dut.io.bpu.resp.bits.pc.poke(0.U)
        dut.io.bpu.resp.bits.lookupId.poke(0.U)
        dut.io.bpu.resp.bits.history.poke(0.U)
        dut.io.bpu.resp.bits.longHistory.poke(0.U)
        for (slot <- 0 until nfch) {
            dut.io.bpu.resp.bits.taken(slot).poke(false.B)
            dut.io.bpu.resp.bits.pretarget(slot).valid.poke(false.B)
            dut.io.bpu.resp.bits.pretarget(slot).bits.poke(0.U)
            dut.io.bpu.resp.bits.predType(slot).poke(0.U)
            dut.io.bpu.resp.bits.btbMeta(slot).writeWay.poke(0.U)
            dut.io.bpu.resp.bits.btbMeta(slot).localCtr.poke(0.U)
            dut.io.bpu.resp.bits.btbMeta(slot).predictorCtr.poke(0.U)
            dut.io.bpu.resp.bits.btbMeta(slot).bias.poke(false.B)
            dut.io.bpu.resp.bits.btbMeta(slot).chooseAgree.poke(0.U)
        }
    }

    private def clearAuxPrediction(dut: InstructionFetchUnit): Unit = {
        val aux = dut.io.bpu.auxResp.bits
        aux.lookupId.poke(0.U)
        aux.pc.poke(0.U)
        aux.miniValid.poke(false.B)
        aux.mini.pc.poke(0.U)
        aux.mini.baseTaken.poke(0.U)
        aux.mini.conditionalMask.poke(0.U)
        aux.mini.candidateTaken.poke(0.U)
        aux.mini.providerHitMask.poke(0.U)
        aux.mini.correctionMask.poke(0.U)
        for (slot <- 0 until nfch) {
            val meta = aux.mini.meta.slots(slot)
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
    }

    private def initialize(dut: InstructionFetchUnit): Unit = {
        dut.io.flush.poke(false.B)
        dut.io.redirectRequest.poke(false.B)

        dut.io.npc.valid.poke(false.B)
        dut.io.npc.req.pc.poke(0.U)
        dut.io.npc.req.mask.poke(0.U)

        dut.io.icache.req.ready.poke(true.B)
        dut.io.icache.resp.valid.poke(false.B)
        clearICacheResponse(dut)
        dut.io.icache.miss.poke(false.B)

        dut.io.bpu.req.ready.poke(true.B)
        dut.io.bpu.resp.valid.poke(false.B)
        clearBasePrediction(dut)
        dut.io.bpu.auxResp.valid.poke(false.B)
        clearAuxPrediction(dut)

        dut.io.fetch.ready.poke(true.B)

        dut.reset.poke(true.B)
        dut.clock.step(2)
        dut.reset.poke(false.B)
    }

    private def issueRequest(dut: InstructionFetchUnit, pc: BigInt): Unit = {
        dut.io.npc.valid.poke(true.B)
        dut.io.npc.req.pc.poke(pc.U)
        dut.io.npc.req.mask.poke(((BigInt(1) << nfch) - 1).U)
        dut.io.icache.req.valid.expect(true.B)
        dut.io.bpu.req.valid.expect(true.B)
        dut.clock.step()
        dut.io.npc.valid.poke(false.B)
    }

    private def driveBasePrediction(
        dut: InstructionFetchUnit,
        pc: BigInt,
        id: Int,
        taken: Boolean,
        target: BigInt
    ): Unit = {
        clearBasePrediction(dut)
        dut.io.bpu.resp.bits.pc.poke(pc.U)
        dut.io.bpu.resp.bits.lookupId.poke(id.U)
        dut.io.bpu.resp.bits.taken(branchSlot).poke(taken.B)
        dut.io.bpu.resp.bits.pretarget(branchSlot).valid.poke(true.B)
        dut.io.bpu.resp.bits.pretarget(branchSlot).bits.poke(target.U)
        dut.io.bpu.resp.bits.predType(branchSlot).poke(BR)
        dut.io.bpu.resp.valid.poke(true.B)
    }

    private def driveMiniPrediction(
        dut: InstructionFetchUnit,
        pc: BigInt,
        id: Int,
        baseTaken: Boolean,
        candidateTaken: Boolean,
        chooserCounter: Int = MiniTageChooserConfig.initialCounter
    ): Unit = {
        clearAuxPrediction(dut)
        dut.io.bpu.auxResp.bits.lookupId.poke(id.U)
        dut.io.bpu.auxResp.bits.pc.poke(pc.U)
        dut.io.bpu.auxResp.bits.miniValid.poke(true.B)
        dut.io.bpu.auxResp.bits.mini.pc.poke(pc.U)
        dut.io.bpu.auxResp.bits.mini.baseTaken.poke(
            (if (baseTaken) branchBit else BigInt(0)).U
        )
        dut.io.bpu.auxResp.bits.mini.conditionalMask.poke(branchBit.U)
        dut.io.bpu.auxResp.bits.mini.candidateTaken.poke(
            (if (candidateTaken) branchBit else BigInt(0)).U
        )
        dut.io.bpu.auxResp.bits.mini.providerHitMask.poke(branchBit.U)
        dut.io.bpu.auxResp.bits.mini.correctionMask.poke(branchBit.U)
        dut.io.bpu.auxResp.bits.mini.meta.slots(branchSlot)
            .chooserCounter.poke(chooserCounter.U)
        dut.io.bpu.auxResp.valid.poke(true.B)
    }

    private def driveICacheResponse(
        dut: InstructionFetchUnit,
        pc: BigInt
    ): Unit = {
        clearICacheResponse(dut)
        dut.io.icache.resp.bits.pc.poke(pc.U)
        dut.io.icache.resp.bits.instrs.poke(packetInstrs.U)
        dut.io.icache.resp.bits.normal.poke(true.B)
        dut.io.icache.resp.bits.mask.poke(((BigInt(1) << nfch) - 1).U)
        dut.io.icache.resp.valid.poke(true.B)
    }

    private def expectOneRegisteredRepair(
        dut: InstructionFetchUnit,
        target: BigInt
    ): Unit = {
        dut.io.lateRedirect.valid.expect(false.B)
        dut.clock.step()
        dut.io.icache.resp.valid.poke(false.B)
        dut.io.lateRedirect.valid.expect(true.B)
        dut.io.lateRedirect.bits.expect(target.U)
        dut.clock.step()
        dut.io.lateRedirect.valid.expect(false.B)
    }

    it should "turn an Agree NT branch into taken and emit one registered repair" in {
        test(new InstructionFetchUnit) { dut =>
            initialize(dut)
            issueRequest(dut, packetPc)

            driveBasePrediction(
                dut,
                packetPc,
                lookupId,
                taken = false,
                branchTarget
            )
            dut.io.earlyRedirect.valid.expect(false.B)
            dut.clock.step()
            dut.io.bpu.resp.valid.poke(false.B)

            driveMiniPrediction(
                dut,
                packetPc,
                lookupId,
                baseTaken = false,
                candidateTaken = true
            )
            dut.clock.step()
            dut.io.bpu.auxResp.valid.poke(false.B)

            driveICacheResponse(dut, packetPc)
            dut.io.fetch.valid.expect(true.B)
            dut.io.fetch.bits.cfiIdx.valid.expect(true.B)
            dut.io.fetch.bits.cfiIdx.bits.expect(branchSlot.U)
            dut.io.fetch.bits.cfiType.expect(1.U)
            dut.io.fetch.bits.taken.expect(true.B)
            dut.io.fetch.bits.predTaken.expect(branchBit.U)
            dut.io.fetch.bits.target.expect(branchTarget.U)
            dut.io.fetchPredictorMeta.miniValid.expect(true.B)
            expectOneRegisteredRepair(dut, branchTarget)
        }
    }

    it should "turn an Agree taken branch into NT through a stalled response" in {
        test(new InstructionFetchUnit) { dut =>
            initialize(dut)
            issueRequest(dut, packetPc)

            driveBasePrediction(
                dut,
                packetPc,
                lookupId,
                taken = true,
                branchTarget
            )
            dut.io.earlyRedirect.valid.expect(true.B)
            dut.io.earlyRedirect.bits.expect(branchTarget.U)
            dut.clock.step()
            dut.io.bpu.resp.valid.poke(false.B)

            // Move the packet to the response slot, then hold it there. The
            // delayed aux result must attach to that stalled packet.
            dut.clock.step(icacheLatency - 2)
            dut.io.fetch.ready.poke(false.B)
            driveICacheResponse(dut, packetPc)
            driveMiniPrediction(
                dut,
                packetPc,
                lookupId,
                baseTaken = true,
                candidateTaken = false
            )
            dut.io.fetch.valid.expect(true.B)
            dut.clock.step()
            dut.io.bpu.auxResp.valid.poke(false.B)
            dut.io.fetch.ready.poke(true.B)

            dut.io.fetch.valid.expect(true.B)
            dut.io.fetch.bits.cfiIdx.valid.expect(false.B)
            dut.io.fetch.bits.taken.expect(false.B)
            dut.io.fetch.bits.predTaken.expect(0.U)
            dut.io.fetchPredictorMeta.miniValid.expect(true.B)
            expectOneRegisteredRepair(dut, sequentialTarget)
        }
    }

    it should "discard an old aux response after flush" in {
        test(new InstructionFetchUnit) { dut =>
            initialize(dut)
            val oldPc = packetPc - 0x100
            val newPc = packetPc
            val oldLookupId = lookupId
            val newLookupId = lookupId + 1

            issueRequest(dut, oldPc)
            driveBasePrediction(
                dut,
                oldPc,
                oldLookupId,
                taken = false,
                oldPc + 20
            )
            dut.clock.step()
            dut.io.bpu.resp.valid.poke(false.B)

            dut.io.flush.poke(true.B)
            dut.clock.step()
            dut.io.flush.poke(false.B)

            issueRequest(dut, newPc)
            driveBasePrediction(
                dut,
                newPc,
                newLookupId,
                taken = false,
                branchTarget
            )
            dut.clock.step()
            dut.io.bpu.resp.valid.poke(false.B)

            // The integrated BPU advances the ID on every request and does not
            // reset it on flush, so a late response keeps the old ID.
            driveMiniPrediction(
                dut,
                oldPc,
                oldLookupId,
                baseTaken = false,
                candidateTaken = true
            )
            dut.clock.step()
            dut.io.bpu.auxResp.valid.poke(false.B)

            driveICacheResponse(dut, newPc)
            dut.io.fetch.valid.expect(true.B)
            dut.io.fetch.bits.cfiIdx.valid.expect(false.B)
            dut.io.fetch.bits.taken.expect(false.B)
            dut.io.fetch.bits.predTaken.expect(0.U)
            dut.io.fetchPredictorMeta.miniValid.expect(false.B)
            dut.clock.step()
            dut.io.icache.resp.valid.poke(false.B)
            dut.io.lateRedirect.valid.expect(false.B)
        }
    }

    it should "retain the fast direction when the chooser lookup is unavailable" in {
        test(new InstructionFetchUnit) { dut =>
            initialize(dut)
            issueRequest(dut, packetPc)

            driveBasePrediction(
                dut,
                packetPc,
                lookupId,
                taken = false,
                branchTarget
            )
            dut.clock.step()
            dut.io.bpu.resp.valid.poke(false.B)

            driveMiniPrediction(
                dut,
                packetPc,
                lookupId,
                baseTaken = false,
                candidateTaken = true,
                chooserCounter = 0
            )
            dut.clock.step()
            dut.io.bpu.auxResp.valid.poke(false.B)

            driveICacheResponse(dut, packetPc)
            dut.io.fetch.valid.expect(true.B)
            dut.io.fetch.bits.cfiIdx.valid.expect(false.B)
            dut.io.fetch.bits.predTaken.expect(0.U)
            dut.io.fetchPredictorMeta.miniProviderHit.expect(branchBit.U)
            dut.io.fetchPredictorMeta.miniCandidateTaken.expect(branchBit.U)
            dut.io.fetchPredictorMeta.miniMeta.slots(branchSlot)
                .chooserCounter.expect(0.U)
            dut.io.lateRedirect.valid.expect(false.B)
        }
    }
}
