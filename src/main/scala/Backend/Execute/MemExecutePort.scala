package CPUSTC.backend.execute

import chisel3._
import chisel3.util._

import CPUSTC.backend.branch.{BranchMask, BranchUpdate}
import CPUSTC.config.FunctionUnit._
import CPUSTC.config.LoadStoreQueue._
import CPUSTC.config.MemIssueOp._
import CPUSTC.config.Memory._
import CPUSTC.config.MemoryException._
import CPUSTC.config.RenameConfig._
import CPUSTC.backend.execute.fu.AddressGenerationUnit
import CPUSTC.memory._

class MemExecutePort extends Module {
    val io = IO(new MemExecutePortIO)

    val outValid  = RegInit(false.B)
    val outBits   = Reg(new BackendInst)
    val outBrMask = Reg(UInt(maxBrCount.W))
    // Keep the late STA dependency release beside the resident output state.
    // Its payload is intentionally narrow so the wide BackendInst register
    // does not fan back into every MemIQ dependency mask.
    val lateStaReleaseValid = RegInit(false.B)
    val lateStaReleasePtr = Reg(chiselTypeOf(io.staDependencyRelease.bits))

    val resolveMask = Mux(
        io.branchUpdate.valid,
        io.branchUpdate.bits.resolveMask,
        0.U(maxBrCount.W)
    )

    val mispredictMask = Mux(
        io.branchUpdate.valid,
        io.branchUpdate.bits.mispredictMask,
        0.U(maxBrCount.W)
    )

    val recoveryMispredict = io.branchUpdate.valid && mispredictMask.orR

    val outKilled = outValid && BranchMask.isKilled(outBrMask, mispredictMask)
    val outNewMask = BranchMask.clearResolved(outBrMask, resolveMask)

    val isLd  = io.in.bits.memOp === MEM_LD
    val isSta = io.in.bits.memOp === MEM_STA
    val isStd = io.in.bits.memOp === MEM_STD
    val isAddrOp = isLd || isSta

    val operandEnable = Mux(
        isStd,
        io.in.bits.uop.reg.lsrc2Valid,
        io.in.bits.uop.reg.lsrc1Valid
    )
    val operandPsrc = Mux(
        isStd,
        io.in.bits.uop.reg.psrc2,
        io.in.bits.uop.reg.psrc1
    )
    val forwardHits = VecInit(io.fastForward.map { forward =>
        io.in.valid &&
            operandEnable &&
            operandPsrc =/= 0.U &&
            forward.valid &&
            forward.bits.pdest === operandPsrc
    })
    val operandData = Mux(
        forwardHits.asUInt.orR,
        Mux1H(forwardHits, io.fastForward.map(_.bits.data)),
        io.in.bits.operandData
    )

    when(io.in.valid && operandEnable) {
        assert(PopCount(forwardHits) <= 1.U)
    }

    val agu = Module(new AddressGenerationUnit)

    agu.io.base    := operandData
    agu.io.offset  := io.in.bits.uop.imm
    agu.io.memType := io.in.bits.uop.mem.memType

    // CSR mode changes are predecoded and registered off the main memory
    // datapath. Select again with the repaired AGU address so a fast-forward
    // that crosses a VSEG boundary cannot reuse stale RegisterRead metadata.
    val fastAddress = io.fastAddressMap.byVseg(agu.io.vaddr(31, 29))

    val addressAlignmentException = isAddrOp && agu.io.addrMisaligned
    val translationPending =
        isAddrOp &&
        !addressAlignmentException &&
        !fastAddress.resolved

    val nextInst = WireDefault(0.U.asTypeOf(new BackendInst))
    val uop = io.in.bits.uop

    nextInst.uop.isLD     := isLd
    nextInst.uop.isSTA    := isSta
    nextInst.uop.isSTD    := isStd
    nextInst.uop.isRefill := false.B
    nextInst.valid        := true.B

    nextInst.pc    := Mux(isAddrOp, agu.io.vaddr, 0.U)
    // The payload is consumed only after address resolution. Pending
    // translations overwrite it with the TLB response, and STD never reads it.
    nextInst.paddr := Cat(fastAddress.pseg, agu.io.vaddr(28, 0))
    nextInst.translationPending := translationPending

    nextInst.uncache := isAddrOp &&
        !addressAlignmentException &&
        fastAddress.resolved &&
        !fastAddress.cacheable

    nextInst.operateData := Mux(isStd, operandData, 0.U)
    nextInst.mask        := Mux(isAddrOp, agu.io.sizeMask, 0.U)
    nextInst.signed      := isLd && uop.mem.memSigned

    nextInst.sqindex     := uop.stqIdx.oh
    nextInst.sqindexHigh := uop.stqIdx.flag
    nextInst.storeDepMask := uop.stOrderMask

    nextInst.ldindex     := Mux(isLd, uop.ldqIdx.oh, 0.U)
    nextInst.ldindexHigh := Mux(isLd, uop.ldqIdx.flag, false.B)

    nextInst.soreceReg := Mux(isLd, uop.reg.pdest, 0.U)

    // The current LoongArch32 test architecture removed ADEM. ADEF/ALE cover
    // misaligned fetch/data addresses; mapped-mode protection faults come from
    // the TLB as PIL/PIS/PIF/PME/PPI/TLBR.
    nextInst.exception := Mux(addressAlignmentException, EXC_ALE, EXC_NONE)
    nextInst.exceptionBadvValid := addressAlignmentException
    nextInst.exceptionBadv := agu.io.vaddr

    nextInst.Poisoned := isSta && agu.io.addrMisaligned
    nextInst.robPtr := uop.robPtr
    nextInst.pdest  := uop.reg.pdest
    nextInst.rfWen  := uop.reg.rfWen

    val directLoad = WireDefault(0.U.asTypeOf(new DirectCachedLoad))
    directLoad.vaddr        := io.in.bits.preVaddr
    directLoad.paddr        := Cat(
        io.in.bits.preFastPseg,
        io.in.bits.preVaddr(28, 0)
    )
    directLoad.mask         := io.in.bits.preSizeMask
    directLoad.signed       := uop.mem.memSigned
    directLoad.sqindex      := uop.stqIdx.oh
    directLoad.sqindexHigh  := uop.stqIdx.flag
    directLoad.storeDepMask := uop.stOrderMask
    directLoad.ldindex      := uop.ldqIdx.oh
    directLoad.ldindexHigh  := uop.ldqIdx.flag
    directLoad.robPtr       := uop.robPtr
    directLoad.pdest        := uop.reg.pdest
    directLoad.rfWen        := uop.reg.rfWen

    // Every qualifier below was registered at the RegisterRead boundary. The
    // direct valid therefore has no path from the live translator, AGU repair,
    // exception logic or Store controls.
    val directLoadCandidate =
        isLd &&
            !io.in.bits.addrSpeculative &&
            !io.in.bits.preMisaligned &&
            io.in.bits.preFastCacheable

    val nextBrMask = BranchMask.clearResolved(uop.spec.brMask, resolveMask)

    val outCanAccept = !outValid || io.backendInst.ready || outKilled

    val inputKilled =
        io.in.valid &&
        BranchMask.isKilled(uop.spec.brMask, mispredictMask)

    val directOpportunity =
        !outValid && io.in.valid && directLoadCandidate

    io.directCachedLoad.valid := directOpportunity && !io.flush
    io.directCachedLoad.bits := directLoad

    // A recovery-cycle cached read is harmless: the registered LDQ flush and
    // result-generation check discard a killed request. Keeping recovery off
    // the direct valid cone prevents the global branch network reaching LST.
    io.in.ready := Mux(
        directOpportunity,
        io.directCachedLoad.ready,
        outCanAccept && !recoveryMispredict
    ) && !io.flush

    val acceptedInput = io.in.fire && !inputKilled
    val captureInput = acceptedInput && !io.directCachedLoad.fire

    // Publish the STA index while the address operation enters the output
    // register, but qualify it only with RegisterRead-registered address state.
    // A speculative address is repaired by this stage's AGU and uses the
    // registered release below.  This keeps the duplicate AGU/translator out
    // of the replicated MemIQ dependency-mask write cone.
    io.staDependencyReleaseEarly.valid :=
        acceptedInput &&
            isSta &&
            !io.in.bits.addrSpeculative &&
            (io.in.bits.preMisaligned ||
                io.in.bits.preTranslationResolved)
    io.staDependencyReleaseEarly.bits.oh := uop.stqIdx.oh
    io.staDependencyReleaseEarly.bits.flag := uop.stqIdx.flag

    // Branch recovery is a registered, rare control event.  Do not let its
    // per-entry mask check gate the normal CPUSTC.memory request valid: doing so
    // extends resolveMask through translation/SQ completion into the ROB write
    // enable.  A killed resident is still cleared at this cycle's edge below.
    // If CPUSTC.memory accepts it on the recovery cycle, the same-cycle LSQ
    // recovery mask rejects its writeback and the registered queue flush removes
    // the speculative entry on the following cycle.
    io.backendInst.valid := outValid && !io.flush

    io.backendInst.bits := outBits

    // A genuine page translation miss waits for Backend's final
    // memRequest.fire event, so younger loads cannot pass a Store whose
    // physical address is unresolved. Keep this as a resident level rather
    // than a pulse: while the output stalls, newly dispatched Loads must also
    // observe that this Store address is already known.
    io.staDependencyRelease.valid := lateStaReleaseValid
    io.staDependencyRelease.bits := lateStaReleasePtr

    when(io.flush) {
        outValid := false.B
        lateStaReleaseValid := false.B
    }.elsewhen(outKilled) {
        outValid := false.B
        lateStaReleaseValid := false.B
    }.elsewhen(outCanAccept) {
        outValid := captureInput
        val captureLateSta = captureInput && isSta && !translationPending
        lateStaReleaseValid := captureLateSta

        when(captureInput) {
            outBits := nextInst
            outBrMask := nextBrMask
        }
        when(captureLateSta) {
            lateStaReleasePtr.oh := uop.stqIdx.oh
            lateStaReleasePtr.flag := uop.stqIdx.flag
        }
    }.elsewhen(io.branchUpdate.valid) {
        outBrMask := outNewMask
    }

    val memTypeLegal =
        uop.mem.memType === MEM_BYTE ||
        uop.mem.memType === MEM_HALF ||
        uop.mem.memType === MEM_WORD

    when(io.in.fire) {
        assert(PopCount(Cat(isStd, isSta, isLd)) === 1.U)
        assert(uop.ctrl.fuType === FU_MEM)
        assert(PopCount(uop.stqIdx.oh) === 1.U)

        when(isLd) {
            assert(uop.mem.isLoad && !uop.mem.isStore)
            assert(PopCount(uop.ldqIdx.oh) === 1.U)
            assert(memTypeLegal)
        }

        when(isSta) {
            assert(uop.mem.isStore && !uop.mem.isLoad)
            assert(!uop.reg.rfWen)
            assert(memTypeLegal)
        }

        when(isStd) {
            assert(uop.mem.isStore && !uop.mem.isLoad)
            assert(!uop.reg.rfWen)
        }
    }

    when(io.staDependencyReleaseEarly.valid) {
        assert(PopCount(io.staDependencyReleaseEarly.bits.oh) === 1.U)
        assert(io.staDependencyReleaseEarly.bits.oh === uop.stqIdx.oh)
        assert(io.staDependencyReleaseEarly.bits.flag === uop.stqIdx.flag)
    }

    when(io.staDependencyRelease.valid) {
        assert(PopCount(io.staDependencyRelease.bits.oh) === 1.U)
        assert(outValid)
        assert(outBits.uop.isSTA)
        assert(!outBits.translationPending)
        assert(io.staDependencyRelease.bits.oh === outBits.sqindex)
        assert(io.staDependencyRelease.bits.flag === outBits.sqindexHigh)
    }
    when(io.backendInst.valid) {
        val inst = io.backendInst.bits

        assert(inst.valid)
        assert(PopCount(Cat(
            inst.uop.isRefill,
            inst.uop.isSTD,
            inst.uop.isSTA,
            inst.uop.isLD
        )) === 1.U)
        assert(!inst.uop.isRefill)
        assert(PopCount(inst.sqindex) === 1.U)

        when(inst.uop.isLD) {
            assert(PopCount(inst.ldindex) === 1.U)
            assert(!inst.Poisoned)
        }

        when(inst.translationPending) {
            assert(inst.uop.isLD || inst.uop.isSTA)
            assert(!inst.uncache)
            assert(inst.exception === EXC_NONE)
            assert(!inst.exceptionBadvValid)
        }

        when(inst.exception === EXC_ALE || inst.exception === EXC_TLBR) {
            assert(inst.exceptionBadvValid)
            assert(inst.exceptionBadv === inst.pc)
        }

        when(!inst.exception.orR) {
            assert(!inst.exceptionBadvValid)
        }

        when(inst.uop.isSTA) {
            assert(inst.operateData === 0.U)
            assert(inst.ldindex === 0.U)
            assert(!inst.signed)
        }

        when(inst.uop.isSTD) {
            assert(inst.pc === 0.U)
            assert(inst.mask === 0.U)
            assert(inst.ldindex === 0.U)
            assert(!inst.ldindexHigh)
            assert(inst.soreceReg === 0.U)
            assert(!inst.signed)
            assert(!inst.Poisoned)
            assert(inst.exception === EXC_NONE)
            assert(!inst.exceptionBadvValid)
        }
    }

    when(io.directCachedLoad.valid) {
        assert(io.in.valid)
        assert(directLoadCandidate)
        assert(!outValid)
        assert(!io.in.bits.addrSpeculative)
        assert(!io.in.bits.preMisaligned)
        assert(io.in.bits.preFastCacheable)
        assert(PopCount(io.directCachedLoad.bits.sqindex) === 1.U)
        assert(PopCount(io.directCachedLoad.bits.ldindex) === 1.U)
    }

    assert(!(io.backendInst.valid && io.directCachedLoad.valid))

    val stalledLastCycle = RegNext(
        io.backendInst.valid && !io.backendInst.ready,
        false.B
    )
    val bitsLastCycle = RegNext(io.backendInst.bits.asUInt)
    val directStalledLastCycle = RegNext(
        io.directCachedLoad.valid && !io.directCachedLoad.ready,
        false.B
    )
    val directBitsLastCycle = RegNext(io.directCachedLoad.bits.asUInt)
    val recoveryControlLastCycle = RegNext(
        io.flush || outKilled || recoveryMispredict,
        false.B
    )

    when(
        stalledLastCycle &&
        !io.flush &&
        !io.branchUpdate.valid &&
        !recoveryControlLastCycle
    ) {
        assert(io.backendInst.valid)
        assert(io.backendInst.bits.asUInt === bitsLastCycle)
    }

    when(
        directStalledLastCycle &&
        !io.flush &&
        !io.branchUpdate.valid &&
        !recoveryControlLastCycle
    ) {
        assert(io.directCachedLoad.valid)
        assert(io.directCachedLoad.bits.asUInt === directBitsLastCycle)
    }

    when(outKilled) {
        assert(recoveryMispredict)
        assert(!io.in.fire)
    }

    when(
        io.backendInst.fire &&
        io.backendInst.bits.uop.isSTA &&
        !io.backendInst.bits.translationPending
    ) {
        assert(io.staDependencyRelease.valid)
    }

    when(io.flush) {
        assert(!io.in.fire)
        assert(!io.backendInst.fire)
        assert(!io.directCachedLoad.fire)
    }

    when(io.in.fire && inputKilled) {
        assert(!acceptedInput)
    }

    when(recoveryMispredict && !io.flush && !io.directCachedLoad.fire) {
        assert(!io.in.fire)
    }

    when(io.directCachedLoad.fire) {
        assert(io.in.fire)
        assert(!captureInput)
    }

    when(io.branchUpdate.valid && !io.flush) {
        assert(PopCount(resolveMask) === 1.U)
        assert(
            (mispredictMask & (~resolveMask).asUInt) === 0.U
        )
    }
}
