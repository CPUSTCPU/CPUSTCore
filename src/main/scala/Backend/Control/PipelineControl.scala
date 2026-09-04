package CPUSTC.backend.control

import chisel3._
import chisel3.util._

import CPUSTC.backend.branch.BranchUpdate
import CPUSTC.config.CtrlFlowInstr._
import CPUSTC.config.RegisterFile._
import CPUSTC.config.RenameConfig._
import CPUSTC.backend.execute.BranchResolve

object BranchUpdateDomain {
    val Rename      = 0
    val DispatchLsq = 1
    val Rob         = 2
    val IntIssue    = 3
    val MemIssue    = 4
    val IntCluster  = 5
    val MemExecute  = 6
    val Count       = 7
}

class BranchUpdateLaunch extends Bundle {
    val resolveMask = UInt(maxBrCount.W)
    val mispredict  = Bool()
    val recoverMask = UInt(maxBrCount.W)
    val robPtr      = new CPUSTC.backend.rob.RobPtr
}

class PipelineControlIO extends Bundle {
    val hardRedirect = Flipped(Valid(UInt(dataWidth.W)))
    val archRedirect = Flipped(Valid(UInt(dataWidth.W)))
    val squashPending = Input(Bool())
    val resolve      = Flipped(Valid(new BranchResolve))

    val fullFlush    = Output(Bool())
    val branchUpdate = Output(Valid(new BranchUpdate))
    val branchUpdateCopies = Output(
        Vec(BranchUpdateDomain.Count, Valid(new BranchUpdate))
    )
    val redirect     = Output(Valid(new PipelineRedirect))
}

class PipelineControl extends Module {
    val io = IO(new PipelineControlIO)

    val updateValidRegs = RegInit(
        VecInit.fill(BranchUpdateDomain.Count)(false.B)
    )
    val updateLaunchRegs = Reg(
        Vec(BranchUpdateDomain.Count, new BranchUpdateLaunch)
    )
    val redirectValidReg = RegInit(false.B)

    val redirectBitsReg = Reg(new PipelineRedirect)

    // Keep one launch register per physical recovery domain. Modules inside
    // the integer datapath share IntCluster so independent copies cannot
    // reconverge through the execute bypass network.
    dontTouch(updateValidRegs)
    dontTouch(updateLaunchRegs)

    val hardRedirect = io.hardRedirect.valid || io.archRedirect.valid

    val isCondBr = io.resolve.bits.cfiType === CFI_BR
    val isDirect = io.resolve.bits.cfiType === CFI_BL
    val isJirl   = io.resolve.bits.cfiType === CFI_JIRL

    val mispredict = io.resolve.bits.mispredict
    val tracked = io.resolve.bits.brTag.valid
    val tagOH = UIntToOH(io.resolve.bits.brTag.bits, maxBrCount)

    val updateLaunch = WireDefault(0.U.asTypeOf(new BranchUpdateLaunch))
    updateLaunch.resolveMask := Mux(tracked, tagOH, 0.U)
    updateLaunch.mispredict  := mispredict
    updateLaunch.recoverMask := io.resolve.bits.brMask
    updateLaunch.robPtr      := io.resolve.bits.robPtr

    val branchRedirectBits = WireDefault(0.U.asTypeOf(new PipelineRedirect))
    branchRedirectBits.kind        := RedirectKind.BRANCH
    branchRedirectBits.target      := io.resolve.bits.actualNextPc
    branchRedirectBits.robPtr      := io.resolve.bits.robPtr
    branchRedirectBits.ftqPtr      := io.resolve.bits.ftqPtr
    branchRedirectBits.ftqOffset   := io.resolve.bits.ftqOffset
    branchRedirectBits.cfiType     := io.resolve.bits.cfiType
    branchRedirectBits.actualTaken := io.resolve.bits.actualTaken
    branchRedirectBits.isCall      := io.resolve.bits.isCall
    branchRedirectBits.isRet       := io.resolve.bits.isRet

    val hardRedirectBits = WireDefault(0.U.asTypeOf(new PipelineRedirect))
    hardRedirectBits.kind   := RedirectKind.HARD
    hardRedirectBits.target := Mux(
        io.hardRedirect.valid,
        io.hardRedirect.bits,
        io.archRedirect.bits
    )

    io.fullFlush := hardRedirect

    for (i <- 0 until BranchUpdateDomain.Count) {
        io.branchUpdateCopies(i).valid := updateValidRegs(i) && !hardRedirect
        io.branchUpdateCopies(i).bits.resolveMask :=
            updateLaunchRegs(i).resolveMask
        io.branchUpdateCopies(i).bits.mispredictMask := Mux(
            updateLaunchRegs(i).mispredict,
            updateLaunchRegs(i).resolveMask,
            0.U
        )
        io.branchUpdateCopies(i).bits.recoverMask :=
            updateLaunchRegs(i).recoverMask
        io.branchUpdateCopies(i).bits.robPtr := updateLaunchRegs(i).robPtr
    }

    io.branchUpdate := io.branchUpdateCopies(BranchUpdateDomain.Rename)

    io.redirect.valid := hardRedirect || (redirectValidReg && !hardRedirect)
    io.redirect.bits  := Mux(hardRedirect, hardRedirectBits, redirectBitsReg)

    // Keep payload capture independent of the accepted-resolve valid path.
    // This prevents recovery feedback from reconverging with branch arithmetic
    // at the payload registers.
    for (i <- 0 until BranchUpdateDomain.Count) {
        updateLaunchRegs(i) := updateLaunch
    }
    redirectBitsReg := branchRedirectBits

    // Rare architectural recovery is redirected outside this module.  It only
    // needs to discard any already registered branch update; keeping this bit
    // off the combinational outputs prevents recovery from feeding the normal
    // issue and allocation domains.
    when(hardRedirect || io.squashPending) {
        updateValidRegs  := VecInit.fill(BranchUpdateDomain.Count)(false.B)
        redirectValidReg := false.B
    }.otherwise {
        updateValidRegs := VecInit.fill(BranchUpdateDomain.Count)(
            io.resolve.valid && tracked
        )
        redirectValidReg := io.resolve.valid && tracked && mispredict

    }

    for (i <- 1 until BranchUpdateDomain.Count) {
        assert(
            io.branchUpdateCopies(i).valid === io.branchUpdateCopies(0).valid
        )
        when(io.branchUpdateCopies(0).valid) {
            assert(
                io.branchUpdateCopies(i).bits.asUInt ===
                    io.branchUpdateCopies(0).bits.asUInt
            )
        }
    }

    when(io.fullFlush) {
        assert(io.redirect.valid)
        assert(io.redirect.bits.kind === RedirectKind.HARD)
        assert(!io.branchUpdate.valid)
    }

    when(io.hardRedirect.valid && io.archRedirect.valid) {
        assert(io.redirect.bits.target === io.hardRedirect.bits)
    }

    when(io.redirect.valid) {
        assert(
            io.redirect.bits.kind === RedirectKind.BRANCH ||
            io.redirect.bits.kind === RedirectKind.HARD
        )
    }

    when(io.redirect.valid && io.redirect.bits.kind === RedirectKind.BRANCH) {
        assert(io.branchUpdate.valid)
        assert(io.branchUpdate.bits.mispredictMask.orR)
    }

    when(io.branchUpdate.valid) {
        assert(PopCount(io.branchUpdate.bits.resolveMask) === 1.U)
        assert(
            (io.branchUpdate.bits.mispredictMask &
                (~io.branchUpdate.bits.resolveMask).asUInt) === 0.U
        )

        when(!io.branchUpdate.bits.mispredictMask.orR) {
            assert(!io.redirect.valid)
        }
    }

    when(io.resolve.valid && tracked && !hardRedirect) {
        assert(isCondBr || isJirl)
    }

    when(io.resolve.valid && isDirect && !hardRedirect) {
        assert(!tracked)
        assert(!mispredict)
    }
}
