package CPUSTC.backend.branch

import chisel3._

object BranchMask {
    def isKilled(
        brMask: UInt,
        mispredictMask: UInt
    ): Bool = {
        (brMask & mispredictMask).orR
    }

    def clearResolved(
        brMask: UInt,
        resolveMask: UInt
    ): UInt = {
        brMask & (~resolveMask).asUInt
    }
}