package CPUSTC.backend.branch

import chisel3._
import chisel3.util._

import CPUSTC.config.RenameConfig._
import CPUSTC.backend.rob.RobPtr

class BranchUpdate extends Bundle {
    val resolveMask    = UInt(maxBrCount.W)
    val mispredictMask = UInt(maxBrCount.W)
    val recoverMask    = UInt(maxBrCount.W)

    val robPtr = new RobPtr
}
