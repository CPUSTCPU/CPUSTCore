package CPUSTC.predict

import chisel3._

import CPUSTC.config.Fetch.nfch

object AdvancedPredictorConfig {
    val historyWidth = MiniTageConfig.historyWidth
    val lookupIdWidth = 5

    // MiniTAGE changes only the registered predecode repair decision, never
    // the existing s2 BTB/Agree fast path.
    val miniTageOverrideEnabled = true
}

class AdvancedPredictorAuxResp extends Bundle {
    val lookupId = UInt(AdvancedPredictorConfig.lookupIdWidth.W)
    val pc = UInt(32.W)

    val miniValid = Bool()
    val mini = new MiniTageLookupResp
}

/** Metadata that follows a fetch packet only as far as the FTQ trainer. */
class AdvancedPredictorPacketMeta extends Bundle {
    val valid = Bool()
    val longHistory = UInt(AdvancedPredictorConfig.historyWidth.W)

    // Direction actually used by the old frontend after static fallback. This
    // is retained separately from MiniTAGE's Agree alternate for exact A/B
    // accounting at retirement.
    val fastTaken = UInt(nfch.W)

    val miniValid = Bool()
    val miniBaseTaken = UInt(nfch.W)
    val miniCandidateTaken = UInt(nfch.W)
    val miniProviderHit = UInt(nfch.W)
    val miniMeta = new MiniTagePredictionMeta
}

class AdvancedPredictorPerfEvents extends Bundle {
    val miniEligible = Bool()
    val miniDisagree = Bool()
    val miniRecover = Bool()
    val miniHarm = Bool()
    val miniWrong = Bool()
    val miniProvider = UInt(MiniTageConfig.tableIdWidth.W)

    val loopEligible = Bool()
    val loopDisagree = Bool()
    val loopRecover = Bool()
    val loopHarm = Bool()
    val loopWrong = Bool()
}
