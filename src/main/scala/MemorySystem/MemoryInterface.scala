package CPUSTC.memory
import CPUSTC.memory._
import chisel3._
import chisel3.util._
import CPUSTC.config.{RotateLeftOH, RotateRightOH}
import CPUSTC.config.RegisterFile._
import CPUSTC.backend.rob.RobPtr

class MemoryUopType extends Bundle {
    val isSTA = Bool()
    val isSTD = Bool()
    val isLD = Bool()
    val isRefill = Bool()
}

class BackendInst extends Bundle {
    val uop = new MemoryUopType
    val pc = UInt(32.W)
    val paddr = UInt(32.W)
    // Page-mode accesses that missed both DMWs are translated at the
    // MemorySubSystem boundary.  Such a request must not be accepted by the
    // LSQ until its MainTLB response is available.
    val translationPending = Bool()
    val uncache = Bool()
    val operateData = UInt(32.W)
    val mask = UInt(4.W)
    val valid = Bool()
    val signed = Bool()
    val sqindex = UInt(StoreQueueConfig.length.W)
    val sqindexHigh = Bool()
    val storeDepMask = UInt(StoreQueueConfig.length.W)
    val ldindex = UInt(LoadStateTableConfig.length.W)
    val ldindexHigh = Bool()
    val soreceReg = UInt(6.W)
    val Poisoned = Bool()
    val exception = UInt(8.W)
    val exceptionBadvValid = Bool()
    val exceptionBadv = UInt(32.W)

    // Carried end-to-end so a delayed memory response can complete the exact
    // ROB/physical-register generation that created the request.
    val robPtr = new RobPtr
    val pdest = UInt(wpreg.W)
    val rfWen = Bool()
}

/** A confirmed aligned, cached Load whose address mode was resolved before the
  * MemExecute output register. Constants such as operation class, exception
  * state and cacheability are reconstructed at the LoadStorePipeline entrance
  * so rare memory controls never enter this fast channel.
  */
class DirectCachedLoad extends Bundle {
    val vaddr = UInt(32.W)
    val paddr = UInt(32.W)
    val mask = UInt(4.W)
    val signed = Bool()
    val sqindex = UInt(StoreQueueConfig.length.W)
    val sqindexHigh = Bool()
    val storeDepMask = UInt(StoreQueueConfig.length.W)
    val ldindex = UInt(LoadStateTableConfig.length.W)
    val ldindexHigh = Bool()
    val robPtr = new RobPtr
    val pdest = UInt(wpreg.W)
    val rfWen = Bool()
}

/** A serialized word Store issued by the ROB-head SC slow path.  The request
  * deliberately carries no LSQ identity: LoadStorePipeline gives it a private
  * internal tag and consumes every retry/completion locally.
  */
class AtomicStoreRequest extends Bundle {
    val paddr = UInt(32.W)
    val data = UInt(32.W)
    val uncache = Bool()
}

class StoreCompletionToken extends Bundle {
    val robPtr = new RobPtr
}

class StoreExceptionEvent extends Bundle {
    val robPtr = new RobPtr
    val sqindex = UInt(StoreQueueConfig.length.W)
    val sqindexHigh = Bool()
    val cause = UInt(8.W)
    val badvValid = Bool()
    val badv = UInt(32.W)
}

class LoadIndex extends Bundle {
    val ldindex = UInt(LoadStateTableConfig.length.W)
    val ldindexHigh = Bool()
}

class RobHeadLoadInfo extends Bundle {
    val robPtr = new RobPtr
    // True only while this head load still blocks retirement.
    val waiting = Bool()
}

class StoreForwardSource extends Bundle {
    val valid = Bool()
    val addrValid = Bool()
    val dataValid = Bool()
    val paddr = UInt(32.W)
    val data = UInt(32.W)
    val alignedMask = UInt((DcacheConfig.DcacheMaskBits * 2).W)
    val sqindex = UInt(StoreQueueConfig.length.W)
    val sqindexHigh = Bool()
    val olderStoreMask = UInt(StoreQueueConfig.length.W)
    val committed = Bool()
}

class LoadResult extends Bundle {
    val inst = new BackendInst
    val data = UInt(32.W)
    val exception = UInt(8.W)
}

class LoadDebugEvent extends Bundle {
    val robPtr = new RobPtr
    val vaddr = UInt(32.W)
    val paddr = UInt(32.W)
    val mask = UInt(4.W)
    val signed = Bool()
}

class StoreCommitTrace extends Bundle {
    val robPtr = new RobPtr
    val vaddr = UInt(32.W)
    val paddr = UInt(32.W)
    val data = UInt(32.W)
    val mask = UInt(4.W)
    val uncache = Bool()
}

class LoadPredictInfo extends Bundle {
    val pdest = UInt(wpreg.W)
    val ldindex = UInt(LoadStateTableConfig.length.W)
    val ldindexHigh = Bool()
    val robPtr = new RobPtr
}

class LoadPredictResolve extends Bundle {
    val info = new LoadPredictInfo
    val success = Bool()
}

class MemoryLsqLiveState extends Bundle {
    val ldqValidMask = UInt(LoadStateTableConfig.length.W)
    val ldqHighMask = UInt(LoadStateTableConfig.length.W)
    val stqValidMask = UInt(StoreQueueConfig.length.W)
    val stqHighMask = UInt(StoreQueueConfig.length.W)
    val stqTailOH = UInt(StoreQueueConfig.length.W)
    val stqTailHigh = Bool()
}

object MemoryPointerUtils {
    def pointerAlive(
        indexOH: UInt,
        indexHigh: Bool,
        validMask: UInt,
        highMask: UInt
    ): Bool = {
        require(indexOH.getWidth == validMask.getWidth)
        require(indexOH.getWidth == highMask.getWidth)

        val generationMask = Mux(indexHigh, highMask, (~highMask).asUInt)
        (indexOH & validMask & generationMask).orR
    }

    def pointerEqual(aOH: UInt, aHigh: Bool, bOH: UInt, bHigh: Bool): Bool = {
        (aOH & bOH).orR && aHigh === bHigh
    }

    def pointerOlderThanBoundary(
        indexOH: UInt,
        indexHigh: Bool,
        boundaryOH: UInt,
        boundaryHigh: Bool
    ): Bool = {
        require(indexOH.getWidth == boundaryOH.getWidth)

        val width = indexOH.getWidth
        val beforeBoundary = VecInit((0 until width).map { index =>
            if (index == width - 1) false.B
            else boundaryOH(width - 1, index + 1).orR
        }).asUInt
        val expectedHigh = Mux(
            (indexOH & beforeBoundary).orR,
            boundaryHigh,
            !boundaryHigh
        )

        indexHigh === expectedHigh
    }

    // Util: to decide whether index alive in head and tail
    def pointerInRange(
        indexOH: UInt,
        indexHigh: Bool,
        headOH: UInt,
        headHigh: Bool,
        tailOH: UInt,
        tailHigh: Bool
    ): Bool = {
        val width = indexOH.getWidth
        val atOrAfterHead = VecInit((0 until width).map { index =>
            headOH(index, 0).orR
        }).asUInt
        val beforeTail = VecInit((0 until width).map { index =>
            if (index == width - 1) false.B else tailOH(width - 1, index + 1).orR
        }).asUInt
        val sameGeneration = headHigh === tailHigh
        val sameRange = atOrAfterHead & beforeTail
        val wrappedHeadRange = atOrAfterHead
        val wrappedTailRange = beforeTail

        Mux(sameGeneration,
            (sameRange & indexOH).orR && indexHigh === headHigh,
            ((wrappedHeadRange & indexOH).orR && indexHigh === headHigh) ||
                ((wrappedTailRange & indexOH).orR && indexHigh === tailHigh))
    }

    def selectOldestOH(candidates: UInt, headOH: UInt): UInt = {
        val rotated = RotateRightOH(candidates, headOH)
        RotateLeftOH(PriorityEncoderOH(rotated), headOH)
    }

    def selectYoungestOH(candidates: UInt, tailOH: UInt): UInt = {
        val rotated = RotateRightOH(candidates, tailOH)
        val highest = Reverse(PriorityEncoderOH(Reverse(rotated)))
        RotateLeftOH(highest, tailOH)
    }
}
