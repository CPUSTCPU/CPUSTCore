package CPUSTC.backend.issue

import chisel3._
import chisel3.util._

import CPUSTC.config.RegisterFile._
import CPUSTC.config.RenameConfig._
import CPUSTC.config.FunctionUnit._
import CPUSTC.config.MemIssueOp._
import CPUSTC.config.Issue._
import CPUSTC.config.WritebackConfig._
import CPUSTC.backend.dispatch.{DispatchUop, IssueDispatchIO, StqPtr}
import CPUSTC.backend.branch.BranchUpdate
import CPUSTC.backend.rob.RobPtr
import CPUSTC.frontend.FtqPtr

class IssueAgeMatrix(
    numEntries: Int,
    enqWidth: Int,
    selectWidth: Int
) extends Module {
    require(numEntries > 1)
    require(enqWidth > 0)
    require(selectWidth > 0)

    val io = IO(new Bundle {
        val flush = Input(Bool())
        val enqOH = Input(Vec(enqWidth, UInt(numEntries.W)))
        val request = Input(Vec(selectWidth, UInt(numEntries.W)))
        val oldestOH = Output(Vec(selectWidth, UInt(numEntries.W)))
    })

    private val pairCount = numEntries * (numEntries - 1) / 2
    private val age = RegInit(VecInit(Seq.fill(pairCount)(false.B)))

    private def pairIndex(left: Int, right: Int): Int = {
        require(left < right)
        left * (2 * numEntries - left - 1) / 2 + right - left - 1
    }

    private def isOlder(left: Int, right: Int): Bool = {
        if (left < right) {
            age(pairIndex(left, right))
        } else if (left == right) {
            true.B
        } else {
            !age(pairIndex(right, left))
        }
    }

    private val enqHit = VecInit((0 until numEntries).map { entry =>
        VecInit(io.enqOH.map(_(entry))).asUInt.orR
    })

    when(io.flush) {
        age := 0.U.asTypeOf(age)
    }.otherwise {
        for (left <- 0 until numEntries; right <- left + 1 until numEntries) {
            val leftOlderInBundle = (for {
                olderLane <- 0 until enqWidth
                youngerLane <- olderLane + 1 until enqWidth
            } yield {
                io.enqOH(olderLane)(left) && io.enqOH(youngerLane)(right)
            }).reduceOption(_ || _).getOrElse(false.B)

            when(enqHit(left) && enqHit(right)) {
                age(pairIndex(left, right)) := leftOlderInBundle
            }.elsewhen(enqHit(left)) {
                age(pairIndex(left, right)) := false.B
            }.elsewhen(enqHit(right)) {
                age(pairIndex(left, right)) := true.B
            }
        }
    }

    for (select <- 0 until selectWidth) {
        val oldestVec = Wire(Vec(numEntries, Bool()))

        for (entry <- 0 until numEntries) {
            val blockedTerms = (0 until numEntries)
                .filter(_ != entry)
                .map { other =>
                    io.request(select)(other) && !isOlder(entry, other)
                }

            // Three pairwise blockers consume at most six LUT inputs. Preserve
            // these group boundaries so the final oldest test is a second LUT.
            val blockedGroups = blockedTerms.grouped(3).toSeq.map { group =>
                val blocked = Wire(Bool())
                blocked := group.reduce(_ || _)
                dontTouch(blocked)
                blocked
            }

            oldestVec(entry) :=
                io.request(select)(entry) &&
                !VecInit(blockedGroups).asUInt.orR
        }

        io.oldestOH(select) := oldestVec.asUInt

        when(io.request(select).orR) {
            assert(PopCount(io.oldestOH(select)) === 1.U)
        }.otherwise {
            assert(!io.oldestOH(select).orR)
        }
    }

    for (lane <- 0 until enqWidth) {
        assert(PopCount(io.enqOH(lane)) <= 1.U)
    }

    for (entry <- 0 until numEntries) {
        assert(PopCount(VecInit(io.enqOH.map(_(entry)))) <= 1.U)
    }
}

class IssueWakeup extends Bundle {
    val pdest = UInt(wpreg.W)
    val fast  = Bool()
}

class IssueOut extends Bundle {
    val uop = new DispatchUop
    val memOp = UInt(MEMQ_SZ.W)

    val src1Read = Bool()
    val src2Read = Bool()
    val src1FastWakeup = Bool()
    val src2FastWakeup = Bool()
}

class IntIssueEntry extends Bundle {
    val valid = Bool()
    val uop = new DispatchUop
    val src1Ready = Bool()
    val src2Ready = Bool()
    val src1FastWakeup = Bool()
    val src2FastWakeup = Bool()
    val src1LoadPoison = UInt(memNissue.W)
    val src2LoadPoison = UInt(memNissue.W)
    val src1IntProducerOH = UInt(intNiq.W)
    val src2IntProducerOH = UInt(intNiq.W)
}

class MemIssueEntry extends Bundle {
    val valid = Bool()
    val uop = new DispatchUop
    val src1Ready = Bool()
    val src2Ready = Bool()
    val src1FastWakeup = Bool()
    val src2FastWakeup = Bool()
    val src1LoadPoison = UInt(memNissue.W)
    val src2LoadPoison = UInt(memNissue.W)
    val src1IntProducerOH = UInt(intNiq.W)
    val src2IntProducerOH = UInt(intNiq.W)
    val staIssued = Bool()
    val stdIssued = Bool()
}

class IssueQueueIO(
    numEntries: Int,
    enqWidth: Int,
    issueWidth: Int
) extends Bundle {
    val flush = Input(Bool())

    val dispatch = new IssueDispatchIO(enqWidth)

    val wakeup = Input(Vec(nwkp, Valid(new IssueWakeup)))
    val loadPredWake = Input(Vec(
        memNissue,
        Valid(new CPUSTC.memory.LoadPredictInfo)
    ))
    val loadPredResolve = Input(Vec(
        memNissue,
        Valid(new CPUSTC.memory.LoadPredictResolve)
    ))

    val portCaps = Input(Vec(issueWidth, UInt(FUC_SZ.W)))

    val issue = Vec(issueWidth, Decoupled(new IssueOut))

    val validMask = Output(UInt(numEntries.W))
    val canIssueMask = Output(UInt(numEntries.W))
    val loadPredIssueCount = Output(UInt(log2Ceil(issueWidth + 1).W))

    val full = Output(Bool())
    val empty = Output(Bool())

    val branchUpdate = Flipped(Valid(new BranchUpdate))
}

class IntIssueQueueIO extends IssueQueueIO(
    numEntries = intNiq,
    enqWidth = CPUSTC.config.Decode.ndcd,
    issueWidth = intNissue
) {
    val p2DeferredWake = Input(Bool())
    val p2FixedWakeAccepted = Input(Bool())
    val fastWakeup = Output(Vec(nFastIntWb, Valid(new IssueWakeup)))
    val residentIntProducers = Output(Vec(
        intNiq,
        Valid(UInt(wpreg.W))
    ))
    val dispatchAllocOH = Output(Vec(
        CPUSTC.config.Decode.ndcd,
        UInt(intNiq.W)
    ))
    val producerFastWakeMask = Output(UInt(intNiq.W))
    val producerReleaseMask = Output(UInt(intNiq.W))
    val robHead = Flipped(Vec(intNiq, Valid(new RobPtr)))
    val robHeadGeneration = Input(Bool())
    val ftqPredictionReadReq = Output(Valid(new FtqPtr))
}

class MemIssueQueueIO extends IssueQueueIO(
    numEntries = CPUSTC.config.Issue.memNiq,
    enqWidth = CPUSTC.config.Decode.ndcd,
    issueWidth = memNissue
) {
    val intFastWakeup = Input(Vec(nFastIntWb, Valid(new IssueWakeup)))
    val intProducerFastWakeMask = Input(UInt(intNiq.W))
    val intProducerReleaseMask = Input(UInt(intNiq.W))
    val staEarlyAccepted = Input(Vec(memNissue, Valid(new StqPtr)))
    val staAccepted = Input(Vec(memNissue, Valid(new StqPtr)))
    val loadPredLoadIssueCount = Output(UInt(log2Ceil(memNissue + 1).W))
}
