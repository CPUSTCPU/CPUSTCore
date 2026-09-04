package CPUSTC.utils

import chisel3._
import chisel3.util._

import CPUSTC.config.PickNRotOHParallel
import CPUSTC.config.ShiftAdd1

class ClusterFreeIndexQueueIO(numEntries: Int, allocWidth: Int) extends Bundle {
    private val countWidth = log2Ceil(numEntries + 1)
    private val allocCountWidth = log2Ceil(allocWidth + 1)

    val flush = Input(Bool())

    val allocCount = Input(UInt(allocCountWidth.W))
    val allocate = Input(Bool())
    val canAllocate = Output(Bool())
    val allocOH = Output(Vec(allocWidth, UInt(numEntries.W)))

    val fastReleaseOH = Input(Vec(allocWidth, UInt(numEntries.W)))
    val registeredReturnOH = Input(Vec(allocWidth, UInt(numEntries.W)))
    val registeredReturnCount = Input(UInt(allocCountWidth.W))
    val normalReleaseMask = Input(UInt(numEntries.W))
    val releaseMask = Input(UInt(numEntries.W))

    val freeCount = Output(UInt(countWidth.W))
    val allocatedMask = Output(UInt(numEntries.W))
    val pendingReleaseMask = Output(UInt(numEntries.W))
}

/**
  * Banked free-index queue for unordered structures such as issue queues.
  *
  * The logical FIFO is striped over allocWidth banks. Consecutive allocation
  * candidates therefore use distinct banks, following ClusterIndexFIFO's
  * compacted multi-lane access pattern. Banks can store compact binary physical
  * indices while registered return and external allocation boundaries remain
  * one-hot.
  *
  * Normal issue releases arrive as independent one-hot lanes and stop directly
  * in a registered candidate stage. Candidate compaction happens after that
  * boundary. Candidates not consumed by allocation spill into the striped banks.
  * Branch recovery entries remain in pendingReleaseMask and drain directly into
  * bank write slots not used by registered candidate spills.
  */
class ClusterFreeIndexQueue(
    val numEntries: Int,
    val allocWidth: Int,
    val useExternalRegisteredReturn: Boolean = false,
    val useCompactBankPayload: Boolean = false
) extends Module {
    require(allocWidth > 0)
    require(numEntries >= allocWidth)
    require(numEntries % allocWidth == 0,
        "ClusterFreeIndexQueue requires equal striped banks")

    val io = IO(new ClusterFreeIndexQueueIO(numEntries, allocWidth))

    private val numBanks = allocWidth
    private val rowsPerBank = numEntries / numBanks
    private val countWidth = log2Ceil(numEntries + 1)
    private val allocCountWidth = log2Ceil(allocWidth + 1)
    private val indexWidth = log2Ceil(numEntries)
    private val bankPayloadWidth =
        if (useCompactBankPayload) indexWidth else numEntries

    private def initialEntry(bank: Int, row: Int): UInt =
        if (useCompactBankPayload) {
            (row * numBanks + bank).U(indexWidth.W)
        } else {
            (BigInt(1) << (row * numBanks + bank)).U(numEntries.W)
        }

    private def initialBanks: Vec[Vec[UInt]] = VecInit.tabulate(numBanks) { bank =>
        VecInit.tabulate(rowsPerBank) { row =>
            initialEntry(bank, row)
        }
    }

    val banks = RegInit(initialBanks)

    val headBankOH = RegInit(1.U(numBanks.W))
    val headRowOH = RegInit(1.U(rowsPerBank.W))
    val tailBankOH = RegInit(1.U(numBanks.W))
    val tailRowOH = RegInit(1.U(rowsPerBank.W))

    val mainFreeCount = RegInit(numEntries.U(countWidth.W))
    val returnRegs: Option[Vec[UInt]] =
        if (useExternalRegisteredReturn) {
            None
        } else {
            Some(RegInit(VecInit.fill(allocWidth)(0.U(numEntries.W))))
        }
    val returnCandidates = returnRegs.getOrElse(io.registeredReturnOH)
    val pendingReleaseMask = RegInit(0.U(numEntries.W))
    val allocatedMask = RegInit(0.U(numEntries.W))

    private def steppedBanks(baseBankOH: UInt): Vec[UInt] = {
        val result = Wire(Vec(allocWidth + 1, UInt(numBanks.W)))
        result(0) := baseBankOH
        for (i <- 0 until allocWidth) {
            result(i + 1) := ShiftAdd1(result(i))
        }
        result
    }

    private def steppedRows(baseBankOH: UInt, baseRowOH: UInt): Vec[UInt] = {
        val banksAtStep = steppedBanks(baseBankOH)
        val result = Wire(Vec(allocWidth + 1, UInt(rowsPerBank.W)))
        result(0) := baseRowOH
        for (i <- 0 until allocWidth) {
            result(i + 1) := Mux(
                banksAtStep(i)(numBanks - 1),
                ShiftAdd1(result(i)),
                result(i)
            )
        }
        result
    }

    private def selectStep(states: Vec[UInt], count: UInt): UInt =
        Mux1H((0 to allocWidth).map(i => count === i.U), states)

    val headBanks = steppedBanks(headBankOH)
    val headRows = steppedRows(headBankOH, headRowOH)
    val bankAllocOH = Wire(Vec(allocWidth, UInt(numEntries.W)))

    for (lane <- 0 until allocWidth) {
        val bankReadData = Wire(Vec(numBanks, UInt(bankPayloadWidth.W)))
        for (bank <- 0 until numBanks) {
            bankReadData(bank) := Mux1H(headRows(lane), banks(bank))
        }

        val bankReadPayload = Mux1H(headBanks(lane), bankReadData)
        val bankReadOH = if (useCompactBankPayload) {
            UIntToOH(bankReadPayload, numEntries)
        } else {
            bankReadPayload
        }
        bankAllocOH(lane) := Mux(
            mainFreeCount > lane.U,
            bankReadOH,
            0.U
        )
    }

    val returnValid = VecInit(returnCandidates.map(_.orR))
    val returnCount = if (useExternalRegisteredReturn) {
        io.registeredReturnCount
    } else {
        PopCount(returnValid)
    }
    val totalFreeCount = mainFreeCount + returnCount

    val compactedReturn = Wire(Vec(allocWidth, UInt(numEntries.W)))
    for (out <- 0 until allocWidth) {
        val matches = (0 until allocWidth).map { in =>
            val priorCount = if (in == 0) {
                0.U(log2Ceil(allocWidth + 1).W)
            } else {
                PopCount(returnValid.take(in))
            }
            returnValid(in) && priorCount === out.U
        }
        compactedReturn(out) := Mux1H(
            matches.zip(returnCandidates).map { case (select, data) =>
                select -> data
            }
        )
    }

    for (lane <- 0 until allocWidth) {
        io.allocOH(lane) := Mux1H((0 to allocWidth).map { count =>
            val candidate = if (lane < count) {
                compactedReturn(lane)
            } else {
                bankAllocOH(lane - count)
            }
            (returnCount === count.U) -> candidate
        })
    }

    io.canAllocate := totalFreeCount >= io.allocCount

    val doAllocate = io.allocate && io.canAllocate && !io.flush
    val allocatedCountNow = Mux(doAllocate, io.allocCount, 0.U)
    val allocatedNowMask = VecInit((0 until allocWidth).map { lane =>
        Mux(doAllocate && lane.U < io.allocCount, io.allocOH(lane), 0.U)
    }).reduce(_ | _)

    val returnConsumedCount = Mux(
        allocatedCountNow > returnCount,
        returnCount,
        allocatedCountNow
    )
    val bankAllocatedCount = allocatedCountNow - returnConsumedCount
    val spillCount = returnCount - returnConsumedCount

    val spillOH = Wire(Vec(allocWidth, UInt(numEntries.W)))
    for (lane <- 0 until allocWidth) {
        spillOH(lane) := Mux1H((0 to allocWidth).map { consumed =>
            val candidate = if (lane + consumed < allocWidth) {
                compactedReturn(lane + consumed)
            } else {
                0.U(numEntries.W)
            }
            (returnConsumedCount === consumed.U) -> candidate
        })
    }

    val fastReleaseMask = io.fastReleaseOH.reduce(_ | _)
    val normalReleaseMask =
        if (useExternalRegisteredReturn) io.normalReleaseMask
        else fastReleaseMask

    // Only registered recovery state enters the wide selector. A new branch
    // recovery is captured this cycle and becomes drainable next cycle.
    val recoveryOH = PickNRotOHParallel(
        req = pendingReleaseMask,
        baseOH = 1.U(numEntries.W),
        count = allocWidth
    )
    val pendingReleaseCount = PopCount(pendingReleaseMask)
    val pendingCountCapped = Mux(
        pendingReleaseCount > allocWidth.U,
        allocWidth.U(allocCountWidth.W),
        pendingReleaseCount(allocCountWidth - 1, 0)
    )

    // Recovery uses only bank write slots left by registered return candidates.
    // Current issue releases therefore stop at returnRegs and cannot affect the
    // pending recovery state or bank write controls in the same cycle.
    val recoveryCapacity = allocWidth.U - spillCount
    val recoveryUsedOH = VecInit((0 until allocWidth).map { lane =>
        Mux(lane.U < recoveryCapacity, recoveryOH(lane), 0.U(numEntries.W))
    })
    val returnedRecoveryMask = recoveryUsedOH.reduce(_ | _)
    val recoveryCount = Mux(
        pendingCountCapped < recoveryCapacity,
        pendingCountCapped,
        recoveryCapacity
    )
    val bankReturnCount = spillCount + recoveryCount
    val bankReturnValid = VecInit((0 until allocWidth).map { lane =>
        bankReturnCount > lane.U
    })

    // Return and recovery candidates already own inactive FIFO slots.  Enable
    // writes for that registered upper bound so current allocation cannot
    // enter the bank clock-enable cone; lanes beyond bankReturnCount write
    // don't-care data outside the live [head, tail) interval.
    val potentialBankWriteCount =
        returnCount +& pendingReleaseCount
    val bankWriteValid = VecInit((0 until allocWidth).map { lane =>
        potentialBankWriteCount > lane.U
    })

    val bankReturnOH = Wire(Vec(allocWidth, UInt(numEntries.W)))
    for (lane <- 0 until allocWidth) {
        bankReturnOH(lane) := Mux1H((0 to allocWidth).map { spills =>
            val candidate = if (lane < spills) {
                spillOH(lane)
            } else {
                recoveryUsedOH(lane - spills)
            }
            (spillCount === spills.U) -> candidate
        })
    }
    val bankReturnPayload = if (useCompactBankPayload) {
        VecInit(bankReturnOH.map(OHToUInt(_)))
    } else {
        bankReturnOH
    }

    val tailBanks = steppedBanks(tailBankOH)
    val tailRows = steppedRows(tailBankOH, tailRowOH)

    when(io.flush) {
        banks := initialBanks
    }.otherwise {
        for (bank <- 0 until numBanks) {
            val writeHits = VecInit((0 until allocWidth).map { lane =>
                bankWriteValid(lane) && tailBanks(lane)(bank)
            })

            when(writeHits.asUInt.orR) {
                val writeRow = Mux1H(writeHits, tailRows.take(allocWidth))
                val writeData = Mux1H(writeHits, bankReturnPayload)
                for (row <- 0 until rowsPerBank) {
                    when(writeRow(row)) {
                        banks(bank)(row) := writeData
                    }
                }
            }

            assert(PopCount(writeHits) <= 1.U)
        }
    }

    when(io.flush) {
        headBankOH := 1.U
        headRowOH := 1.U
        tailBankOH := 1.U
        tailRowOH := 1.U
        mainFreeCount := numEntries.U
        returnRegs.foreach(_ := VecInit.fill(allocWidth)(0.U))
        pendingReleaseMask := 0.U
        allocatedMask := 0.U
    }.otherwise {
        headBankOH := selectStep(headBanks, bankAllocatedCount)
        headRowOH := selectStep(headRows, bankAllocatedCount)
        tailBankOH := selectStep(tailBanks, bankReturnCount)
        tailRowOH := selectStep(tailRows, bankReturnCount)

        mainFreeCount := mainFreeCount - bankAllocatedCount + bankReturnCount
        returnRegs.foreach(_ := io.fastReleaseOH)
        pendingReleaseMask :=
            (pendingReleaseMask | io.releaseMask) &
                (~returnedRecoveryMask).asUInt
        allocatedMask := (allocatedMask | allocatedNowMask) &
            (~(io.releaseMask | normalReleaseMask)).asUInt
    }

    io.freeCount := totalFreeCount
    io.allocatedMask := allocatedMask
    io.pendingReleaseMask := pendingReleaseMask

    assert(PopCount(headBankOH) === 1.U)
    assert(PopCount(headRowOH) === 1.U)
    assert(PopCount(tailBankOH) === 1.U)
    assert(PopCount(tailRowOH) === 1.U)
    assert(mainFreeCount <= numEntries.U)
    assert(totalFreeCount <= numEntries.U)
    assert(recoveryCount <= recoveryCapacity)
    assert(recoveryCount === PopCount(VecInit(recoveryUsedOH.map(_.orR))))
    assert((returnedRecoveryMask & normalReleaseMask) === 0.U)
    assert(bankReturnCount <= allocWidth.U)
    assert(bankReturnCount === PopCount(VecInit(bankReturnOH.map(_.orR))))
    assert(bankReturnCount <= PopCount(bankWriteValid))
    assert(
        mainFreeCount +& PopCount(bankWriteValid) <= numEntries.U
    )

    when(io.allocate && !io.flush) {
        assert(io.canAllocate)
    }

    when(io.releaseMask.orR && !io.flush) {
        assert((io.releaseMask & (~allocatedMask).asUInt) === 0.U)
        assert((io.releaseMask & pendingReleaseMask) === 0.U)
    }

    when(normalReleaseMask.orR && !io.flush) {
        assert((normalReleaseMask & (~allocatedMask).asUInt) === 0.U)
        assert((normalReleaseMask & io.releaseMask) === 0.U)
        assert((normalReleaseMask & pendingReleaseMask) === 0.U)
    }

    for (lane <- 0 until allocWidth) {
        assert(PopCount(io.allocOH(lane)) <= 1.U)
        if (useExternalRegisteredReturn) {
            assert(PopCount(io.registeredReturnOH(lane)) <= 1.U)
        } else {
            assert(PopCount(io.fastReleaseOH(lane)) <= 1.U)
        }
        assert(PopCount(compactedReturn(lane)) <= 1.U)
        assert(PopCount(spillOH(lane)) <= 1.U)
        assert(PopCount(bankReturnOH(lane)) <= 1.U)
        assert(bankReturnValid(lane) === bankReturnOH(lane).orR)
    }

    for (i <- 0 until allocWidth; j <- i + 1 until allocWidth) {
        assert((io.allocOH(i) & io.allocOH(j)) === 0.U)
        if (useExternalRegisteredReturn) {
            assert(
                (io.registeredReturnOH(i) & io.registeredReturnOH(j)) === 0.U
            )
        } else {
            assert((io.fastReleaseOH(i) & io.fastReleaseOH(j)) === 0.U)
        }
        assert((compactedReturn(i) & compactedReturn(j)) === 0.U)
        assert((spillOH(i) & spillOH(j)) === 0.U)
        assert((bankReturnOH(i) & bankReturnOH(j)) === 0.U)
    }

    val returnCandidateMask = returnCandidates.reduce(_ | _)
    assert((returnCandidateMask & allocatedMask) === 0.U)
    assert((returnCandidateMask & pendingReleaseMask) === 0.U)

    assert(
        PopCount(allocatedMask) + totalFreeCount + PopCount(pendingReleaseMask) ===
            numEntries.U
    )
}
