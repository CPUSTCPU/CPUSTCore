package CPUSTC.backend.rename

import chisel3._
import chisel3.util._

import CPUSTC.config.Commit._
import CPUSTC.config.Decode._
import CPUSTC.config.RegisterFile._
import CPUSTC.config.RenameConfig._

/**
  * A clustered FIFO of free physical-register numbers.
  *
  * This keeps ClusterIndexFIFO's compacted multi-lane enqueue/dequeue model,
  * but adds the speculative and architectural heads required by rename
  * recovery. Four 16-entry banks make every three-entry window use at most one
  * read and one write per bank on the configured three-wide core.
  */
class PregFreeQueue extends Module {
    val io = IO(new FreeListIO)

    private val depth = npreg
    private val numBanks = 4
    private val rowsPerBank = depth / numBanks
    private val indexWidth = log2Ceil(depth)
    private val ptrWidth = indexWidth + 1
    private val bankWidth = log2Ceil(numBanks)
    private val rowWidth = log2Ceil(rowsPerBank)

    require(isPow2(depth), "PregFreeQueue requires a power-of-two physical register count")
    require(depth % numBanks == 0)
    require(ndcd < numBanks, "consecutive allocation candidates must occupy distinct banks")
    require(ncmt < numBanks, "consecutive commit frees must occupy distinct banks")

    // Keep the payload reset-free so each bank can infer as a single-read,
    // single-write asynchronous LUTRAM. All four banks are initialized in
    // parallel before rename is allowed to advance.
    val banks = Seq.tabulate(numBanks) { bank =>
        Mem(rowsPerBank, UInt(wpreg.W)).suggestName(s"bank_$bank")
    }

    val initRow = RegInit(0.U(rowWidth.W))
    val initialized = RegInit(false.B)

    when(!initialized) {
        when(initRow === (rowsPerBank - 1).U) {
            initialized := true.B
        }.otherwise {
            initRow := initRow + 1.U
        }
    }

    // Extended pointers use the high bit as the circular generation. The queue
    // has 64 storage positions but contains at most preg 1 through preg 63.
    val specHead = RegInit(0.U(ptrWidth.W))
    val archHead = RegInit(0.U(ptrWidth.W))
    val tail     = RegInit((npreg - 1).U(ptrWidth.W))

    val branchHeadSnapshots = Reg(Vec(maxBrCount, UInt(ptrWidth.W)))

    private def distance(young: UInt, old: UInt): UInt =
        (young - old)(ptrWidth - 1, 0)

    private def addPtr(ptr: UInt, amount: UInt): UInt =
        (ptr + amount)(ptrWidth - 1, 0)

    val freeCount = distance(tail, specHead)
    val allocatedNotCommitted = distance(specHead, archHead)
    io.freeCount := Mux(initialized, freeCount, 0.U)

    val allocReqCount = PopCount(io.allocReqs)
    // This must also block bundles with no destination-register requests.
    io.canAllocate := initialized && freeCount >= allocReqCount

    // Read the next three FIFO entries in parallel. Consecutive addresses map
    // to distinct banks, matching ClusterIndexFIFO's striped-bank principle.
    val candidatePtrs = Wire(Vec(ndcd, UInt(ptrWidth.W)))
    val candidateBanks = Wire(Vec(ndcd, UInt(bankWidth.W)))
    val candidateRows = Wire(Vec(ndcd, UInt(rowWidth.W)))

    for (i <- 0 until ndcd) {
        candidatePtrs(i) := addPtr(specHead, i.U)
        candidateBanks(i) := candidatePtrs(i)(bankWidth - 1, 0)
        candidateRows(i) := candidatePtrs(i)(indexWidth - 1, bankWidth)
    }

    val bankReadRows = Wire(Vec(numBanks, UInt(rowWidth.W)))
    val bankReadData = Wire(Vec(numBanks, UInt(wpreg.W)))

    for (bank <- 0 until numBanks) {
        val hits = VecInit((0 until ndcd).map { i =>
            candidateBanks(i) === bank.U
        })

        bankReadRows(bank) := Mux(
            hits.asUInt.orR,
            Mux1H(hits, candidateRows),
            0.U
        )
        bankReadData(bank) := banks(bank).read(bankReadRows(bank))
        assert(PopCount(hits) <= 1.U)
    }

    val candidates = Wire(Vec(ndcd, UInt(wpreg.W)))
    for (i <- 0 until ndcd) {
        candidates(i) := Mux1H(
            UIntToOH(candidateBanks(i), numBanks),
            bankReadData
        )
    }

    for (i <- 0 until ndcd) {
        val rank = if (i == 0) {
            0.U(log2Ceil(ndcd + 1).W)
        } else {
            PopCount(io.allocReqs.take(i))
        }

        io.allocResps(i).valid := io.allocReqs(i) && io.canAllocate
        io.allocResps(i).bits := MuxLookup(rank, 0.U)(
            (0 until ndcd).map { candidate =>
                candidate.U -> candidates(candidate)
            }
        )
    }

    val mispredict =
        initialized &&
            io.branchUpdate.valid &&
            io.branchUpdate.bits.mispredictMask.orR
    val doAllocate = io.doAllocate && io.canAllocate && !io.flush && !mispredict
    val allocatedCount = Mux(doAllocate, allocReqCount, 0.U)
    val allocatedHeadNext = addPtr(specHead, allocatedCount)

    // A branch snapshot points immediately after all allocations through that
    // lane, preserving the branch's own destination while discarding younger
    // same-bundle allocations on a misprediction.
    for (lane <- 0 until ndcd) {
        val snapshotCount = PopCount(io.allocReqs.take(lane + 1))
        when(
            initialized &&
                io.brSnapshotReqs(lane).valid &&
                !io.flush &&
                !mispredict
        ) {
            branchHeadSnapshots(io.brSnapshotReqs(lane).bits) :=
                addPtr(specHead, snapshotCount)
        }
    }

    val restoreHead = Mux1H(
        io.branchUpdate.bits.resolveMask.asBools,
        branchHeadSnapshots
    )

    val commitAlloc = VecInit(io.commit.map { commit =>
        initialized &&
            commit.valid &&
            commit.bits.ldest =/= 0.U &&
            commit.bits.ldestValid &&
            commit.bits.rfWen &&
            commit.bits.pdest =/= 0.U
    })
    val commitFree = VecInit(io.commit.zip(commitAlloc).map { case (commit, alloc) =>
        alloc && commit.bits.pprd =/= 0.U
    })

    val commitAllocCount = PopCount(commitAlloc)
    val commitFreeCount = PopCount(commitFree)
    val archHeadNext = addPtr(archHead, commitAllocCount)
    val tailNext = addPtr(tail, commitFreeCount)

    // Compact valid frees at the tail. Since at most three consecutive writes
    // occur, the four-bank layout guarantees at most one write to each bank.
    val freePtrs = Wire(Vec(ncmt, UInt(ptrWidth.W)))
    val freeBanks = Wire(Vec(ncmt, UInt(bankWidth.W)))
    val freeRows = Wire(Vec(ncmt, UInt(rowWidth.W)))

    for (i <- 0 until ncmt) {
        val rank = if (i == 0) 0.U else PopCount(commitFree.take(i))
        freePtrs(i) := addPtr(tail, rank)
        freeBanks(i) := freePtrs(i)(bankWidth - 1, 0)
        freeRows(i) := freePtrs(i)(indexWidth - 1, bankWidth)
    }

    for (bank <- 0 until numBanks) {
        val writeHits = VecInit((0 until ncmt).map { i =>
            commitFree(i) && freeBanks(i) === bank.U
        })

        val normalWriteEnable = writeHits.asUInt.orR
        val normalWriteRow = Mux1H(writeHits, freeRows)
        val normalWriteData = Mux1H(writeHits, io.commit.map(_.bits.pprd))

        val scrubAddress = Cat(initRow, bank.U(bankWidth.W))
        val scrubData = Mux(
            scrubAddress === (npreg - 1).U,
            0.U(wpreg.W),
            (scrubAddress + 1.U)(wpreg - 1, 0)
        )

        // One write call per bank keeps the inferred memory at one write port.
        when(!initialized || normalWriteEnable) {
            banks(bank).write(
                Mux(initialized, normalWriteRow, initRow),
                Mux(initialized, normalWriteData, scrubData)
            )
        }

        assert(PopCount(writeHits) <= 1.U)
    }

    archHead := archHeadNext
    tail := tailNext

    when(io.flush) {
        specHead := archHeadNext
    }.elsewhen(mispredict) {
        specHead := restoreHead
    }.otherwise {
        specHead := allocatedHeadNext
    }

    // Structural and protocol assertions stay narrow; full ownership and
    // uniqueness are checked by the randomized software-model test.
    assert(freeCount <= (npreg - 1).U)
    assert(allocatedNotCommitted <= (npreg - 1).U)

    when(commitAllocCount.orR) {
        assert(commitAllocCount <= allocatedNotCommitted)
    }

    for (i <- 0 until ndcd) {
        when(io.brSnapshotReqs(i).valid) {
            assert(doAllocate)
        }
        when(io.allocResps(i).valid) {
            assert(io.allocResps(i).bits =/= 0.U)
        }
    }

    for (i <- 0 until ndcd; j <- i + 1 until ndcd) {
        when(io.allocResps(i).valid && io.allocResps(j).valid) {
            assert(io.allocResps(i).bits =/= io.allocResps(j).bits)
        }
        when(io.brSnapshotReqs(i).valid && io.brSnapshotReqs(j).valid) {
            assert(io.brSnapshotReqs(i).bits =/= io.brSnapshotReqs(j).bits)
        }
    }

    when(initialized && io.branchUpdate.valid && !io.flush) {
        assert(PopCount(io.branchUpdate.bits.resolveMask) === 1.U)
        assert(
            (io.branchUpdate.bits.mispredictMask &
                (~io.branchUpdate.bits.resolveMask).asUInt) === 0.U
        )
    }

}
