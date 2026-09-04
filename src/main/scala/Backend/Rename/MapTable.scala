package CPUSTC.backend.rename

import chisel3._
import chisel3.util._

import CPUSTC.config.RegisterFile._
import CPUSTC.config.Decode._
import CPUSTC.config.Commit._
import CPUSTC.config.RenameConfig._
import CPUSTC.backend.branch.BranchUpdate

class RenameMapTableIO extends Bundle {
    val flush = Input(Bool())

    val branchUpdate = Flipped(Valid(new BranchUpdate))

    val brSnapshotReqs = Input(Vec(ndcd, Valid(UInt(wBrTag.W))))

    val readReqs  = Input(Vec(ndcd, new RenameMapReadReq))
    val readResps = Output(Vec(ndcd, new RenameMapReadResp))

    val rnmReqs = Input(Vec(ndcd, Valid(new RenameMapWriteReq)))
    val cmtReqs = Input(Vec(ncmt, Valid(new RenameMapWriteReq)))
}

class MapTable extends Module {
    val io = IO(new RenameMapTableIO)

    private val ratInit = VecInit(Seq.fill(nlreg)(0.U(wpreg.W)))

    val rnmRat = RegInit(ratInit)
    val cmtRat = RegInit(ratInit)

    // Each rename lane owns one single-write snapshot bank. The branch tag's
    // owner mask selects the bank during recovery, avoiding a wide multi-write
    // register array while preserving the same-cycle post-lane snapshots.
    val brSnapshotBanks = Seq.fill(ndcd)(
        Mem(maxBrCount, Vec(nlreg - 1, UInt(wpreg.W)))
    )
    val brSnapshotOwnerMasks = RegInit(
        VecInit.fill(ndcd)(0.U(maxBrCount.W))
    )
    val pendingSnapshotOwnerValid = RegInit(
        VecInit.fill(ndcd)(false.B)
    )
    val pendingSnapshotOwnerTag = Reg(Vec(ndcd, UInt(wBrTag.W)))
    val pendingSnapshotPayload = Reg(
        Vec(ndcd, Vec(nlreg - 1, UInt(wpreg.W)))
    )

    for (i <- 0 until ndcd) {
        io.readResps(i).psrc1 := rnmRat(io.readReqs(i).lsrc1)
        io.readResps(i).psrc2 := rnmRat(io.readReqs(i).lsrc2)
        io.readResps(i).pprd  := rnmRat(io.readReqs(i).ldest)
    }

    val rnmRatTables = io.rnmReqs.scanLeft(rnmRat) { case (prevTable, req) =>
        VecInit(prevTable.zipWithIndex.map { case (oldPreg, lreg) =>
            if (lreg == 0) {
                0.U(wpreg.W)
            } else {
                Mux(req.valid && req.bits.ldest === lreg.U, req.bits.pdest, oldPreg)
            }
        })
    }

    val cmtRatTables = io.cmtReqs.scanLeft(cmtRat) { case (prevTable, req) =>
        VecInit(prevTable.zipWithIndex.map { case (oldPreg, lreg) =>
            if (lreg == 0) {
                0.U(wpreg.W)
            } else {
                Mux(req.valid && req.bits.ldest === lreg.U, req.bits.pdest, oldPreg)
            }
        })
    }

    val rnmRatNext = rnmRatTables.last
    val cmtRatNext = cmtRatTables.last

    cmtRat := cmtRatNext

    val mispredict = io.branchUpdate.valid && io.branchUpdate.bits.mispredictMask.orR

    val snapshotWriteValid = Wire(Vec(ndcd, Bool()))

    for (i <- 0 until ndcd) {
        snapshotWriteValid(i) :=
            io.brSnapshotReqs(i).valid && !io.flush && !mispredict

        // Payload and tag are deliberately reset-free and written every cycle;
        // pendingSnapshotOwnerValid is their only validity state. The LUTRAM
        // write therefore starts at this registered boundary instead of at the
        // Decode/Rename allocation cone. A killed pending snapshot may write
        // unused data, but it cannot install ownership below and cannot be read
        // by a live branch tag.
        pendingSnapshotOwnerTag(i) := io.brSnapshotReqs(i).bits
        pendingSnapshotPayload(i) := VecInit(rnmRatTables(i + 1).tail)

        when(pendingSnapshotOwnerValid(i)) {
            brSnapshotBanks(i).write(
                pendingSnapshotOwnerTag(i),
                pendingSnapshotPayload(i)
            )
        }

        pendingSnapshotOwnerValid(i) := snapshotWriteValid(i)
    }

    val pendingSnapshotOwnerMasks = VecInit((0 until ndcd).map { i =>
        Mux(
            pendingSnapshotOwnerValid(i) && !io.flush && !mispredict,
            UIntToOH(pendingSnapshotOwnerTag(i), maxBrCount),
            0.U(maxBrCount.W)
        )
    })
    val allSnapshotOwnerWrites = pendingSnapshotOwnerMasks.reduce(_ | _)

    when(allSnapshotOwnerWrites.orR) {
        for (i <- 0 until ndcd) {
            brSnapshotOwnerMasks(i) :=
                (brSnapshotOwnerMasks(i) & ~allSnapshotOwnerWrites) |
                    pendingSnapshotOwnerMasks(i)
        }
    }

    val restoreRat = Wire(Vec(nlreg, UInt(wpreg.W)))
    val restoreTag = OHToUInt(io.branchUpdate.bits.resolveMask)
    val restoreBankData = VecInit(brSnapshotBanks.map(_(restoreTag)))
    val restoreOwnerOH = VecInit(brSnapshotOwnerMasks.map { ownerMask =>
        (ownerMask & io.branchUpdate.bits.resolveMask).orR
    })

    restoreRat(0) := 0.U
    for (lreg <- 1 until nlreg) {
        restoreRat(lreg) := Mux1H(
            restoreOwnerOH,
            restoreBankData.map(_(lreg - 1))
        )
    }

    when(io.flush) {
        rnmRat := cmtRatNext
    }.elsewhen(mispredict) {
        rnmRat := restoreRat
    }.otherwise {
        rnmRat := rnmRatNext
    }

    when(io.branchUpdate.valid && !io.flush) {
        assert(PopCount(io.branchUpdate.bits.resolveMask) === 1.U)
        assert(
            (allSnapshotOwnerWrites & io.branchUpdate.bits.resolveMask) === 0.U,
            "branch resolved before its snapshot owner was installed"
        )
        assert(
            (io.branchUpdate.bits.mispredictMask &
                (~io.branchUpdate.bits.resolveMask).asUInt) === 0.U
        )
    }

    when(mispredict && !io.flush) {
        assert(
            io.branchUpdate.bits.mispredictMask ===
                io.branchUpdate.bits.resolveMask
        )
        assert(PopCount(restoreOwnerOH) === 1.U)
    }

    for (i <- 0 until ndcd; j <- i + 1 until ndcd) {
        when(
            io.brSnapshotReqs(i).valid &&
            io.brSnapshotReqs(j).valid &&
            !io.flush &&
            !mispredict
        ) {
            assert(io.brSnapshotReqs(i).bits =/= io.brSnapshotReqs(j).bits)
        }
    }

    assert(rnmRat(0) === 0.U)
    assert(cmtRat(0) === 0.U)
}
