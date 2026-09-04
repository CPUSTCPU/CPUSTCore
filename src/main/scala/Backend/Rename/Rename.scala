package CPUSTC.backend.rename

import chisel3._
import chisel3.util._

import CPUSTC.config.Decode._
import CPUSTC.config.Commit._
import CPUSTC.config.RegisterFile._
import CPUSTC.config.RenameConfig._
import CPUSTC.config.WritebackConfig._

class Rename extends Module {
    val io = IO(new RenameIO)

    val mapTable  = Module(new MapTable)
    val busyTable = Module(new BusyTable)
    val freeList  = Module(new FreeList)

    val branchTags = Module(new BranchTagAllocator)

    busyTable.io.flush := io.flush
    branchTags.io.hardFlush := io.flush
    branchTags.io.branchUpdate := io.branchUpdate

    val willNeedBrTag = Wire(Vec(ndcd, Bool()))

    for (i <- 0 until ndcd) {
        willNeedBrTag(i) :=
            io.in(i).valid &&
            io.in(i).bits.ctrl.legal &&
            (
                io.in(i).bits.br.isBr ||
                io.in(i).bits.br.isJirl
            )

        branchTags.io.req(i) := willNeedBrTag(i)
    }

    val headValids = RegInit(VecInit(Seq.fill(ndcd)(false.B)))
    val headBits   = Reg(Vec(ndcd, new RenameOut))

    // A second packet absorbs a newly renamed batch when Dispatch first
    // backpressures. Decode readiness then depends only on registered local
    // capacity, rather than on the same-cycle ROB/LSQ/IQ ready network.
    val tailValids = RegInit(VecInit(Seq.fill(ndcd)(false.B)))
    val tailBits   = Reg(Vec(ndcd, new RenameOut))

    val headCanDrain = (0 until ndcd).map(i =>
        !headValids(i) || io.out(i).ready
    ).reduce(_ && _)

    val headOccupied = headValids.asUInt.orR
    val tailOccupied = tailValids.asUInt.orR
    val localCanAccept = !tailOccupied

    val willNeedPdest = Wire(Vec(ndcd, Bool()))

    for (i <- 0 until ndcd) {
        willNeedPdest(i) :=
            io.in(i).valid &&
            io.in(i).bits.ctrl.legal &&
            io.in(i).bits.reg.rfWen &&
            io.in(i).bits.reg.ldestValid &&
            io.in(i).bits.reg.ldest =/= 0.U
    }

    for (i <- 0 until ndcd) {
        freeList.io.allocReqs(i) := willNeedPdest(i)
    }

    val freeEnough  = freeList.io.canAllocate
    val tagEnough   = branchTags.io.canAllocate

    val branchMispredict = io.branchUpdate.valid && io.branchUpdate.bits.mispredictMask.orR

    val canAccept =
        localCanAccept &&
        freeEnough &&
        tagEnough &&
        !io.flush &&
        !branchMispredict

    val inputActive = io.in.map(_.valid).reduce(_ || _)
    io.status.outputBlocked := inputActive && !localCanAccept
    io.status.freeBlocked   := inputActive && localCanAccept && !freeEnough
    io.status.tagBlocked    := inputActive && localCanAccept && freeEnough && !tagEnough

    for (i <- 0 until ndcd) {
        io.in(i).ready := canAccept
    }

    val inFire = Wire(Vec(ndcd, Bool()))

    for (i <- 0 until ndcd) {
        inFire(i) := io.in(i).valid && io.in(i).ready
    }

    val hasFire = inFire.asUInt.orR

    val doRename = hasFire && canAccept && !branchMispredict

    freeList.io.doAllocate := doRename
    branchTags.io.doAllocate := doRename

    val brSnapshotReqs = Wire(Vec(ndcd, Valid(UInt(wBrTag.W))))

    for (i <- 0 until ndcd) {
        brSnapshotReqs(i).valid :=
            doRename && inFire(i) && branchTags.io.brTag(i).valid
        brSnapshotReqs(i).bits  := branchTags.io.brTag(i).bits
    }

    mapTable.io.brSnapshotReqs := brSnapshotReqs
    freeList.io.brSnapshotReqs := brSnapshotReqs

    freeList.io.branchUpdate := io.branchUpdate

    val needPdest = Wire(Vec(ndcd, Bool()))

    for (i <- 0 until ndcd) {
        needPdest(i) := doRename && inFire(i) && willNeedPdest(i)
    }

    for (i <- 0 until ndcd) {
        mapTable.io.readReqs(i).lsrc1 := io.in(i).bits.reg.lsrc1
        mapTable.io.readReqs(i).lsrc2 := io.in(i).bits.reg.lsrc2
        mapTable.io.readReqs(i).ldest := io.in(i).bits.reg.ldest
    }

    freeList.io.flush  := io.flush
    freeList.io.commit := io.commit

    val pdest = Wire(Vec(ndcd, UInt(wpreg.W)))

    for (i <- 0 until ndcd) {
        pdest(i) := Mux(needPdest(i), freeList.io.allocResps(i).bits, 0.U)
    }

    def bypassNewest(
        base: UInt,
        hits: Seq[Bool],
        values: Seq[UInt]
    ): UInt = {
        if (hits.isEmpty) {
            base
        } else {
            MuxCase(base, hits.zip(values).reverse.map {
                case (hit, value) => hit -> value
            })
        }
    }

    val finalPsrc1 = Wire(Vec(ndcd, UInt(wpreg.W)))
    val finalPsrc2 = Wire(Vec(ndcd, UInt(wpreg.W)))

    for (i <- 0 until ndcd) {
        val older = 0 until i

        val hit1 = older.map { j =>
            needPdest(j) &&
            io.in(j).bits.reg.ldest === io.in(i).bits.reg.lsrc1
        }

        val hit2 = older.map { j =>
            needPdest(j) &&
            io.in(j).bits.reg.ldest === io.in(i).bits.reg.lsrc2
        }

        finalPsrc1(i) := bypassNewest(
            mapTable.io.readResps(i).psrc1,
            hit1,
            older.map(j => pdest(j))
        )

        finalPsrc2(i) := bypassNewest(
            mapTable.io.readResps(i).psrc2,
            hit2,
            older.map(j => pdest(j))
        )
    }

    val finalPprd = Wire(Vec(ndcd, UInt(wpreg.W)))

    for (i <- 0 until ndcd) {
        val older = 0 until i

        val hitDst = older.map { j =>
            needPdest(j) &&
            io.in(j).bits.reg.ldest === io.in(i).bits.reg.ldest
        }

        finalPprd(i) := bypassNewest(
            mapTable.io.readResps(i).pprd,
            hitDst,
            older.map(j => pdest(j))
        )
    }

    for (i <- 0 until ndcd) {
        busyTable.io.reqs(i).psrc1 := Mux(
            headValids(i),
            headBits(i).reg.psrc1,
            0.U
        )
        busyTable.io.reqs(i).psrc2 := Mux(
            headValids(i),
            headBits(i).reg.psrc2,
            0.U
        )

        busyTable.io.reqs(i).psrc1Valid :=
            headValids(i) && headBits(i).reg.lsrc1Valid
        busyTable.io.reqs(i).psrc2Valid :=
            headValids(i) && headBits(i).reg.lsrc2Valid

        busyTable.io.allocPregs(i).valid := needPdest(i)
        busyTable.io.allocPregs(i).bits  := pdest(i)
    }

    for (i <- 0 until nwkp) {
        busyTable.io.wakeups(i).valid := io.wakeup(i).valid
        busyTable.io.wakeups(i).bits  := io.wakeup(i).bits.pdest
    }

    for (i <- 0 until nFastIntWb) {
        busyTable.io.earlyWakeups(i).valid := io.earlyWakeup(i).valid
        busyTable.io.earlyWakeups(i).bits  := io.earlyWakeup(i).bits.pdest
    }

    mapTable.io.flush := io.flush
    mapTable.io.branchUpdate := io.branchUpdate

    for (i <- 0 until ncmt) {
        mapTable.io.cmtReqs(i).valid :=
            io.commit(i).valid &&
            io.commit(i).bits.rfWen &&
            io.commit(i).bits.ldestValid &&
            io.commit(i).bits.ldest =/= 0.U

        mapTable.io.cmtReqs(i).bits.ldest := io.commit(i).bits.ldest
        mapTable.io.cmtReqs(i).bits.pdest := io.commit(i).bits.pdest
    }

    for (i <- 0 until ndcd) {
        mapTable.io.rnmReqs(i).valid := needPdest(i)
        mapTable.io.rnmReqs(i).bits.ldest := io.in(i).bits.reg.ldest
        mapTable.io.rnmReqs(i).bits.pdest := pdest(i)
    }

    val renamed = Wire(Vec(ndcd, new RenameOut))

    for (i <- 0 until ndcd) {
        renamed(i) := 0.U.asTypeOf(new RenameOut)

        renamed(i).meta := io.in(i).bits.meta
        renamed(i).ctrl := io.in(i).bits.ctrl
        renamed(i).mem  := io.in(i).bits.mem
        renamed(i).br   := io.in(i).bits.br

        renamed(i).reg.lsrc1 := io.in(i).bits.reg.lsrc1
        renamed(i).reg.lsrc2 := io.in(i).bits.reg.lsrc2
        renamed(i).reg.ldest := io.in(i).bits.reg.ldest

        renamed(i).reg.psrc1 := finalPsrc1(i)
        renamed(i).reg.psrc2 := finalPsrc2(i)
        renamed(i).reg.pdest := pdest(i)
        renamed(i).reg.pprd  := finalPprd(i)

        renamed(i).reg.lsrc1Valid := io.in(i).bits.reg.lsrc1Valid
        renamed(i).reg.lsrc2Valid := io.in(i).bits.reg.lsrc2Valid
        renamed(i).reg.ldestValid := io.in(i).bits.reg.ldestValid
        renamed(i).reg.rfWen      := io.in(i).bits.reg.rfWen

        renamed(i).reg.psrc1Ready := false.B
        renamed(i).reg.psrc2Ready := false.B

        renamed(i).spec.brMask := branchTags.io.brMask(i)
        renamed(i).spec.brTag  := branchTags.io.brTag(i)
    }

    val resolveMask = Mux(io.branchUpdate.valid, io.branchUpdate.bits.resolveMask, 0.U(maxBrCount.W))

    val resolvedTailBits = WireInit(tailBits)
    for (i <- 0 until ndcd) {
        resolvedTailBits(i).spec.brMask :=
            tailBits(i).spec.brMask & (~resolveMask).asUInt
    }

    val headDequeue =
        headOccupied && headCanDrain && !io.flush && !branchMispredict

    when(io.flush || branchMispredict) {
        headValids := VecInit(Seq.fill(ndcd)(false.B))
        tailValids := VecInit(Seq.fill(ndcd)(false.B))
    }.elsewhen(headDequeue) {
        when(tailOccupied) {
            // A full buffer cannot also accept from Decode because readiness
            // is intentionally independent of the combinational dequeue.
            headValids := tailValids
            headBits := resolvedTailBits
            tailValids := VecInit(Seq.fill(ndcd)(false.B))
        }.elsewhen(hasFire) {
            headValids := inFire
            headBits := renamed
        }.otherwise {
            headValids := VecInit(Seq.fill(ndcd)(false.B))
        }
    }.elsewhen(hasFire) {
        when(headOccupied) {
            tailValids := inFire
            tailBits := renamed

            for (i <- 0 until ndcd) {
                headBits(i).spec.brMask :=
                    headBits(i).spec.brMask & (~resolveMask).asUInt
            }
        }.otherwise {
            headValids := inFire
            headBits := renamed
        }
    }.elsewhen(io.branchUpdate.valid) {
        for (i <- 0 until ndcd) {
            headBits(i).spec.brMask :=
                headBits(i).spec.brMask & (~resolveMask).asUInt
            tailBits(i).spec.brMask :=
                tailBits(i).spec.brMask & (~resolveMask).asUInt
        }
    }

    for (i <- 0 until ndcd) {
        // Dispatch has its own cycle-aligned flush copy and suppresses every
        // allocation/fire on a recovery cycle.  Keep the registered packet
        // visible here so the rare flush does not enter the normal
        // Rename-to-Dispatch validity and allocation cone.
        io.out(i).valid := headValids(i)

        io.out(i).bits := headBits(i)

        io.out(i).bits.reg.psrc1Ready :=
            busyTable.io.resps(i).psrc1Ready
        io.out(i).bits.reg.psrc2Ready :=
            busyTable.io.resps(i).psrc2Ready

        io.out(i).bits.spec.brMask :=
            headBits(i).spec.brMask & (~resolveMask).asUInt
    }

    // The output bundle is older than allocations accepted into the rename
    // register this cycle. A newly allocated preg therefore cannot still be a
    // live source of the output bundle.
    for (outLane <- 0 until ndcd; allocLane <- 0 until ndcd) {
        when(io.out(outLane).valid && needPdest(allocLane)) {
            when(headBits(outLane).reg.lsrc1Valid && headBits(outLane).reg.psrc1 =/= 0.U) {
                assert(headBits(outLane).reg.psrc1 =/= pdest(allocLane))
            }
            when(headBits(outLane).reg.lsrc2Valid && headBits(outLane).reg.psrc2 =/= 0.U) {
                assert(headBits(outLane).reg.psrc2 =/= pdest(allocLane))
            }
        }
    }

    when(branchMispredict) {
        assert(!doRename)
        assert(!freeList.io.doAllocate)
        assert(!branchTags.io.doAllocate)
    }

    when(io.flush) {
        assert(!hasFire)
        assert(!doRename)
        assert(!freeList.io.doAllocate)
        assert(!branchTags.io.doAllocate)
    }

    for (i <- 0 until ndcd) {
        when(inFire(i)) {
            assert(
                renamed(i).spec.brTag.valid === willNeedBrTag(i)
            )

            when(renamed(i).spec.brTag.valid) {
                val tagOH = UIntToOH(
                    renamed(i).spec.brTag.bits,
                    maxBrCount
                )

                assert((renamed(i).spec.brMask & tagOH) === 0.U)
            }
        }

        when(io.out(i).valid && io.branchUpdate.valid) {
            assert(
                (io.out(i).bits.spec.brMask & resolveMask) === 0.U
            )
        }
    }

    for (i <- 1 until ndcd) {
        when(headValids(i)) {
            assert(headValids(i - 1))
        }
        when(tailValids(i)) {
            assert(tailValids(i - 1))
        }
    }

    assert(!tailOccupied || headOccupied)
    when(headDequeue && tailOccupied) {
        assert(!hasFire)
    }
}
