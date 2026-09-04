package CPUSTC.backend.rename

import chisel3._
import chisel3.util._

import CPUSTC.config.RegisterFile._
import CPUSTC.config.Decode._
import CPUSTC.config.RenameConfig._
import CPUSTC.config.WritebackConfig._
import CPUSTC.config.PregMaskOr

class BusyTableReq extends Bundle {
    val psrc1 = UInt(wpreg.W)
    val psrc2 = UInt(wpreg.W)

    val psrc1Valid = Bool()
    val psrc2Valid = Bool()
}

class BusyTableResp extends Bundle {
    val psrc1Ready = Bool()
    val psrc2Ready = Bool()
}

class BusyTableIO extends Bundle {
    val flush      = Input(Bool())
    val allocPregs = Input(Vec(ndcd, Valid(UInt(wpreg.W))))
    val wakeups    = Input(Vec(nwkp, Valid(UInt(wpreg.W))))
    val earlyWakeups = Input(Vec(nFastIntWb, Valid(UInt(wpreg.W))))

    val reqs  = Input(Vec(ndcd, new BusyTableReq))
    val resps = Output(Vec(ndcd, new BusyTableResp))
}

class BusyTable extends Module {
    val io = IO(new BusyTableIO)

    private val p0Mask = 1.U(npreg.W)

    val busyTable = RegInit(0.U(npreg.W))

    val allocMask = PregMaskOr(io.allocPregs, npreg)
    val wakeupMask = PregMaskOr(io.wakeups, npreg)
    val earlyWakeupMask = PregMaskOr(io.earlyWakeups, npreg)

    val busyAfterWakeup =
        busyTable & (~(wakeupMask | earlyWakeupMask)).asUInt
    val busyNext = (busyAfterWakeup | allocMask) & (~p0Mask).asUInt

    // A full architectural flush restores the committed RAT. Every physical
    // register reachable from that RAT has completed, so clearing BusyTable is
    // exact. Branch-only recovery does not drive this input and therefore keeps
    // the busy state of older speculative instructions intact. This local
    // priority also prevents flush from being absorbed into the wide allocation
    // mask cone.
    when(io.flush) {
        busyTable := 0.U
    }.otherwise {
        busyTable := busyNext
    }

    def readReady(psrc: UInt, valid: Bool): Bool = {
        // Reads describe the registered state at the start of the cycle. New
        // allocations become visible after the edge. Registered early Int and
        // delayed Load wakeups are bypassed here for an instruction currently
        // in the Rename output register; consumers already at Dispatch are
        // covered by the IQ-local same-cycle wakeup paths.
        val earlyWakeupHit = io.earlyWakeups.map { wakeup =>
            wakeup.valid && wakeup.bits === psrc
        }.reduce(_ || _)

        // Writeback guarantees that the Load ports occupy the suffix and that
        // their Rename events come from the registered PRF-write relay. Do not
        // include the raw integer ports here: that would reconnect execution
        // response logic to the Rename read path.
        val delayedLoadWakeupHit = io.wakeups
            .slice(nIntWb, nDataWb)
            .map { wakeup =>
                wakeup.valid && wakeup.bits === psrc
            }
            .reduce(_ || _)

        !valid || psrc === 0.U || !busyTable(psrc) ||
            earlyWakeupHit || delayedLoadWakeupHit
    }

    for (i <- 0 until ndcd) {
        io.resps(i).psrc1Ready := readReady(io.reqs(i).psrc1, io.reqs(i).psrc1Valid)
        io.resps(i).psrc2Ready := readReady(io.reqs(i).psrc2, io.reqs(i).psrc2Valid)
    }

    for (i <- 0 until nFastIntWb) {
        when(io.earlyWakeups(i).valid) {
            assert(io.earlyWakeups(i).bits =/= 0.U)
        }
    }

    val flushedLastCycle = RegNext(io.flush, false.B)
    when(flushedLastCycle) {
        assert(busyTable === 0.U)
    }
}
