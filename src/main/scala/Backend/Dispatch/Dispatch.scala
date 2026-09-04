package CPUSTC.backend.dispatch

import chisel3._
import chisel3.util._

import CPUSTC.config.Decode._
import CPUSTC.config.Issue._
import CPUSTC.config.IssueQueue._
import CPUSTC.config.{ShiftAdd1, Transpose}
import CPUSTC.backend.rename.RenameOut
import CPUSTC.backend.rob.{RobEntry, RobPtr}

class Dispatch extends Module {
    val io = IO(new DispatchIO)

    val prefixValid = Wire(Vec(ndcd, Bool()))
    for (i <- 0 until ndcd) {
        prefixValid(i) := (if (i == 0) true.B else prefixValid(i - 1) && io.in(i - 1).valid)
    }

    val inValid = VecInit((0 until ndcd).map { i =>
        io.in(i).valid && prefixValid(i)
    })

    val prefixOk = (0 until ndcd).map { i =>
        !io.in(i).valid || prefixValid(i)
    }.reduce(_ && _)

    val hasValid = inValid.asUInt.orR

    val needLsq = VecInit((0 until ndcd).map(i =>
        inValid(i) && (io.in(i).bits.mem.isLoad || io.in(i).bits.mem.isStore)
    )).asUInt.orR

    val isInt = VecInit((0 until ndcd).map(i =>
        inValid(i) &&
        !io.in(i).bits.ctrl.exceptionValid &&
        io.in(i).bits.ctrl.iqType === IQT_INT
    ))

    val isMem = VecInit((0 until ndcd).map(i =>
        inValid(i) &&
        !io.in(i).bits.ctrl.exceptionValid &&
        io.in(i).bits.ctrl.iqType === IQT_MEM
    ))

    val needIntIq = isInt.asUInt.orR
    val needMemIq = isMem.asUInt.orR

    val robCanAccept = io.rob.canAccept
    val lsqCanAccept = !needLsq || io.lsq.canAccept

    val iqCanAccept =
        (!needIntIq || io.intIq.canAccept) &&
        (!needMemIq || io.memIq.canAccept)

    val structuralCanDispatch =
        hasValid && prefixOk && robCanAccept && lsqCanAccept && iqCanAccept
    val canDispatch =
        structuralCanDispatch && !io.flush && !io.branchMispredict

    private def makeRobEntry(in: RenameOut): RobEntry = {
        val e = WireDefault(0.U.asTypeOf(new RobEntry))

        e.valid    := false.B
        e.complete := in.ctrl.exceptionValid
        e.ptrHigh  := false.B
        e.epoch    := 0.U
        e.exceptionValid := in.ctrl.exceptionValid
        e.exceptionCause := in.ctrl.exceptionCause
        e.exceptionBadvValid := in.ctrl.exceptionBadvValid
        e.exceptionBadv      := in.ctrl.exceptionBadv

        e.pc        := in.meta.pc
        e.instr     := in.meta.instr
        e.ftqPtr    := in.meta.ftqPtr
        e.ftqOffset := in.meta.ftqOffset
        e.ftqLast   := in.meta.ftqLast

        e.uop    := in.ctrl.uop
        e.fuType := in.ctrl.fuType

        e.ldest      := in.reg.ldest
        e.pdest      := in.reg.pdest
        e.pprd       := in.reg.pprd
        e.ldestValid := in.reg.ldestValid
        e.rfWen      := in.reg.rfWen

        e.isLoad  := in.mem.isLoad
        e.isStore := in.mem.isStore

        e.isBr   := in.br.isBr
        e.isBl   := in.br.isBl
        e.isJirl := in.br.isJirl

        e
    }

    private def makeDispatchUop(
        in: RenameOut,
        robPtr: RobPtr,
        lsqResp: LsqAllocResp
    ): DispatchUop = {
        val u = Wire(new DispatchUop)
        u := 0.U.asTypeOf(new DispatchUop)

        u.meta := in.meta
        u.ctrl := in.ctrl
        u.mem  := in.mem
        u.br   := in.br
        u.reg  := in.reg
        u.spec := in.spec

        u.robPtr    := robPtr
        u.ldqIdx    := lsqResp.ldqIdx
        u.stqIdx    := lsqResp.stqIdx
        u.stDepMask := lsqResp.stDepMask
        u.stOrderMask := lsqResp.stOrderMask

        u.imm := in.ctrl.imm

        when (!(in.mem.isLoad || in.mem.isStore)) {
            u.ldqIdx    := 0.U.asTypeOf(new LdqPtr)
            u.stqIdx    := 0.U.asTypeOf(new StqPtr)
            u.stDepMask := 0.U
            u.stOrderMask := 0.U
        }

        u
    }

    val fireVec = VecInit((0 until ndcd).map(i =>
        inValid(i) && canDispatch
    ))

    for (i <- 0 until ndcd) {
        io.in(i).ready := canDispatch
        io.fire(i) := fireVec(i)
    }

    when(io.branchMispredict) {
        assert(!fireVec.asUInt.orR)
    }

    for (i <- 0 until ndcd) {
        io.rob.req(i).valid := fireVec(i)
        io.rob.req(i).bits  := makeRobEntry(io.in(i).bits)
    }

    for (i <- 0 until ndcd) {
        val isMem = inValid(i) && (io.in(i).bits.mem.isLoad || io.in(i).bits.mem.isStore)

        io.lsq.req(i).valid := isMem
        io.lsq.req(i).bits.isLoad  := io.in(i).bits.mem.isLoad
        io.lsq.req(i).bits.isStore := io.in(i).bits.mem.isStore

        io.lsq.brSnapshotReqs(i).valid := fireVec(i) && io.in(i).bits.spec.brTag.valid
        io.lsq.brSnapshotReqs(i).bits  := io.in(i).bits.spec.brTag.bits
    }

    io.lsq.doAllocate := canDispatch && needLsq

    private def packToIq(
        route: Vec[Bool],
        uops: Vec[DispatchUop]
    ): (UInt, Vec[Valid[DispatchUop]]) = {
        val portMap = Wire(Vec(ndcd, UInt(ndcd.W)))

        var enqPtr = 1.U(ndcd.W)
        var countOH = 1.U((ndcd + 1).W)

        for (i <- 0 until ndcd) {
            portMap(i) := Mux(route(i), enqPtr, 0.U(ndcd.W))

            enqPtr = Mux(route(i), ShiftAdd1(enqPtr), enqPtr)

            val countNext = Cat(countOH(ndcd - 1, 0), 0.U(1.W))
            countOH = Mux(route(i), countNext, countOH)
        }

        val portMapTrans = Transpose(portMap)

        val out = Wire(Vec(ndcd, Valid(new DispatchUop)))
        for (j <- 0 until ndcd) {
            val hit = portMapTrans(j).orR

            out(j).valid := hit
            out(j).bits  := Mux(
                hit,
                Mux1H(portMapTrans(j), uops),
                0.U.asTypeOf(new DispatchUop)
            )
        }

        (countOH, out)
    }

    val dispatchUops = Wire(Vec(ndcd, new DispatchUop))

    for (i <- 0 until ndcd) {
        dispatchUops(i) := makeDispatchUop(
            io.in(i).bits,
            io.rob.resp(i).bits,
            io.lsq.resp(i).bits
        )
    }

    val (intReqCountOH, intEnq) = packToIq(isInt, dispatchUops)
    val (memReqCountOH, memEnq) = packToIq(isMem, dispatchUops)

    private def rawIntProducerMask(preg: UInt): UInt = {
        val residentMatches = VecInit((0 until intNiq).map { slot =>
            io.intResidentProducers(slot).valid &&
                io.intResidentProducers(slot).bits === preg
        }).asUInt

        val bundleMatches = VecInit((0 until ndcd).map { lane =>
            val producer = intEnq(lane)
            Mux(
                producer.bits.reg.rfWen &&
                    producer.bits.reg.pdest =/= 0.U &&
                    producer.bits.reg.pdest === preg,
                io.intAllocOH(lane),
                0.U(intNiq.W)
            )
        }).reduce(_ | _)

        residentMatches | bundleMatches
    }

    io.intIq.reqCountOH := intReqCountOH
    io.memIq.reqCountOH := memReqCountOH

    for (i <- 0 until ndcd) {
        val memSrc1NeedsProducer =
            memEnq(i).valid &&
            memEnq(i).bits.reg.lsrc1Valid &&
            memEnq(i).bits.reg.psrc1 =/= 0.U &&
            !memEnq(i).bits.reg.psrc1Ready
        val memSrc2NeedsProducer =
            memEnq(i).valid &&
            memEnq(i).bits.reg.lsrc2Valid &&
            memEnq(i).bits.reg.psrc2 =/= 0.U &&
            !memEnq(i).bits.reg.psrc2Ready

        val intSrc1RawProducerOH =
            rawIntProducerMask(intEnq(i).bits.reg.psrc1)
        val intSrc2RawProducerOH =
            rawIntProducerMask(intEnq(i).bits.reg.psrc2)
        val memSrc1RawProducerOH =
            rawIntProducerMask(memEnq(i).bits.reg.psrc1)
        val memSrc2RawProducerOH =
            rawIntProducerMask(memEnq(i).bits.reg.psrc2)

        // IntIQ factors the ready/source-valid gate at its local state write.
        // Keep the wide resident/bundle owner compare independent of Rename.
        io.intIq.src1IntProducerOH(i) := intSrc1RawProducerOH
        io.intIq.src2IntProducerOH(i) := intSrc2RawProducerOH

        // MemIQ still consumes the legacy, fully-qualified owner contract.
        io.memIq.src1IntProducerOH(i) := Mux(
            memSrc1NeedsProducer,
            memSrc1RawProducerOH,
            0.U
        )
        io.memIq.src2IntProducerOH(i) := Mux(
            memSrc2NeedsProducer,
            memSrc2RawProducerOH,
            0.U
        )

        io.intIq.enq(i).valid := canDispatch && intEnq(i).valid
        io.intIq.enq(i).bits  := intEnq(i).bits

        io.memIq.enq(i).valid := canDispatch && memEnq(i).valid
        io.memIq.enq(i).bits  := memEnq(i).bits

        assert(PopCount(io.intIq.src1IntProducerOH(i)) <= 1.U)
        assert(PopCount(io.intIq.src2IntProducerOH(i)) <= 1.U)
        assert(PopCount(io.memIq.src1IntProducerOH(i)) <= 1.U)
        assert(PopCount(io.memIq.src2IntProducerOH(i)) <= 1.U)
    }
}
