package CPUSTC.predict

import chisel3._
import chisel3.util._
import CPUSTC.utils.BLevelPAdder32
import CPUSTC.config.Fetch._
import CPUSTC.config.Predict.RAS._
import CPUSTC.config.JumpOp._
import CPUSTC.config.ZE

class RASBTBMiniIO extends Bundle {
    val predType      = Input(Vec(nfch, UInt(2.W)))
    val returnOffset  = Output(UInt(32.W))
}

class RASBPUIO extends Bundle {
    val fetchCall         = Input(Bool())
    val fetchRet          = Input(Bool())
    val fetchReturnOffset = Input(UInt(30.W))

    val pdPredType     = Input(UInt(2.W))
    val pdPc           = Input(UInt(32.W))
    val pdReturnOffset = Output(UInt(32.W))
    val pdFlush        = Input(Bool())

    val cmtPredType = Input(UInt(2.W))
    val cmtPc       = Input(UInt(32.W))
    val cmtFlush    = Input(Bool())

    val fcStall = Input(Bool())
    val pdStall = Input(Bool())
}

class RASIO extends Bundle {
    val btbM = new RASBTBMiniIO
    val bpu  = new RASBPUIO
}

class RAS(useBlackBoxRam: Boolean = false) extends Module {
    require(size > 1 && isPow2(size))

    val io = IO(new RASIO)

    private val addrWidth = 30
    private val ptrWidth = log2Ceil(size)
    private val tagWidth = 3

    // Recovery state only flows from older to younger: C -> PD -> F. A C
    // flush updates PD and F together, so they can retain at most one common
    // older C generation. C and PD therefore need two versions each, while F
    // needs one. Static ownership removes cross-stage write arbitration.
    private val cmtTag0 = 1
    private val pdTagBase = 4
    private val fetchTagValue = 6

    class State extends Bundle {
        val ptr = UInt(ptrWidth.W)
        val tags = Vec(size, UInt(tagWidth.W))
    }

    private def incPtr(ptr: UInt): UInt = (ptr + 1.U)(ptrWidth - 1, 0)
    private def decPtr(ptr: UInt): UInt = (ptr - 1.U)(ptrWidth - 1, 0)

    val fetch = RegInit(0.U.asTypeOf(new State))
    val predecode = RegInit(0.U.asTypeOf(new State))
    val commit = RegInit(0.U.asTypeOf(new State))

    // Commit recovery supersedes predecode recovery, which supersedes the
    // speculative fetch update. Hidden updates do not write their version
    // pools.
    val commitCall = io.bpu.cmtPredType === CALL
    val commitRet = io.bpu.cmtPredType === RET
    val predecodeCall =
        !io.bpu.cmtFlush && !io.bpu.pdStall && io.bpu.pdPredType === CALL
    val predecodeRet =
        !io.bpu.cmtFlush && !io.bpu.pdStall && io.bpu.pdPredType === RET
    val fetchCall =
        !io.bpu.cmtFlush && !io.bpu.pdFlush &&
            !io.bpu.fcStall && io.bpu.fetchCall
    val fetchRet =
        !io.bpu.cmtFlush && !io.bpu.pdFlush &&
            !io.bpu.fcStall && io.bpu.fetchRet

    val commitSlot = commit.ptr
    val predecodeSlot = predecode.ptr
    val fetchSlot = fetch.ptr

    // A commit push replaces C's old value. Final PD and F can only retain the
    // same older C generation at the written slot, so select the other of two
    // C versions.
    val cmtProtectedByPd = Mux(
        io.bpu.cmtFlush,
        0.U,
        Mux(
            predecodeCall && predecodeSlot === commitSlot,
            0.U,
            predecode.tags(commitSlot)
        )
    )
    val cmtProtectedByFetch = Mux(
        io.bpu.cmtFlush,
        0.U,
        Mux(
            io.bpu.pdFlush,
            cmtProtectedByPd,
            Mux(
                fetchCall && fetchSlot === commitSlot,
                0.U,
                fetch.tags(commitSlot)
            )
        )
    )
    def isCmtTag(tag: UInt): Bool = !tag(2) && tag.orR

    val cmtProtectedTag = Mux(
        isCmtTag(cmtProtectedByPd),
        cmtProtectedByPd,
        cmtProtectedByFetch
    )
    val commitGrantIndex = (cmtProtectedTag === cmtTag0.U).asUInt
    val commitGrantTag = Cat(0.U(1.W), commitGrantIndex, 1.U(1.W))

    // A predecode push replaces PD's old value. Only final F can retain one
    // older PD generation, so select the other of two PD versions.
    val pdProtectedByFetch = Mux(
        io.bpu.pdFlush,
        0.U,
        Mux(
            fetchCall && fetchSlot === predecodeSlot,
            0.U,
            fetch.tags(predecodeSlot)
        )
    )
    val predecodeGrantIndex =
        (pdProtectedByFetch === pdTagBase.U).asUInt
    val predecodeGrantTag =
        (predecodeGrantIndex + pdTagBase.U(tagWidth.W))(tagWidth - 1, 0)

    val commitOwnNext = WireDefault(commit)
    when(commitCall) {
        commitOwnNext.ptr := incPtr(commit.ptr)
        for (i <- 0 until size) {
            when(commitSlot === i.U) {
                commitOwnNext.tags(i) := commitGrantTag
            }
        }
    }.elsewhen(commitRet) {
        commitOwnNext.ptr := decPtr(commit.ptr)
    }

    val predecodeOwnNext = WireDefault(predecode)
    when(predecodeCall) {
        predecodeOwnNext.ptr := incPtr(predecode.ptr)
        for (i <- 0 until size) {
            when(predecodeSlot === i.U) {
                predecodeOwnNext.tags(i) := predecodeGrantTag
            }
        }
    }.elsewhen(predecodeRet) {
        predecodeOwnNext.ptr := decPtr(predecode.ptr)
    }

    val fetchOwnNext = WireDefault(fetch)
    when(fetchCall) {
        fetchOwnNext.ptr := incPtr(fetch.ptr)
        for (i <- 0 until size) {
            when(fetchSlot === i.U) {
                fetchOwnNext.tags(i) := fetchTagValue.U
            }
        }
    }.elsewhen(fetchRet) {
        fetchOwnNext.ptr := decPtr(fetch.ptr)
    }

    val commitNext = commitOwnNext
    val predecodeNext = Mux(io.bpu.cmtFlush, commitNext, predecodeOwnNext)
    val fetchNext = Mux(
        io.bpu.cmtFlush,
        commitNext,
        Mux(io.bpu.pdFlush, predecodeNext, fetchOwnNext)
    )

    commit := commitNext
    predecode := predecodeNext
    fetch := fetchNext

    val cmtReturnOffset =
        BLevelPAdder32(io.bpu.cmtPc, 4.U, 0.U).io.res(31, 2)
    val predecodeReturnOffset = io.bpu.pdPc(31, 2)
    val fetchReturnOffset = io.bpu.fetchReturnOffset

    // These version pools are only 8/16 entries deep. Asynchronous distributed
    // RAM keeps the current top directly available and removes the BTB response
    // from a synchronous BRAM address cone without changing BPU latency.
    val cmtMem = Module(new RasAsyncRam(addrWidth, size * 2, useBlackBoxRam))
    val pdMem = Module(new RasAsyncRam(addrWidth, size * 2, useBlackBoxRam))
    val fetchMem = Module(new RasAsyncRam(addrWidth, size, useBlackBoxRam))

    cmtMem.io.wen := commitCall
    cmtMem.io.waddr := Cat(commitGrantIndex, commitSlot)
    cmtMem.io.wdata := cmtReturnOffset
    pdMem.io.wen := predecodeCall
    pdMem.io.waddr := Cat(predecodeGrantIndex, predecodeSlot)
    pdMem.io.wdata := predecodeReturnOffset
    fetchMem.io.wen := fetchCall
    fetchMem.io.waddr := fetchSlot
    fetchMem.io.wdata := fetchReturnOffset

    val fetchTopSlot = decPtr(fetch.ptr)
    val fetchTopTag = fetch.tags(fetchTopSlot)
    cmtMem.io.raddr := Cat(fetchTopTag(1), fetchTopSlot)
    pdMem.io.raddr := Cat(fetchTopTag(0), fetchTopSlot)
    fetchMem.io.raddr := fetchTopSlot

    // Only the state selected by the recovery priority can expose a value
    // written this cycle as next cycle's top. The version allocator guarantees
    // that hidden writes cannot alias a tag retained by a younger state.
    val commitBypass = io.bpu.cmtFlush && commitCall
    val predecodeBypass = !io.bpu.cmtFlush && io.bpu.pdFlush && predecodeCall
    val fetchBypass = fetchCall
    val fetchBypassHit = commitBypass || predecodeBypass || fetchBypass
    val fetchBypassData = Mux(
        io.bpu.cmtFlush,
        cmtReturnOffset,
        Mux(io.bpu.pdFlush, predecodeReturnOffset, fetchReturnOffset)
    )
    val fetchBypassHitReg = RegNext(fetchBypassHit, false.B)
    val fetchBypassDataReg = Reg(UInt(addrWidth.W))
    fetchBypassDataReg := fetchBypassData

    def selectPool(tag: UInt, cmtData: UInt, pdData: UInt, fData: UInt): UInt =
        Mux(
            !tag.orR,
            0.U,
            Mux(!tag(2), cmtData, Mux(!tag(1), pdData, fData))
        )

    val fetchRamTop = selectPool(
        fetchTopTag,
        cmtMem.io.rdata,
        pdMem.io.rdata,
        fetchMem.io.rdata
    )
    val fetchTop = Mux(
        fetchBypassHitReg,
        fetchBypassDataReg,
        fetchRamTop
    )
    io.btbM.returnOffset := ZE(fetchTop) << 2

    // Preserve the standalone one-cycle PD output with mirrored read RAMs.
    // The full core has no consumer for this legacy output, so synthesis
    // removes all three mirrors and their write logic.
    val pdCmtMem = Module(new BpuSdpRam(addrWidth, size * 2, useBlackBoxRam))
    val pdPdMem = Module(new BpuSdpRam(addrWidth, size * 2, useBlackBoxRam))
    val pdFetchMem = Module(new BpuSdpRam(addrWidth, size, useBlackBoxRam))

    pdCmtMem.io.wen := commitCall
    pdCmtMem.io.waddr := Cat(commitGrantIndex, commitSlot)
    pdCmtMem.io.wdata := cmtReturnOffset
    pdPdMem.io.wen := predecodeCall
    pdPdMem.io.waddr := Cat(predecodeGrantIndex, predecodeSlot)
    pdPdMem.io.wdata := predecodeReturnOffset
    pdFetchMem.io.wen := fetchCall
    pdFetchMem.io.waddr := fetchSlot
    pdFetchMem.io.wdata := fetchReturnOffset

    val predecodeTopSlot = decPtr(predecode.ptr)
    val predecodeTopTag = predecode.tags(predecodeTopSlot)
    val predecodeReadTag = RegInit(0.U(tagWidth.W))
    predecodeReadTag := predecodeTopTag
    pdCmtMem.io.ren := true.B
    pdCmtMem.io.raddr := Cat(predecodeTopTag(1), predecodeTopSlot)
    pdPdMem.io.ren := true.B
    pdPdMem.io.raddr := Cat(predecodeTopTag(0), predecodeTopSlot)
    pdFetchMem.io.ren := true.B
    pdFetchMem.io.raddr := predecodeTopSlot

    val predecodeTop = selectPool(
        predecodeReadTag,
        pdCmtMem.io.rdata,
        pdPdMem.io.rdata,
        pdFetchMem.io.rdata
    )
    io.bpu.pdReturnOffset := ZE(predecodeTop) << 2

    assert(
        !commitCall || !(
            isCmtTag(cmtProtectedByPd) &&
                isCmtTag(cmtProtectedByFetch) &&
                cmtProtectedByPd =/= cmtProtectedByFetch
        )
    )
    assert(
        !commitCall || (
            cmtProtectedByPd =/= commitGrantTag &&
                cmtProtectedByFetch =/= commitGrantTag
        )
    )
    assert(!predecodeCall || pdProtectedByFetch =/= predecodeGrantTag)
}
