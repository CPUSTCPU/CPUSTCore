package CPUSTC.memory.external

import chisel3._
import chisel3.util._
import CPUSTC.memory._
import CPUSTC.memory.backend._
import CPUSTC.memory.frontend._

class AXIIO extends Bundle {
    val araddr = Output(UInt(32.W))
    val arburst = Output(UInt(2.W))
    val arid = Output(UInt(AXIConfig.IdBits.W))
    val arlen = Output(UInt(8.W))
    val arready = Input(Bool())
    val arsize = Output(UInt(3.W))
    val arvalid = Output(Bool())

    val awaddr = Output(UInt(32.W))
    val awburst = Output(UInt(2.W))
    val awid = Output(UInt(AXIConfig.IdBits.W))
    val awlen = Output(UInt(8.W))
    val awready = Input(Bool())
    val awsize = Output(UInt(3.W))
    val awvalid = Output(Bool())

    val bid = Input(UInt(AXIConfig.IdBits.W))
    val bready = Output(Bool())
    val bresp = Input(UInt(2.W))
    val bvalid = Input(Bool())

    val rdata = Input(UInt(AXIConfig.DataBits.W))
    val rid = Input(UInt(AXIConfig.IdBits.W))
    val rlast = Input(Bool())
    val rready = Output(Bool())
    val rresp = Input(UInt(2.W))
    val rvalid = Input(Bool())

    val wdata = Output(UInt(AXIConfig.DataBits.W))
    val wlast = Output(Bool())
    val wready = Input(Bool())
    val wstrb = Output(UInt(AXIConfig.DataBytes.W))
    val wvalid = Output(Bool())
}

class UncacheLoadResponse extends Bundle {
    val inst = new BackendInst
    val data = UInt(32.W)
}

class UncacheMemoryIO extends Bundle {
    val loadReq = Decoupled(new BackendInst)
    val storeReq = Decoupled(new BackendInst)
    val loadResp = Flipped(Decoupled(new UncacheLoadResponse))
    val full = Input(Bool())
    val writeDone = Input(Bool())
}

class UncacheBridgeRequest extends Bundle {
    val isWrite = Bool()
    val inst = new BackendInst
}

class ExternalAXIArbiterIO extends Bundle {
    val l2 = Flipped(new MshrMemoryIO)
    val icache = Flipped(new IcacheMissBus)
    val uncache = Flipped(new UncacheMemoryIO)
    val uncacheWriteSnoopReq = Decoupled(new L2UncacheWriteSnoopRequest)
    val uncacheWriteSnoopResp = Flipped(Decoupled(new L2UncacheWriteSnoopResponse))
    val maintenanceWrite = Flipped(Decoupled(new WritebackRequest))
    val maintenanceWriteDone = Output(Bool())
    val flush = Input(Bool())
    val uncacheReadStart = Output(Bool())
    val uncacheWriteStart = Output(Bool())
    val uncacheIdle = Output(Bool())
    val axi = new AXIIO
}

class ExternalAXIArbiter extends Module {
    val io = IO(new ExternalAXIArbiterIO)

    private val lineBits = DcacheConfig.DcacheLineBits
    private val lineBeats = lineBits / AXIConfig.DataBits
    private val icacheBeats = IcacheConfig.IcacheFetchBits / AXIConfig.DataBits
    private val beatWidth = log2Ceil(lineBeats)
    private val sourceIcache = 0.U(AXIConfig.IdBits.W)
    private val sourceL2 = 1.U(AXIConfig.IdBits.W)
    private val sourceUncache = 2.U(AXIConfig.IdBits.W)
    private val sourceMaintenance = 3.U(AXIConfig.IdBits.W)

    require(AXIConfig.DataBits == 32)
    require(IcacheConfig.IcacheLineBits == lineBits)
    require(IcacheConfig.IcacheFetchBits % AXIConfig.DataBits == 0)
    require(lineBeats == 16, "ExternalAXIArbiter requires 16-beat cache lines")

    private def accessSize(mask: UInt): UInt = {
        Mux(mask === "b0001".U, 0.U,
            Mux(mask === "b0011".U, 1.U, 2.U))
    }

    private def extendLoadData(data: UInt, inst: BackendInst): UInt = {
        val shifted = data >> (inst.paddr(1, 0) << 3)
        val byteData = shifted(7, 0)
        val halfData = shifted(15, 0)
        Mux(inst.mask === "b0001".U,
            Cat(Fill(24, inst.signed && byteData(7)), byteData),
            Mux(inst.mask === "b0011".U,
                Cat(Fill(16, inst.signed && halfData(15)), halfData),
                shifted(31, 0)))
    }

    val uncacheBuffer = Module(new Queue(new UncacheBridgeRequest, 1, pipe = false))
    val uncacheStoreValid = io.uncache.storeReq.valid
    uncacheBuffer.io.enq.valid := io.uncache.loadReq.valid || uncacheStoreValid
    uncacheBuffer.io.enq.bits.isWrite := uncacheStoreValid
    uncacheBuffer.io.enq.bits.inst := Mux(
        uncacheStoreValid,
        io.uncache.storeReq.bits,
        io.uncache.loadReq.bits
    )
    // The LSU serializes uncached Loads behind every pending uncached Store,
    // while atomic Stores hold the LSU quiesced.  The two request channels are
    // therefore mutually exclusive, so neither ready needs the other valid in
    // its combinational cone.
    io.uncache.storeReq.ready := uncacheBuffer.io.enq.ready
    io.uncache.loadReq.ready := uncacheBuffer.io.enq.ready
    io.uncache.full := !uncacheBuffer.io.enq.ready
    uncacheBuffer.io.deq.ready := false.B

    val uncacheActive = RegInit(false.B)
    val uncacheIncoming = io.uncache.loadReq.valid || io.uncache.storeReq.valid
    val maintenanceActive = RegInit(false.B)
    val maintenanceIncoming = io.maintenanceWrite.valid
    val blockNormalTraffic =
        uncacheBuffer.io.deq.valid || uncacheActive || uncacheIncoming ||
            maintenanceActive || maintenanceIncoming

    val l2ReadRespValid = RegInit(false.B)
    val l2ReadRespAddr = Reg(UInt(32.W))
    val l2ReadRespLine = Reg(UInt(lineBits.W))
    io.l2.readResp.valid := l2ReadRespValid
    io.l2.readResp.bits.paddr := l2ReadRespAddr
    io.l2.readResp.bits.data := l2ReadRespLine
    io.l2.readResp.bits.dirty := false.B

    val icacheRespValid = RegInit(false.B)
    val icacheRespBlock = Reg(UInt(IcacheConfig.IcacheFetchBits.W))
    io.icache.resp.valid := icacheRespValid && !io.flush
    io.icache.resp.bits.refillLine := icacheRespBlock.pad(lineBits)

    val uncacheRespValid = RegInit(false.B)
    val uncacheRespData = Reg(UInt(AXIConfig.DataBits.W))
    io.uncache.loadResp.valid := uncacheRespValid
    io.uncache.loadResp.bits.inst := uncacheBuffer.io.deq.bits.inst
    io.uncache.loadResp.bits.data := extendLoadData(
        uncacheRespData,
        uncacheBuffer.io.deq.bits.inst
    )

    val l2ReadRespCanAccept = !l2ReadRespValid || io.l2.readResp.ready
    val icacheRespCanAccept = !icacheRespValid || io.icache.resp.ready
    val uncacheRespCanAccept = !uncacheRespValid || io.uncache.loadResp.ready

    // DBAR only needs the architecturally ordered uncached stream. Cached L2
    // traffic has its own completion tracking in the load/store pipeline.
    io.uncacheIdle :=
        !uncacheBuffer.io.deq.valid &&
        !uncacheActive &&
        !uncacheRespValid &&
        !maintenanceActive &&
        !maintenanceIncoming

    when(io.l2.readResp.fire) {
        l2ReadRespValid := false.B
    }
    when(io.icache.resp.fire || io.flush) {
        icacheRespValid := false.B
    }
    when(io.uncache.loadResp.fire) {
        uncacheRespValid := false.B
        when(uncacheActive) {
            uncacheBuffer.io.deq.ready := true.B
            uncacheActive := false.B
        }
    }

    val rIdle :: rAddr :: rData :: Nil = Enum(3)
    val readState = RegInit(rIdle)
    val readSource = Reg(UInt(AXIConfig.IdBits.W))
    val readAddr = Reg(UInt(32.W))
    val readLen = Reg(UInt(8.W))
    val readSize = Reg(UInt(3.W))
    val readBeat = RegInit(0.U(beatWidth.W))
    val readIcacheKilled = RegInit(false.B)

    val wIdle :: wAddr :: wData :: wResp :: Nil = Enum(4)
    val writeState = RegInit(wIdle)
    val writeSource = Reg(UInt(AXIConfig.IdBits.W))
    val writeAddr = Reg(UInt(32.W))
    val writeLine = Reg(UInt(lineBits.W))
    val writeMask = Reg(UInt(AXIConfig.DataBytes.W))
    val writeLen = Reg(UInt(8.W))
    val writeSize = Reg(UInt(3.W))
    val writeBeat = RegInit(0.U(beatWidth.W))
    val uncacheSnoopPending = RegInit(false.B)
    val uncacheSnoopReqSent = RegInit(false.B)

    val uncacheCanStart = uncacheBuffer.io.deq.valid && !uncacheActive &&
        !maintenanceActive && !maintenanceIncoming &&
        readState === rIdle && writeState === wIdle && !l2ReadRespValid
    val startUncacheRead = uncacheCanStart && !uncacheBuffer.io.deq.bits.isWrite &&
        uncacheRespCanAccept
    val startUncacheWrite = uncacheCanStart && uncacheBuffer.io.deq.bits.isWrite

    val grantIcacheRead = readState === rIdle && !blockNormalTraffic && !io.flush &&
        io.icache.req.valid && icacheRespCanAccept
    // If a store reaches its post-B snoop while L2 still owns a miss, let the
    // already-issued downstream traffic drain.  New demand/ICache traffic
    // remains blocked by uncacheActive.
    val allowL2Drain = !blockNormalTraffic || uncacheSnoopPending
    val grantL2Read = readState === rIdle && allowL2Drain && !grantIcacheRead &&
        io.l2.readReq.valid && l2ReadRespCanAccept
    val grantMaintenanceWrite =
        writeState === wIdle && readState === rIdle &&
        !uncacheBuffer.io.deq.valid && !uncacheActive && !uncacheRespValid &&
        !l2ReadRespValid && !icacheRespValid &&
        io.maintenanceWrite.valid
    val grantL2Write = writeState === wIdle && allowL2Drain &&
        !grantMaintenanceWrite && io.l2.writeReq.valid

    io.uncacheReadStart := startUncacheRead || grantIcacheRead
    io.uncacheWriteStart := startUncacheWrite
    io.uncache.writeDone := false.B
    io.uncacheWriteSnoopReq.valid :=
        uncacheSnoopPending && !uncacheSnoopReqSent
    io.uncacheWriteSnoopReq.bits.paddr :=
        uncacheBuffer.io.deq.bits.inst.paddr
    io.uncacheWriteSnoopReq.bits.data := uncacheBuffer.io.deq.bits.inst.operateData
    io.uncacheWriteSnoopReq.bits.mask := uncacheBuffer.io.deq.bits.inst.mask
    io.uncacheWriteSnoopResp.ready :=
        uncacheSnoopPending && uncacheSnoopReqSent
    io.maintenanceWrite.ready := grantMaintenanceWrite
    io.maintenanceWriteDone := false.B
    io.icache.req.ready := io.flush || grantIcacheRead
    io.l2.readReq.ready := grantL2Read
    io.l2.writeReq.ready := grantL2Write

    when(startUncacheRead) {
        val inst = uncacheBuffer.io.deq.bits.inst
        uncacheActive := true.B
        readSource := sourceUncache
        readAddr := inst.paddr
        readLen := 0.U
        readSize := accessSize(inst.mask)
        readBeat := 0.U
        readIcacheKilled := false.B
        readState := rAddr
    }.elsewhen(grantIcacheRead) {
        readSource := sourceIcache
        readAddr := io.icache.req.bits.paddr
        readLen := (icacheBeats - 1).U
        readSize := log2Ceil(AXIConfig.DataBytes).U
        readBeat := 0.U
        readIcacheKilled := false.B
        readState := rAddr
    }.elsewhen(grantL2Read) {
        readSource := sourceL2
        readAddr := io.l2.readReq.bits.paddr
        readLen := (lineBeats - 1).U
        readSize := log2Ceil(AXIConfig.DataBytes).U
        readBeat := 0.U
        readIcacheKilled := false.B
        readState := rAddr
    }

    when(io.flush && readState =/= rIdle && readSource === sourceIcache) {
        readIcacheKilled := true.B
    }

    io.axi.araddr := readAddr
    io.axi.arburst := 1.U
    io.axi.arid := readSource
    io.axi.arlen := readLen
    io.axi.arsize := readSize
    io.axi.arvalid := readState === rAddr
    io.axi.rready := readState === rData

    when(io.axi.arvalid && io.axi.arready) {
        readState := rData
    }

    when(io.axi.rvalid && io.axi.rready) {
        assert(io.axi.rid === readSource,
            "ExternalAXIArbiter: read response ID mismatch")
        assert(io.axi.rresp === 0.U,
            "ExternalAXIArbiter: read response must be OKAY")
        assert(io.axi.rlast === (readBeat === readLen),
            "ExternalAXIArbiter: RLAST position mismatch")

        when(readSource === sourceL2) {
            l2ReadRespLine := Cat(io.axi.rdata,
                l2ReadRespLine(lineBits - 1, AXIConfig.DataBits))
        }.elsewhen(readSource === sourceIcache) {
            icacheRespBlock := Cat(io.axi.rdata,
                icacheRespBlock(IcacheConfig.IcacheFetchBits - 1, AXIConfig.DataBits))
        }
        when(io.axi.rlast) {
            when(readSource === sourceL2) {
                l2ReadRespValid := true.B
                l2ReadRespAddr := readAddr
            }.elsewhen(readSource === sourceIcache) {
                when(!readIcacheKilled && !io.flush) {
                    icacheRespValid := true.B
                }
            }.otherwise {
                uncacheRespValid := true.B
                uncacheRespData := io.axi.rdata
            }
            readState := rIdle
        }.otherwise {
            readBeat := readBeat + 1.U
        }
    }

    // The L2 request is stable while backpressured. Preload its payload while
    // the write channel is idle so arbitration only controls the state change.
    val preloadL2Write = writeState === wIdle && io.l2.writeReq.valid
    when(preloadL2Write) {
        writeSource := sourceL2
        writeAddr := io.l2.writeReq.bits.paddr
        writeLine := io.l2.writeReq.bits.data
        writeMask := Fill(AXIConfig.DataBytes, 1.U(1.W))
        writeLen := (lineBeats - 1).U
        writeSize := log2Ceil(AXIConfig.DataBytes).U
        writeBeat := 0.U
    }

    when(startUncacheWrite) {
        val inst = uncacheBuffer.io.deq.bits.inst
        val byteShift = inst.paddr(1, 0) << 3
        uncacheActive := true.B
        writeSource := sourceUncache
        writeAddr := inst.paddr
        writeLine := (inst.operateData << byteShift).pad(lineBits)
        writeMask := (inst.mask << inst.paddr(1, 0))(AXIConfig.DataBytes - 1, 0)
        writeLen := 0.U
        writeSize := accessSize(inst.mask)
        writeBeat := 0.U
        uncacheSnoopPending := false.B
        uncacheSnoopReqSent := false.B
        writeState := wAddr
    }.elsewhen(grantMaintenanceWrite) {
        maintenanceActive := true.B
        writeSource := sourceMaintenance
        writeAddr := io.maintenanceWrite.bits.paddr
        writeLine := io.maintenanceWrite.bits.data
        writeMask := Fill(AXIConfig.DataBytes, 1.U(1.W))
        writeLen := (lineBeats - 1).U
        writeSize := log2Ceil(AXIConfig.DataBytes).U
        writeBeat := 0.U
        writeState := wAddr
    }.elsewhen(grantL2Write) {
        writeState := wAddr
    }

    io.axi.awaddr := writeAddr
    io.axi.awburst := 1.U
    io.axi.awid := writeSource
    io.axi.awlen := writeLen
    io.axi.awsize := writeSize
    io.axi.awvalid := writeState === wAddr

    io.axi.wdata := writeLine(AXIConfig.DataBits - 1, 0)
    io.axi.wstrb := writeMask
    io.axi.wlast := writeBeat === writeLen
    io.axi.wvalid := writeState === wData
    io.axi.bready := writeState === wResp

    io.l2.writeResp.valid := false.B
    io.l2.writeResp.bits.paddr := writeAddr

    when(io.axi.awvalid && io.axi.awready) {
        writeState := wData
    }
    when(io.axi.wvalid && io.axi.wready) {
        writeLine := writeLine >> AXIConfig.DataBits
        when(io.axi.wlast) {
            writeState := wResp
        }.otherwise {
            writeBeat := writeBeat + 1.U
        }
    }
    when(io.axi.bvalid && io.axi.bready) {
        assert(io.axi.bid === writeSource,
            "ExternalAXIArbiter: write response ID mismatch")
        assert(io.axi.bresp === 0.U,
            "ExternalAXIArbiter: write response must be OKAY")
        when(writeSource === sourceL2) {
            io.l2.writeResp.valid := true.B
            writeState := wIdle
        }.elsewhen(writeSource === sourceMaintenance) {
            io.maintenanceWriteDone := true.B
            maintenanceActive := false.B
            writeState := wIdle
        }.otherwise {
            uncacheSnoopPending := true.B
            uncacheSnoopReqSent := false.B
            writeState := wIdle
        }
    }

    when(io.uncacheWriteSnoopReq.fire) {
        uncacheSnoopReqSent := true.B
    }

    when(io.uncacheWriteSnoopResp.fire) {
        assert(io.uncacheWriteSnoopResp.bits.paddr ===
            uncacheBuffer.io.deq.bits.inst.paddr,
            "ExternalAXIArbiter: uncache write snoop response address mismatch")
        io.uncache.writeDone := true.B
        uncacheBuffer.io.deq.ready := true.B
        uncacheActive := false.B
        uncacheSnoopPending := false.B
        uncacheSnoopReqSent := false.B
    }

    when(!reset.asBool) {
        assert(!(io.uncache.loadReq.valid && io.uncache.storeReq.valid),
            "ExternalAXIArbiter: uncached Load and Store requests must be serialized")
        when(io.icache.req.fire) {
            assert(!io.icache.req.bits.cacheable,
                "ExternalAXIArbiter accepts only uncached ICache requests")
        }
        when(io.l2.readReq.fire) {
            assert(io.l2.readReq.bits.paddr(log2Ceil(lineBits / 8) - 1, 0) === 0.U,
                "ExternalAXIArbiter: L2 read must be line aligned")
        }
        when(io.l2.writeReq.fire) {
            assert(io.l2.writeReq.bits.paddr(log2Ceil(lineBits / 8) - 1, 0) === 0.U,
                "ExternalAXIArbiter: L2 write must be line aligned")
        }
        when(io.maintenanceWrite.fire) {
            assert(io.maintenanceWrite.bits.paddr(
                log2Ceil(lineBits / 8) - 1, 0) === 0.U,
                "ExternalAXIArbiter: maintenance write must be line aligned")
        }
        when(io.uncache.loadReq.fire) {
            assert(io.uncache.loadReq.bits.uncache && io.uncache.loadReq.bits.uop.isLD,
                "ExternalAXIArbiter: uncache load input must be an uncached Load")
        }
        when(io.uncache.storeReq.fire) {
            assert(io.uncache.storeReq.bits.uncache && io.uncache.storeReq.bits.uop.isSTD,
                "ExternalAXIArbiter: uncache Store input must be an uncached Store")
        }
        assert(!uncacheActive || uncacheBuffer.io.deq.valid,
            "ExternalAXIArbiter: active uncache request must retain its buffer entry")
        when(uncacheRespValid) {
            assert(uncacheActive && uncacheBuffer.io.deq.valid &&
                !uncacheBuffer.io.deq.bits.isWrite,
                "ExternalAXIArbiter: uncache load response must retain its request")
        }
        when(uncacheSnoopPending) {
            assert(uncacheActive && uncacheBuffer.io.deq.valid &&
                uncacheBuffer.io.deq.bits.isWrite,
                "ExternalAXIArbiter: uncache write snoop must retain its request")
        }
    }
}
