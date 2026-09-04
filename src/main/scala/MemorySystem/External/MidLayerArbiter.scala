package CPUSTC.memory.external

import chisel3._
import chisel3.util._
import CPUSTC.memory._
import CPUSTC.memory.backend._
import CPUSTC.memory.frontend._
import _root_.circt.stage.ChiselStage

object L2RequestSource {
    val ICache = 0.U(2.W)
    val DCacheRead = 1.U(2.W)
    val DCacheWrite = 2.U(2.W)
}

class L2Request extends Bundle {
    val source = UInt(2.W)
    val write = Bool()
    val paddr = UInt(32.W)
    val data = UInt(DcacheConfig.DcacheLineBits.W)
}

class L2Response extends Bundle {
    val source = UInt(2.W)
    val write = Bool()
    val paddr = UInt(32.W)
    val data = UInt(DcacheConfig.DcacheLineBits.W)
    val dirty = Bool()
}

class L2MissStatus extends Bundle {
    val accepting = Bool()
    val active = Bool()
    val write = Bool()
    val setMask = UInt(32.W)
    val setValue = UInt(32.W)
}

class L2ClientIO extends Bundle {
    val req = Decoupled(new L2Request)
    val resp = Flipped(Decoupled(new L2Response))
    val miss = Flipped(new L2MissStatus)
}

class MidLayerArbiterIO extends Bundle {
    val icache = Flipped(new IcacheMissBus)
    val dcache = Flipped(new MshrMemoryIO)
    val l2 = new L2ClientIO
    val flush = Input(Bool())
}

class MidLayerArbiter extends Module {
    val io = IO(new MidLayerArbiterIO)

    require(IcacheConfig.IcacheLineBits == DcacheConfig.DcacheLineBits)

    val icacheOutstanding = RegInit(false.B)
    val icacheKilled = RegInit(false.B)

    val grantNone :: grantWrite :: grantICache :: grantDCacheRead :: Nil = Enum(4)
    val heldGrantValid = RegInit(false.B)
    val heldGrant = RegInit(grantNone)

    private def readAllowed(addr: UInt): Bool =
        !io.l2.miss.active || (!io.l2.miss.write &&
            (addr & io.l2.miss.setMask) =/= io.l2.miss.setValue)

    // Flush is sampled into icacheKilled below. A request already presented in
    // the flush cycle may still enter L2; its response is discarded using the
    // registered killed state. Keep flush out of the request arbitration cone.
    val icachePending = io.icache.req.valid && io.icache.req.bits.cacheable &&
        !icacheOutstanding
    val writeEligible = io.dcache.writeReq.valid && !io.l2.miss.active
    val icacheEligible = icachePending && readAllowed(io.icache.req.bits.paddr)
    val dcacheReadEligible = io.dcache.readReq.valid &&
        readAllowed(io.dcache.readReq.bits.paddr)
    val selectedGrant = Mux(writeEligible,
        grantWrite,
        Mux(icacheEligible,
            grantICache,
            Mux(dcacheReadEligible, grantDCacheRead, grantNone)))
    // Payload is don't-care when no grant is valid.  Defaulting to the lowest
    // priority candidate keeps DCache-read eligibility out of the wide payload
    // mux while preserving every valid request bit-for-bit.
    val addressGrant = Mux(heldGrantValid,
        heldGrant,
        Mux(writeEligible,
            grantWrite,
            Mux(icacheEligible, grantICache, grantDCacheRead)))
    val heldSourceValid = Mux(heldGrant === grantWrite,
        io.dcache.writeReq.valid,
        Mux(heldGrant === grantICache,
            io.icache.req.valid,
            io.dcache.readReq.valid))
    val cancelHeldGrant = heldGrantValid && !heldSourceValid
    val activeGrant = Mux(heldGrantValid, heldGrant, selectedGrant)
    val grantValid = !cancelHeldGrant && activeGrant =/= grantNone &&
        (heldGrantValid || io.l2.miss.accepting)
    val chooseWrite = grantValid && activeGrant === grantWrite
    val chooseICache = grantValid && activeGrant === grantICache
    val chooseDCacheRead = grantValid && activeGrant === grantDCacheRead

    io.l2.req.valid := chooseWrite || chooseICache || chooseDCacheRead
    io.l2.req.bits.source := Mux(addressGrant === grantWrite,
        L2RequestSource.DCacheWrite,
        Mux(addressGrant === grantICache,
            L2RequestSource.ICache,
            L2RequestSource.DCacheRead))
    io.l2.req.bits.write := addressGrant === grantWrite
    io.l2.req.bits.paddr := Mux(addressGrant === grantWrite,
        io.dcache.writeReq.bits.paddr,
        Mux(addressGrant === grantICache,
            io.icache.req.bits.paddr,
            io.dcache.readReq.bits.paddr))
    io.l2.req.bits.data := Mux(addressGrant === grantWrite,
        io.dcache.writeReq.bits.data,
        0.U)

    io.icache.req.ready := chooseICache && io.l2.req.ready
    io.dcache.readReq.ready := chooseDCacheRead && io.l2.req.ready
    io.dcache.writeReq.ready := chooseWrite && io.l2.req.ready

    when(!heldGrantValid && io.l2.req.valid && !io.l2.req.ready) {
        heldGrantValid := true.B
        heldGrant := activeGrant
    }
    when((heldGrantValid && io.l2.req.fire) || cancelHeldGrant) {
        heldGrantValid := false.B
        heldGrant := grantNone
    }

    val responseICache = io.l2.resp.bits.source === L2RequestSource.ICache
    val responseDCacheRead = io.l2.resp.bits.source === L2RequestSource.DCacheRead
    val responseDCacheWrite = io.l2.resp.bits.source === L2RequestSource.DCacheWrite
    val dropICacheResponse = responseICache && icacheKilled

    io.icache.resp.valid := io.l2.resp.valid && responseICache && !dropICacheResponse
    io.icache.resp.bits.refillLine := io.l2.resp.bits.data
    io.dcache.readResp.valid := io.l2.resp.valid && responseDCacheRead
    io.dcache.readResp.bits.paddr := io.l2.resp.bits.paddr
    io.dcache.readResp.bits.data := io.l2.resp.bits.data
    io.dcache.readResp.bits.dirty := false.B
    io.dcache.writeResp.valid := io.l2.resp.valid && responseDCacheWrite
    io.dcache.writeResp.bits.paddr := io.l2.resp.bits.paddr

    io.l2.resp.ready := Mux(responseICache,
        dropICacheResponse || io.icache.resp.ready,
        Mux(responseDCacheRead, io.dcache.readResp.ready, responseDCacheWrite))

    when(io.icache.req.fire) {
        icacheOutstanding := true.B
        icacheKilled := io.flush
    }
    when(io.flush && icacheOutstanding) {
        icacheKilled := true.B
    }
    when(io.l2.resp.fire && responseICache) {
        icacheOutstanding := false.B
        icacheKilled := false.B
    }

    when(!reset.asBool) {
        when(io.l2.req.fire) {
            assert(io.l2.req.bits.paddr(DcacheConfig.DcacheOffset - 1, 0) === 0.U,
                "MidLayerArbiter: cache-line request must be aligned")
        }
        when(io.icache.req.fire) {
            assert(io.icache.req.bits.cacheable,
                "MidLayerArbiter: uncached fetch must use ExternalAXIArbiter")
        }
        when(io.l2.resp.valid) {
            assert(responseICache || responseDCacheRead || responseDCacheWrite,
                "MidLayerArbiter: invalid L2 response source")
        }
    }
}

class LineMemoryBypassIO extends Bundle {
    val upstream = Flipped(new L2ClientIO)
    val downstream = new MshrMemoryIO
}

class LineMemoryBypass extends Module {
    val io = IO(new LineMemoryBypassIO)

    val sIdle :: sReadWait :: sWriteWait :: sResp :: Nil = Enum(4)
    val state = RegInit(sIdle)
    val request = Reg(new L2Request)
    val response = Reg(new L2Response)

    io.upstream.miss := 0.U.asTypeOf(new L2MissStatus)
    io.upstream.miss.accepting := state === sIdle

    val requestWrite = io.upstream.req.bits.write
    io.downstream.readReq.valid := state === sIdle && io.upstream.req.valid && !requestWrite
    io.downstream.readReq.bits.paddr := io.upstream.req.bits.paddr
    io.downstream.writeReq.valid := state === sIdle && io.upstream.req.valid && requestWrite
    io.downstream.writeReq.bits.paddr := io.upstream.req.bits.paddr
    io.downstream.writeReq.bits.data := io.upstream.req.bits.data
    io.upstream.req.ready := state === sIdle && Mux(
        requestWrite,
        io.downstream.writeReq.ready,
        io.downstream.readReq.ready
    )

    io.downstream.readResp.ready := state === sReadWait
    io.upstream.resp.valid := state === sResp
    io.upstream.resp.bits := response

    when(io.upstream.req.fire) {
        request := io.upstream.req.bits
        state := Mux(io.upstream.req.bits.write, sWriteWait, sReadWait)
    }

    when(io.downstream.readResp.fire) {
        assert(io.downstream.readResp.bits.paddr === request.paddr,
            "LineMemoryBypass: read response address mismatch")
        response.source := request.source
        response.write := false.B
        response.paddr := request.paddr
        response.data := io.downstream.readResp.bits.data
        response.dirty := false.B
        state := sResp
    }

    when(state === sWriteWait && io.downstream.writeResp.valid) {
        assert(io.downstream.writeResp.bits.paddr === request.paddr,
            "LineMemoryBypass: write response address mismatch")
        response.source := request.source
        response.write := true.B
        response.paddr := request.paddr
        response.data := 0.U
        response.dirty := false.B
        state := sResp
    }

    when(io.upstream.resp.fire) {
        state := sIdle
    }
}

object GenerateMidLayerArbiter extends App {
    ChiselStage.emitSystemVerilogFile(
        new MidLayerArbiter,
        args = Array("--target-dir", "generated/mid-layer-arbiter"),
        firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info")
    )
}
