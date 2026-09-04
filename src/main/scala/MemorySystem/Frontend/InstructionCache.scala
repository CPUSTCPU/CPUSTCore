package CPUSTC.memory.frontend
/*
* 这里是指令cache说明：
* 单读写口全阻塞
* 共三级寄存器，其中前后信号均直接连通reg
* 可接受字级不对齐读取，超出边界部分返回0（mask需要对应适配）
* 对于触发异常的指令会被标记except而不触发miss流程直接给出
* 以上
* */

import chisel3._
import chisel3.util._
import CPUSTC.memory.mmu._
import CPUSTC.utils.XilinxSinglePortRamReadFirst
import CPUSTC.memory._
import CPUSTC.config.ExpCode
import CPUSTC.perf.ICachePerfEvents

class IcachePpResp extends Bundle {
    val pc     = UInt(32.W)
    val instrs = UInt((32 * IcacheConfig.nfetch).W)
    val exceps = Vec(IcacheConfig.nfetch, Bool())
    val exceptionCauses = Vec(IcacheConfig.nfetch, UInt(8.W))
    val normal = Bool()
    val mask   = UInt(IcacheConfig.nfetch.W)
}

class IcachePpReq extends Bundle {
    val pc   = UInt(32.W)
    val mask = UInt(IcacheConfig.nfetch.W)
}

class IcachePpBus extends Bundle {
    val req = Flipped(Decoupled(new IcachePpReq))
    val resp = Decoupled(new IcachePpResp)
}

class IcacheTlbBus extends Bundle {
    val req = Decoupled(new TLBReq)
    val resp = Flipped(Decoupled(new TLBResp))
}

class IcacheMissReq extends Bundle {
    val paddr = UInt(32.W)
    val cacheable = Bool()
}

class IcacheMissResp extends Bundle {
    val refillLine = UInt(IcacheConfig.IcacheLineBits.W)
}

class IcacheMissBus extends Bundle {
    val req = Decoupled(new IcacheMissReq)
    val resp = Flipped(Decoupled(new IcacheMissResp))
}

class ICacheIO extends Bundle {
    val pp = new IcachePpBus
    val tlb = new IcacheTlbBus
    val flush = Input(Bool())
    val redirect = Input(Bool())
    val invalidate = Input(Bool())
    val invalidateDone = Output(Bool())
    val missReq = new IcacheMissBus
    val perf = Output(new ICachePerfEvents)
}

class IcacheMeta extends Bundle {
    val tag = UInt(IcacheConfig.IcacheTag.W)
}

class IcacheStageReq extends Bundle {
    val valid = Bool()
    val req = new IcachePpReq
}

class IcacheStageResp extends IcacheStageReq {
    val paddr = UInt(32.W)
    val uncache = Bool()
    val exception = UInt(8.W)
    val meta = Vec(IcacheConfig.IcacheWay, new IcacheMeta)
    val rdata = Vec(IcacheConfig.IcacheWay, UInt(IcacheConfig.IcacheLineBits.W))
    val selectedData = UInt(IcacheConfig.IcacheFetchBits.W)
    val hits = UInt(IcacheConfig.IcacheWay.W)
    val replaceOH = UInt(IcacheConfig.IcacheWay.W)
    val hit = Bool()
}

class IcacheSinglePortRamReadFirst(width: Int, depth: Int, useBlackBox: Boolean) extends Module {
    val io = IO(new Bundle {
        val addra = Input(UInt(log2Ceil(depth).W))
        val dina = Input(UInt(width.W))
        val clka = Input(Clock())
        val wea = Input(Bool())
        val ena = Input(Bool())
        val douta = Output(UInt(width.W))
    })

    if (useBlackBox) {
        val ram = Module(new XilinxSinglePortRamReadFirst(RAMWIDTH = width, RAMDEPTH = depth))
        ram.io.addra := io.addra
        ram.io.dina := io.dina
        ram.io.clka := io.clka
        ram.io.wea := io.wea
        ram.io.ena := io.ena
        io.douta := ram.io.douta
    } else {
        withClock(io.clka) {
            val ram = RegInit(VecInit.fill(depth)(0.U(width.W)))
            val addrReg = RegInit(0.U(log2Ceil(depth).W))

            when(io.ena) {
                when(io.wea) {
                    ram(io.addra) := io.dina
                }
                addrReg := io.addra
            }

            io.douta := ram(addrReg)
        }
    }
}

class ICache(useBlackBoxRam: Boolean = true) extends Module {
    val io = IO(new ICacheIO)

    private val metaWidth = (new IcacheMeta).getWidth
    private val wayIndexWidth = log2Ceil(IcacheConfig.IcacheWay)
    private val fetchBlockBytes = IcacheConfig.IcacheFetchBits / 8
    private val fetchBlockOffsetBits = log2Ceil(fetchBlockBytes)
    private val fetchBlocksPerLine = IcacheConfig.IcacheLineBits / IcacheConfig.IcacheFetchBits

    require(IcacheConfig.IcacheLineBits % IcacheConfig.IcacheFetchBits == 0,
        "ICache line must contain an integer number of fetch blocks")

    val meta = Seq.fill(IcacheConfig.IcacheWay)(
        Module(new IcacheSinglePortRamReadFirst(
            width = metaWidth,
            depth = IcacheConfig.IcacheSet,
            useBlackBox = useBlackBoxRam
        )).io
    )
    val dataBanks = Seq.fill(fetchBlocksPerLine)(
        Module(new IcacheSinglePortRamReadFirst(
            width = IcacheConfig.IcacheFetchBits,
            depth = IcacheConfig.IcacheSet * IcacheConfig.IcacheWay,
            useBlackBox = useBlackBoxRam
        )).io
    )
    val metaValid = RegInit(VecInit(Seq.fill(IcacheConfig.IcacheWay)(
        VecInit(Seq.fill(IcacheConfig.IcacheSet)(false.B))
    )))
    val metaValidReadIndex = RegInit(0.U(log2Ceil(IcacheConfig.IcacheSet).W))
    val replacePtr = RegInit(VecInit(Seq.fill(IcacheConfig.IcacheSet)(0.U(wayIndexWidth.W))))

    // IBAR is rare and may take multiple cycles.  Sweep one set per cycle so
    // invalidation adds no generation compare or fanout to the normal hit path.
    val invalidateActive = RegInit(false.B)
    val invalidateIndex = RegInit(0.U(log2Ceil(IcacheConfig.IcacheSet).W))
    val invalidateStart = io.invalidate && !invalidateActive
    val invalidateLast = invalidateIndex === (IcacheConfig.IcacheSet - 1).U
    io.invalidateDone := invalidateActive && invalidateLast
    val maintenanceFlush = invalidateStart || invalidateActive
    val cacheFlush = io.flush || invalidateStart || invalidateActive
    val redirectReplace = io.redirect && io.flush && !maintenanceFlush

    def getTag(x: UInt): UInt = x(31, IcacheConfig.IcacheOffset + IcacheConfig.IcacheIndex)
    def getIndex(x: UInt): UInt = x(IcacheConfig.IcacheOffset + IcacheConfig.IcacheIndex - 1, IcacheConfig.IcacheOffset)
    def getLineAddr(x: UInt): UInt = x(31, IcacheConfig.IcacheOffset) ## 0.U(IcacheConfig.IcacheOffset.W)
    def getFetchBlock(x: UInt): UInt = x(IcacheConfig.IcacheOffset - 1, fetchBlockOffsetBits)
    def getFetchWindow(line: UInt, pc: UInt): UInt = {
        VecInit.tabulate(fetchBlocksPerLine) { block =>
            line(
                (block + 1) * IcacheConfig.IcacheFetchBits - 1,
                block * IcacheConfig.IcacheFetchBits
            )
        }(getFetchBlock(pc))
    }
    def incWay(ptr: UInt): UInt = {
        Mux(ptr === (IcacheConfig.IcacheWay - 1).U, 0.U, ptr + 1.U)
    }

    val sNormal :: sMissReq :: sRefilling :: sReStart :: Nil = Enum(4)
    val missState = RegInit(sNormal)
    val missActive = missState =/= sNormal

    // channel 1: pipeline request
    //==================== c1s1: accept pipeline request, get TLB/meta results, and calculate hits ==================
    val c1s1 = RegInit(0.U.asTypeOf(new IcacheStageReq))

    //==================== c1s2: get data RAM results, select data, and calculate hit/miss ==================
    val metaRdata = VecInit(meta.map(_.douta.asTypeOf(new IcacheMeta)))
    val metaValidRdata = VecInit(metaValid.map(_(metaValidReadIndex)))
    val dataBankRdata = VecInit(dataBanks.map(_.douta))
    val c1s2 = RegInit(0.U.asTypeOf(new IcacheStageResp))

    val tlbResp = io.tlb.resp.bits
    val deferredFetchException = MuxCase(0.U(8.W), Seq(
        !tlbResp.fetchPageValid -> ExpCode.PIF,
        (tlbResp.fetchRequestPlv > tlbResp.fetchPagePlv) -> ExpCode.PPI
    ))
    val tlbRespException = Mux(
        tlbResp.deferredFetchCheck,
        deferredFetchException,
        tlbResp.exception
    )

    val c1s1Hits = VecInit(metaRdata.zip(metaValidRdata).map { case (m, valid) =>
        valid && m.tag === getTag(tlbResp.paddr)
    }).asUInt
    val invalidWays = VecInit(metaValidRdata.map(valid => !valid)).asUInt
    val invalidReplaceOH = PriorityEncoderOH(invalidWays)
    val chooseInvalidWay = invalidWays.orR

    val c1s2Exception = c1s2.exception.orR
    val restartHits = VecInit(metaRdata.zip(metaValidRdata).map { case (m, valid) =>
        valid && m.tag === getTag(c1s2.paddr)
    }).asUInt
    val cacheSelectedData = dataBankRdata(getFetchBlock(c1s2.req.pc))
    val refillDataHeld = c1s2.hit
    val selectedData = Mux(refillDataHeld, c1s2.selectedData, cacheSelectedData)
    val c1s2CacheHit = c1s2.hits.orR && !c1s2.uncache && !c1s2Exception
    val c1s2Hit = c1s2CacheHit || (refillDataHeld && !c1s2Exception)
    val c1s2Miss = c1s2.valid && !c1s2Exception && !c1s2Hit

    //===================== c1s3: register selected hit/refill data ============================
    val c1s3 = RegInit(0.U.asTypeOf(new IcacheStageResp))

    val c1s3Exception = c1s3.exception.orR
    val hitRespValid = c1s3.valid && (c1s3.hit || c1s3Exception)
    val currentRefill = c1s2.valid && missState === sRefilling
    val respValid = hitRespValid && !cacheFlush
    val respFire = io.pp.resp.fire
    val missReqFire = io.missReq.req.fire
    val refillFire = io.missReq.resp.fire

    // A redirect may race a pending miss request. Let that request either fire
    // or be withdrawn after c1s2 is cleared at this clock edge; MidLayer tracks
    // and drops a response for a request that fired with flush asserted. This
    // keeps redirect out of the ICache -> L2 request timing cone.
    io.missReq.req.valid :=
        c1s2.valid && missState === sMissReq && !c1s2Exception &&
            !maintenanceFlush
    io.missReq.req.bits.paddr := Mux(c1s2.uncache, c1s2.paddr, getLineAddr(c1s2.paddr))
    io.missReq.req.bits.cacheable := !c1s2.uncache
    io.missReq.resp.ready := true.B

    val refillCacheable = refillFire && currentRefill && !c1s2.uncache && !cacheFlush

    //================================= Block RAM set ==========================================

    val metaWrite = Wire(new IcacheMeta)
    metaWrite.tag := getTag(c1s2.paddr)

    //================== Icache Pipeline Control ==================================
    val external_stall = hitRespValid && !io.pp.resp.ready
    val startMiss = missState === sNormal && c1s2Miss
    io.perf.hit := missState === sNormal && c1s2.valid && c1s2Hit &&
        !external_stall
    io.perf.miss := startMiss
    val basePipelineBlocked =
        !redirectReplace &&
            (external_stall || c1s2Miss || missActive || cacheFlush)
    val tlbRespMatches =
        io.tlb.resp.bits.token === c1s1.req.pc(15, 0)
    val tlbRespUsable =
        io.tlb.resp.valid && c1s1.valid && tlbRespMatches
    val s1CanAdvance =
        redirectReplace ||
            (!basePipelineBlocked && (!c1s1.valid || tlbRespUsable))
    val pipelineBlocked = !s1CanAdvance

    // A stale response is consumed but cannot advance c1s1. Normal responses
    // remain backpressured whenever c1s2/c1s3 cannot move.
    io.tlb.resp.ready := !basePipelineBlocked
    io.tlb.req.valid := io.pp.req.valid && s1CanAdvance
    io.pp.req.ready := s1CanAdvance && io.tlb.req.ready
    val reqFire = io.pp.req.fire
    when(reqFire) {
        assert(io.pp.req.bits.pc(fetchBlockOffsetBits - 1, 0) === 0.U,
            "ICache fetch PC must be aligned to the fetch block")
    }
    io.tlb.req.bits.vaddr := io.pp.req.bits.pc
    io.tlb.req.bits.stall := false.B
    io.tlb.req.bits.access := TLBAccess.Fetch
    io.tlb.req.bits.token := io.pp.req.bits.pc(15, 0)
    val metaReadPc = Mux(reqFire, io.pp.req.bits.pc, c1s1.req.pc)
    val metaReadIndex = getIndex(metaReadPc)
    // A MicroTLB miss holds c1s1/c1s2 metadata. Keep the synchronous data RAM
    // on c1s2 as well, otherwise its output can be paired with the old c1s2 PC.
    val tlbWaitHoldsS2 =
        c1s1.valid && !tlbRespUsable && !basePipelineBlocked
    val holdS2DataRead =
        (external_stall || tlbWaitHoldsS2) && !refillDataHeld
    val dataReadPc = Mux(holdS2DataRead, c1s2.req.pc, c1s1.req.pc)
    val dataReadIndex = getIndex(dataReadPc)
    val dataReadHits = Mux(holdS2DataRead, c1s2.hits, c1s1Hits)
    val dataReadWay = OHToUInt(dataReadHits)
    val dataReadAddr = Cat(dataReadIndex, dataReadWay)
    val ramWriteIndex = getIndex(c1s2.paddr)
    val ramWriteWay = OHToUInt(c1s2.replaceOH)
    val dataWriteAddr = Cat(ramWriteIndex, ramWriteWay)
    val useMissRamAddr =
        (missState === sMissReq || missState === sRefilling) &&
            !redirectReplace
    metaValidReadIndex := Mux(useMissRamAddr, ramWriteIndex, metaReadIndex)
    meta.zipWithIndex.foreach { case (metat, i) =>
        metat.clka := clock
        metat.addra := Mux(useMissRamAddr, ramWriteIndex, metaReadIndex)
        metat.ena := true.B
        metat.dina := metaWrite.asUInt
        metat.wea := refillCacheable && c1s2.replaceOH(i)
        when(refillCacheable && c1s2.replaceOH(i)) {
            metaValid(i)(ramWriteIndex) := true.B
        }
    }
    dataBanks.zipWithIndex.foreach { case (bank, block) =>
        bank.clka := clock
        bank.addra := Mux(useMissRamAddr, dataWriteAddr, dataReadAddr)
        bank.ena := true.B
        bank.dina := io.missReq.resp.bits.refillLine(
            (block + 1) * IcacheConfig.IcacheFetchBits - 1,
            block * IcacheConfig.IcacheFetchBits
        )
        bank.wea := refillCacheable
    }

    when(cacheFlush) {
        c1s1.valid := redirectReplace && reqFire
        when(redirectReplace && reqFire) {
            c1s1.req := io.pp.req.bits
        }
        c1s2.valid := false.B
        c1s3.valid := false.B
    }.elsewhen(respFire && hitRespValid && pipelineBlocked) {
        c1s3.valid := false.B
    }.elsewhen(s1CanAdvance) {
        c1s2.valid := c1s1.valid && tlbRespUsable
        c1s2.req := c1s1.req
        c1s2.paddr := tlbResp.paddr
        c1s2.uncache := tlbResp.uncache && !tlbRespException.orR
        c1s2.exception := tlbRespException
        c1s2.meta := metaRdata
        c1s2.hits := c1s1Hits
        c1s2.hit := false.B
        c1s2.selectedData := 0.U
        c1s2.replaceOH := Mux(
            chooseInvalidWay,
            invalidReplaceOH,
            UIntToOH(replacePtr(getIndex(tlbResp.paddr)), IcacheConfig.IcacheWay)
        )

        c1s1.valid := reqFire
        c1s1.req := io.pp.req.bits

        c1s3 := c1s2
        // Fault packets carry no executable instruction data.  Clear them at
        // this registered boundary so exception decode cannot feed the IFU's
        // normal predecode/repair cone.
        c1s3.selectedData := Mux(c1s2Exception, 0.U, selectedData)
        c1s3.hit := c1s2Hit
    }

    when(io.tlb.resp.fire && c1s1.valid && !tlbRespMatches) {
        assert(false.B, "ICache discarded a stale TLB response")
    }

    when(!cacheFlush && refillFire && currentRefill && c1s2.uncache) {
        c1s2.selectedData := io.missReq.resp.bits.refillLine(IcacheConfig.IcacheFetchBits - 1, 0)
        c1s2.hit := true.B
    }
    when(!cacheFlush && missState === sReStart && c1s2.valid && !c1s2.uncache) {
        c1s2.hits := restartHits
        c1s2.selectedData := cacheSelectedData
        c1s2.hit := restartHits.orR
    }

    when(io.redirect) {
        assert(io.flush)
    }
    when(refillCacheable) {
        replacePtr(ramWriteIndex) := incWay(OHToUInt(c1s2.replaceOH))
    }

    when(invalidateStart) {
        invalidateActive := true.B
        invalidateIndex := 0.U
    }.elsewhen(invalidateActive) {
        for (way <- 0 until IcacheConfig.IcacheWay) {
            metaValid(way)(invalidateIndex) := false.B
        }
        when(invalidateLast) {
            invalidateActive := false.B
        }.otherwise {
            invalidateIndex := invalidateIndex + 1.U
        }
    }

    //================================ main pipeline interface ================================

    io.pp.resp.valid := respValid
    io.pp.resp.bits.pc := c1s3.req.pc
    io.pp.resp.bits.mask := c1s3.req.mask
    // Keep the rare exception response out of IFU's normal predecode/repair
    // cone. c1s3.hit is already registered and is false for every exception.
    io.pp.resp.bits.normal := c1s3.hit
    io.pp.resp.bits.exceps := VecInit.tabulate(IcacheConfig.nfetch) { i =>
        c1s3Exception && c1s3.req.mask(i)
    }
    io.pp.resp.bits.exceptionCauses := VecInit.tabulate(IcacheConfig.nfetch) { i =>
        Mux(c1s3Exception && c1s3.req.mask(i), c1s3.exception, 0.U)
    }

    // Fault data was cleared at the c1s2 -> c1s3 registered boundary.  Keep the
    // response data direct here so exception state does not control predecode.
    io.pp.resp.bits.instrs := c1s3.selectedData

    //================== refill FSM & register update ==============================

    when(cacheFlush) {
        when(missState === sRefilling) {
            missState := sReStart
        }.otherwise {
            missState := sNormal
        }
    }.otherwise {
        when(startMiss) {
            missState := sMissReq
        }.elsewhen(missState === sMissReq && missReqFire) {
            missState := sRefilling
        }.elsewhen(missState === sRefilling && refillFire) {
            missState := sReStart
        }.elsewhen(missState === sReStart) {
            missState := sNormal
        }
    }

    assert(!c1s2.valid || PopCount(c1s2.hits) <= 1.U, "ICache: channel 1: multiple hits")
    when(c1s3.valid && c1s3Exception) {
        assert(!c1s3.hit, "ICache exception response cannot be a normal hit")
        assert(c1s3.selectedData === 0.U,
            "ICache exception response must not expose instruction data")
    }
}
