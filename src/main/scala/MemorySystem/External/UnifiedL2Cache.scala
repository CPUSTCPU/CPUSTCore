package CPUSTC.memory.external

import chisel3._
import chisel3.util._
import CPUSTC.memory._
import CPUSTC.memory.backend._
import CPUSTC.memory.frontend.IcacheSinglePortRamReadFirst

class UnifiedL2Meta(tagBits: Int) extends Bundle {
    val valid = Bool()
    val dirty = Bool()
    val tag = UInt(tagBits.W)
}

class UnifiedL2PerfEvents extends Bundle {
    val iReadHit = Bool()
    val iReadMiss = Bool()
    val dReadHit = Bool()
    val dReadMiss = Bool()
    val writeHit = Bool()
    val writeMiss = Bool()
    val dirtyWriteback = Bool()
    val uncacheRead = Bool()
    val uncacheWrite = Bool()
    val busy = Bool()
}

/** Serialize an uncached store against the L2 and update an already-resident
  * copy of the word.  `data` and `mask` use the same unshifted convention as
  * BackendInst: paddr(1, 0) selects the first byte lane in the 32-bit word.
  */
class L2UncacheWriteSnoopRequest extends Bundle {
    val paddr = UInt(32.W)
    val data = UInt(AXIConfig.DataBits.W)
    val mask = UInt(AXIConfig.DataBytes.W)
}

class L2UncacheWriteSnoopResponse extends Bundle {
    val paddr = UInt(32.W)
}

object L2MaintenanceOperation {
    val StoreTag = 0.U(2.W)
    val Index = 1.U(2.W)
    val Hit = 2.U(2.W)
}

class L2MaintenanceRequest extends Bundle {
    val operation = UInt(2.W)
    val vaddr = UInt(32.W)
    val paddr = UInt(32.W)
}

class L2MaintenanceResponse extends Bundle {
    val operation = UInt(2.W)
}

class UnifiedL2CacheIO extends Bundle {
    val upstream = Flipped(new L2ClientIO)
    val downstream = new MshrMemoryIO
    val uncacheWriteSnoopReq = Flipped(Decoupled(new L2UncacheWriteSnoopRequest))
    val uncacheWriteSnoopResp = Decoupled(new L2UncacheWriteSnoopResponse)
    val maintenanceReq = Flipped(Decoupled(new L2MaintenanceRequest))
    val maintenanceResp = Decoupled(new L2MaintenanceResponse)
    val perf = Output(new UnifiedL2PerfEvents)
}

class UnifiedL2Cache(
    memSysConfig: MemSysConfig = MemSysConfig(),
    useBlackBoxRam: Boolean = true
) extends Module {
    private val nSets = memSysConfig.l2Sets
    private val nWays = memSysConfig.l2Ways
    private val lineBytes = memSysConfig.l2LineBytes

    val io = IO(new UnifiedL2CacheIO)

    private val lineBits = lineBytes * 8
    private val lineWords = lineBits / AXIConfig.DataBits
    private val byteOffsetBits = log2Ceil(AXIConfig.DataBytes)
    private val offsetBits = log2Ceil(lineBytes)
    private val indexBits = log2Ceil(nSets)
    private val wayBits = log2Ceil(nWays)
    private val tagBits = 32 - indexBits - offsetBits
    private val dataDepth = nSets * nWays
    private val dataAddrBits = log2Ceil(dataDepth)
    private val metaBits = (new UnifiedL2Meta(tagBits)).getWidth

    require(IcacheConfig.IcacheLineBits == lineBits)
    require(DcacheConfig.DcacheLineBits == lineBits)
    require(AXIConfig.DataBits == 32)
    require(isPow2(nSets) && nSets > 1)
    require(isPow2(nWays) && nWays > 1)
    require(lineBytes == 64)

    private def getIndex(addr: UInt): UInt =
        addr(offsetBits + indexBits - 1, offsetBits)
    private def getTag(addr: UInt): UInt =
        addr(31, offsetBits + indexBits)
    private def lineAddr(addr: UInt): UInt =
        Cat(addr(31, offsetBits), 0.U(offsetBits.W))
    private def dataAddr(index: UInt, way: UInt): UInt = Cat(index, way)
    private def nextWay(way: UInt): UInt =
        Mux(way === (nWays - 1).U, 0.U, way + 1.U)

    val meta = Seq.fill(nWays)(Module(new IcacheSinglePortRamReadFirst(
        width = metaBits,
        depth = nSets,
        useBlackBox = useBlackBoxRam
    )).io)
    val data = Seq.fill(lineWords)(Module(new IcacheSinglePortRamReadFirst(
        width = AXIConfig.DataBits,
        depth = dataDepth,
        useBlackBox = useBlackBoxRam
    )).io)
    val replacePtr = RegInit(VecInit.fill(nSets)(0.U(wayBits.W)))

    val sInit :: sIdle :: sLookup :: sData :: sWaitMiss :: sSnoopLookup :: sSnoopData :: sMaintenanceLookup :: sMaintenanceWriteIssue :: sMaintenanceWriteWait :: Nil = Enum(10)
    val state = RegInit(sInit)
    val initIndex = RegInit(0.U(indexBits.W))

    val lookupReq = Reg(new L2Request)
    val lookupHitReg = RegInit(false.B)
    val lookupHitBankRegs = RegInit(VecInit.fill(lineWords)(false.B))
    dontTouch(lookupHitBankRegs)
    val lookupWayReg = Reg(UInt(wayBits.W))
    val lookupMetaReg = Reg(new UnifiedL2Meta(tagBits))

    val respValid = RegInit(false.B)
    val respBits = Reg(new L2Response)

    val missValid = RegInit(false.B)
    val missWrite = RegInit(false.B)
    val missReq = Reg(new L2Request)
    val missSet = Reg(UInt(indexBits.W))
    val missWay = Reg(UInt(wayBits.W))
    val missVictimDirty = RegInit(false.B)
    val missVictimAddr = Reg(UInt(32.W))
    val missVictimData = Reg(UInt(lineBits.W))
    val missReadSent = RegInit(false.B)
    val missRefillValid = RegInit(false.B)
    val linePayload = Reg(UInt(lineBits.W))
    val missWriteSent = RegInit(false.B)
    val missWriteDone = RegInit(false.B)

    val snoopReq = Reg(new L2UncacheWriteSnoopRequest)
    val snoopHitReg = RegInit(false.B)
    val snoopWayReg = Reg(UInt(wayBits.W))
    val snoopRespValid = RegInit(false.B)
    val snoopRespBits = Reg(new L2UncacheWriteSnoopResponse)

    val maintenanceReq = Reg(new L2MaintenanceRequest)
    val maintenanceWriteIndex = Reg(UInt(indexBits.W))
    val maintenanceWriteWay = Reg(UInt(wayBits.W))
    val maintenanceWriteAddr = Reg(UInt(32.W))
    val maintenanceRespValid = RegInit(false.B)
    val maintenanceRespBits = Reg(new L2MaintenanceResponse)

    val metaRead = VecInit(meta.map(_.douta.asTypeOf(new UnifiedL2Meta(tagBits))))
    val dataReadLine = Cat(data.reverse.map(_.douta))
    val lookupIndex = getIndex(lookupReq.paddr)
    val lookupTag = getTag(lookupReq.paddr)
    val lookupHits = VecInit(metaRead.map(entry => entry.valid && entry.tag === lookupTag)).asUInt
    val lookupInvalid = VecInit(metaRead.map(entry => !entry.valid)).asUInt
    val lookupHit = lookupHits.orR
    val lookupWayOH = Mux(lookupHit,
        lookupHits,
        Mux(lookupInvalid.orR,
            PriorityEncoderOH(lookupInvalid),
            UIntToOH(replacePtr(lookupIndex), nWays)))
    val lookupWay = OHToUInt(lookupWayOH)
    val lookupMeta = Mux1H(lookupWayOH, metaRead)

    val snoopWord = snoopReq.paddr(offsetBits - 1, byteOffsetBits)
    val snoopByteShift = snoopReq.paddr(byteOffsetBits - 1, 0)
    val snoopBitShift = snoopByteShift << 3
    val snoopShiftedData = (snoopReq.data << snoopBitShift)(AXIConfig.DataBits - 1, 0)
    val snoopShiftedMask =
        (snoopReq.mask << snoopByteShift)(AXIConfig.DataBytes - 1, 0)
    val snoopBitMask = Cat((0 until AXIConfig.DataBytes).reverse.map { byte =>
        Fill(8, snoopShiftedMask(byte))
    })
    val maintenanceHitOperation =
        maintenanceReq.operation === L2MaintenanceOperation.Hit
    val maintenanceIndex = getIndex(Mux(
        maintenanceHitOperation,
        maintenanceReq.paddr,
        maintenanceReq.vaddr
    ))
    val maintenanceTag = getTag(maintenanceReq.paddr)
    val maintenanceHitOH = VecInit(metaRead.map { entry =>
        entry.valid && entry.tag === maintenanceTag
    }).asUInt
    val maintenanceDirectWay = maintenanceReq.vaddr(wayBits - 1, 0)
    val maintenanceSelectedOH = Mux(
        maintenanceHitOperation,
        maintenanceHitOH,
        UIntToOH(maintenanceDirectWay, nWays)
    )
    val maintenanceSelectedWay = OHToUInt(maintenanceSelectedOH)
    val maintenanceSelectedMeta = Mux1H(maintenanceSelectedOH, metaRead)
    val maintenanceSelectedValid = maintenanceSelectedOH.orR &&
        maintenanceSelectedMeta.valid
    val maintenanceInvalidateOperation =
        maintenanceReq.operation === L2MaintenanceOperation.Index ||
        maintenanceHitOperation
    val maintenanceNeedsWriteback =
        state === sMaintenanceLookup &&
        maintenanceInvalidateOperation &&
        maintenanceSelectedValid && maintenanceSelectedMeta.dirty
    val maintenanceWriteResponse =
        state === sMaintenanceWriteWait && io.downstream.writeResp.valid
    val maintenanceStoreTag = state === sMaintenanceLookup &&
        maintenanceReq.operation === L2MaintenanceOperation.StoreTag
    val maintenanceCleanInvalidate = state === sMaintenanceLookup &&
        maintenanceInvalidateOperation &&
        maintenanceSelectedValid && !maintenanceSelectedMeta.dirty
    val maintenanceMetaWrite = maintenanceStoreTag ||
        maintenanceCleanInvalidate || maintenanceWriteResponse
    val missReadyToInstall = missValid && missRefillValid && missWriteDone
    val installStateAvailable = state === sIdle || state === sWaitMiss
    val installMiss = missReadyToInstall && installStateAvailable && !respValid
    val requestSet = getIndex(io.upstream.req.bits.paddr)
    val requestBlockedByMiss = missValid && (
        missWrite || io.upstream.req.bits.write || requestSet === missSet
    )
    val maintenanceCanStart = state === sIdle && !respValid && !installMiss &&
        !missValid && !snoopRespValid && !maintenanceRespValid
    io.maintenanceReq.ready := maintenanceCanStart
    val maintenanceFire = io.maintenanceReq.fire
    io.maintenanceResp.valid := maintenanceRespValid
    io.maintenanceResp.bits := maintenanceRespBits

    val snoopCanStart = state === sIdle && !respValid && !installMiss &&
        !missValid && !snoopRespValid && !maintenanceRespValid &&
        !io.maintenanceReq.valid
    io.uncacheWriteSnoopReq.ready := snoopCanStart
    val snoopFire = io.uncacheWriteSnoopReq.fire
    io.uncacheWriteSnoopResp.valid := snoopRespValid
    io.uncacheWriteSnoopResp.bits := snoopRespBits

    // A snoop is rare and serialized.  Giving it priority when valid avoids
    // exposing a stale resident line after the uncached store has completed.
    io.upstream.req.ready := state === sIdle && !respValid && !installMiss &&
        !requestBlockedByMiss && !snoopRespValid && !maintenanceRespValid &&
        !io.maintenanceReq.valid && !io.uncacheWriteSnoopReq.valid
    val requestFire = io.upstream.req.fire

    val setMask = ((((BigInt(1) << indexBits) - 1) << offsetBits).U(32.W))
    io.upstream.miss.accepting := state === sIdle && !respValid && !installMiss &&
        !snoopRespValid && !maintenanceRespValid &&
        !io.maintenanceReq.valid && !io.uncacheWriteSnoopReq.valid
    io.upstream.miss.active := missValid
    io.upstream.miss.write := missWrite
    io.upstream.miss.setMask := setMask
    io.upstream.miss.setValue := missReq.paddr & setMask

    io.upstream.resp.valid := respValid
    io.upstream.resp.bits := respBits
    val responseFire = io.upstream.resp.fire

    io.downstream.readReq.valid := missValid && !missWrite && !missReadSent
    io.downstream.readReq.bits.paddr := missReq.paddr
    io.downstream.readResp.ready := missValid && !missWrite && missReadSent &&
        !missRefillValid
    val maintenanceWriteIssue = state === sMaintenanceWriteIssue
    io.downstream.writeReq.valid := maintenanceWriteIssue ||
        (missValid && missVictimDirty && !missWriteSent)
    io.downstream.writeReq.bits.paddr := Mux(
        maintenanceWriteIssue,
        maintenanceWriteAddr,
        missVictimAddr
    )
    io.downstream.writeReq.bits.data := Mux(
        maintenanceWriteIssue,
        dataReadLine,
        missVictimData
    )

    val metaEnable = WireDefault(false.B)
    val metaWrite = WireDefault(false.B)
    val metaWriteAll = WireDefault(false.B)
    val metaWriteWay = WireDefault(0.U(nWays.W))
    val metaWriteData = WireDefault(0.U.asTypeOf(new UnifiedL2Meta(tagBits)))

    val retryLookup = state === sWaitMiss && !missValid && !respValid
    val hitWrite = state === sData && lookupHitReg && lookupReq.write
    // The address is don't-care while metaEnable is low.  Keep the address
    // selector physically independent of request acceptance/backpressure;
    // requestFire below controls only whether the offered set is read.
    val metaAddress = MuxCase(requestSet, Seq(
        (state === sInit) -> initIndex,
        installMiss -> missSet,
        hitWrite -> lookupIndex,
        maintenanceMetaWrite -> Mux(
            maintenanceWriteResponse,
            maintenanceWriteIndex,
            maintenanceIndex
        ),
        snoopFire -> getIndex(io.uncacheWriteSnoopReq.bits.paddr),
        maintenanceFire -> getIndex(Mux(
            io.maintenanceReq.bits.operation === L2MaintenanceOperation.Hit,
            io.maintenanceReq.bits.paddr,
            io.maintenanceReq.bits.vaddr
        )),
        retryLookup -> getIndex(lookupReq.paddr)
    ))

    when(state === sInit) {
        metaEnable := true.B
        metaWrite := true.B
        metaWriteAll := true.B
    }.elsewhen(installMiss) {
        metaEnable := true.B
        metaWrite := true.B
        metaWriteWay := UIntToOH(missWay, nWays)
        metaWriteData.valid := true.B
        metaWriteData.dirty := missWrite
        metaWriteData.tag := getTag(missReq.paddr)
    }.elsewhen(hitWrite) {
        metaEnable := true.B
        metaWrite := true.B
        metaWriteWay := UIntToOH(lookupWayReg, nWays)
        metaWriteData.valid := true.B
        metaWriteData.dirty := true.B
        metaWriteData.tag := lookupTag
    }.elsewhen(maintenanceMetaWrite) {
        metaEnable := true.B
        metaWrite := true.B
        metaWriteWay := UIntToOH(Mux(
            maintenanceWriteResponse,
            maintenanceWriteWay,
            maintenanceSelectedWay
        ), nWays)
    }.elsewhen(state === sIdle) {
        metaEnable := true.B
    }.elsewhen(retryLookup) {
        metaEnable := true.B
    }

    meta.zipWithIndex.foreach { case (ram, way) =>
        ram.clka := clock
        ram.ena := metaEnable
        ram.addra := metaAddress
        ram.dina := metaWriteData.asUInt
        ram.wea := metaWrite && (metaWriteAll || metaWriteWay(way))
    }
    data.zipWithIndex.foreach { case (ram, word) =>
        val bankHitWrite = state === sData && lookupHitBankRegs(word) &&
            lookupReq.write
        val snoopBankRead = state === sSnoopLookup && lookupHit &&
            snoopWord === word.U
        val snoopBankWrite = state === sSnoopData && snoopHitReg &&
            snoopWord === word.U
        val maintenanceBankRead = maintenanceNeedsWriteback
        val snoopMergedBankWord =
            (ram.douta & ~snoopBitMask) | (snoopShiftedData & snoopBitMask)
        val bankWrite = installMiss || bankHitWrite || snoopBankWrite
        ram.clka := clock
        ram.ena := state === sLookup || bankWrite || snoopBankRead ||
            maintenanceBankRead
        ram.addra := Mux(
            maintenanceBankRead,
            dataAddr(maintenanceIndex, maintenanceSelectedWay),
            Mux(
                snoopBankRead || snoopBankWrite,
                dataAddr(lookupIndex, Mux(snoopBankRead, lookupWay, snoopWayReg)),
                Mux(
                    installMiss,
                    dataAddr(missSet, missWay),
                    Mux(
                        bankHitWrite,
                        dataAddr(lookupIndex, lookupWayReg),
                        dataAddr(lookupIndex, lookupWay)
                    )
                )
            )
        )
        ram.dina := Mux(
            snoopBankWrite,
            snoopMergedBankWord,
            linePayload(
                (word + 1) * AXIConfig.DataBits - 1,
                word * AXIConfig.DataBits
            )
        )
        ram.wea := bankWrite
    }

    val perfPulse = WireDefault(0.U.asTypeOf(new UnifiedL2PerfEvents))
    perfPulse.busy := state =/= sIdle || missValid || respValid ||
        snoopRespValid || maintenanceRespValid
    perfPulse.dirtyWriteback := io.downstream.writeReq.fire
    when(state === sData) {
        when(lookupReq.write) {
            perfPulse.writeHit := lookupHitReg
            perfPulse.writeMiss := !lookupHitReg && !missValid
        }.elsewhen(lookupReq.source === L2RequestSource.ICache) {
            perfPulse.iReadHit := lookupHitReg
            perfPulse.iReadMiss := !lookupHitReg && !missValid
        }.otherwise {
            perfPulse.dReadHit := lookupHitReg
            perfPulse.dReadMiss := !lookupHitReg && !missValid
        }
    }
    io.perf := perfPulse

    when(responseFire) {
        respValid := false.B
    }
    when(io.uncacheWriteSnoopResp.fire) {
        snoopRespValid := false.B
    }
    when(io.maintenanceResp.fire) {
        maintenanceRespValid := false.B
    }

    when(io.downstream.readReq.fire) {
        missReadSent := true.B
    }
    when(io.downstream.readResp.fire) {
        assert(io.downstream.readResp.bits.paddr === missReq.paddr,
            "UnifiedL2Cache: refill address mismatch")
        missRefillValid := true.B
    }
    when(io.downstream.writeReq.fire && !maintenanceWriteIssue) {
        missWriteSent := true.B
    }
    when(io.downstream.writeResp.valid && missValid && missVictimDirty && missWriteSent) {
        assert(io.downstream.writeResp.bits.paddr === missVictimAddr,
            "UnifiedL2Cache: victim write response address mismatch")
        missWriteDone := true.B
    }

    when(installMiss) {
        replacePtr(missSet) := nextWay(missWay)
        respValid := true.B
        respBits.source := missReq.source
        respBits.write := missWrite
        respBits.paddr := missReq.paddr
        respBits.data := Mux(missWrite, 0.U, linePayload)
        respBits.dirty := false.B
        missValid := false.B
        missVictimDirty := false.B
        missReadSent := false.B
        missRefillValid := false.B
        missWriteSent := false.B
        missWriteDone := false.B
    }

    switch(state) {
        is(sInit) {
            when(initIndex === (nSets - 1).U) {
                state := sIdle
            }.otherwise {
                initIndex := initIndex + 1.U
            }
        }
        is(sIdle) {
            lookupReq.source := io.upstream.req.bits.source
            lookupReq.write := io.upstream.req.bits.write
            lookupReq.paddr := io.upstream.req.bits.paddr
            when(maintenanceFire) {
                maintenanceReq := io.maintenanceReq.bits
                state := sMaintenanceLookup
            }.elsewhen(snoopFire) {
                lookupReq.source := 0.U
                lookupReq.write := false.B
                lookupReq.paddr := io.uncacheWriteSnoopReq.bits.paddr
                lookupReq.data := 0.U
                snoopReq := io.uncacheWriteSnoopReq.bits
                state := sSnoopLookup
            }.elsewhen(requestFire) {
                lookupReq.data := io.upstream.req.bits.data
                state := sLookup
            }
        }
        is(sLookup) {
            lookupHitReg := lookupHit
            lookupHitBankRegs.foreach(_ := lookupHit)
            lookupWayReg := lookupWay
            lookupMetaReg := lookupMeta
            state := sData
        }
        is(sData) {
            when(lookupHitReg) {
                respValid := true.B
                respBits.source := lookupReq.source
                respBits.write := lookupReq.write
                respBits.paddr := lookupReq.paddr
                respBits.data := Mux(lookupReq.write, 0.U, dataReadLine)
                respBits.dirty := false.B
                state := sIdle
            }.elsewhen(missValid) {
                state := sWaitMiss
            }.otherwise {
                missValid := true.B
                missWrite := lookupReq.write
                missReq := lookupReq
                missSet := lookupIndex
                missWay := lookupWayReg
                missVictimDirty := lookupMetaReg.valid && lookupMetaReg.dirty
                missVictimAddr := Cat(
                    lookupMetaReg.tag,
                    lookupIndex,
                    0.U(offsetBits.W)
                )
                missVictimData := dataReadLine
                missReadSent := false.B
                missRefillValid := lookupReq.write
                missWriteSent := false.B
                missWriteDone := !lookupMetaReg.valid || !lookupMetaReg.dirty
                state := sIdle
            }
        }
        is(sWaitMiss) {
            when(!missValid && !respValid) {
                state := sLookup
            }
        }
        is(sSnoopLookup) {
            snoopHitReg := lookupHit
            snoopWayReg := lookupWay
            state := sSnoopData
        }
        is(sSnoopData) {
            snoopRespValid := true.B
            snoopRespBits.paddr := snoopReq.paddr
            state := sIdle
        }
        is(sMaintenanceLookup) {
            when(maintenanceNeedsWriteback) {
                maintenanceWriteIndex := maintenanceIndex
                maintenanceWriteWay := maintenanceSelectedWay
                maintenanceWriteAddr := Cat(
                    maintenanceSelectedMeta.tag,
                    maintenanceIndex,
                    0.U(offsetBits.W)
                )
                state := sMaintenanceWriteIssue
            }.otherwise {
                maintenanceRespValid := true.B
                maintenanceRespBits.operation := maintenanceReq.operation
                state := sIdle
            }
        }
        is(sMaintenanceWriteIssue) {
            when(io.downstream.writeReq.fire) {
                state := sMaintenanceWriteWait
            }
        }
        is(sMaintenanceWriteWait) {
            when(io.downstream.writeResp.valid) {
                assert(io.downstream.writeResp.bits.paddr === maintenanceWriteAddr,
                    "UnifiedL2Cache: maintenance write response address mismatch")
                maintenanceRespValid := true.B
                maintenanceRespBits.operation := maintenanceReq.operation
                state := sIdle
            }
        }
    }

    // A write payload is observable only after its request is accepted.  Load
    // it speculatively while idle so request acceptance does not drive the
    // clock enable of the 512-bit line register.
    when(state === sIdle && io.upstream.req.bits.write) {
        linePayload := io.upstream.req.bits.data
    }
    when(io.downstream.readResp.fire) {
        linePayload := io.downstream.readResp.bits.data
    }

    when(!reset.asBool) {
        when(requestFire) {
            assert(io.upstream.req.bits.paddr(offsetBits - 1, 0) === 0.U,
                "UnifiedL2Cache: request must be cache-line aligned")
            assert(
                (io.upstream.req.bits.write &&
                    io.upstream.req.bits.source === L2RequestSource.DCacheWrite) ||
                (!io.upstream.req.bits.write &&
                    (io.upstream.req.bits.source === L2RequestSource.ICache ||
                        io.upstream.req.bits.source === L2RequestSource.DCacheRead)),
                "UnifiedL2Cache: request source/type mismatch"
            )
            when(missValid) {
                assert(!io.upstream.req.bits.write && !missWrite && requestSet =/= missSet,
                    "UnifiedL2Cache: hit-under-miss request violates conflict policy")
            }
        }
        when(snoopFire) {
            assert(io.uncacheWriteSnoopReq.bits.mask.orR,
                "UnifiedL2Cache: uncached write snoop mask must not be empty")
            assert((io.uncacheWriteSnoopReq.bits.mask <<
                io.uncacheWriteSnoopReq.bits.paddr(byteOffsetBits - 1, 0)) <
                (BigInt(1) << AXIConfig.DataBytes).U,
                "UnifiedL2Cache: uncached write snoop must not cross a word")
        }
        when(state === sMaintenanceLookup && maintenanceHitOperation) {
            assert(PopCount(maintenanceHitOH) <= 1.U,
                "UnifiedL2Cache: multiple ways matched one maintenance line")
        }
        when(requestFire && io.downstream.readResp.fire) {
            assert(!io.upstream.req.bits.write,
                "UnifiedL2Cache: refill may overlap only a read request")
        }
        when(state === sLookup) {
            assert(PopCount(lookupHits) <= 1.U,
                "UnifiedL2Cache: multiple ways matched one line")
        }
        when(state === sSnoopLookup) {
            assert(PopCount(lookupHits) <= 1.U,
                "UnifiedL2Cache: multiple ways matched one snoop line")
        }
        assert(!(missValid && missWrite && !missRefillValid),
            "UnifiedL2Cache: write miss must retain its full line")
        assert(!(maintenanceWriteIssue && missValid),
            "UnifiedL2Cache: maintenance writeback must not overlap a miss")
    }
}
