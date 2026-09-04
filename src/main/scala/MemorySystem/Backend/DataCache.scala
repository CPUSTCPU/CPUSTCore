package CPUSTC.memory.backend

import chisel3._
import chisel3.util._
import CPUSTC.memory._
import CPUSTC.memory.MemoryPointerUtils._
import CPUSTC.utils.{XilinxTrueDualPortReadFirst1ClockRam, XilinxTrueDualPortReadFirstByteWrite1ClockRam}
import CPUSTC.perf.DCachePerfEvents

class LoadForwardResult extends Bundle {
    val result = new LoadResult
    val forwarded = Bool()
    val forwardSqindex = UInt(StoreQueueConfig.length.W)
    val forwardSqindexHigh = Bool()
    val forwardCommitted = Bool()
}

class StoreDataBypass extends Bundle {
    val data = UInt(32.W)
    val sqindex = UInt(StoreQueueConfig.length.W)
    val sqindexHigh = Bool()
}

class DcachePpResp extends Bundle {
    val inst = new BackendInst
    val vaddr = UInt(32.W)
    val paddr = UInt(32.W)
    val rdata = UInt(DcacheConfig.DcacheDataBits.W)
    val exception = UInt(8.W)
    val forwarded = Bool()
    val forwardSqindex = UInt(StoreQueueConfig.length.W)
    val forwardSqindexHigh = Bool()
    val forwardCommitted = Bool()
    val predictReserved = Bool()
}

class DcachePpReq extends BackendInst

class StoreLinePayload extends Bundle {
    val data = UInt(DcacheConfig.DcacheLineBits.W)
    val mask = UInt(DcacheConfig.DcacheLineBytes.W)
    val contributors = UInt(StoreQueueConfig.length.W)
}

class DcachePpBus extends Bundle {
    val req = Flipped(Decoupled(new DcachePpReq))
    val resp = Valid(new DcachePpResp)
}

class DcacheMshrPort0Req extends Bundle {
    val linePaddr = UInt((32 - DcacheConfig.DcacheOffset).W)
    val byteOffset = UInt(DcacheConfig.DcacheOffset.W)
    val loadMetadata = new MshrLoadMetadata
    val store = Bool()
    val storeData = UInt(DcacheConfig.DcacheDataBits.W)
    val storeMask = UInt(DcacheConfig.DcacheMaskBits.W)
    val sqindex = UInt(StoreQueueConfig.length.W)
    val sqindexHigh = Bool()
}

class DcacheMshrPort1Req extends Bundle {
    val linePaddr = UInt((32 - DcacheConfig.DcacheOffset).W)
    val byteOffset = UInt(DcacheConfig.DcacheOffset.W)
    val loadMetadata = new MshrLoadMetadata
}

class DcacheMshrRefill extends Bundle {
    val paddr = UInt(32.W)
    val data = UInt(DcacheConfig.DcacheLineBits.W)
    val dirty = Bool()
}

/** A serialized cache-line maintenance request.  The request is accepted only
  * after the normal DCache pipeline has drained, so it never competes with a
  * demand access for architectural ordering.
  */
class DcacheMaintenanceRequest extends Bundle {
    val paddr = UInt(32.W)
    val indexOnly = Bool()
    val way = UInt(log2Ceil(DcacheConfig.DcacheWay).W)
    val writeback = Bool()
    val invalidate = Bool()
}

class DcacheMaintenanceResponse extends Bundle {
    val writeback = Valid(new WritebackRequest)
}

class DcacheMshrIO extends Bundle {
    val req0 = Decoupled(new DcacheMshrPort0Req)
    val req1 = Decoupled(new DcacheMshrPort1Req)
    val storeAdmissionReady = Input(Bool())
    val victimAvailable = Input(Bool())
    val victimReq = Valid(new WritebackRequest)
    val resp = Flipped(Decoupled(new DcacheMshrRefill))
}

class DCacheIO extends Bundle {
    val mainPp = Vec(DcacheConfig.nPorts, new DcachePpBus)
    val flush = Input(Bool())
    val ldqValidMask = Input(UInt(LoadStateTableConfig.length.W))
    val ldqHighMask = Input(UInt(LoadStateTableConfig.length.W))
    val mshrIO = new DcacheMshrIO
    val maintenanceReq = Flipped(Decoupled(new DcacheMaintenanceRequest))
    val maintenanceResp = Decoupled(new DcacheMaintenanceResponse)
    val forwardSources = Input(Vec(StoreQueueConfig.length, new StoreForwardSource))
    val stdBypass = Input(Vec(LoadQueueConfig.EnqNum, Valid(new StoreDataBypass)))
    val loadStoreFail = Output(Vec(DcacheConfig.nPorts, new DcacheLoadFailBus))
    val loadMshrFail = Output(Vec(DcacheConfig.nPorts, new DcacheLoadFailBus))
    val storeRetry = Output(Valid(new DcacheStoreRetryBus))
    val loadPredWake = Output(Vec(
        DcacheConfig.nPorts,
        Valid(new LoadPredictInfo)
    ))
    val loadPredResolve = Output(Vec(
        DcacheConfig.nPorts,
        Valid(new LoadPredictResolve)
    ))
    val storeComplete = Output(Valid(new StoreReadyEvent))
    val requestAvailable = Output(Vec(DcacheConfig.nPorts, Bool()))
    val idle = Output(Bool())
    val perf = Output(new DCachePerfEvents)
}

class DcacheMeta extends Bundle {
    val tag = UInt(DcacheConfig.DcacheTag.W)
}

class DcacheStage1 extends Bundle {
    val valid = Bool()
    val req = new DcachePpReq
}

class DcacheForwardFail extends Bundle {
    val valid = Bool()
    val waitSqindex = UInt(StoreQueueConfig.length.W)
    val waitSqindexHigh = Bool()
    val partialOverlap = Bool()
    val waitStoreData = Bool()
}

class DcacheStage2 extends DcacheStage1 {
    val storeLine = new StoreLinePayload
    val hits = UInt(DcacheConfig.DcacheWay.W)
    val forwardFail = new DcacheForwardFail
    val forwardValid = Bool()
    val forwardData = UInt(DcacheConfig.DcacheDataBits.W)
    val forwardSqindex = UInt(StoreQueueConfig.length.W)
    val forwardSqindexHigh = Bool()
    val forwardCommitted = Bool()
}

class DcacheStage3 extends Bundle {
    val valid = Bool()
    val req = new DcachePpReq
    val hits = UInt(DcacheConfig.DcacheWay.W)
    val forwarded = Bool()
    val forwardData = UInt(DcacheConfig.DcacheDataBits.W)
    val forwardSqindex = UInt(StoreQueueConfig.length.W)
    val forwardSqindexHigh = Bool()
    val forwardCommitted = Bool()
    val predictReserved = Bool()
}

class DcacheTrueDualPortRamReadFirst(width: Int, depth: Int, useBlackBox: Boolean) extends Module {
    val io = IO(new Bundle {
        val addra = Input(UInt(log2Ceil(depth).W))
        val addrb = Input(UInt(log2Ceil(depth).W))
        val dina = Input(UInt(width.W))
        val dinb = Input(UInt(width.W))
        val clka = Input(Clock())
        val wea = Input(Bool())
        val web = Input(Bool())
        val ena = Input(Bool())
        val enb = Input(Bool())
        val douta = Output(UInt(width.W))
        val doutb = Output(UInt(width.W))
    })

    if (useBlackBox) {
        val ram = Module(new XilinxTrueDualPortReadFirst1ClockRam(RAMWIDTH = width, RAMDEPTH = depth))
        ram.io.addra := io.addra
        ram.io.addrb := io.addrb
        ram.io.dina := io.dina
        ram.io.dinb := io.dinb
        ram.io.clka := io.clka
        ram.io.wea := io.wea
        ram.io.web := io.web
        ram.io.ena := io.ena
        ram.io.enb := io.enb
        io.douta := ram.io.douta
        io.doutb := ram.io.doutb
    } else {
        withClock(io.clka) {
            val ram = RegInit(VecInit.fill(depth)(0.U(width.W)))
            val doutA = RegInit(0.U(width.W))
            val doutB = RegInit(0.U(width.W))

            when(io.ena) {
                doutA := ram(io.addra)
                when(io.wea) {
                    ram(io.addra) := io.dina
                }
            }

            when(io.enb) {
                doutB := ram(io.addrb)
                when(io.web) {
                    ram(io.addrb) := io.dinb
                }
            }

            io.douta := doutA
            io.doutb := doutB
        }
    }
}

class DcacheTrueDualPortByteWriteRamReadFirst(
    nBytes: Int,
    depth: Int,
    useBlackBox: Boolean
) extends Module {
    private val width = nBytes * 8
    val io = IO(new Bundle {
        val addra = Input(UInt(log2Ceil(depth).W))
        val addrb = Input(UInt(log2Ceil(depth).W))
        val dina = Input(UInt(width.W))
        val dinb = Input(UInt(width.W))
        val clka = Input(Clock())
        val wea = Input(UInt(nBytes.W))
        val web = Input(UInt(nBytes.W))
        val ena = Input(Bool())
        val enb = Input(Bool())
        val douta = Output(UInt(width.W))
        val doutb = Output(UInt(width.W))
    })

    if (useBlackBox) {
        val ram = Module(new XilinxTrueDualPortReadFirstByteWrite1ClockRam(
            NBCOL = nBytes,
            COLWIDTH = 8,
            RAMDEPTH = depth
        ))
        ram.io.addra := io.addra
        ram.io.addrb := io.addrb
        ram.io.dina := io.dina
        ram.io.dinb := io.dinb
        ram.io.clka := io.clka
        ram.io.wea := io.wea
        ram.io.web := io.web
        ram.io.ena := io.ena
        ram.io.enb := io.enb
        io.douta := ram.io.douta
        io.doutb := ram.io.doutb
    } else {
        withClock(io.clka) {
            val ram = RegInit(VecInit.fill(depth)(0.U(width.W)))
            val doutA = RegInit(0.U(width.W))
            val doutB = RegInit(0.U(width.W))

            def mergeBytes(oldData: UInt, newData: UInt, byteMask: UInt): UInt = {
                val bitMask = VecInit(byteMask.asBools.map(bit => Fill(8, bit))).asUInt
                (oldData & ~bitMask) | (newData & bitMask)
            }

            when(io.ena) {
                doutA := ram(io.addra)
                when(io.wea.orR) {
                    ram(io.addra) := mergeBytes(ram(io.addra), io.dina, io.wea)
                }
            }

            when(io.enb) {
                doutB := ram(io.addrb)
                when(io.web.orR) {
                    ram(io.addrb) := mergeBytes(ram(io.addrb), io.dinb, io.web)
                }
            }

            io.douta := doutA
            io.doutb := doutB
        }
    }
}

class DCache(useBlackBoxRam: Boolean = true) extends Module {
    val io = IO(new DCacheIO)

    private val metaWidth = (new DcacheMeta).getWidth
    private val wayIndexWidth = log2Ceil(DcacheConfig.DcacheWay)
    private val accessMaskWidth = DcacheConfig.DcacheMaskBits * 2
    private val wordsPerDataBank = 2
    private val dataBankCount = DcacheConfig.DcacheLineWord / wordsPerDataBank
    private val dataBankDepth = DcacheConfig.DcacheSet *
        DcacheConfig.DcacheWay * wordsPerDataBank
    private val wordIndexWidth = log2Ceil(DcacheConfig.DcacheLineWord)
    private val dataBankIndexWidth = log2Ceil(dataBankCount)

    require(DcacheConfig.DcacheLineWord % wordsPerDataBank == 0)

    val meta = Seq.fill(DcacheConfig.DcacheWay)(
        Module(new DcacheTrueDualPortRamReadFirst(
            width = metaWidth,
            depth = DcacheConfig.DcacheSet,
            useBlackBox = useBlackBoxRam
        )).io
    )
    val data = Seq.fill(dataBankCount)(
        Module(new DcacheTrueDualPortByteWriteRamReadFirst(
            nBytes = DcacheConfig.DcacheMaskBits,
            depth = dataBankDepth,
            useBlackBox = useBlackBoxRam
        )).io
    )

    val validArray = RegInit(VecInit(Seq.fill(DcacheConfig.DcacheWay)(
        VecInit(Seq.fill(DcacheConfig.DcacheSet)(false.B))
    )))
    val dirtyArray = RegInit(VecInit(Seq.fill(DcacheConfig.DcacheWay)(
        VecInit(Seq.fill(DcacheConfig.DcacheSet)(false.B))
    )))
    val replacePtr = RegInit(VecInit(Seq.fill(DcacheConfig.DcacheSet)(0.U(wayIndexWidth.W))))

    def isStore(req: DcachePpReq): Bool = req.uop.isSTD
    def isLoad(req: DcachePpReq): Bool = req.uop.isLD
    def getTag(x: UInt): UInt = x(31, DcacheConfig.DcacheOffset + DcacheConfig.DcacheIndex)
    def getIndex(x: UInt): UInt = x(DcacheConfig.DcacheOffset + DcacheConfig.DcacheIndex - 1, DcacheConfig.DcacheOffset)
    def getOffset(x: UInt): UInt = x(DcacheConfig.DcacheOffset - 1, 0)
    def getWordBank(x: UInt): UInt = x(DcacheConfig.DcacheOffset - 1, log2Ceil(DcacheConfig.DcacheMaskBits))
    def getDataBank(x: UInt): UInt = getWordBank(x)(wordIndexWidth - 1, 1)
    def getDataRowBit(x: UInt): Bool = getWordBank(x)(0)
    def getLineAddr(x: UInt): UInt = x(31, DcacheConfig.DcacheOffset) ## 0.U(DcacheConfig.DcacheOffset.W)

    def singleStoreLine(req: DcachePpReq): StoreLinePayload = {
        val result = WireDefault(0.U.asTypeOf(new StoreLinePayload))
        val byteOffset = getOffset(req.paddr)
        val bitOffset = byteOffset << 3
        result.data := (req.operateData.pad(DcacheConfig.DcacheLineBits) << bitOffset)(DcacheConfig.DcacheLineBits - 1, 0)
        result.mask := (req.mask.pad(DcacheConfig.DcacheLineBytes) << byteOffset)(DcacheConfig.DcacheLineBytes - 1, 0)
        result.contributors := req.sqindex
        result
    }

    def extendLoadData(word: UInt, mask: UInt, signed: Bool): UInt = {
        val byteData = word(7, 0)
        val halfData = word(15, 0)
        val byteResult = Cat(Fill(24, signed && byteData(7)), byteData)
        val halfResult = Cat(Fill(16, signed && halfData(15)), halfData)

        Mux(mask === "b0001".U, byteResult,
            Mux(mask === "b0011".U, halfResult, word))
    }

    def accessMask(paddr: UInt, mask: UInt): UInt = {
        (mask.pad(accessMaskWidth) << paddr(1, 0))(accessMaskWidth - 1, 0)
    }

    def alignForwardData(data: UInt, storeOffset: UInt, loadOffset: UInt): UInt = {
        val byteDelta = loadOffset - storeOffset
        MuxLookup(byteDelta, 0.U(DcacheConfig.DcacheDataBits.W))(Seq(
            0.U -> data,
            1.U -> Cat(0.U(8.W), data(31, 8)),
            2.U -> Cat(0.U(16.W), data(31, 16)),
            3.U -> Cat(0.U(24.W), data(31, 24))
        ))
    }

    def overlap(loadPaddr: UInt, loadAccessMask: UInt, store: StoreForwardSource): Bool = {
        store.valid && store.addrValid &&
            loadPaddr(31, 2) === store.paddr(31, 2) &&
            (loadAccessMask & store.alignedMask).orR
    }

    def matchForwardSources(
        load: DcachePpReq,
        loadPaddr: UInt,
        loadAccessMask: UInt,
        loadBoundaryOH: UInt,
        loadBoundaryHigh: Bool
    ): UInt = {
        VecInit((0 until StoreQueueConfig.length).map { index =>
            val store = io.forwardSources(index)
            overlap(loadPaddr, loadAccessMask, store) &&
                load.storeDepMask(index) &&
                pointerOlderThanBoundary(
                    store.sqindex,
                    store.sqindexHigh,
                    loadBoundaryOH,
                    loadBoundaryHigh
                )
        }).asUInt
    }

    def selectYoungestBanked(
        candidates: UInt,
        boundaryOH: UInt
    ): (UInt, UInt, UInt, Bool) = {
        require(candidates.getWidth == boundaryOH.getWidth)
        val width = candidates.getWidth
        require(width % 2 == 0)
        val bankDepth = width / 2
        val beforeBoundary = VecInit((0 until width).map { index =>
            if (index == width - 1) false.B
            else boundaryOH(width - 1, index + 1).orR
        }).asUInt

        def selectBank(parity: Int): (UInt, Bool, Bool) = {
            val bankCandidates = VecInit((0 until bankDepth).map { index =>
                candidates(2 * index + parity)
            }).asUInt
            val bankBeforeBoundary = VecInit((0 until bankDepth).map { index =>
                beforeBoundary(2 * index + parity)
            }).asUInt
            val beforeCandidates = bankCandidates & bankBeforeBoundary
            val wrappedCandidates = bankCandidates & (~bankBeforeBoundary).asUInt
            val beforeSelectOH = Reverse(
                PriorityEncoderOH(Reverse(beforeCandidates))
            )
            val wrappedSelectOH = Reverse(
                PriorityEncoderOH(Reverse(wrappedCandidates))
            )
            val winnerOH = Mux(beforeCandidates.orR, beforeSelectOH, wrappedSelectOH)

            (winnerOH, winnerOH.orR, (winnerOH & bankBeforeBoundary).orR)
        }

        val (bank0WinnerOH, bank0Valid, bank0BeforeBoundary) = selectBank(0)
        val (bank1WinnerOH, bank1Valid, bank1BeforeBoundary) = selectBank(1)
        val bank1HigherInSameSegment = VecInit((0 until bankDepth).map { index =>
            bank1WinnerOH(index) && bank0WinnerOH(index, 0).orR
        }).asUInt.orR
        val sameSegment =
            (bank0BeforeBoundary && bank1BeforeBoundary) ||
                (!bank0BeforeBoundary && !bank1BeforeBoundary)
        val chooseBank1 = bank1Valid && (
            !bank0Valid ||
                (bank1BeforeBoundary && !bank0BeforeBoundary) ||
                (sameSegment && bank1HigherInSameSegment)
        )
        val bank0WinnerFullOH = VecInit((0 until width).map { index =>
            if (index % 2 == 0) bank0WinnerOH(index / 2) else false.B
        }).asUInt
        val bank1WinnerFullOH = VecInit((0 until width).map { index =>
            if (index % 2 == 1) bank1WinnerOH(index / 2) else false.B
        }).asUInt

        (
            Mux(chooseBank1, bank1WinnerFullOH, bank0WinnerFullOH),
            bank0WinnerOH,
            bank1WinnerOH,
            chooseBank1
        )
    }

    def incWay(ptr: UInt): UInt = {
        Mux(ptr === (DcacheConfig.DcacheWay - 1).U, 0.U, ptr + 1.U)
    }

    def loadKilled(valid: Bool, req: DcachePpReq): Bool = {
        io.flush && valid && isLoad(req)
    }

    def loadPointerLive(indexOH: UInt, indexHigh: Bool): Bool =
        pointerAlive(indexOH, indexHigh, io.ldqValidMask, io.ldqHighMask)

    //==================================== basic info =========================================
    val s1 = RegInit(VecInit(Seq.fill(DcacheConfig.nPorts)(0.U.asTypeOf(new DcacheStage1))))
    val s1ForwardPaddr = RegInit(VecInit(Seq.fill(DcacheConfig.nPorts)(0.U(32.W))))
    val s1ForwardAccessMask = RegInit(VecInit(Seq.fill(DcacheConfig.nPorts)(0.U(accessMaskWidth.W))))
    val s1ForwardBoundaryOH = RegInit(VecInit(Seq.fill(DcacheConfig.nPorts)(0.U(StoreQueueConfig.length.W))))
    val s1ForwardBoundaryHigh = RegInit(VecInit(Seq.fill(DcacheConfig.nPorts)(false.B)))
    dontTouch(s1ForwardPaddr)
    dontTouch(s1ForwardAccessMask)
    dontTouch(s1ForwardBoundaryOH)
    dontTouch(s1ForwardBoundaryHigh)
    val s1Valid = VecInit((0 until DcacheConfig.nPorts).map { port =>
        s1(port).valid && !(isLoad(s1(port).req) && io.flush)
    })

    val s2 = RegInit(VecInit(Seq.fill(DcacheConfig.nPorts)(0.U.asTypeOf(new DcacheStage2))))
    val s2Valid = VecInit((0 until DcacheConfig.nPorts).map { port =>
        s2(port).valid && !(isLoad(s2(port).req) && io.flush)
    })

    val sNormal :: sRestart :: sMaintenanceLookup :: sMaintenanceData :: Nil = Enum(4)
    val ramState = RegInit(sNormal)

    val maintenanceReq = Reg(new DcacheMaintenanceRequest)
    val maintenanceTargetOH = Reg(UInt(DcacheConfig.DcacheWay.W))
    val maintenanceTargetIndex = Reg(UInt(DcacheConfig.DcacheIndex.W))
    val maintenanceLinePaddr = Reg(UInt(32.W))
    val maintenanceRespValid = RegInit(false.B)
    val maintenanceRespBits = Reg(new DcacheMaintenanceResponse)

    io.maintenanceReq.ready :=
        ramState === sNormal &&
        !maintenanceRespValid
    io.maintenanceResp.valid := maintenanceRespValid
    io.maintenanceResp.bits := maintenanceRespBits

    when(io.maintenanceResp.fire) {
        maintenanceRespValid := false.B
    }

    val maintenanceFire = io.maintenanceReq.fire
    when(maintenanceFire) {
        assert(!s1Valid.asUInt.orR && !s2Valid.asUInt.orR,
            "DCache: maintenance must start with an empty demand pipeline")
        assert(!io.mshrIO.resp.valid,
            "DCache: maintenance must not overlap a refill response")
        assert(!io.mainPp.map(_.req.valid).reduce(_ || _),
            "DCache: maintenance must not overlap a demand request")
        maintenanceReq := io.maintenanceReq.bits
    }

    //================================== s1 logic ======================================
    val metaRdata = Seq(
        VecInit(meta.map(_.douta.asTypeOf(new DcacheMeta))),
        VecInit(meta.map(_.doutb.asTypeOf(new DcacheMeta)))
    )

    val s1BaseHits = Wire(Vec(DcacheConfig.nPorts, UInt(DcacheConfig.DcacheWay.W)))
    for (port <- 0 until DcacheConfig.nPorts) {
        val currentSet = getIndex(s1(port).req.paddr)
        val currentValid = VecInit((0 until DcacheConfig.DcacheWay).map { way =>
            validArray(way)(currentSet)
        }).asUInt
        s1BaseHits(port) := VecInit((0 until DcacheConfig.DcacheWay).map { way =>
            currentValid(way) && metaRdata(port)(way).tag === getTag(s1(port).req.paddr)
        }).asUInt
    }

    val s1ForwardFail = Wire(Vec(DcacheConfig.nPorts, new DcacheForwardFail))
    val s1ForwardValid = Wire(Vec(DcacheConfig.nPorts, Bool()))
    val s1ForwardData = Wire(Vec(DcacheConfig.nPorts, UInt(DcacheConfig.DcacheDataBits.W)))
    val s1ForwardSqindex = Wire(Vec(DcacheConfig.nPorts, UInt(StoreQueueConfig.length.W)))
    val s1ForwardSqindexHigh = Wire(Vec(DcacheConfig.nPorts, Bool()))
    val s1ForwardCommitted = Wire(Vec(DcacheConfig.nPorts, Bool()))

    for (port <- 0 until DcacheConfig.nPorts) {
        val req = s1(port).req
        val forwardPaddr = s1ForwardPaddr(port)
        val forwardAccessMask = s1ForwardAccessMask(port)
        val forwardBoundaryOH = s1ForwardBoundaryOH(port)
        val forwardBoundaryHigh = s1ForwardBoundaryHigh(port)
        val forwardPayloadPresent = s1(port).valid && isLoad(req)
        val candidates = Mux(forwardPayloadPresent,
            matchForwardSources(
                req,
                forwardPaddr,
                forwardAccessMask,
                forwardBoundaryOH,
                forwardBoundaryHigh
            ), 0.U)
        val (selectOH, bank0SelectOH, bank1SelectOH, chooseBank1) =
            selectYoungestBanked(candidates, forwardBoundaryOH)
        val legacySelectOH = selectYoungestOH(candidates, forwardBoundaryOH)
        val bank0SelectedSource = Mux1H(bank0SelectOH, VecInit(
            (0 until StoreQueueConfig.length / 2).map { index =>
                io.forwardSources(2 * index)
            }
        ))
        val bank1SelectedSource = Mux1H(bank1SelectOH, VecInit(
            (0 until StoreQueueConfig.length / 2).map { index =>
                io.forwardSources(2 * index + 1)
            }
        ))
        val selectedSource = Mux(chooseBank1, bank1SelectedSource, bank0SelectedSource)
        val bypassOH = VecInit(io.stdBypass.map { bypass =>
            bypass.valid &&
                (bypass.bits.sqindex & selectOH).orR &&
                bypass.bits.sqindexHigh === selectedSource.sqindexHigh
        }).asUInt
        val bypassHit = bypassOH.orR
        val selectedData = Mux(bypassHit,
            Mux1H(bypassOH, io.stdBypass.map(_.bits.data)),
            selectedSource.data)
        val sourceValid = forwardPayloadPresent && selectOH.orR && !req.exception.orR
        val loadMask = forwardAccessMask
        val sourceMask = selectedSource.alignedMask
        val fullCoverage = (sourceMask & loadMask) === loadMask
        val partialOverlap = sourceValid && !fullCoverage
        val waitStoreData = sourceValid && fullCoverage &&
            !selectedSource.dataValid && !bypassHit
        val canForward = sourceValid && fullCoverage &&
            (selectedSource.dataValid || bypassHit)
        val alignedWord = alignForwardData(
            selectedData,
            selectedSource.paddr(1, 0),
            forwardPaddr(1, 0)
        )

        s1ForwardFail(port) := 0.U.asTypeOf(new DcacheForwardFail)
        s1ForwardFail(port).valid := sourceValid && !canForward
        s1ForwardFail(port).waitSqindex := selectedSource.sqindex
        s1ForwardFail(port).waitSqindexHigh := selectedSource.sqindexHigh
        s1ForwardFail(port).partialOverlap := partialOverlap
        s1ForwardFail(port).waitStoreData := waitStoreData

        s1ForwardValid(port) := canForward
        s1ForwardData(port) := alignedWord
        s1ForwardSqindex(port) := selectedSource.sqindex
        s1ForwardSqindexHigh(port) := selectedSource.sqindexHigh
        s1ForwardCommitted(port) := selectedSource.committed

        when(forwardPayloadPresent) {
            assert(PopCount(forwardBoundaryOH) === 1.U,
                s"DCache: channel $port Store boundary must be one-hot")
            assert(selectOH === legacySelectOH,
                s"DCache: channel $port fixed Store priority must match rotating priority")
        }
        assert(PopCount(bypassOH) <= 1.U,
            s"DCache: channel $port has multiple STD bypasses for one Store")
        when(canForward) {
            assert(loadMask.orR,
                s"DCache: channel $port forwarding requires a non-empty Load mask")
            assert(selectedSource.paddr(1, 0) <= forwardPaddr(1, 0),
                s"DCache: channel $port full-cover Store must start before the Load")
        }
        assert(!s1ForwardFail(port).valid || partialOverlap || waitStoreData,
            s"DCache: channel $port forwarding failure must have one replay reason")
    }

    //================================= s2 logic =======================================
    val dataRdata = Seq(
        VecInit(data.map(_.douta)),
        VecInit(data.map(_.doutb))
    )
    val victimDataWords = VecInit((0 until DcacheConfig.DcacheLineWord).map { word =>
        if ((word & 1) == 0) data(word / wordsPerDataBank).douta
        else data(word / wordsPerDataBank).doutb
    })

    val s2Exception = VecInit((0 until DcacheConfig.nPorts).map { port =>
        s2(port).req.exception.orR
    })
    val s2Hit = VecInit((0 until DcacheConfig.nPorts).map { port =>
        s2(port).hits.orR && !s2(port).req.uncache && !s2Exception(port)
    })
    val s2Miss = VecInit((0 until DcacheConfig.nPorts).map { port =>
        s2Valid(port) && !s2Hit(port) && !s2Exception(port)
    })
    val s2ForwardSuccess = VecInit((0 until DcacheConfig.nPorts).map { port =>
        s2Valid(port) && isLoad(s2(port).req) && s2(port).forwardValid
    })
    val s2ForwardFail = VecInit((0 until DcacheConfig.nPorts).map { port =>
        s2Valid(port) && isLoad(s2(port).req) && s2(port).forwardFail.valid
    })
    val s2ForwardMatched = VecInit((0 until DcacheConfig.nPorts).map { port =>
        s2ForwardSuccess(port) || s2ForwardFail(port)
    })

    val s2NormalResponse = VecInit((0 until DcacheConfig.nPorts).map { port =>
        s2Valid(port) && isLoad(s2(port).req) && !s2ForwardMatched(port) &&
            (s2Hit(port) || s2Exception(port))
    })
    val s2ResponseCandidate = VecInit((0 until DcacheConfig.nPorts).map { port =>
        s2ForwardSuccess(port) || s2NormalResponse(port)
    })
    //=============================== refilling logic =====================================
    val restartActive = ramState === sRestart
    val maintenanceLookup = ramState === sMaintenanceLookup
    val maintenanceData = ramState === sMaintenanceData
    val refillWaiting = io.mshrIO.resp.valid
    val refillIndex = getIndex(io.mshrIO.resp.bits.paddr)
    val refillTag = getTag(io.mshrIO.resp.bits.paddr)

    val refillValidVec = VecInit((0 until DcacheConfig.DcacheWay).map { way =>
        validArray(way)(refillIndex)
    }).asUInt
    val refillDirtyVec = VecInit((0 until DcacheConfig.DcacheWay).map { way =>
        dirtyArray(way)(refillIndex)
    }).asUInt
    val refillInvalidWays = ~refillValidVec
    val defaultRefillWayOH = Mux(refillInvalidWays.asUInt.orR,
        PriorityEncoderOH(refillInvalidWays),
        UIntToOH(replacePtr(refillIndex), DcacheConfig.DcacheWay))

    val refillS2HitOH = (0 until DcacheConfig.nPorts).map { port =>
        Mux(s2Valid(port) &&
            getLineAddr(s2(port).req.paddr) === getLineAddr(io.mshrIO.resp.bits.paddr),
            s2(port).hits, 0.U)
    }.reduce(_ | _)
    val refillWayOH = defaultRefillWayOH
    val refillVictimValid = (refillWayOH & refillValidVec).orR
    val refillVictimDirty = (refillWayOH & refillDirtyVec).orR
    val victimWayReg = RegInit(0.U(DcacheConfig.DcacheWay.W))
    val victimIndexReg = RegInit(0.U(DcacheConfig.DcacheIndex.W))
    val victimValidReg = RegInit(false.B)
    val victimDirtyReg = RegInit(false.B)

    val refillS2LineMatch = VecInit((0 until DcacheConfig.nPorts).map { port =>
        s2Valid(port) &&
            getLineAddr(s2(port).req.paddr) === getLineAddr(io.mshrIO.resp.bits.paddr)
    })
    val refillS2VictimMatch = VecInit((0 until DcacheConfig.nPorts).map { port =>
        s2Valid(port) && !refillS2LineMatch(port) &&
            getIndex(s2(port).req.paddr) === refillIndex &&
            (s2(port).hits & refillWayOH).orR
    })
    io.mshrIO.resp.ready := ramState === sNormal && io.mshrIO.victimAvailable
    val refillFire = io.mshrIO.resp.fire
    // The registered maintenance request is accepted only after S1/S2 drain.
    // ramState blocks demand traffic from the following cycle onward; feeding
    // maintenanceFire back into a current S2 response creates a false common-
    // case StoreQueue-to-wakeup timing path.
    // Demand traffic only yields the RAM/result lanes on the refill write and
    // synchronous restart cycles. MSHR keeps a pending refill stable while
    // victim capacity is unavailable.
    val globalRamStall = ramState =/= sNormal || refillFire
    val refillS2StoreCollision = refillFire && refillS2LineMatch(0) &&
        s2Miss(0) && isStore(s2(0).req)

    val s2BaseRun = VecInit((0 until DcacheConfig.nPorts).map { port =>
        !globalRamStall
    })
    val needsMshrReq = Wire(Vec(DcacheConfig.nPorts, Bool()))
    val mshrReqFire = Wire(Vec(DcacheConfig.nPorts, Bool()))
    for (port <- 0 until DcacheConfig.nPorts) {
        needsMshrReq(port) := s2BaseRun(port) && s2Miss(port) &&
            !s2ForwardMatched(port)
    }

    def mshrLoadMetadata(req: DcachePpReq): MshrLoadMetadata = {
        val metadata = Wire(new MshrLoadMetadata)
        metadata.token := OHToUInt(req.ldindex)
        metadata.robPtr := req.robPtr
        metadata.pdest := req.pdest
        metadata.ldindexHigh := req.ldindexHigh
        metadata.format := MshrLoadFormat.encode(req.mask, req.signed)
        metadata.vaddrVpn := req.pc(31, MshrLoadWaiterConfig.pageOffsetWidth)
        metadata
    }

    val port0StoreMiss = isStore(s2(0).req)
    io.mshrIO.req0.valid := needsMshrReq(0)
    io.mshrIO.req0.bits.linePaddr :=
        s2(0).req.paddr(31, DcacheConfig.DcacheOffset)
    io.mshrIO.req0.bits.byteOffset := getOffset(s2(0).req.paddr)
    io.mshrIO.req0.bits.loadMetadata := mshrLoadMetadata(s2(0).req)
    io.mshrIO.req0.bits.store := port0StoreMiss
    io.mshrIO.req0.bits.storeData := Mux(port0StoreMiss,
        s2(0).req.operateData, 0.U)
    io.mshrIO.req0.bits.storeMask := Mux(port0StoreMiss,
        s2(0).req.mask, 0.U)
    io.mshrIO.req0.bits.sqindex := Mux(port0StoreMiss,
        s2(0).req.sqindex, 0.U)
    io.mshrIO.req0.bits.sqindexHigh :=
        port0StoreMiss && s2(0).req.sqindexHigh

    io.mshrIO.req1.valid := needsMshrReq(1)
    io.mshrIO.req1.bits.linePaddr :=
        s2(1).req.paddr(31, DcacheConfig.DcacheOffset)
    io.mshrIO.req1.bits.byteOffset := getOffset(s2(1).req.paddr)
    io.mshrIO.req1.bits.loadMetadata := mshrLoadMetadata(s2(1).req)

    mshrReqFire(0) := io.mshrIO.req0.fire
    mshrReqFire(1) := io.mshrIO.req1.fire
    for (port <- 0 until DcacheConfig.nPorts) {
        when(mshrReqFire(port) && isLoad(s2(port).req)) {
            assert(PopCount(s2(port).req.ldindex) === 1.U,
                s"DCache: MSHR Load request on port $port must carry one ldindex")
            assert(s2(port).req.rfWen === s2(port).req.pdest.orR,
                s"DCache: MSHR Load request on port $port must derive rfWen from pdest")
            assert(s2(port).req.mask === "b0001".U ||
                s2(port).req.mask === "b0011".U ||
                s2(port).req.mask === "b1111".U,
                s"DCache: MSHR Load request on port $port must carry a legal format")
        }
    }
    val mshrReqReady = VecInit(io.mshrIO.req0.ready, io.mshrIO.req1.ready)
    // Store admission is intentionally independent of the Load waiter-ready
    // cone. A rejected Store is withdrawn through the SQ retry boundary while
    // Loads already in S1 and at the entrance continue to advance.
    val storeMshrFullRetry = needsMshrReq(0) && isStore(s2(0).req) &&
        !io.mshrIO.storeAdmissionReady
    val storeMissAccepted = mshrReqFire(0) && isStore(s2(0).req)
    val s1StoreMshrFullKill = storeMshrFullRetry && s1Valid(0) &&
        isStore(s1(0).req)

    val channelStall = VecInit.fill(DcacheConfig.nPorts)(globalRamStall)

    val loadNormalMiss = Wire(Vec(DcacheConfig.nPorts, Bool()))
    val loadMshrFull = Wire(Vec(DcacheConfig.nPorts, Bool()))
    for (port <- 0 until DcacheConfig.nPorts) {
        val ownAccepted = mshrReqFire(port) && isLoad(s2(port).req)
        val ownFull = needsMshrReq(port) && isLoad(s2(port).req) &&
            !mshrReqReady(port)
        loadNormalMiss(port) := s2BaseRun(port) && s2Miss(port) &&
            !s2ForwardMatched(port) && ownAccepted
        loadMshrFull(port) := ownFull

        val storeReplay = s2BaseRun(port) && s2ForwardFail(port)
        io.loadStoreFail(port) := 0.U.asTypeOf(new DcacheLoadFailBus)
        when(storeReplay) {
            io.loadStoreFail(port).valid := s2(port).forwardFail.valid
            io.loadStoreFail(port).inst := s2(port).req
            io.loadStoreFail(port).waitSqindex := s2(port).forwardFail.waitSqindex
            io.loadStoreFail(port).waitSqindexHigh :=
                s2(port).forwardFail.waitSqindexHigh
            io.loadStoreFail(port).storeData := s2(port).forwardFail.valid
            io.loadStoreFail(port).partialOverlap :=
                s2(port).forwardFail.partialOverlap
            io.loadStoreFail(port).waitStoreData :=
                s2(port).forwardFail.waitStoreData
        }

        io.loadMshrFail(port) := 0.U.asTypeOf(new DcacheLoadFailBus)
        when(!storeReplay && loadMshrFull(port)) {
            io.loadMshrFail(port).valid := true.B
            io.loadMshrFail(port).inst := s2(port).req
            io.loadMshrFail(port).mshrFull := true.B
        }
    }

    val s2ResponseFire = VecInit((0 until DcacheConfig.nPorts).map { port =>
        !channelStall(port) && s2ResponseCandidate(port)
    })
    val s1DataRead = VecInit((0 until DcacheConfig.nPorts).map { port =>
        !channelStall(port) && s1Valid(port) && isLoad(s1(port).req) &&
            s1BaseHits(port).orR && !s1(port).req.uncache &&
            !s1(port).req.exception.orR
    })
    val s1StoreLine = singleStoreLine(s1(0).req)
    val s1StoreWrite = !channelStall(0) && !s1StoreMshrFullKill && s1Valid(0) &&
        isStore(s1(0).req) && s1BaseHits(0).orR &&
        !s1(0).req.uncache && !s1(0).req.exception.orR
    val s1StoreWriteIndex = getIndex(s1(0).req.paddr)
    val storeWriteSuccessByBank = Wire(Vec(dataBankCount, Bool()))
    val storeWriteSuccess = storeWriteSuccessByBank.asUInt.orR

    val maintenanceIndex = getIndex(maintenanceReq.paddr)
    val maintenanceValidVec = VecInit((0 until DcacheConfig.DcacheWay).map { way =>
        validArray(way)(maintenanceIndex)
    }).asUInt
    val maintenanceHitOH = VecInit((0 until DcacheConfig.DcacheWay).map { way =>
        maintenanceValidVec(way) &&
            metaRdata(0)(way).tag === getTag(maintenanceReq.paddr)
    }).asUInt
    val maintenanceSelectedOH = Mux(
        maintenanceReq.indexOnly,
        UIntToOH(maintenanceReq.way, DcacheConfig.DcacheWay),
        maintenanceHitOH
    )
    val maintenanceSelectedValidOH = maintenanceSelectedOH & maintenanceValidVec
    val maintenanceSelectedDirty = VecInit((0 until DcacheConfig.DcacheWay).map { way =>
        maintenanceSelectedValidOH(way) && dirtyArray(way)(maintenanceIndex)
    }).asUInt.orR
    val maintenanceNeedsWriteback =
        maintenanceLookup &&
        maintenanceReq.writeback &&
        maintenanceSelectedDirty
    val maintenanceDataRead = maintenanceNeedsWriteback

    //========================== simple predict wake bus =================================
    def loadPredictInfo(req: DcachePpReq): LoadPredictInfo = {
        val info = Wire(new LoadPredictInfo)
        info.pdest := req.pdest
        info.ldindex := req.ldindex
        info.ldindexHigh := req.ldindexHigh
        info.robPtr := req.robPtr
        info
    }

    val loadPredWakeReg = RegInit(VecInit.fill(DcacheConfig.nPorts)(
        0.U.asTypeOf(Valid(new LoadPredictInfo))
    ))
    val loadPredSuccessNow = Wire(Vec(DcacheConfig.nPorts, Bool()))

    for (port <- 0 until DcacheConfig.nPorts) {
        val s1PredictionLive = loadPointerLive(
            s1(port).req.ldindex,
            s1(port).req.ldindexHigh
        )
        val registeredPredictionLive = loadPointerLive(
            loadPredWakeReg(port).bits.ldindex,
            loadPredWakeReg(port).bits.ldindexHigh
        )
        val wakeNow =
            s1Valid(port) &&
            isLoad(s1(port).req) &&
            s1PredictionLive &&
            !s1(port).req.uncache &&
            !s1(port).req.exception.orR &&
            s1(port).req.rfWen &&
            s1(port).req.pdest =/= 0.U &&
            !channelStall(port)

        val resolveIdentityMatches =
            s2Valid(port) &&
            isLoad(s2(port).req) &&
            s2(port).req.pdest === loadPredWakeReg(port).bits.pdest &&
            s2(port).req.ldindex === loadPredWakeReg(port).bits.ldindex &&
            s2(port).req.ldindexHigh === loadPredWakeReg(port).bits.ldindexHigh &&
            s2(port).req.robPtr.asUInt === loadPredWakeReg(port).bits.robPtr.asUInt

        loadPredSuccessNow(port) :=
            loadPredWakeReg(port).valid &&
            registeredPredictionLive &&
            resolveIdentityMatches &&
            s2ResponseFire(port) &&
            !s2Exception(port)

        when(io.flush) {
            loadPredWakeReg(port) := 0.U.asTypeOf(loadPredWakeReg(port))
        }.otherwise {
            loadPredWakeReg(port).valid := wakeNow
            when(wakeNow) {
                loadPredWakeReg(port).bits := loadPredictInfo(s1(port).req)
            }
        }

        // Wake speculatively from S1 without waiting for the tag BRAM.  S2
        // confirms a hit/forward or cancels the poisoned IQ source on a miss.
        io.loadPredWake(port).valid := wakeNow && !io.flush
        io.loadPredWake(port).bits := loadPredictInfo(s1(port).req)
        io.loadPredResolve(port).valid :=
            loadPredWakeReg(port).valid &&
            !io.flush
        io.loadPredResolve(port).bits.info := loadPredWakeReg(port).bits
        io.loadPredResolve(port).bits.success := loadPredSuccessNow(port)

        when(io.loadPredResolve(port).valid) {
            assert(PopCount(loadPredWakeReg(port).bits.ldindex) === 1.U)
            when(registeredPredictionLive) {
                assert(resolveIdentityMatches,
                    s"DCache: channel $port predictive wake lost its S2 request")
            }

        }
        when(io.loadPredWake(port).valid) {
            assert(s1PredictionLive,
                s"DCache: channel $port exported a stale predictive wake")
        }
    }

    //========================== store complete signal ===============================
    val storeCompleteValid = RegNext(storeWriteSuccess, false.B)
    // The payload is unobservable while valid is low.  Keep reset only on the
    // validity bit so a cache hit cannot drive the payload registers' reset pins.
    val storeCompleteBits = Reg(new StoreReadyEvent)
    when(storeWriteSuccess) {
        storeCompleteBits.paddr := s1(0).req.paddr
        storeCompleteBits.sqindex := s1(0).req.sqindex
        storeCompleteBits.sqindexHigh := s1(0).req.sqindexHigh
        storeCompleteBits.sqMask := s1StoreLine.contributors
        storeCompleteBits.sqHighMask := Mux(
            s1(0).req.sqindexHigh,
            s1StoreLine.contributors,
            0.U
        )
    }

    io.storeComplete.valid := storeCompleteValid
    io.storeComplete.bits := storeCompleteBits

    //=============================== pipeline entrance control ==========================
    val reqFire = Wire(Vec(DcacheConfig.nPorts, Bool()))
    val enterS1 = Wire(Vec(DcacheConfig.nPorts, Bool()))
    val storeRetryReg = RegInit(0.U.asTypeOf(Valid(new DcacheStoreRetryBus)))
    val entranceStoreRetry = WireDefault(false.B)
    storeRetryReg := 0.U.asTypeOf(Valid(new DcacheStoreRetryBus))
    io.storeRetry := storeRetryReg
    io.idle :=
        !s1Valid.asUInt.orR &&
        !s2Valid.asUInt.orR &&
        ramState === sNormal &&
        !storeCompleteValid &&
        !maintenanceRespValid

    for (port <- 0 until DcacheConfig.nPorts) {
        io.requestAvailable(port) := ramState === sNormal &&
            !maintenanceRespValid && !refillFire
        io.mainPp(port).req.ready := io.requestAvailable(port)
        reqFire(port) := io.mainPp(port).req.fire
        val entranceStoreKill = if (port == 0) {
            storeMshrFullRetry && reqFire(port) &&
                isStore(io.mainPp(port).req.bits)
        } else {
            false.B
        }
        if (port == 0) {
            entranceStoreRetry := entranceStoreKill
        }
        enterS1(port) := reqFire(port) && !entranceStoreKill
        assert(PopCount(VecInit(
            enterS1(port), entranceStoreKill
        )) === reqFire(port).asUInt,
            s"DCache: channel $port request must enter S1 or be Store-squashed")
        when(reqFire(port)) {
            assert(!channelStall(port),
                s"DCache: channel $port accepted a request during a global RAM stall")
            assert(!io.mainPp(port).req.bits.uncache,
                s"DCache: channel $port received an uncache request")
            if (port == 1) {
                assert(!isStore(io.mainPp(port).req.bits),
                    "DCache: channel 1 received a Store request")
            }
        }
    }

    val s2StoreRetry = refillS2StoreCollision || storeMshrFullRetry
    val s1StoreRetry = s2StoreRetry && s1Valid(0) && isStore(s1(0).req)
    storeRetryReg.valid := s2StoreRetry
    storeRetryReg.bits.sqindex := s2(0).req.sqindex
    storeRetryReg.bits.sqindexHigh := s2(0).req.sqindexHigh

    when(s2StoreRetry) {
        assert(PopCount(s2(0).req.sqindex) === 1.U,
            "DCache: S2 Store retry boundary must identify one SQ entry")
    }
    when(s1StoreRetry) {
        assert(PopCount(s1(0).req.sqindex) === 1.U,
            "DCache: S1 Store retry must identify one SQ entry")
    }
    when(entranceStoreRetry) {
        assert(PopCount(io.mainPp(0).req.bits.sqindex) === 1.U,
            "DCache: entrance Store squash must identify one SQ entry")
    }

    //=============================== miss mshr req & dirty write back handshake & miss FSM======================
    val restartVictimValid = restartActive && victimValidReg && victimDirtyReg
    val maintenanceVictimValid =
        maintenanceData && io.mshrIO.victimAvailable && !maintenanceRespValid
    io.mshrIO.victimReq.valid := restartVictimValid || maintenanceVictimValid
    io.mshrIO.victimReq.bits.paddr := Mux(
        maintenanceVictimValid,
        maintenanceLinePaddr,
        Mux1H(victimWayReg, metaRdata(0).map(_.tag)) ## victimIndexReg ##
            0.U(DcacheConfig.DcacheOffset.W)
    )
    io.mshrIO.victimReq.bits.data := Cat(victimDataWords.reverse)

    for (way <- 0 until DcacheConfig.DcacheWay) {
        when(refillFire && refillWayOH(way)) {
            validArray(way)(refillIndex) := true.B
            dirtyArray(way)(refillIndex) := io.mshrIO.resp.bits.dirty
        }
        when(s1StoreWrite && s1BaseHits(0)(way)) {
            validArray(way)(s1StoreWriteIndex) := true.B
            dirtyArray(way)(s1StoreWriteIndex) := true.B
        }
        when(
            maintenanceLookup &&
            !maintenanceNeedsWriteback &&
            maintenanceReq.invalidate &&
            maintenanceSelectedValidOH(way)
        ) {
            validArray(way)(maintenanceIndex) := false.B
            dirtyArray(way)(maintenanceIndex) := false.B
        }
        when(
            maintenanceVictimValid &&
            maintenanceReq.invalidate &&
            maintenanceTargetOH(way)
        ) {
            validArray(way)(maintenanceTargetIndex) := false.B
            dirtyArray(way)(maintenanceTargetIndex) := false.B
        }
    }

    when(refillFire) {
        replacePtr(refillIndex) := incWay(OHToUInt(refillWayOH))
        victimWayReg := refillWayOH
        victimIndexReg := refillIndex
        victimValidReg := refillVictimValid
        victimDirtyReg := refillVictimDirty
    }

    when(maintenanceLookup) {
        when(maintenanceNeedsWriteback) {
            maintenanceTargetOH := maintenanceSelectedValidOH
            maintenanceTargetIndex := maintenanceIndex
            maintenanceLinePaddr :=
                Mux1H(maintenanceSelectedValidOH, metaRdata(0).map(_.tag)) ##
                    maintenanceIndex ## 0.U(DcacheConfig.DcacheOffset.W)
            ramState := sMaintenanceData
        }.otherwise {
            maintenanceRespBits := 0.U.asTypeOf(maintenanceRespBits)
            maintenanceRespValid := true.B
            ramState := sNormal
        }
    }.elsewhen(maintenanceData) {
        when(maintenanceVictimValid) {
            maintenanceRespBits := 0.U.asTypeOf(maintenanceRespBits)
            maintenanceRespBits.writeback.valid := true.B
            maintenanceRespBits.writeback.bits.paddr := maintenanceLinePaddr
            maintenanceRespBits.writeback.bits.data := Cat(victimDataWords.reverse)
            maintenanceRespValid := true.B
            ramState := sNormal
        }
    }.elsewhen(maintenanceFire) {
        ramState := sMaintenanceLookup
    }.elsewhen(restartActive) {
        when(victimValidReg && victimDirtyReg) {
            assert(io.mshrIO.victimAvailable,
                "DCache: WBB capacity confirmed at refill must remain available in sRestart")
        }
        ramState := sNormal
        victimValidReg := false.B
        victimDirtyReg := false.B
    }.elsewhen(refillFire) {
        ramState := sRestart
    }

    //============================= bram wire connexion ====================================
    for (way <- 0 until DcacheConfig.DcacheWay) {
        val refillWrite = refillFire && refillWayOH(way)
        val refillVictimRead = refillFire && refillWayOH(way)
        val restartMetaRead0 = restartActive && s1Valid(0)
        val restartMetaRead1 = restartActive && s1Valid(1)
        val metaWrite = Wire(new DcacheMeta)
        metaWrite.tag := refillTag

        meta(way).clka := clock
        meta(way).addra := Mux(refillFire, refillIndex,
            Mux(maintenanceFire, getIndex(io.maintenanceReq.bits.paddr),
                Mux(restartActive, getIndex(s1(0).req.paddr), getIndex(io.mainPp(0).req.bits.paddr))))
        meta(way).addrb := Mux(refillFire, refillIndex,
            Mux(restartActive, getIndex(s1(1).req.paddr), getIndex(io.mainPp(1).req.bits.paddr)))
        meta(way).dina := 0.U
        meta(way).dinb := metaWrite.asUInt
        meta(way).wea := false.B
        meta(way).web := refillWrite
        meta(way).ena := enterS1(0) || restartMetaRead0 || refillVictimRead ||
            maintenanceFire
        meta(way).enb := enterS1(1) || restartMetaRead1 || refillWrite
    }

    val refillDataAddr = Cat(refillIndex, OHToUInt(refillWayOH))
    val s1DataAddr0 = Cat(getIndex(s1(0).req.paddr), OHToUInt(s1BaseHits(0)))
    val s1DataAddr1 = Cat(getIndex(s1(1).req.paddr), OHToUInt(s1BaseHits(1)))
    val restartDataAddr0 = Cat(getIndex(s2(0).req.paddr), OHToUInt(s2(0).hits))
    val restartDataAddr1 = Cat(getIndex(s2(1).req.paddr), OHToUInt(s2(1).hits))
    val refillEvenDataAddr = Cat(refillDataAddr, 0.U(1.W))
    val refillOddDataAddr = Cat(refillDataAddr, 1.U(1.W))
    val s1DataBank0 = getDataBank(s1(0).req.paddr)
    val s1DataBank1 = getDataBank(s1(1).req.paddr)
    val restartDataBank0 = getDataBank(s2(0).req.paddr)
    val restartDataBank1 = getDataBank(s2(1).req.paddr)
    val s1DataRow0 = Cat(s1DataAddr0, getDataRowBit(s1(0).req.paddr))
    val s1DataRow1 = Cat(s1DataAddr1, getDataRowBit(s1(1).req.paddr))
    val restartDataRow0 = Cat(restartDataAddr0, getDataRowBit(s2(0).req.paddr))
    val restartDataRow1 = Cat(restartDataAddr1, getDataRowBit(s2(1).req.paddr))
    val maintenanceDataAddr = Cat(maintenanceIndex, OHToUInt(maintenanceSelectedValidOH))
    val maintenanceEvenDataRow = Cat(maintenanceDataAddr, 0.U(1.W))
    val maintenanceOddDataRow = Cat(maintenanceDataAddr, 1.U(1.W))
    val restartDataRead0 = restartActive && s2Valid(0) &&
        isLoad(s2(0).req) && s2(0).hits.orR
    val restartDataRead1 = restartActive && s2Valid(1) &&
        isLoad(s2(1).req) && s2(1).hits.orR

    for (bank <- 0 until dataBankCount) {
        val evenWord = bank * wordsPerDataBank
        val oddWord = evenWord + 1
        val evenDataLo = evenWord * DcacheConfig.DcacheDataBits
        val evenDataHi = evenDataLo + DcacheConfig.DcacheDataBits - 1
        val oddDataLo = oddWord * DcacheConfig.DcacheDataBits
        val oddDataHi = oddDataLo + DcacheConfig.DcacheDataBits - 1
        val evenMaskLo = evenWord * DcacheConfig.DcacheMaskBits
        val evenMaskHi = evenMaskLo + DcacheConfig.DcacheMaskBits - 1
        val oddMaskLo = oddWord * DcacheConfig.DcacheMaskBits
        val oddMaskHi = oddMaskLo + DcacheConfig.DcacheMaskBits - 1
        val s1BankSelected0 = s1DataBank0 === bank.U(dataBankIndexWidth.W)
        val s1BankSelected1 = s1DataBank1 === bank.U(dataBankIndexWidth.W)
        val restartBankSelected0 = restartDataBank0 === bank.U(dataBankIndexWidth.W)
        val restartBankSelected1 = restartDataBank1 === bank.U(dataBankIndexWidth.W)
        val storeData = Mux(
            getDataRowBit(s1(0).req.paddr),
            s1StoreLine.data(oddDataHi, oddDataLo),
            s1StoreLine.data(evenDataHi, evenDataLo)
        )
        val storeMask = Mux(
            getDataRowBit(s1(0).req.paddr),
            s1StoreLine.mask(oddMaskHi, oddMaskLo),
            s1StoreLine.mask(evenMaskHi, evenMaskLo)
        )

        data(bank).clka := clock
        data(bank).addra := Mux(refillFire, refillEvenDataAddr,
            Mux(maintenanceDataRead, maintenanceEvenDataRow,
                Mux(restartActive, restartDataRow0, s1DataRow0)))
        data(bank).addrb := Mux(refillFire, refillOddDataAddr,
            Mux(maintenanceDataRead, maintenanceOddDataRow,
                Mux(restartActive, restartDataRow1, s1DataRow1)))
        data(bank).dina := Mux(
            refillFire,
            io.mshrIO.resp.bits.data(evenDataHi, evenDataLo),
            storeData
        )
        data(bank).dinb := io.mshrIO.resp.bits.data(oddDataHi, oddDataLo)
        val dataPortAWriteMask = Mux(
            refillFire,
            Fill(DcacheConfig.DcacheMaskBits, 1.U(1.W)),
            Mux(s1StoreWrite && s1BankSelected0,
                storeMask, 0.U(DcacheConfig.DcacheMaskBits.W))
        )
        data(bank).web := Fill(DcacheConfig.DcacheMaskBits, refillFire)
        val dataPortAEnable = refillFire || maintenanceDataRead ||
            Mux(restartActive,
                restartBankSelected0 && restartDataRead0,
                s1BankSelected0 && (s1DataRead(0) || s1StoreWrite))
        data(bank).wea := dataPortAWriteMask
        data(bank).ena := dataPortAEnable
        data(bank).enb := refillFire || maintenanceDataRead ||
            Mux(restartActive,
                restartBankSelected1 && restartDataRead1,
                s1BankSelected1 && s1DataRead(1))
        storeWriteSuccessByBank(bank) :=
            s1StoreWrite && ramState === sNormal &&
                dataPortAEnable && dataPortAWriteMask.orR
    }

    //====================== pipeline registers =============================
    for (port <- 0 until DcacheConfig.nPorts) {
        when(!channelStall(port)) {
            val s1StoreKill = if (port == 0) s1StoreMshrFullKill else false.B
            val s1Advances = s1Valid(port) && !s1StoreKill
            s2(port).valid := s1Advances
            when(s1Advances) {
                s2(port).req := s1(port).req
                s2(port).storeLine := singleStoreLine(s1(port).req)
                s2(port).hits := s1BaseHits(port)
                s2(port).forwardFail := s1ForwardFail(port)
                s2(port).forwardValid := s1ForwardValid(port)
                s2(port).forwardData := s1ForwardData(port)
                s2(port).forwardSqindex := s1ForwardSqindex(port)
                s2(port).forwardSqindexHigh := s1ForwardSqindexHigh(port)
                s2(port).forwardCommitted := s1ForwardCommitted(port)
            }

            s1(port).valid := enterS1(port)
            when(enterS1(port)) {
                s1(port).req := io.mainPp(port).req.bits
                s1ForwardPaddr(port) := io.mainPp(port).req.bits.paddr
                s1ForwardAccessMask(port) := accessMask(
                    io.mainPp(port).req.bits.paddr,
                    io.mainPp(port).req.bits.mask
                )
                s1ForwardBoundaryOH(port) := io.mainPp(port).req.bits.sqindex
                s1ForwardBoundaryHigh(port) := io.mainPp(port).req.bits.sqindexHigh
            }
        }.otherwise {
            when(loadKilled(s1(port).valid, s1(port).req)) {
                s1(port).valid := false.B
            }
            when(loadKilled(s2(port).valid, s2(port).req)) {
                s2(port).valid := false.B
            }
        }

        // Loads still consume the corrected S2 hit result after a refill.
        // Store hits are final in S1, so reclassifying their retained S2 copy
        // can enqueue an already-completed Store into the MSHR a second time.
        when(refillFire && s2Valid(port) && isLoad(s2(port).req)) {
            when(refillS2LineMatch(port)) {
                s2(port).hits := refillWayOH
            }.elsewhen(refillS2VictimMatch(port)) {
                s2(port).hits := 0.U
            }
        }
        if (port == 0) {
            // A refill collision uses the global hold path. Remove only the
            // colliding Stores; a held S1 Load must remain resident. MSHR-full
            // Store squash was handled by the normal advance assignments above.
            when(refillS2StoreCollision) {
                s2(port).valid := false.B
            }
            when(refillS2StoreCollision && s1Valid(port) && isStore(s1(port).req)) {
                s1(port).valid := false.B
            }
        }
    }

    //=========================== pipeline output interface ===============================
    for (port <- 0 until DcacheConfig.nPorts) {
        val cacheWord = dataRdata(port)(getDataBank(s2(port).req.paddr))
        val alignedCacheWord = (cacheWord >> (s2(port).req.paddr(1, 0) << 3))(
            DcacheConfig.DcacheDataBits - 1, 0)
        val selectedWord = Mux(s2ForwardSuccess(port),
            s2(port).forwardData,
            alignedCacheWord)
        io.mainPp(port).resp.valid := !globalRamStall && s2ResponseCandidate(port)
        io.mainPp(port).resp.bits.inst := s2(port).req
        io.mainPp(port).resp.bits.vaddr := s2(port).req.pc
        io.mainPp(port).resp.bits.paddr := s2(port).req.paddr
        io.mainPp(port).resp.bits.exception := s2(port).req.exception
        io.mainPp(port).resp.bits.rdata := Mux(s2(port).req.exception.orR,
            0.U,
            extendLoadData(
                selectedWord,
                s2(port).req.mask,
                s2(port).req.signed
            ))
        io.mainPp(port).resp.bits.forwarded := s2ForwardSuccess(port)
        io.mainPp(port).resp.bits.forwardSqindex := Mux(s2ForwardSuccess(port),
            s2(port).forwardSqindex, 0.U)
        io.mainPp(port).resp.bits.forwardSqindexHigh := s2ForwardSuccess(port) &&
            s2(port).forwardSqindexHigh
        io.mainPp(port).resp.bits.forwardCommitted := s2ForwardSuccess(port) &&
            s2(port).forwardCommitted
        io.mainPp(port).resp.bits.predictReserved := loadPredSuccessNow(port)

        when(s1Valid(port) && ramState === sNormal && !refillFire) {
            assert(PopCount(s1BaseHits(port)) <= 1.U,
                s"DCache: channel $port: multiple hits")
        }
        when(s2Valid(port)) {
            assert(PopCount(s2(port).hits) <= 1.U,
                s"DCache: channel $port: S2 hits must be one-hot")
            assert(!(s2ForwardSuccess(port) && s2ForwardFail(port)),
                s"DCache: channel $port: forwarding cannot succeed and fail together")
        }
    }

    when(refillWaiting) {
        assert(!refillS2HitOH.orR,
            "DCache: refill line must not already be resident")
    }
    when(refillFire) {
        assert(refillEvenDataAddr =/= refillOddDataAddr,
            "DCache: folded refill ports must target distinct words")
    }
    assert(!(restartVictimValid && maintenanceVictimValid),
        "DCache: refill victim and maintenance victim must not be emitted together")
    when(maintenanceDataRead) {
        assert(!refillFire && !restartActive,
            "DCache: maintenance data read must not overlap refill or restart")
    }
    when(maintenanceVictimValid) {
        assert(io.mshrIO.victimAvailable,
            "DCache: maintenance victim requires available WBB capacity")
    }

    //================================ perf counter ===========================================
    io.perf.loadHitCount := PopCount(VecInit((0 until DcacheConfig.nPorts).map { port =>
        s2ResponseFire(port) && s2Hit(port) && !s2ForwardMatched(port)
    }))
    io.perf.loadNormalMissCount := PopCount(loadNormalMiss)
    io.perf.loadMshrFullCount := PopCount(loadMshrFull)
    io.perf.storeHitCount := s1StoreWrite.asUInt
    io.perf.storeMissCount := storeMissAccepted.asUInt
    io.perf.storeRetryCount := PopCount(VecInit(
        s2StoreRetry,
        s1StoreRetry,
        entranceStoreRetry
    ))
    io.perf.storeMshrFullRetryCount := storeMshrFullRetry.asUInt
    io.perf.storeRefillRetryCount := refillS2StoreCollision.asUInt
    io.perf.storeEntranceRetryCount := entranceStoreRetry.asUInt
}
