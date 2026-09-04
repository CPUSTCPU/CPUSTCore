package CPUSTC.backend.rob

import chisel3._
import chisel3.util._

import CPUSTC.config.Decode._
import CPUSTC.config.Commit._
import CPUSTC.config.MemoryException._
import CPUSTC.config.WritebackConfig._
import CPUSTC.utils.ClusterIndexFIFO
import CPUSTC.config.{MaskLower, ShiftAdd1}
import CPUSTC.config.EXEOp._
import CPUSTC.config.FunctionUnit.{FU_ALU, FU_CNT, FU_MUL, FU_SYS}

class RobPayload extends Bundle {
    val pc        = UInt(32.W)
    val instr     = UInt(32.W)
    val ftqPtr    = new CPUSTC.frontend.FtqPtr
    val ftqOffset = UInt(log2Ceil(CPUSTC.config.Fetch.nfch).W)
    val ftqLast   = Bool()

    val uop    = UInt(OP_SZ.W)
    val fuType = UInt(CPUSTC.config.FunctionUnit.FUC_SZ.W)
    val fastFixedInt = Bool()

    val ldest      = UInt(CPUSTC.config.RegisterFile.wlreg.W)
    val pdest      = UInt(CPUSTC.config.RegisterFile.wpreg.W)
    val pprd       = UInt(CPUSTC.config.RegisterFile.wpreg.W)
    val ldestValid = Bool()
    val rfWen      = Bool()

    val isLoad  = Bool()
    val isStore = Bool()
    val isBr    = Bool()
    val isBl    = Bool()
    val isJirl  = Bool()
}

class RobExceptionRecord extends Bundle {
    val robPtr    = new RobPtr
    val cause     = UInt(8.W)
    val badvValid = Bool()
    val badv      = UInt(CPUSTC.config.RegisterFile.dataWidth.W)
}

class Rob(
    maxCommitPerCycle: Int = ncmt,
    enableDebug: Boolean = false
) extends Module {
    val io = IO(new RobIO)

    require(ndcd == ncmt)
    require(maxCommitPerCycle > 0 && maxCommitPerCycle <= ncmt)

    // ClusterIndexFIFO remains the ordering, occupancy and tail-restore engine.
    // Wide ROB state is stored separately so seven completion ports do not turn
    // every payload bit into a multi-written register.
    val q = Module(new ClusterIndexFIFO(
        Bool(),
        nrob,
        ndcd,
        ncmt,
        0,
        0,
        supportTailRestore = true
    ))

    val payloadBanks = Seq.tabulate(ndcd) { bank =>
        val mem = Mem(nrobQ, new RobPayload)
        mem.suggestName(s"payloadBank_$bank")
        mem
    }

    // Random completion validation still uses metaEpoch. Retirement uses a
    // private bank-local FF copy so the heavily distributed validation array
    // does not feed the dequeue-control cone. Store the copy as bit planes:
    // updating a whole plane prevents Vivado from inferring an asynchronous
    // LUTRAM whose address would sit in series with the registered ROB head.
    val retirementEpochPlanes = Seq.tabulate(ndcd) { bank =>
        val planes = Reg(Vec(RobConfig.epochBits, UInt(nrobQ.W)))
        planes.suggestName(s"retirementEpochPlanes_$bank")
        planes
    }

    val completeState = RegInit(
        VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B))
    )
    val branchResolvedState = RegInit(
        VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B))
    )
    val storeState = RegInit(
        VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B))
    )
    val loadState = RegInit(
        VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B))
    )
    val commitBoundaryState = RegInit(
        VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B))
    )
    val completionData =
        Reg(Vec(nrobQ, Vec(ndcd, UInt(CPUSTC.config.RegisterFile.dataWidth.W))))

    // Valid qidx values are 0, 1 and 2. Cat leaves one unused rank between
    // rows, but remains strictly monotonic in physical ROB slot order.
    def robRank(ptr: RobPtr): UInt = Cat(ptr.offset, ptr.qidx)

    def isOlder(
        leftHigh: Bool,
        leftIndex: UInt,
        rightHigh: Bool,
        rightIndex: UInt
    ): Bool = Mux(
        leftHigh === rightHigh,
        leftIndex < rightIndex,
        leftIndex > rightIndex
    )

    val wbPipeValid = RegInit(VecInit.fill(nRobComplete)(false.B))
    val wbPipeBits  = Reg(Vec(nRobComplete, new RobWriteback))

    val robEpoch = RegInit(0.U(RobConfig.epochBits.W))
    val metaValid = RegInit(VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B)))
    val metaHigh = RegInit(VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B)))
    val metaEpoch = RegInit(
        VecInit.fill(nrobQ)(VecInit.fill(ndcd)(0.U(RobConfig.epochBits.W)))
    )
    // Dequeue advances the authoritative FIFO immediately. Keep the retired
    // physical identities for one cycle and clear the distributed lifetime
    // state from this registered mask, so retirement does not directly drive
    // 33 slot clock enables. A pending slot is already dead to every observer.
    val retireCleanupPending = RegInit(0.U(nrob.W))
    val ftqLastOverride = RegInit(
        VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B))
    )

    def cleanupPending(offset: Int, bank: Int): Bool =
        retireCleanupPending(offset * ndcd + bank)

    val visibleMetaValid = Wire(Vec(nrobQ, Vec(ndcd, Bool())))
    for (offset <- 0 until nrobQ) {
        for (bank <- 0 until ndcd) {
            visibleMetaValid(offset)(bank) :=
                metaValid(offset)(bank) && !cleanupPending(offset, bank)
        }
    }

    val branchMispredict =
        io.branchUpdate.valid && io.branchUpdate.bits.mispredictMask.orR

    val brPtr = io.branchUpdate.bits.robPtr
    val brBankOH = UIntToOH(brPtr.qidx, ndcd)
    val brOffsetOH = UIntToOH(brPtr.offset, nrobQ)

    val nextOffsetOH = ShiftAdd1(brOffsetOH)
    val consumedBanks = MaskLower(brBankOH)
    val offsetWrap = brOffsetOH(nrobQ - 1)

    val restore = q.io.tailRestore.get
    restore.valid := branchMispredict && !io.flush

    io.enq.canAccept := q.io.enqCapacity && !io.flush

    for (bank <- 0 until ndcd) {
        restore.bits.tailOH(bank) := Mux(
            consumedBanks(bank),
            nextOffsetOH,
            brOffsetOH
        )
        restore.bits.tailHigh(bank) :=
            brPtr.high ^ (offsetWrap && consumedBanks(bank))
    }
    restore.bits.enqBaseOH := ShiftAdd1(brBankOH)

    for (i <- 1 until ndcd) {
        assert(!(io.enq.req(i).valid && !io.enq.req(i - 1).valid))
    }

    val enqPayload = Wire(Vec(ndcd, new RobPayload))
    val enqCommitBoundary = Wire(Vec(ndcd, Bool()))
    for (i <- 0 until ndcd) {
        val in = io.enq.req(i).bits
        enqCommitBoundary(i) :=
            (in.fuType & FU_SYS).orR ||
            in.uop === opCSRWR ||
            in.uop === opCSRXCHG ||
            in.uop === opERTN ||
            in.uop === opIDLE
        enqPayload(i).pc        := in.pc
        enqPayload(i).instr     := in.instr
        enqPayload(i).ftqPtr    := in.ftqPtr
        enqPayload(i).ftqOffset := in.ftqOffset
        enqPayload(i).ftqLast   := in.ftqLast
        enqPayload(i).uop       := in.uop
        enqPayload(i).fuType    := in.fuType
        enqPayload(i).fastFixedInt :=
            in.fuType === FU_ALU ||
            in.fuType === FU_MUL ||
            in.fuType === FU_CNT
        enqPayload(i).ldest      := in.ldest
        enqPayload(i).pdest      := in.pdest
        enqPayload(i).pprd       := in.pprd
        enqPayload(i).ldestValid := in.ldestValid
        enqPayload(i).rfWen      := in.rfWen
        enqPayload(i).isLoad  := in.isLoad
        enqPayload(i).isStore := in.isStore
        enqPayload(i).isBr    := in.isBr
        enqPayload(i).isBl    := in.isBl
        enqPayload(i).isJirl  := in.isJirl

        q.io.enq(i).valid :=
            io.enq.req(i).valid && io.enq.canAccept && !branchMispredict
        q.io.enq(i).bits := true.B

        io.enq.resp(i).valid       := q.io.enq(i).fire
        io.enq.resp(i).bits.qidx   := OHToUInt(q.io.enqIdx(i).qidx)
        io.enq.resp(i).bits.offset := OHToUInt(q.io.enqIdx(i).offset)
        io.enq.resp(i).bits.high   := q.io.enqIdx(i).high.asBool
        io.enq.resp(i).bits.epoch  := robEpoch
    }

    for (bank <- 0 until ndcd) {
        val select = VecInit((0 until ndcd).map { lane =>
            q.io.enq(lane).fire && q.io.enqIdx(lane).qidx(bank)
        })
        val selectAny = select.asUInt.orR
        val rowOH = q.io.bankEnqOffset(bank)

        when(selectAny) {
            payloadBanks(bank).write(
                OHToUInt(rowOH),
                Mux1H(select, enqPayload)
            )
            for (bit <- 0 until RobConfig.epochBits) {
                val oldPlane = retirementEpochPlanes(bank)(bit)
                retirementEpochPlanes(bank)(bit) := Mux(
                    robEpoch(bit),
                    oldPlane | rowOH,
                    oldPlane & ~rowOH
                )
            }
        }
        assert(PopCount(select) <= 1.U)
    }

    val completionValid = Wire(Vec(nRobComplete, Bool()))
    val completionBits = Wire(Vec(nRobComplete, new RobWriteback))
    val completionAccepted = Wire(Vec(nRobComplete, Bool()))

    // Slow-path completion validation remains behind this ROB-local boundary.
    // Clean integer/branch results use fastWbAccepted below. Every load, all
    // exceptional results and stores retain this staged path.
    for (i <- 0 until nRobComplete) {
        completionValid(i) := wbPipeValid(i)
        completionBits(i) := wbPipeBits(i)

        val wbPtr      = completionBits(i).robPtr
        val wbQidxOH   = UIntToOH(wbPtr.qidx, ndcd)
        val wbOffsetOH = UIntToOH(wbPtr.offset, nrobQ)
        val slotValid = Mux1H(wbOffsetOH, visibleMetaValid.map { row =>
            Mux1H(wbQidxOH, row)
        })
        val slotHigh = Mux1H(wbOffsetOH, metaHigh.map { row =>
            Mux1H(wbQidxOH, row)
        })
        val slotEpoch = Mux1H(wbOffsetOH, metaEpoch.map { row =>
            Mux1H(wbQidxOH, row)
        })
        val slotIsLoad = Mux1H(wbOffsetOH, loadState.map { row =>
            Mux1H(wbQidxOH, row)
        })
        val slotIsStore = Mux1H(wbOffsetOH, storeState.map { row =>
            Mux1H(wbQidxOH, row)
        })

        when(completionValid(i)) {
            assert(PopCount(wbQidxOH) === 1.U)
            assert(PopCount(wbOffsetOH) === 1.U)
        }

        val sameEntry =
            slotValid && slotHigh === wbPtr.high && slotEpoch === wbPtr.epoch

        // RobPtr has a finite epoch and can eventually alias after repeated
        // recovery.  A delayed result must also come from the execution class
        // that owns the current slot, otherwise (for example) an old integer
        // result can make a newly allocated Load retire before its response.
        val portTypeMatches = if (i < nIntWb) {
            !slotIsLoad && !slotIsStore
        } else if (i < nDataWb) {
            slotIsLoad
        } else {
            slotIsStore
        }

        completionAccepted(i) :=
            completionValid(i) && sameEntry && portTypeMatches && !io.flush

        when(completionValid(i) && !completionBits(i).exceptionValid) {
            assert(completionBits(i).exceptionCause === EXC_NONE)
            assert(!completionBits(i).exceptionBadvValid)
        }
    }

    def sameRobPtr(a: RobPtr, b: RobPtr): Bool =
        a.qidx === b.qidx &&
        a.offset === b.offset &&
        a.high === b.high &&
        a.epoch === b.epoch

    def ptrIsLive(ptr: RobPtr): Bool = {
        val bankOH = UIntToOH(ptr.qidx, ndcd)
        val rowOH = UIntToOH(ptr.offset, nrobQ)
        val valid = Mux1H(
            rowOH,
            visibleMetaValid.map(row => Mux1H(bankOH, row))
        )
        val high = Mux1H(rowOH, metaHigh.map(row => Mux1H(bankOH, row)))
        val epoch = Mux1H(rowOH, metaEpoch.map(row => Mux1H(bankOH, row)))
        valid && high === ptr.high && epoch === ptr.epoch
    }

    def ptrIsLoad(ptr: RobPtr): Bool = {
        val bankOH = UIntToOH(ptr.qidx, ndcd)
        val rowOH = UIntToOH(ptr.offset, nrobQ)
        Mux1H(rowOH, loadState.map(row => Mux1H(bankOH, row)))
    }

    def ptrIsStore(ptr: RobPtr): Bool = {
        val bankOH = UIntToOH(ptr.qidx, ndcd)
        val rowOH = UIntToOH(ptr.offset, nrobQ)
        Mux1H(rowOH, storeState.map(row => Mux1H(bankOH, row)))
    }

    for (i <- nDataWb until nRobComplete) {
        when(completionValid(i)) {
            assert(!completionBits(i).exceptionValid)
        }
        when(completionAccepted(i)) {
            assert(ptrIsStore(completionBits(i).robPtr))
        }
    }

    val brRank = robRank(brPtr)

    def killedByMispredict(ptrHigh: Bool, ptrRank: UInt): Bool =
        branchMispredict && isOlder(
            brPtr.high,
            brRank,
            ptrHigh,
            ptrRank
        )

    // Clean integer/branch writebacks need neither exception arbitration nor
    // normal completion staging. Every Load crosses wbPipe so a late
    // CPUSTC.memory result cannot drive ROB retirement or dequeue control.
    val fastIntWbCandidate = Wire(Vec(nRobComplete, Bool()))
    val fastBranchWbAccepted = Wire(Vec(nRobComplete, Bool()))
    val fastWbAccepted = Wire(Vec(nRobComplete, Bool()))
    for (i <- 0 until nRobComplete) {
        val wb = io.wb(i)
        val isIntPort = (i < nIntWb).B

        fastIntWbCandidate(i) :=
            isIntPort &&
            wb.bits.fastEligible
        when(fastIntWbCandidate(i)) {
            // Writeback's fastEligible contract already includes valid and
            // flush qualification and is restricted to clean fixed results.
            // Do not pull the shared CSR/exception payload into this path.
            assert(wb.valid)
            assert(!wb.bits.exceptionValid)
            assert(wb.bits.exceptionCause === EXC_NONE)
            assert(!wb.bits.exceptionBadvValid)
        }
        fastWbAccepted(i) :=
            fastIntWbCandidate(i) &&
            ptrIsLive(wb.bits.robPtr) &&
            !ptrIsLoad(wb.bits.robPtr) &&
            !ptrIsStore(wb.bits.robPtr) &&
            !io.flush
        fastBranchWbAccepted(i) :=
            fastWbAccepted(i) && isIntPort && wb.bits.branchResolved
    }

    when(io.flush) {
        wbPipeValid := VecInit.fill(nRobComplete)(false.B)
    }.otherwise {
        for (i <- 0 until nRobComplete) {
            // Every Load crosses this boundary. Other unaccepted candidates
            // stay on the ordinary path, where liveness validation drops stale
            // or recovered pointers.
            wbPipeValid(i) := io.wb(i).valid && !fastWbAccepted(i)
            when(io.wb(i).valid && !fastWbAccepted(i)) {
                wbPipeBits(i) := io.wb(i).bits
            }
        }
    }

    // Store-completion payload is meaningful only when wbPipeValid is set.
    // Sample the identity and exception leaves every cycle so translation and
    // Store completion control cannot become clock enables on the ROB pointer
    // path. Invalid-cycle payload remains don't-care; validity is still held
    // separately in wbPipeValid.
    for (i <- nDataWb until nRobComplete) {
        wbPipeBits(i).robPtr := io.wb(i).bits.robPtr
        wbPipeBits(i).exceptionValid := io.wb(i).bits.exceptionValid
        wbPipeBits(i).exceptionCause := io.wb(i).bits.exceptionCause
        wbPipeBits(i).exceptionBadvValid := io.wb(i).bits.exceptionBadvValid
        wbPipeBits(i).exceptionBadv := io.wb(i).bits.exceptionBadv
    }

    // Exception selection is intentionally one stage behind ordinary
    // completion. This keeps the uncommon age-selection path out of the
    // normal completion and retirement timing paths.
    val wbExceptionPipeValid = RegInit(VecInit.fill(nRobComplete)(false.B))
    val wbExceptionPipeBits = Reg(Vec(nRobComplete, new RobExceptionRecord))

    for (i <- 0 until nDataWb) {
        val capture =
            completionAccepted(i) &&
            completionBits(i).exceptionValid &&
            !killedByMispredict(
                completionBits(i).robPtr.high,
                robRank(completionBits(i).robPtr)
            )

        when(io.flush) {
            wbExceptionPipeValid(i) := false.B
        }.otherwise {
            wbExceptionPipeValid(i) := capture
            when(capture) {
                wbExceptionPipeBits(i).robPtr := completionBits(i).robPtr
                wbExceptionPipeBits(i).cause := completionBits(i).exceptionCause
                wbExceptionPipeBits(i).badvValid :=
                    completionBits(i).exceptionBadvValid
                wbExceptionPipeBits(i).badv := completionBits(i).exceptionBadv
            }
        }
    }

    // Store exceptions arrive through a registered, recovery-filtered sidecar.
    // Reuse the two former Store writeback slots in the existing seven-way age
    // tree so this rare path does not enlarge ordinary completion selection.
    for (s <- 0 until nStoreComplete) {
        val i = nDataWb + s
        val in = io.storeException(s)
        val inRank = robRank(in.bits.robPtr)
        val capture =
            in.valid &&
            ptrIsLive(in.bits.robPtr) &&
            ptrIsStore(in.bits.robPtr) &&
            !killedByMispredict(in.bits.robPtr.high, inRank) &&
            !io.flush

        when(io.flush) {
            wbExceptionPipeValid(i) := false.B
        }.otherwise {
            wbExceptionPipeValid(i) := capture
        }
        wbExceptionPipeBits(i).robPtr := in.bits.robPtr
        wbExceptionPipeBits(i).cause := in.bits.cause
        wbExceptionPipeBits(i).badvValid := in.bits.badvValid
        wbExceptionPipeBits(i).badv := in.bits.badv

        when(in.valid) {
            assert(in.bits.cause =/= EXC_NONE)
        }
    }

    type ExceptionChoice = (Bool, RobExceptionRecord)

    def selectOlder(
        left: ExceptionChoice,
        right: ExceptionChoice
    ): ExceptionChoice = {
        val takeRight = right._1 && (
            !left._1 || isOlder(
                right._2.robPtr.high,
                robRank(right._2.robPtr),
                left._2.robPtr.high,
                robRank(left._2.robPtr)
            )
        )
        (
            left._1 || right._1,
            Mux(takeRight, right._2, left._2)
        )
    }

    def selectOldest(candidates: Seq[ExceptionChoice]): ExceptionChoice = {
        require(candidates.nonEmpty)
        if (candidates.length == 1) {
            candidates.head
        } else {
            selectOldest(candidates.grouped(2).map {
                case Seq(left, right) => selectOlder(left, right)
                case Seq(left)        => left
            }.toSeq)
        }
    }

    // Check recovery independently at each registered candidate. If the
    // oldest is younger than the redirecting branch, every younger candidate
    // is killed too; otherwise the oldest itself is one of the survivors.
    // The OR below is therefore equivalent to filtering the selected oldest,
    // without putting the age/mux tree in selectedExceptionValid's path.
    val anySurvivingPipeException = VecInit(
        (0 until nRobComplete).map { i =>
            wbExceptionPipeValid(i) &&
                !killedByMispredict(
                    wbExceptionPipeBits(i).robPtr.high,
                    robRank(wbExceptionPipeBits(i).robPtr)
                )
        }
    ).asUInt.orR

    val (oldestPipeValid, oldestPipeBits) = selectOldest(
        (0 until nRobComplete).map { i =>
            (wbExceptionPipeValid(i), wbExceptionPipeBits(i))
        }
    )

    val selectedExceptionValid = RegInit(false.B)
    val selectedExceptionBits = Reg(new RobExceptionRecord)

    when(io.flush) {
        selectedExceptionValid := false.B
    }.otherwise {
        selectedExceptionValid := anySurvivingPipeException
        selectedExceptionBits := oldestPipeBits
    }

    val selectedExceptionAccepted =
        selectedExceptionValid &&
        !killedByMispredict(
            selectedExceptionBits.robPtr.high,
            robRank(selectedExceptionBits.robPtr)
        ) &&
        !io.flush

    when(selectedExceptionAccepted) {
        assert(selectedExceptionBits.cause =/= EXC_NONE)
    }

    when(io.flush) {
        completeState := VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B))
        branchResolvedState := VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B))
        storeState := VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B))
        loadState := VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B))
        commitBoundaryState :=
            VecInit.fill(nrobQ)(VecInit.fill(ndcd)(false.B))
    }.otherwise {
        for (i <- 0 until nRobComplete) {
            val ptr = io.wb(i).bits.robPtr
            when(fastWbAccepted(i)) {
                completeState(ptr.offset)(ptr.qidx) := true.B
                when(io.wb(i).bits.branchResolved) {
                    branchResolvedState(ptr.offset)(ptr.qidx) := true.B
                }
                completionData(ptr.offset)(ptr.qidx) := io.wb(i).bits.data
            }
        }

        for (i <- 0 until nRobComplete) {
            val ptr = completionBits(i).robPtr
            when(completionAccepted(i) && !completionBits(i).exceptionValid) {
                completeState(ptr.offset)(ptr.qidx) := true.B
                when(completionBits(i).branchResolved) {
                    branchResolvedState(ptr.offset)(ptr.qidx) := true.B
                }
                completionData(ptr.offset)(ptr.qidx) := completionBits(i).data
            }
        }

        // Allocation has final priority over a stale completion to a reused slot.
        for (i <- 0 until ndcd) {
            when(q.io.enq(i).fire) {
                val offset = OHToUInt(q.io.enqIdx(i).offset)
                val bank = OHToUInt(q.io.enqIdx(i).qidx)
                completeState(offset)(bank) := io.enq.req(i).bits.complete
                branchResolvedState(offset)(bank) :=
                    io.enq.req(i).bits.branchResolved
                storeState(offset)(bank) := io.enq.req(i).bits.isStore
                loadState(offset)(bank) := io.enq.req(i).bits.isLoad
                commitBoundaryState(offset)(bank) := enqCommitBoundary(i)
                completionData(offset)(bank) := io.enq.req(i).bits.data
            }
        }
    }

    val bankReadRowOH = Wire(Vec(ndcd, UInt(nrobQ.W)))
    val bankPayload = Wire(Vec(ndcd, new RobPayload))
    val bankComplete = Wire(Vec(ndcd, Bool()))
    val bankBranchResolved = Wire(Vec(ndcd, Bool()))
    val bankIsStore = Wire(Vec(ndcd, Bool()))
    val bankIsLoad = Wire(Vec(ndcd, Bool()))
    val bankCommitBoundary = Wire(Vec(ndcd, Bool()))
    val bankEpoch = Wire(Vec(ndcd, UInt(RobConfig.epochBits.W)))
    val bankDebugData = Wire(Vec(ndcd, UInt(CPUSTC.config.RegisterFile.dataWidth.W)))

    for (bank <- 0 until ndcd) {
        // ClusterIndexFIFO's logical dequeue lanes are a rotation of the three
        // physical banks.  Read each bank from its own registered head and do
        // only the final bank-to-lane rotation below.  This removes the old
        // bank -> lane -> bank round trip from the ordinary retirement cone.
        bankReadRowOH(bank) := q.io.bankDeqOffset(bank)
        val row = OHToUInt(bankReadRowOH(bank))

        bankPayload(bank) := payloadBanks(bank).read(row)
        bankComplete(bank) := Mux1H(
            bankReadRowOH(bank),
            completeState.map(_(bank))
        )
        bankBranchResolved(bank) := Mux1H(
            bankReadRowOH(bank),
            branchResolvedState.map(_(bank))
        )
        bankIsStore(bank) := Mux1H(
            bankReadRowOH(bank),
            storeState.map(_(bank))
        )
        bankIsLoad(bank) := Mux1H(
            bankReadRowOH(bank),
            loadState.map(_(bank))
        )
        bankCommitBoundary(bank) := Mux1H(
            bankReadRowOH(bank),
            commitBoundaryState.map(_(bank))
        )
        val retirementEpochRead = Wire(Vec(RobConfig.epochBits, Bool()))
        for (bit <- 0 until RobConfig.epochBits) {
            retirementEpochRead(bit) :=
                (retirementEpochPlanes(bank)(bit) & bankReadRowOH(bank)).orR
        }
        bankEpoch(bank) := retirementEpochRead.asUInt
        bankDebugData(bank) := Mux1H(
            bankReadRowOH(bank),
            completionData.map(_(bank))
        )
        assert(PopCount(bankReadRowOH(bank)) === 1.U)
    }

    val deqPayload = Wire(Vec(ncmt, new RobPayload))
    val deqComplete = Wire(Vec(ncmt, Bool()))
    val deqBranchResolved = Wire(Vec(ncmt, Bool()))
    val deqIsStore = Wire(Vec(ncmt, Bool()))
    val deqIsLoad = Wire(Vec(ncmt, Bool()))
    val deqCommitBoundary = Wire(Vec(ncmt, Bool()))
    val deqEpoch = Wire(Vec(ncmt, UInt(RobConfig.epochBits.W)))
    val deqData = Wire(Vec(ncmt, UInt(CPUSTC.config.RegisterFile.dataWidth.W)))
    val deqPtr = Wire(Vec(ncmt, new RobPtr))

    for (i <- 0 until ncmt) {
        deqPayload(i) := Mux1H(q.io.deqIdx(i).qidx, bankPayload)
        deqComplete(i) := Mux1H(q.io.deqIdx(i).qidx, bankComplete)
        deqBranchResolved(i) :=
            Mux1H(q.io.deqIdx(i).qidx, bankBranchResolved)
        deqIsStore(i) := Mux1H(q.io.deqIdx(i).qidx, bankIsStore)
        deqIsLoad(i) := Mux1H(q.io.deqIdx(i).qidx, bankIsLoad)
        deqCommitBoundary(i) :=
            Mux1H(q.io.deqIdx(i).qidx, bankCommitBoundary)
        deqEpoch(i) := Mux1H(q.io.deqIdx(i).qidx, bankEpoch)
        deqData(i) := Mux1H(q.io.deqIdx(i).qidx, bankDebugData)

        deqPtr(i).qidx   := OHToUInt(q.io.deqIdx(i).qidx)
        deqPtr(i).offset := OHToUInt(q.io.deqIdx(i).offset)
        deqPtr(i).high   := q.io.deqIdx(i).high.asBool
        deqPtr(i).epoch  := deqEpoch(i)

        when(q.io.deqPresent(i)) {
            val validationValid = Mux1H(
                q.io.deqIdx(i).offset,
                visibleMetaValid.map(row => Mux1H(q.io.deqIdx(i).qidx, row))
            )
            val validationHigh = Mux1H(
                q.io.deqIdx(i).offset,
                metaHigh.map(row => Mux1H(q.io.deqIdx(i).qidx, row))
            )
            val validationEpoch = Mux1H(
                q.io.deqIdx(i).offset,
                metaEpoch.map(row => Mux1H(q.io.deqIdx(i).qidx, row))
            )
            assert(validationValid)
            assert(q.io.deqIdx(i).high.asBool === validationHigh)
            assert(deqEpoch(i) === validationEpoch)
        }
    }

    val exceptionRecordValid = RegInit(false.B)
    val exceptionRecord = Reg(new RobExceptionRecord)
    val currentExceptionValid =
        exceptionRecordValid &&
        ptrIsLive(exceptionRecord.robPtr) &&
        !killedByMispredict(
            exceptionRecord.robPtr.high,
            robRank(exceptionRecord.robPtr)
        )

    val enqExceptionValid = Wire(Vec(ndcd, Bool()))
    val enqExceptionBits = Wire(Vec(ndcd, new RobExceptionRecord))
    for (i <- 0 until ndcd) {
        enqExceptionValid(i) :=
            q.io.enq(i).fire && io.enq.req(i).bits.exceptionValid
        enqExceptionBits(i).robPtr := io.enq.resp(i).bits
        enqExceptionBits(i).cause := io.enq.req(i).bits.exceptionCause
        enqExceptionBits(i).badvValid :=
            io.enq.req(i).bits.exceptionBadvValid
        enqExceptionBits(i).badv := io.enq.req(i).bits.exceptionBadv

        when(enqExceptionValid(i)) {
            assert(io.enq.req(i).bits.complete)
        }
    }

    val oldestWbValid = selectedExceptionAccepted
    val oldestWbBits = selectedExceptionBits

    val takeWb = oldestWbValid && (
        !currentExceptionValid || isOlder(
            oldestWbBits.robPtr.high,
            robRank(oldestWbBits.robPtr),
            exceptionRecord.robPtr.high,
            robRank(exceptionRecord.robPtr)
        )
    )
    val retainedExceptionValid = currentExceptionValid || oldestWbValid
    val retainedExceptionBits = Mux(takeWb, oldestWbBits, exceptionRecord)

    // Newly allocated entries are younger than every live ROB entry. They can
    // seed an empty exception record, but never need to enter the age tree.
    val enqExceptionAny = enqExceptionValid.asUInt.orR
    val oldestEnqBits = Mux1H(
        PriorityEncoderOH(enqExceptionValid.asUInt),
        enqExceptionBits
    )
    val oldestExceptionValid = retainedExceptionValid || enqExceptionAny
    val oldestExceptionBits = Mux(
        retainedExceptionValid,
        retainedExceptionBits,
        oldestEnqBits
    )

    when(io.flush) {
        exceptionRecordValid := false.B
    }.otherwise {
        exceptionRecordValid := oldestExceptionValid
        when(oldestExceptionValid) {
            exceptionRecord := oldestExceptionBits
        }
    }

    val deqHasException = Wire(Vec(ncmt, Bool()))
    for (i <- 0 until ncmt) {
        deqHasException(i) :=
            exceptionRecordValid && sameRobPtr(exceptionRecord.robPtr, deqPtr(i))
    }

    val retireCandidate = Wire(Vec(ncmt, Bool()))
    val retireNow = Wire(Vec(ncmt, Bool()))
    val retirementEnabled = !io.commitBlock && !io.flush

    // A registered Load completion can retire directly from wbPipe. This
    // avoids a second completion-state cycle while keeping every retirement
    // input on the ROB side of the CPUSTC.memory boundary.
    val pipeLoadCompleteAtDeq = Wire(Vec(ncmt, Bool()))
    val pipeLoadDataAtDeq = Wire(
        Vec(ncmt, UInt(CPUSTC.config.RegisterFile.dataWidth.W))
    )
    for (lane <- 0 until ncmt) {
        val pipeMatches = VecInit((0 until nRobComplete).map { port =>
            (port >= nIntWb && port < nDataWb).B &&
            completionValid(port) &&
            !completionBits(port).exceptionValid &&
            !io.flush &&
            q.io.deqPresent(lane) &&
            deqIsLoad(lane) &&
            sameRobPtr(completionBits(port).robPtr, deqPtr(lane))
        })
        val acceptedMatches = VecInit((0 until nRobComplete).map { port =>
            (port >= nIntWb && port < nDataWb).B &&
            completionAccepted(port) &&
            !completionBits(port).exceptionValid &&
            q.io.deqPresent(lane) &&
            deqIsLoad(lane) &&
            sameRobPtr(completionBits(port).robPtr, deqPtr(lane))
        })
        pipeLoadCompleteAtDeq(lane) := pipeMatches.asUInt.orR
        pipeLoadDataAtDeq(lane) := Mux1H(
            pipeMatches,
            completionBits.map(_.data)
        )
        assert(PopCount(pipeMatches) <= 1.U)
        assert(pipeMatches.asUInt === acceptedMatches.asUInt)
    }

    // A present dequeue slot plus a full-pointer match is already a local
    // liveness proof.  Keep the ROB-wide fastWbAccepted scan for completion
    // state writes, but do not place it in the same-cycle retirement cone.
    // This mirrors the fixed-integer retirement bypass below and prevents a
    // branch result from scanning all 33 slots before matching three heads.
    val fastBranchCompleteAtDeq = Wire(Vec(ncmt, Bool()))
    val fastBranchDataAtDeq = Wire(
        Vec(ncmt, UInt(CPUSTC.config.RegisterFile.dataWidth.W))
    )
    for (lane <- 0 until ncmt) {
        val dequeueIsBranch =
            deqPayload(lane).isBr ||
            deqPayload(lane).isBl ||
            deqPayload(lane).isJirl
        val matches = VecInit((0 until nRobComplete).map { port =>
            fastIntWbCandidate(port) &&
            io.wb(port).bits.branchResolved &&
            q.io.deqPresent(lane) &&
            !deqIsLoad(lane) &&
            !deqIsStore(lane) &&
            dequeueIsBranch &&
            !io.flush &&
            sameRobPtr(io.wb(port).bits.robPtr, deqPtr(lane))
        })
        val legacyMatches = VecInit((0 until nRobComplete).map { port =>
            fastBranchWbAccepted(port) &&
            dequeueIsBranch &&
            sameRobPtr(io.wb(port).bits.robPtr, deqPtr(lane))
        })
        fastBranchCompleteAtDeq(lane) := matches.asUInt.orR
        fastBranchDataAtDeq(lane) := Mux1H(matches, io.wb.map(_.bits.data))
        assert(PopCount(matches) <= 1.U)
        assert(matches.asUInt === legacyMatches.asUInt)

        when(matches.asUInt.orR) {
            assert(q.io.deqPresent(lane))
            assert(!deqIsLoad(lane))
            assert(!deqIsStore(lane))
            assert(dequeueIsBranch)
        }
    }

    val correctBranchCompleteAtDeq = VecInit((0 until ncmt).map { lane =>
        fastBranchCompleteAtDeq(lane) && !branchMispredict
    })

    // A full-pointer match against a present dequeue entry is a ROB-local
    // liveness check. Keep this retirement-only matcher off the 33-entry
    // fastWbAccepted metadata scan. Only fixed-latency ALU, MUL and CNT
    // operations may complete an otherwise incomplete ROB head directly.
    val fastFixedIntCompleteAtDeq = Wire(Vec(ncmt, Bool()))
    val fastFixedIntDataAtDeq = Wire(
        Vec(ncmt, UInt(CPUSTC.config.RegisterFile.dataWidth.W))
    )
    for (lane <- 0 until ncmt) {
        val fixedIntFu = deqPayload(lane).fastFixedInt
        val matches = VecInit((0 until nIntWb).map { port =>
            fastIntWbCandidate(port) &&
            q.io.deqPresent(lane) &&
            fixedIntFu &&
            sameRobPtr(io.wb(port).bits.robPtr, deqPtr(lane))
        })
        fastFixedIntCompleteAtDeq(lane) := matches.asUInt.orR
        fastFixedIntDataAtDeq(lane) := Mux1H(
            matches,
            io.wb.take(nIntWb).map(_.bits.data)
        )
        assert(PopCount(matches) <= 1.U)

        when(fastFixedIntCompleteAtDeq(lane)) {
            assert(q.io.deqPresent(lane))
            assert(!deqIsLoad(lane))
            assert(!deqIsStore(lane))
            assert(fixedIntFu)
        }
    }

    val correctFixedIntCompleteAtDeq = VecInit((0 until ncmt).map { lane =>
        fastFixedIntCompleteAtDeq(lane) && !branchMispredict
    })

    val entryReady = Wire(Vec(ncmt, Bool()))
    for (lane <- 0 until ncmt) {
        entryReady(lane) :=
            q.io.deqPresent(lane) &&
            (
                deqComplete(lane) ||
                pipeLoadCompleteAtDeq(lane) ||
                correctBranchCompleteAtDeq(lane) ||
                correctFixedIntCompleteAtDeq(lane)
            ) &&
            !deqHasException(lane)
    }

    for (i <- 0 until ncmt) {
        val readyPrefix = entryReady.take(i + 1).reduce(_ && _)
        val olderBoundary = if (i == 0) {
            false.B
        } else {
            deqCommitBoundary.take(i).reduce(_ || _)
        }
        val storeCount = PopCount(deqIsStore.take(i + 1))

        retireCandidate(i) :=
            retirementEnabled &&
            (i < maxCommitPerCycle).B &&
            readyPrefix &&
            !olderBoundary &&
            storeCount <= 1.U
        retireNow(i) := retireCandidate(i) && !branchMispredict
    }

    val commitBitsNow = Wire(Vec(ncmt, new RobCommitEntry))
    for (i <- 0 until ncmt) {
        val payload = deqPayload(i)
        commitBitsNow(i).robPtr := deqPtr(i)
        commitBitsNow(i).pc        := payload.pc
        commitBitsNow(i).instr     := payload.instr
        commitBitsNow(i).data :=
            Mux(
                pipeLoadCompleteAtDeq(i),
                pipeLoadDataAtDeq(i),
                Mux(
                    correctBranchCompleteAtDeq(i),
                    fastBranchDataAtDeq(i),
                    Mux(
                        correctFixedIntCompleteAtDeq(i),
                        fastFixedIntDataAtDeq(i),
                        deqData(i)
                    )
                )
            )
        commitBitsNow(i).ftqPtr    := payload.ftqPtr
        commitBitsNow(i).ftqOffset := payload.ftqOffset
        commitBitsNow(i).ftqLast   := payload.ftqLast || Mux1H(
            q.io.deqIdx(i).offset,
            ftqLastOverride.map(row => Mux1H(q.io.deqIdx(i).qidx, row))
        )
        commitBitsNow(i).ldest      := payload.ldest
        commitBitsNow(i).pdest      := payload.pdest
        commitBitsNow(i).pprd       := payload.pprd
        commitBitsNow(i).ldestValid := payload.ldestValid
        commitBitsNow(i).rfWen      := payload.rfWen
        commitBitsNow(i).isLoad     := deqIsLoad(i)
        commitBitsNow(i).isStore    := deqIsStore(i)
        commitBitsNow(i).isBr       := payload.isBr
        commitBitsNow(i).isBl       := payload.isBl
        commitBitsNow(i).isJirl     := payload.isJirl
        commitBitsNow(i).uncache    := false.B
        commitBitsNow(i).uop        := payload.uop
        commitBitsNow(i).fuType     := payload.fuType
        commitBitsNow(i).commitBoundary := deqCommitBoundary(i)

        when(retireNow(i) && (payload.isBr || payload.isBl || payload.isJirl)) {
            assert(deqBranchResolved(i) || correctBranchCompleteAtDeq(i))
        }

        // Tail restoration already forces q.io.deq.valid low during a branch
        // recovery.  Keep that rare recovery bit out of the ordinary dequeue
        // candidate cone; architectural commit remains gated by retireNow.
        q.io.deq(i).ready := retireCandidate(i)
    }

    val commitPipeValid = RegInit(VecInit.fill(ncmt)(false.B))
    val commitPipeBits = Reg(Vec(ncmt, new RobCommitEntry))
    commitPipeValid := retireNow
    for (i <- 0 until ncmt) {
        when(retireNow(i)) {
            commitPipeBits(i) := commitBitsNow(i)
        }

        io.cmt(i).valid := commitPipeValid(i)
        io.cmt(i).bits := commitPipeBits(i)

        io.rnmCmt(i).valid := commitPipeValid(i)
        io.rnmCmt(i).bits.ldest      := commitPipeBits(i).ldest
        io.rnmCmt(i).bits.pdest      := commitPipeBits(i).pdest
        io.rnmCmt(i).bits.pprd       := commitPipeBits(i).pprd
        io.rnmCmt(i).bits.ldestValid := commitPipeBits(i).ldestValid
        io.rnmCmt(i).bits.rfWen      := commitPipeBits(i).rfWen
    }

    io.retireMask := retireNow.asUInt
    for (i <- 0 until ncmt) {
        io.loadCommit(i) := retireNow(i) && deqIsLoad(i)
    }
    io.storeCommit.valid := VecInit((0 until ncmt).map { i =>
        retireNow(i) && deqIsStore(i)
    }).asUInt.orR
    io.storeCommit.bits := true.B
    io.boundaryCommit := VecInit((0 until ncmt).map { i =>
        retireNow(i) && deqCommitBoundary(i)
    }).asUInt.orR

    val retirePhysicalMask = VecInit((0 until nrobQ).flatMap { offset =>
        (0 until ndcd).map { bank =>
            VecInit((0 until ncmt).map { lane =>
                q.io.deq(lane).fire &&
                q.io.deqIdx(lane).offset(offset) &&
                q.io.deqIdx(lane).qidx(bank)
            }).asUInt.orR
        }
    }).asUInt

    when(io.flush) {
        retireCleanupPending := 0.U
    }.otherwise {
        retireCleanupPending := retirePhysicalMask
    }

    when(io.flush || branchMispredict) {
        robEpoch := robEpoch + 1.U
    }

    val branchKilledMetaSlot = Wire(Vec(nrobQ, Vec(ndcd, Bool())))
    for (offset <- 0 until nrobQ) {
        for (bank <- 0 until ndcd) {
            val slotRank = Cat(
                offset.U(log2Ceil(nrobQ).W),
                bank.U(log2Ceil(ndcd).W)
            )
            branchKilledMetaSlot(offset)(bank) :=
                visibleMetaValid(offset)(bank) &&
                killedByMispredict(metaHigh(offset)(bank), slotRank)
        }
    }

    // Override is observed only through a present dequeue slot. Recovery or
    // dequeue may leave a stale bit in dead storage; allocation clears it
    // before the physical slot can be reused.
    when(io.flush) {
        for (offset <- 0 until nrobQ) {
            for (bank <- 0 until ndcd) {
                metaValid(offset)(bank) := false.B
            }
        }
    }.otherwise {
        for (offset <- 0 until nrobQ) {
            for (bank <- 0 until ndcd) {
                when(branchKilledMetaSlot(offset)(bank)) {
                    metaValid(offset)(bank) := false.B
                }
            }
        }

        for (i <- 0 until ncmt) {
            when(q.io.deq(i).fire) {
                val offset = OHToUInt(q.io.deqIdx(i).offset)
                val bank = OHToUInt(q.io.deqIdx(i).qidx)

                assert(PopCount(q.io.deqIdx(i).offset) === 1.U)
                assert(PopCount(q.io.deqIdx(i).qidx) === 1.U)
                assert(metaValid(offset)(bank))
                assert(metaHigh(offset)(bank) === q.io.deqIdx(i).high.asBool)
                assert(metaEpoch(offset)(bank) === deqEpoch(i))
            }
        }

        for (offset <- 0 until nrobQ) {
            for (bank <- 0 until ndcd) {
                when(cleanupPending(offset, bank)) {
                    metaValid(offset)(bank) := false.B
                }
            }
        }

        // Prewrite the epoch into each bank's registered free tail.  A real
        // allocation uses exactly these rows, so its returned RobPtr observes
        // the same epoch without putting the cross-pipeline enqueue-valid cone
        // on every metaEpoch clock enable.  Do not write while full: a full
        // bank's tail can alias a live head entry.
        for (bank <- 0 until ndcd) {
            for (offset <- 0 until nrobQ) {
                when(
                    q.io.enqCapacity &&
                    q.io.bankEnqOffset(bank)(offset)
                ) {
                    metaEpoch(offset)(bank) := robEpoch
                }
            }
        }

        for (i <- 0 until ndcd) {
            when(q.io.enq(i).fire) {
                val offset = OHToUInt(q.io.enqIdx(i).offset)
                val bank = OHToUInt(q.io.enqIdx(i).qidx)

                assert(PopCount(q.io.enqIdx(i).offset) === 1.U)
                assert(PopCount(q.io.enqIdx(i).qidx) === 1.U)

                metaValid(offset)(bank) := true.B
                metaHigh(offset)(bank) := q.io.enqIdx(i).high.asBool
                ftqLastOverride(offset)(bank) := false.B
            }
        }

        when(branchMispredict) {
            ftqLastOverride(brPtr.offset)(brPtr.qidx) := true.B
        }
    }

    when(io.branchUpdate.valid && !io.flush) {
        assert(PopCount(io.branchUpdate.bits.resolveMask) === 1.U)
        assert(
            (io.branchUpdate.bits.mispredictMask &
                (~io.branchUpdate.bits.resolveMask).asUInt) === 0.U
        )
    }

    when(branchMispredict && !io.flush) {
        assert(PopCount(brBankOH) === 1.U)
        assert(PopCount(brOffsetOH) === 1.U)
        assert(metaValid(brPtr.offset)(brPtr.qidx))
        assert(metaHigh(brPtr.offset)(brPtr.qidx) === brPtr.high)
        assert(metaEpoch(brPtr.offset)(brPtr.qidx) === brPtr.epoch)
    }

    when(io.flush || branchMispredict) {
        assert(!q.io.enq.map(_.fire).reduce(_ || _))
        assert(!q.io.deq.map(_.fire).reduce(_ || _))
    }

    for (i <- 1 until ncmt) {
        assert(!(commitPipeValid(i) && !commitPipeValid(i - 1)))
    }

    q.io.flush := io.flush

    io.empty := Mux(io.flush, true.B, !branchMispredict && !q.io.deqPresent(0))
    io.full := !io.enq.canAccept
    io.occupancy := PopCount(visibleMetaValid.flatten)
    io.status.headValid :=
        q.io.deqPresent(0) && !io.flush && !branchMispredict
    // An exception record can only be created by an accepted completion or
    // by an already-complete decode exception. It is therefore the completion
    // proof for a trapping head without writing the normal completion matrix.
    io.status.headComplete :=
        deqComplete(0) || pipeLoadCompleteAtDeq(0) || deqHasException(0)
    // Use the dedicated registered type state instead of rereading the wide
    // payload bank for the memory-side ROB-head qualification path.
    io.status.headIsLoad   := deqIsLoad(0)
    io.status.headIsStore  := deqPayload(0).isStore
    io.status.headIsBranch :=
        deqPayload(0).isBr || deqPayload(0).isBl || deqPayload(0).isJirl
    io.status.headRobPtr := deqPtr(0)
    io.status.headPc := deqPayload(0).pc
    // T16B deliberately removes the raw P0 Load handoff. Keep the historical
    // counters at zero rather than mixing registered wbPipe events into them.
    io.status.p0Handoff := 0.U.asTypeOf(new RobP0HandoffStatus)

    if (enableDebug) {
        val debugCycle = RegInit(0.U(64.W))
        debugCycle := debugCycle + 1.U

        val previousHeadValid = RegNext(io.status.headValid, false.B)
        val previousHeadComplete = RegNext(io.status.headComplete, false.B)
        val previousHeadPtr = RegNext(io.status.headRobPtr.asUInt, 0.U)
        val headChanged = previousHeadValid =/= io.status.headValid ||
            (io.status.headValid && previousHeadPtr =/= io.status.headRobPtr.asUInt) ||
            (io.status.headValid && previousHeadComplete =/= io.status.headComplete)
        val blockedHeartbeat = io.status.headValid && !io.status.headComplete &&
            debugCycle(7, 0) === 0.U

        when(debugCycle >= 300000.U) {
        when(headChanged || blockedHeartbeat) {
            printf(
                p"[DBG][ROB][HEAD] cycle=${debugCycle} valid=${io.status.headValid} " +
                    p"complete=${io.status.headComplete} pc=0x${Hexadecimal(deqPayload(0).pc)} " +
                    p"instr=0x${Hexadecimal(deqPayload(0).instr)} " +
                    p"robQ=${deqPtr(0).qidx} robOff=${deqPtr(0).offset} " +
                    p"robH=${deqPtr(0).high} robEpoch=${deqPtr(0).epoch} " +
                    p"uop=${deqPayload(0).uop} fu=0x${Hexadecimal(deqPayload(0).fuType)} " +
                    p"load=${deqPayload(0).isLoad} store=${deqPayload(0).isStore} " +
                    p"branch=${io.status.headIsBranch} commitBlock=${io.commitBlock}\n"
            )
        }

        for (port <- 0 until nRobComplete) {
            when(io.wb(port).valid) {
                printf(
                    p"[DBG][ROB][WB_IN] cycle=${debugCycle} port=${port.U} " +
                        p"robQ=${io.wb(port).bits.robPtr.qidx} " +
                        p"robOff=${io.wb(port).bits.robPtr.offset} " +
                        p"robH=${io.wb(port).bits.robPtr.high} " +
                        p"robEpoch=${io.wb(port).bits.robPtr.epoch}\n"
                )
            }
            when(completionValid(port)) {
                printf(
                    p"[DBG][ROB][WB_PIPE] cycle=${debugCycle} port=${port.U} " +
                        p"accepted=${completionAccepted(port)} " +
                        p"robQ=${completionBits(port).robPtr.qidx} " +
                        p"robOff=${completionBits(port).robPtr.offset} " +
                        p"robH=${completionBits(port).robPtr.high} " +
                        p"robEpoch=${completionBits(port).robPtr.epoch}\n"
                )
            }
        }
        }
    }

    // Expose the raw head exception independently of redirect control. Backend
    // uses this to prioritize a precise trap over memory replay; feeding flush
    // back into this predicate would create a recovery arbitration loop.
    io.trap.valid :=
        q.io.deqPresent(0) &&
        deqHasException(0)
    io.trap.bits.robPtr := deqPtr(0)
    io.trap.bits.pc := deqPayload(0).pc
    io.trap.bits.instr := deqPayload(0).instr
    io.trap.bits.exceptionCause := exceptionRecord.cause
    io.trap.bits.exceptionBadvValid := exceptionRecord.badvValid
    io.trap.bits.exceptionBadv := exceptionRecord.badv

    when(io.trap.valid) {
        assert(!retireNow.asUInt.orR)
        assert(io.trap.bits.exceptionCause =/= EXC_NONE)
    }

    when(exceptionRecordValid) {
        assert(ptrIsLive(exceptionRecord.robPtr) || branchMispredict)
        assert(exceptionRecord.cause =/= EXC_NONE)
    }

    when(io.commitBlock) {
        assert(!retireNow.asUInt.orR)
    }

    val previousBranchKilledMetaMask = RegNext(
        branchKilledMetaSlot.asUInt,
        0.U((nrobQ * ndcd).W)
    )
    val enqPhysicalMask = VecInit((0 until ndcd).map { lane =>
        val slot = OHToUInt(q.io.enqIdx(lane).offset) * ndcd.U +
            OHToUInt(q.io.enqIdx(lane).qidx)
        Mux(
            q.io.enq(lane).fire,
            UIntToOH(slot, nrobQ * ndcd),
            0.U((nrobQ * ndcd).W)
        )
    }).reduce(_ | _)
    val previousEnqPhysicalMask = RegNext(
        enqPhysicalMask,
        0.U((nrobQ * ndcd).W)
    )
    val metaValidFlat = visibleMetaValid.asUInt
    val ftqLastOverrideFlat = ftqLastOverride.asUInt

    // Lazy payload bits may remain set in dead physical slots. Allocation is
    // the visibility boundary: both bits must be clear before a reused slot can
    // be observed as a new ROB entry.
    assert((ftqLastOverrideFlat & previousEnqPhysicalMask) === 0.U)
    assert((metaValidFlat & previousBranchKilledMetaMask) === 0.U)
    assert((retireCleanupPending & enqPhysicalMask) === 0.U ||
        q.io.enq.map(_.fire).reduce(_ || _))
    assert((retireCleanupPending & retirePhysicalMask) === 0.U)

    assert(PopCount(retireNow) <= maxCommitPerCycle.U)
    assert(PopCount(VecInit((0 until ncmt).map { i =>
        retireNow(i) && deqIsStore(i)
    })) <= 1.U)
    assert(PopCount(VecInit((0 until ncmt).map { i =>
        retireNow(i) && deqCommitBoundary(i)
    })) <= 1.U)
}
