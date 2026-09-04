package CPUSTC.memory.mmu

import chisel3._
import chisel3.util._

import CPUSTC.backend.{AddressTranslationState, DMW, TLBEntry, TLBPageEntry, TLBELO}
import CPUSTC.config.ExpCode
import CPUSTC.config.SystemOp
import CPUSTC.memory.{SysMemCmd, SysMemResp}

object TLBAccess {
    val Fetch = 0.U(2.W)
    val Load  = 1.U(2.W)
    val Store = 2.U(2.W)
}

class TLBReq extends Bundle {
    val vaddr = UInt(32.W)
    val stall = Bool()
    val access = UInt(2.W)
    val token = UInt(16.W)
}

class TLBResp extends Bundle {
    val paddr = UInt(32.W)
    val uncache = Bool()
    val exception = UInt(8.W)
    val token = UInt(16.W)

    // A MicroTLB hit on the instruction port carries raw permission metadata.
    // ICache resolves it after this registered boundary so the rare PIF/PPI
    // encoder cannot extend the BTB-target translation path.
    val deferredFetchCheck = Bool()
    val fetchPageValid = Bool()
    val fetchPagePlv = UInt(2.W)
    val fetchRequestPlv = UInt(2.W)
}

private class FetchCheckMeta extends Bundle {
    val deferred = Bool()
    val requestPlv = UInt(2.W)
}

private class PendingTLBReq extends Bundle {
    val vaddr = UInt(32.W)
    val access = UInt(2.W)
}

private class MainTLBPayload extends Bundle {
    val vppn = UInt(19.W)
    val ps = UInt(6.W)
    val g = Bool()
    val asid = UInt(10.W)
    val page0 = new TLBPageEntry
    val page1 = new TLBPageEntry
}

class TLBIO(nPorts: Int, nEntries: Int, epochBits: Int) extends Bundle {
    val flush = Input(Bool())
    val cancel = Input(Vec(nPorts, Bool()))
    val state = Input(new AddressTranslationState)
    val req = Vec(nPorts, Flipped(Decoupled(new TLBReq)))
    val resp = Vec(nPorts, Decoupled(new TLBResp))

    val sysCmd = Flipped(Decoupled(new SysMemCmd(log2Ceil(nEntries), epochBits)))
    val sysResp = Decoupled(new SysMemResp(log2Ceil(nEntries), epochBits))
}

/** A single architectural MainTLB with three small translation caches.
  *
  * Direct-address, DMW, and MicroTLB hits return through a one-entry registered
  * response per port. A MicroTLB miss is serialized through the one MainTLB
  * scanner, which examines four entries per cycle. The 32-entry associative
  * compare therefore never appears on an ICache or AGU timing path.
  */
class TLB(
    nPorts: Int = 3,
    nEntries: Int = 32,
    scanWidth: Int = 4,
    microEntries: Int = 4,
    epochBits: Int = 4
) extends Module {
    require(nPorts > 0)
    require(nEntries >= 2 && isPow2(nEntries))
    require(scanWidth > 0 && isPow2(scanWidth) && nEntries % scanWidth == 0)
    require(microEntries >= 2 && isPow2(microEntries))

    private val indexBits = log2Ceil(nEntries)
    private val nSlices = nEntries / scanWidth
    private val sliceBits = math.max(1, log2Ceil(nSlices))
    private val bankBits = math.max(1, log2Ceil(scanWidth))
    private val portBits = math.max(1, log2Ceil(nPorts))
    private val microIndexBits = log2Ceil(microEntries)

    val io = IO(new TLBIO(nPorts, nEntries, epochBits))

    private def pageMatch(entry: TLBEntry, vaddr: UInt): Bool =
        entry.e && (
            (entry.ps === 12.U && entry.vppn === vaddr(31, 13)) ||
            (entry.ps === 21.U && entry.vppn(18, 9) === vaddr(31, 22))
        )

    private def entryMatch(entry: TLBEntry, vaddr: UInt, asid: UInt): Bool =
        pageMatch(entry, vaddr) && (entry.g || entry.asid === asid)

    private def bankOfIndex(index: UInt): UInt =
        if (scanWidth == 1) 0.U else index(bankBits - 1, 0)

    private def rowOfIndex(index: UInt): UInt =
        if (nSlices == 1) 0.U
        else if (scanWidth == 1) index
        else index(indexBits - 1, bankBits)

    private def payloadFromEntry(entry: TLBEntry): MainTLBPayload = {
        val payload = Wire(new MainTLBPayload)
        payload.vppn := entry.vppn
        payload.ps := entry.ps
        payload.g := entry.g
        payload.asid := entry.asid
        payload.page0.ppn := entry.page0.ppn
        payload.page0.plv := entry.page0.plv
        payload.page0.mat := entry.page0.mat
        payload.page0.d := entry.page0.d
        payload.page0.v := entry.page0.v
        payload.page1.ppn := entry.page1.ppn
        payload.page1.plv := entry.page1.plv
        payload.page1.mat := entry.page1.mat
        payload.page1.d := entry.page1.d
        payload.page1.v := entry.page1.v
        payload
    }

    private def entryFromPayload(
        payload: MainTLBPayload,
        valid: Bool
    ): TLBEntry = {
        val entry = Wire(new TLBEntry)
        entry.vppn := payload.vppn
        entry.ps := payload.ps
        entry.g := payload.g
        entry.asid := payload.asid
        entry.e := valid
        entry.page0.ppn := payload.page0.ppn
        entry.page0.plv := payload.page0.plv
        entry.page0.mat := payload.page0.mat
        entry.page0.d := payload.page0.d
        entry.page0.v := payload.page0.v
        entry.page1.ppn := payload.page1.ppn
        entry.page1.plv := payload.page1.plv
        entry.page1.mat := payload.page1.mat
        entry.page1.d := payload.page1.d
        entry.page1.v := payload.page1.v
        entry
    }

    // A resident MicroTLB entry was filled only after a legal MainTLB hit, so
    // its enable and page-size legality are already represented by microValid.
    // Keep the fetch matcher narrow because it sits on the predicted-target
    // path; the other ports retain the defensive architectural matcher.
    private def residentFetchMicroMatch(
        entry: TLBEntry,
        vaddr: UInt,
        asid: UInt
    ): Bool =
        (entry.g || entry.asid === asid) && Mux(
            entry.ps === 12.U,
            entry.vppn === vaddr(31, 13),
            entry.vppn(18, 9) === vaddr(31, 22)
        )

    private def plvAllowed(dmw: DMW, plv: UInt): Bool = MuxLookup(
        plv,
        false.B
    )(Seq(
        0.U -> dmw.plv0.asBool,
        3.U -> dmw.plv3.asBool
    ))

    private def dmwHit(dmw: DMW, vaddr: UInt, plv: UInt, pageMode: Bool): Bool =
        pageMode && plvAllowed(dmw, plv) && vaddr(31, 29) === dmw.vseg

    private def selectedPage(entry: TLBEntry, vaddr: UInt): TLBPageEntry = {
        val odd = Mux(entry.ps === 12.U, vaddr(12), vaddr(21))
        Mux(odd, entry.page1, entry.page0)
    }

    private def physicalAddress(entry: TLBEntry, page: TLBPageEntry, vaddr: UInt): UInt =
        Mux(
            entry.ps === 12.U,
            Cat(page.ppn, vaddr(11, 0)),
            Cat(page.ppn(19, 9), vaddr(20, 0))
        )

    private def pageException(found: Bool, page: TLBPageEntry, access: UInt, plv: UInt): UInt = {
        val invalidCause = MuxLookup(access, ExpCode.PIF)(Seq(
            TLBAccess.Fetch -> ExpCode.PIF,
            TLBAccess.Load  -> ExpCode.PIL,
            TLBAccess.Store -> ExpCode.PIS
        ))
        MuxCase(0.U(8.W), Seq(
            !found -> ExpCode.TLBR,
            !page.v -> invalidCause,
            (plv > page.plv) -> ExpCode.PPI,
            (access === TLBAccess.Store && !page.d) -> ExpCode.PME
        ))
    }

    private def translatedResponse(
        entry: TLBEntry,
        found: Bool,
        vaddr: UInt,
        access: UInt,
        plv: UInt,
        token: UInt
    ): TLBResp = {
        val result = WireDefault(0.U.asTypeOf(new TLBResp))
        val page = selectedPage(entry, vaddr)
        val exception = pageException(found, page, access, plv)
        result.paddr := Mux(found, physicalAddress(entry, page, vaddr), 0.U)
        result.uncache := found && !exception.orR && page.mat === 0.U
        result.exception := exception
        result.token := token
        result
    }

    private val mainPayloadBanks =
        Seq.fill(scanWidth)(Mem(nSlices, new MainTLBPayload))
    val mainValidBanks = RegInit(VecInit.fill(scanWidth)(
        VecInit.fill(nSlices)(false.B)
    ))
    val fillIndex = RegInit(0.U(indexBits.W))

    val microValid = RegInit(VecInit.fill(nPorts)(
        VecInit.fill(microEntries)(false.B)
    ))
    val micro = Reg(Vec(nPorts, Vec(microEntries, new TLBEntry)))
    val microReplace = RegInit(VecInit.fill(nPorts)(0.U(microIndexBits.W)))

    val portRespValid = RegInit(VecInit.fill(nPorts)(false.B))
    val portRespBits = Reg(Vec(nPorts, new TLBResp))
    // Fast I-side responses never carry a pre-encoded exception.  Keep that
    // identity and its raw page metadata outside the ordinary response bundle
    // so clearing a fast response cannot be implemented on the synchronous
    // reset pins of the slow-path exception/token registers.
    val portRespFast = RegInit(VecInit.fill(nPorts)(false.B))
    private val fetchCheckMeta = Reg(new FetchCheckMeta)
    // On an I-side MicroTLB hit, stop the predicted-target path after the
    // parallel tag compares.  The registered one-hot and request address
    // select the resident entry during the existing response cycle, before
    // ICache captures the result.  This preserves hit latency and throughput
    // while removing page/PPN selection from the BTB-to-TLB register path.
    private val fetchMicroHitOH = Reg(UInt(microEntries.W))
    // The I-side fast result has its own payload registers. Slow MainTLB
    // responses remain in portRespBits, so fast request classification cannot
    // become the write enable of the slow response bank.
    private val fetchFastPaddr = Reg(UInt(32.W))
    private val fetchFastUncache = Reg(Bool())
    val missPending = RegInit(VecInit.fill(nPorts)(false.B))
    private val missReq = Reg(Vec(nPorts, new PendingTLBReq))
    val missAsid = Reg(Vec(nPorts, UInt(10.W)))
    val missPlv = Reg(Vec(nPorts, UInt(2.W)))
    val portEpoch = RegInit(VecInit.fill(nPorts)(0.U(epochBits.W)))

    object State extends ChiselEnum {
        val Idle, Lookup, Search, Read, Write, Fill, Invalidate = Value
    }

    val state = RegInit(State.Idle)
    val pending = Reg(new SysMemCmd(indexBits, epochBits))

    private val lookupReq = Reg(new PendingTLBReq)
    private val lookupToken = Reg(UInt(16.W))
    val lookupAsid = Reg(UInt(10.W))
    val lookupPlv = Reg(UInt(2.W))
    val lookupEpoch = Reg(UInt(epochBits.W))
    val lookupFound = RegInit(false.B)
    val lookupEntry = Reg(new TLBEntry)
    val lookupOwnerOH = RegInit(0.U(nPorts.W))
    val lookupFinishValid = RegInit(false.B)

    val slice = RegInit(0.U(sliceBits.W))
    val searchFound = RegInit(false.B)
    val searchIndex = Reg(UInt(indexBits.W))
    val searchMultiHit = RegInit(false.B)

    val respValid = RegInit(false.B)
    val respBits = Reg(new SysMemResp(indexBits, epochBits))

    // All MainTLB operations share one asynchronous read port per bank.
    // Scans address the current row in every bank; TLBRD changes only the row
    // and selects its indexed bank from the same four read results.
    val mainReadRow = Mux(
        state === State.Read,
        rowOfIndex(pending.tlbidx.index),
        slice
    )
    private val mainReadPayloads = VecInit(mainPayloadBanks.map(_(mainReadRow)))
    val mainReadEntries = VecInit((0 until scanWidth).map { bank =>
        entryFromPayload(
            mainReadPayloads(bank),
            mainValidBanks(bank)(mainReadRow)
        )
    })

    val heldFetchMicroPages = VecInit((0 until microEntries).map { slot =>
        selectedPage(micro(0)(slot), fetchFastPaddr)
    })
    val heldFetchMicroPaddrs = VecInit((0 until microEntries).map { slot =>
        physicalAddress(
            micro(0)(slot),
            heldFetchMicroPages(slot),
            fetchFastPaddr
        )
    })
    val heldFetchMicroPage = Mux1H(fetchMicroHitOH.asBools, heldFetchMicroPages)
    val heldFetchMicroPaddr = Mux1H(fetchMicroHitOH.asBools, heldFetchMicroPaddrs)
    val heldFetchMicroResponse =
        portRespFast(0) &&
            fetchCheckMeta.deferred &&
            fetchMicroHitOH.orR
    // The I-side request reserves its response slot before the MicroTLB hit
    // decision. A registered miss is suppressed here and handed to MainTLB on
    // the following edge; direct/DMW/MicroTLB hits keep their original latency.
    val fetchProvisionalMiss =
        portRespValid(0) &&
            portRespFast(0) &&
            fetchCheckMeta.deferred &&
            !fetchMicroHitOH.orR &&
            !io.flush &&
            !io.cancel(0)

    for (i <- 0 until nPorts) {
        // A redirect may cancel the old I-side transaction while presenting
        // its replacement in the same cycle. Treat that request as a new epoch:
        // old serialized lookup results fail the epoch check, while direct,
        // DMW and MicroTLB requests retain the normal one-cycle response.
        val replaceRequest = if (i == 0) {
            io.cancel(i) && io.req(i).valid && !io.flush
        } else {
            false.B
        }
        val responsePending = portRespValid(i)
        val visibleResponsePending = if (i == 0) {
            responsePending && !fetchProvisionalMiss
        } else {
            responsePending
        }
        io.resp(i).valid := visibleResponsePending && !io.flush && !io.cancel(i)
        io.resp(i).bits := portRespBits(i)
        if (i == 0) {
            io.resp(i).bits.paddr := Mux(
                portRespFast(i),
                Mux(
                    heldFetchMicroResponse,
                    heldFetchMicroPaddr,
                    fetchFastPaddr
                ),
                portRespBits(i).paddr
            )
            io.resp(i).bits.exception := Mux(
                portRespFast(i),
                0.U,
                portRespBits(i).exception
            )
            io.resp(i).bits.uncache := Mux(
                portRespFast(i),
                Mux(
                    heldFetchMicroResponse,
                    heldFetchMicroPage.mat === 0.U,
                    fetchFastUncache
                ),
                portRespBits(i).uncache
            )
            io.resp(i).bits.deferredFetchCheck :=
                heldFetchMicroResponse
            io.resp(i).bits.fetchPageValid :=
                heldFetchMicroResponse && heldFetchMicroPage.v
            io.resp(i).bits.fetchPagePlv := Mux(
                heldFetchMicroResponse,
                heldFetchMicroPage.plv,
                0.U
            )
            io.resp(i).bits.fetchRequestPlv := Mux(
                heldFetchMicroResponse,
                fetchCheckMeta.requestPlv,
                0.U
            )
        } else {
            io.resp(i).bits.deferredFetchCheck := false.B
            io.resp(i).bits.fetchPageValid := false.B
            io.resp(i).bits.fetchPagePlv := 0.U
            io.resp(i).bits.fetchRequestPlv := 0.U
        }

        val scanOwnsPort =
            (state === State.Lookup || lookupFinishValid) && lookupOwnerOH(i)
        // A serialized MainTLB lookup is intentionally not preempted. In that
        // rare case cancel invalidates it by epoch and Frontend retries the
        // redirect target on the next cycle.
        val portAvailable = !missPending(i) && !scanOwnsPort
        val responseSlotAvailable = if (i == 0) {
            replaceRequest ||
                !responsePending ||
                (io.resp(i).ready && !fetchProvisionalMiss)
        } else {
            replaceRequest || !responsePending || io.resp(i).ready
        }
        io.req(i).ready :=
            portAvailable &&
            responseSlotAvailable &&
            !io.req(i).bits.stall &&
            !io.flush &&
            (!io.cancel(i) || replaceRequest)

        val pageMode = !io.state.crmd.da.asBool && io.state.crmd.pg.asBool
        val hit0 = dmwHit(io.state.dmw0, io.req(i).bits.vaddr, io.state.crmd.plv, pageMode)
        val hit1 = !hit0 && dmwHit(
            io.state.dmw1,
            io.req(i).bits.vaddr,
            io.state.crmd.plv,
            pageMode
        )
        val directMode = !pageMode
        val microHits = VecInit((0 until microEntries).map { slot =>
            val residentMatch = if (i == 0) {
                residentFetchMicroMatch(
                    micro(i)(slot),
                    io.req(i).bits.vaddr,
                    io.state.asid
                )
            } else {
                entryMatch(
                    micro(i)(slot),
                    io.req(i).bits.vaddr,
                    io.state.asid
                )
            }
            microValid(i)(slot) && residentMatch
        })
        val microFound = microHits.asUInt.orR
        // Build each translation candidate beside its resident MicroTLB entry.
        // Selecting a narrow result avoids the serial wide-entry mux -> page
        // select -> paddr cone on the predicted-target fetch path.
        val microPages = VecInit((0 until microEntries).map { slot =>
            selectedPage(micro(i)(slot), io.req(i).bits.vaddr)
        })
        val microPaddrs = VecInit((0 until microEntries).map { slot =>
            physicalAddress(
                micro(i)(slot),
                microPages(slot),
                io.req(i).bits.vaddr
            )
        })
        val microPage = Mux1H(microHits, microPages)
        val microPaddr = Mux1H(microHits, microPaddrs)
        when(io.resp(i).fire) {
            portRespValid(i) := false.B
        }

        when(io.req(i).fire) {
            if (i == 0) {
                // The registered MicroTLB hit vector validates or cancels this
                // provisional response on the next cycle.
                portRespValid(i) := true.B
            } else {
                when(directMode || hit0 || hit1 || microFound) {
                    portRespValid(i) := true.B
                }.otherwise {
                    missPending(i) := true.B
                }
            }

            // The request identity is independent of the translation outcome.
            // Capture it before the direct/DMW/MicroTLB decision so the full
            // page-match cone cannot become the token register's write control.
            // A miss keeps response valid low and overwrites this field with the
            // same saved token when the serialized lookup eventually completes.
            portRespBits(i).token := io.req(i).bits.token
            portRespFast(i) := true.B

            missReq(i).vaddr := io.req(i).bits.vaddr
            missReq(i).access := io.req(i).bits.access
            missAsid(i) := io.state.asid
            missPlv(i) := io.state.crmd.plv

            if (i == 0) {
                fetchMicroHitOH := microHits.asUInt
                fetchFastPaddr := Mux(
                    hit0,
                    Cat(io.state.dmw0.pseg, io.req(i).bits.vaddr(28, 0)),
                    Mux(
                        hit1,
                        Cat(io.state.dmw1.pseg, io.req(i).bits.vaddr(28, 0)),
                        io.req(i).bits.vaddr
                    )
                )
                fetchFastUncache := Mux(
                    directMode,
                    io.state.crmd.datf === 0.U,
                    Mux(
                        hit0,
                        io.state.dmw0.mat === 0.U,
                        io.state.dmw1.mat === 0.U
                    )
                )
                fetchCheckMeta.deferred :=
                    pageMode && !hit0 && !hit1
                fetchCheckMeta.requestPlv := io.state.crmd.plv
            }

            when(directMode) {
                if (i != 0) {
                    val directUncache = Mux(
                        io.req(i).bits.access === TLBAccess.Fetch,
                        io.state.crmd.datf === 0.U,
                        io.state.crmd.datm === 0.U
                    )

                    portRespBits(i).paddr := io.req(i).bits.vaddr
                    portRespBits(i).uncache := directUncache
                    portRespBits(i).exception := 0.U
                }
            }.elsewhen(hit0 || hit1) {
                if (i != 0) {
                    val dmwPaddr = Mux(
                        hit0,
                        Cat(io.state.dmw0.pseg, io.req(i).bits.vaddr(28, 0)),
                        Cat(io.state.dmw1.pseg, io.req(i).bits.vaddr(28, 0))
                    )
                    val dmwUncache =
                        Mux(hit0, io.state.dmw0.mat, io.state.dmw1.mat) === 0.U

                    portRespBits(i).paddr := dmwPaddr
                    portRespBits(i).uncache := dmwUncache
                    portRespBits(i).exception := 0.U
                }
            }.elsewhen(microFound) {
                val exception = pageException(
                    true.B,
                    microPage,
                    io.req(i).bits.access,
                    io.state.crmd.plv
                )

                if (i != 0) {
                    portRespBits(i).paddr := microPaddr
                    portRespBits(i).uncache :=
                        !exception.orR && microPage.mat === 0.U
                    portRespBits(i).exception := exception
                }
            }
        }

        if (i == 0) {
            when(fetchProvisionalMiss) {
                portRespValid(i) := false.B
                missPending(i) := true.B
            }

            when(!reset.asBool) {
                when(fetchProvisionalMiss) {
                    assert(!io.resp(i).valid)
                    assert(!io.resp(i).fire)
                    assert(!io.req(i).ready)
                    assert(!io.req(i).fire)
                }
                when(
                    io.resp(i).valid &&
                    portRespFast(i) &&
                    fetchCheckMeta.deferred
                ) {
                    assert(fetchMicroHitOH.orR)
                }
            }
        }

        when(io.flush || (io.cancel(i) && !io.req(i).fire)) {
            portRespValid(i) := false.B
            missPending(i) := false.B
        }
        when(io.flush || io.cancel(i)) {
            portEpoch(i) := portEpoch(i) + 1.U
        }

        when(!reset.asBool) {
            assert(PopCount(microHits) <= 1.U, "MicroTLB contains multiple matching entries")
            if (i == 0) {
                for (slot <- 0 until microEntries) {
                    when(microValid(i)(slot)) {
                        assert(micro(i)(slot).e,
                            "Fetch MicroTLB valid entry must remain enabled")
                        assert(micro(i)(slot).ps === 12.U || micro(i)(slot).ps === 21.U,
                            "Fetch MicroTLB valid entry must have a supported page size")
                    }
                }
            }
        }
    }

    io.sysCmd.ready :=
        state === State.Idle && !respValid && !lookupFinishValid && !io.flush
    io.sysResp.valid := respValid
    io.sysResp.bits := respBits

    when(io.sysResp.fire) {
        respValid := false.B
    }

    private def commandEntry(cmd: SysMemCmd): TLBEntry = {
        val entry = WireDefault(0.U.asTypeOf(new TLBEntry))
        entry.e := cmd.inTlbRefill || !cmd.tlbidx.ne.asBool
        entry.ps := cmd.tlbidx.ps
        entry.vppn := cmd.tlbehi.vppn
        entry.asid := cmd.asid
        entry.g := cmd.tlbelo0.g.asBool && cmd.tlbelo1.g.asBool

        entry.page0.ppn := cmd.tlbelo0.ppn
        entry.page0.plv := cmd.tlbelo0.plv
        entry.page0.mat := cmd.tlbelo0.mat
        entry.page0.d := cmd.tlbelo0.d.asBool
        entry.page0.v := cmd.tlbelo0.v.asBool

        entry.page1.ppn := cmd.tlbelo1.ppn
        entry.page1.plv := cmd.tlbelo1.plv
        entry.page1.mat := cmd.tlbelo1.mat
        entry.page1.d := cmd.tlbelo1.d.asBool
        entry.page1.v := cmd.tlbelo1.v.asBool
        entry
    }

    val mainWriteEnable =
        (state === State.Write || state === State.Fill) && !io.flush
    val mainWriteIndex = Mux(
        state === State.Fill,
        fillIndex,
        pending.tlbidx.index
    )
    val mainWriteBank = bankOfIndex(mainWriteIndex)
    val mainWriteRow = rowOfIndex(mainWriteIndex)
    val mainWriteEntry = commandEntry(pending)
    private val mainWritePayload = payloadFromEntry(mainWriteEntry)

    for (bank <- 0 until scanWidth) {
        when(mainWriteEnable && mainWriteBank === bank.U) {
            mainPayloadBanks(bank).write(mainWriteRow, mainWritePayload)
            mainValidBanks(bank)(mainWriteRow) := mainWriteEntry.e
        }
    }

    private def startResponse(): Unit = {
        respBits := 0.U.asTypeOf(respBits)
        respBits.robPtr := pending.robPtr
        respBits.epoch := pending.epoch
        respValid := true.B
        state := State.Idle
    }

    val pendingMisses = missPending.asUInt
    val nextMissOH = PriorityEncoderOH(pendingMisses)
    val nextMissPort = OHToUInt(nextMissOH)

    val finishReservesSysCmd = lookupFinishValid && io.sysCmd.valid
    when(state === State.Idle && !respValid && !finishReservesSysCmd) {
        when(io.sysCmd.fire) {
            pending := io.sysCmd.bits
            slice := 0.U
            searchFound := false.B
            searchMultiHit := false.B

            switch(io.sysCmd.bits.op) {
                is(SystemOp.TLBSRCH) { state := State.Search }
                is(SystemOp.TLBRD)   { state := State.Read }
                is(SystemOp.TLBWR)   { state := State.Write }
                is(SystemOp.TLBFILL) { state := State.Fill }
                is(SystemOp.INVTLB)  { state := State.Invalidate }
            }

            when(
                io.sysCmd.bits.op === SystemOp.TLBWR ||
                io.sysCmd.bits.op === SystemOp.TLBFILL ||
                io.sysCmd.bits.op === SystemOp.INVTLB
            ) {
                for (port <- 0 until nPorts; slot <- 0 until microEntries) {
                    microValid(port)(slot) := false.B
                }
            }

            assert(
                io.sysCmd.bits.op === SystemOp.TLBSRCH ||
                    io.sysCmd.bits.op === SystemOp.TLBRD ||
                    io.sysCmd.bits.op === SystemOp.TLBWR ||
                    io.sysCmd.bits.op === SystemOp.TLBFILL ||
                    io.sysCmd.bits.op === SystemOp.INVTLB,
                "MainTLB received a non-TLB system command"
            )
        }.elsewhen(pendingMisses.orR) {
            lookupOwnerOH := nextMissOH
            lookupReq := missReq(nextMissPort)
            lookupToken := portRespBits(nextMissPort).token
            lookupAsid := missAsid(nextMissPort)
            lookupPlv := missPlv(nextMissPort)
            lookupEpoch := portEpoch(nextMissPort)
            lookupFound := false.B
            slice := 0.U
            missPending(nextMissPort) := false.B
            state := State.Lookup
        }
    }

    val sliceBase = (slice * scanWidth.U)(indexBits - 1, 0)
    val lookupSliceHits = VecInit((0 until scanWidth).map { lane =>
        entryMatch(mainReadEntries(lane), lookupReq.vaddr, lookupAsid)
    })
    val lookupSliceFound = lookupSliceHits.asUInt.orR
    val lookupSliceEntry = Mux1H(lookupSliceHits, (0 until scanWidth).map { lane =>
        mainReadEntries(lane)
    })
    val lastSlice = slice === (nSlices - 1).U

    // The last MainTLB slice only captures a registered result.  Response
    // generation and MicroTLB refill use this sidecar on the following cycle,
    // while the scanner may already accept a miss from another client.
    lookupFinishValid := false.B

    val lookupFinishResponse = translatedResponse(
        lookupEntry,
        lookupFound,
        lookupReq.vaddr,
        lookupReq.access,
        lookupPlv,
        lookupToken
    )
    for (client <- 0 until nPorts) {
        val finishCurrent =
            lookupFinishValid &&
            lookupOwnerOH(client) &&
            portEpoch(client) === lookupEpoch &&
            !io.cancel(client) &&
            !io.flush

        when(finishCurrent) {
            assert(!portRespValid(client),
                "MainTLB finish must own an empty client response slot")
            portRespBits(client) := lookupFinishResponse
            portRespFast(client) := false.B
            portRespValid(client) := true.B

            when(lookupFound) {
                micro(client)(microReplace(client)) := lookupEntry
                microValid(client)(microReplace(client)) := true.B
                microReplace(client) := microReplace(client) + 1.U
            }
        }
    }

    when(state === State.Lookup) {
        val foundNext = lookupFound || lookupSliceFound
        lookupFound := foundNext
        when(!lookupFound && lookupSliceFound) {
            lookupEntry := lookupSliceEntry
        }

        when(lastSlice) {
            lookupFinishValid := true.B
            state := State.Idle
        }.otherwise {
            slice := slice + 1.U
        }

        assert(PopCount(lookupSliceHits) <= 1.U, "MainTLB slice contains multiple matching entries")
    }

    val searchVaddr = Cat(pending.tlbehi.vppn, 0.U(13.W))
    val searchSliceHits = VecInit((0 until scanWidth).map { lane =>
        entryMatch(mainReadEntries(lane), searchVaddr, pending.asid)
    })
    val searchSliceHitCount = PopCount(searchSliceHits)
    val searchSliceFound = searchSliceHits.asUInt.orR
    val searchSliceIndex =
        (sliceBase + PriorityEncoder(searchSliceHits.asUInt))(indexBits - 1, 0)

    when(state === State.Search) {
        val foundNext = searchFound || searchSliceFound
        val indexNext = Mux(searchFound, searchIndex, searchSliceIndex)
        val multiHitNext = searchMultiHit ||
            searchSliceHitCount > 1.U ||
            (searchFound && searchSliceFound)

        searchFound := foundNext
        when(!searchFound && searchSliceFound) {
            searchIndex := searchSliceIndex
        }
        searchMultiHit := multiHitNext

        when(lastSlice) {
            startResponse()
            respBits.tlbSearch.valid := true.B
            respBits.tlbSearch.bits.found := foundNext
            respBits.tlbSearch.bits.index := Mux(foundNext, indexNext, 0.U)
            assert(!multiHitNext, "MainTLB contains multiple matching entries")
        }.otherwise {
            slice := slice + 1.U
        }
    }

    when(state === State.Read) {
        val rawEntry = mainReadEntries(bankOfIndex(pending.tlbidx.index))
        val entry = Mux(
            rawEntry.e,
            rawEntry,
            0.U.asTypeOf(new TLBEntry)
        )
        val elo0 = WireDefault(0.U.asTypeOf(new TLBELO))
        val elo1 = WireDefault(0.U.asTypeOf(new TLBELO))

        elo0.ppn := entry.page0.ppn
        elo0.plv := entry.page0.plv
        elo0.mat := entry.page0.mat
        elo0.d := entry.page0.d
        elo0.v := entry.page0.v
        elo0.g := entry.g
        elo1.ppn := entry.page1.ppn
        elo1.plv := entry.page1.plv
        elo1.mat := entry.page1.mat
        elo1.d := entry.page1.d
        elo1.v := entry.page1.v
        elo1.g := entry.g

        startResponse()
        respBits.tlbRead.valid := true.B
        respBits.tlbRead.bits.entryValid := rawEntry.e
        respBits.tlbRead.bits.ps := entry.ps
        respBits.tlbRead.bits.vppn := entry.vppn
        respBits.tlbRead.bits.asid := entry.asid
        respBits.tlbRead.bits.elo0 := elo0
        respBits.tlbRead.bits.elo1 := elo1
    }

    when(state === State.Write && !io.flush) {
        startResponse()
    }

    when(state === State.Fill && !io.flush) {
        val committedFillIndex = fillIndex
        fillIndex := fillIndex + 1.U
        startResponse()
        respBits.tlbFill.valid := true.B
        respBits.tlbFill.bits := committedFillIndex
    }

    when(state === State.Invalidate && !io.flush) {
        for (lane <- 0 until scanWidth) {
            val entry = mainReadEntries(lane)
            val global = entry.g
            val sameAsid = entry.asid === pending.asid
            val samePage = pageMatch(entry, pending.vaddr)
            val invalidate = MuxLookup(pending.auxOp, false.B)(Seq(
                0.U -> true.B,
                1.U -> true.B,
                2.U -> global,
                3.U -> !global,
                4.U -> (!global && sameAsid),
                5.U -> (!global && sameAsid && samePage),
                6.U -> ((global || sameAsid) && samePage)
            ))

            when(entry.e && invalidate) {
                mainValidBanks(lane)(slice) := false.B
            }
        }

        when(lastSlice) {
            startResponse()
        }.otherwise {
            slice := slice + 1.U
        }
    }

    when(io.flush) {
        state := State.Idle
        respValid := false.B
        lookupFinishValid := false.B
    }

}
