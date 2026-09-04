package CPUSTC.backend

import chisel3._
import chisel3.util._

class TLBPageEntry extends Bundle {
  val ppn = UInt(20.W)
  val plv = UInt(2.W)
  val mat = UInt(2.W)
  val d   = Bool()
  val v   = Bool()
}

class TLBEntry extends Bundle {
  val vppn = UInt(19.W)
  val ps   = UInt(6.W)
  val g    = Bool()
  val asid = UInt(10.W)
  val e    = Bool()
  val page0 = new TLBPageEntry
  val page1 = new TLBPageEntry
}

class TLBLookupReq extends Bundle {
  val vaddr = UInt(32.W)
  val asid  = UInt(10.W)
}

class TLBLookupResp(indexBits: Int) extends Bundle {
  val found    = Bool()
  val multiHit = Bool()
  val index    = UInt(indexBits.W)
  val ps       = UInt(6.W)
  val paddr    = UInt(32.W)
  val plv      = UInt(2.W)
  val mat      = UInt(2.W)
  val d        = Bool()
  val v        = Bool()
}

object TLBInvalidateOp {
  val width = 5
  val all0       = 0.U(width.W)
  val all1       = 1.U(width.W)
  val global     = 2.U(width.W)
  val nonGlobal  = 3.U(width.W)
  val asid       = 4.U(width.W)
  val asidVaddr  = 5.U(width.W)
  val vaddr      = 6.U(width.W)
}

class TLBInvalidateReq extends Bundle {
  val op    = UInt(TLBInvalidateOp.width.W)
  val asid  = UInt(10.W)
  val vaddr = UInt(32.W)
}

class TLBControlIO(indexBits: Int) extends Bundle {
  val search = Input(Bool())
  val read   = Input(Bool())
  val write  = Input(Bool())
  val fill   = Input(Bool())
  val inv    = Flipped(Valid(new TLBInvalidateReq))

  val tlbidx  = Input(new TLBIDX(indexBits))
  val tlbehi  = Input(new TLBEHI)
  val tlbelo0 = Input(new TLBELO)
  val tlbelo1 = Input(new TLBELO)
  val asid    = Input(UInt(10.W))

  val searchResult = Output(Valid(new TLBSrchResult(indexBits)))
  val readResult   = Output(Valid(new TLBRDResult))
  val fillIndex    = Output(UInt(indexBits.W))
}

class TLBIO(nPorts: Int, nEntries: Int) extends Bundle {
  private val indexBits = log2Ceil(nEntries)
  val lookup  = Vec(nPorts, new Bundle {
    val req  = Input(Valid(new TLBLookupReq))
    val resp = Output(Valid(new TLBLookupResp(indexBits)))
  })
  val control = new TLBControlIO(indexBits)
}

/** Fully-associative LoongArch dual-page TLB table.
  * Address-mode selection and access-permission exceptions belong in the MMU.
  */
class TLB(nPorts: Int = 2, nEntries: Int = 32) extends Module {
  require(nPorts > 0)
  require(nEntries >= 2 && isPow2(nEntries))

  private val indexBits = log2Ceil(nEntries)
  val io = IO(new TLBIO(nPorts, nEntries))

  private val entries = RegInit(VecInit(Seq.fill(nEntries)(0.U.asTypeOf(new TLBEntry))))
  private val fillCounter = RegInit(0.U(indexBits.W))

  private def pageMatch(entry: TLBEntry, vaddr: UInt): Bool = {
    val entryAddress = Cat(entry.vppn, 0.U(13.W))
    (vaddr >> (entry.ps +& 1.U)) === (entryAddress >> (entry.ps +& 1.U))
  }

  private def entryMatch(entry: TLBEntry, vaddr: UInt, asid: UInt): Bool =
    entry.e && (entry.g || entry.asid === asid) &&
      pageMatch(entry, vaddr)

  for (port <- io.lookup) {
    val hits = VecInit(entries.map(entry => entryMatch(entry, port.req.bits.vaddr, port.req.bits.asid)))
    val hitBits = hits.asUInt
    val found = hitBits.orR
    val index = PriorityEncoder(hitBits)
    val selected = Mux1H(hits, entries)
    val oddPage = (port.req.bits.vaddr >> selected.ps)(0)
    val page = Mux(oddPage, selected.page1, selected.page0)
    val pageMask = ((1.U(33.W) << selected.ps) - 1.U)(31, 0)
    val pageBase = Cat(page.ppn, 0.U(12.W)) & ~pageMask

    port.resp.valid         := port.req.valid
    port.resp.bits.found    := found
    port.resp.bits.multiHit := PopCount(hitBits) > 1.U
    port.resp.bits.index    := index
    port.resp.bits.ps       := Mux(found, selected.ps, 0.U)
    port.resp.bits.paddr    := Mux(found, pageBase | (port.req.bits.vaddr & pageMask), 0.U)
    port.resp.bits.plv      := Mux(found, page.plv, 0.U)
    port.resp.bits.mat      := Mux(found, page.mat, 0.U)
    port.resp.bits.d        := found && page.d
    port.resp.bits.v        := found && page.v
  }

  val searchHits = VecInit(entries.map(entry =>
    entryMatch(entry, Cat(io.control.tlbehi.vppn, 0.U(13.W)), io.control.asid)))
  io.control.searchResult.valid      := io.control.search
  io.control.searchResult.bits.found := searchHits.asUInt.orR
  io.control.searchResult.bits.index := PriorityEncoder(searchHits.asUInt)

  val readEntry = entries(io.control.tlbidx.index)
  io.control.readResult.valid           := io.control.read
  io.control.readResult.bits.entryValid := readEntry.e
  io.control.readResult.bits.ps         := readEntry.ps
  io.control.readResult.bits.vppn       := readEntry.vppn
  io.control.readResult.bits.asid       := readEntry.asid
  io.control.readResult.bits.elo0       := 0.U.asTypeOf(new TLBELO)
  io.control.readResult.bits.elo1       := 0.U.asTypeOf(new TLBELO)
  io.control.readResult.bits.elo0.ppn   := readEntry.page0.ppn
  io.control.readResult.bits.elo0.plv   := readEntry.page0.plv
  io.control.readResult.bits.elo0.mat   := readEntry.page0.mat
  io.control.readResult.bits.elo0.d     := readEntry.page0.d
  io.control.readResult.bits.elo0.v     := readEntry.page0.v
  io.control.readResult.bits.elo0.g     := readEntry.g
  io.control.readResult.bits.elo1.ppn   := readEntry.page1.ppn
  io.control.readResult.bits.elo1.plv   := readEntry.page1.plv
  io.control.readResult.bits.elo1.mat   := readEntry.page1.mat
  io.control.readResult.bits.elo1.d     := readEntry.page1.d
  io.control.readResult.bits.elo1.v     := readEntry.page1.v
  io.control.readResult.bits.elo1.g     := readEntry.g
  io.control.fillIndex                  := fillCounter

  val writeEntry = Wire(new TLBEntry)
  writeEntry.e      := !io.control.tlbidx.ne.asBool
  writeEntry.ps     := io.control.tlbidx.ps
  writeEntry.vppn   := io.control.tlbehi.vppn
  writeEntry.asid   := io.control.asid
  writeEntry.g      := io.control.tlbelo0.g.asBool && io.control.tlbelo1.g.asBool
  writeEntry.page0.ppn := io.control.tlbelo0.ppn
  writeEntry.page0.plv := io.control.tlbelo0.plv
  writeEntry.page0.mat := io.control.tlbelo0.mat
  writeEntry.page0.d   := io.control.tlbelo0.d.asBool
  writeEntry.page0.v   := io.control.tlbelo0.v.asBool
  writeEntry.page1.ppn := io.control.tlbelo1.ppn
  writeEntry.page1.plv := io.control.tlbelo1.plv
  writeEntry.page1.mat := io.control.tlbelo1.mat
  writeEntry.page1.d   := io.control.tlbelo1.d.asBool
  writeEntry.page1.v   := io.control.tlbelo1.v.asBool

  when(io.control.inv.valid) {
    for (entry <- entries) {
      val global = entry.g
      val sameAsid = entry.asid === io.control.inv.bits.asid
      val samePage = pageMatch(entry, io.control.inv.bits.vaddr)
      val invalidate = MuxLookup(io.control.inv.bits.op, false.B)(Seq(
        TLBInvalidateOp.all0      -> true.B,
        TLBInvalidateOp.all1      -> true.B,
        TLBInvalidateOp.global    -> global,
        TLBInvalidateOp.nonGlobal -> !global,
        TLBInvalidateOp.asid      -> (!global && sameAsid),
        TLBInvalidateOp.asidVaddr -> (!global && sameAsid && samePage),
        TLBInvalidateOp.vaddr     -> ((global || sameAsid) && samePage)
      ))
      when(entry.e && invalidate) {
        entry.e := false.B
      }
    }
  }.elsewhen(io.control.write) {
    entries(io.control.tlbidx.index) := writeEntry
  }.elsewhen(io.control.fill) {
    entries(fillCounter) := writeEntry
    fillCounter := fillCounter + 1.U
  }

  assert(PopCount(Seq(io.control.write, io.control.fill, io.control.inv.valid)) <= 1.U,
    "only one TLB mutation may commit per cycle")
  assert(!(io.control.search && io.control.read), "TLBSRCH and TLBRD cannot commit together")
}
