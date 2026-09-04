package CPUSTC.backend

import chisel3._
import chisel3.util._

class AddressFastMapEntry extends Bundle {
  val resolved  = Bool()
  val cacheable = Bool()
  val pseg      = UInt(3.W)
}

class AddressFastMap extends Bundle {
  val byVseg = Vec(8, new AddressFastMapEntry)
}

object AddressFastMap {
  private def plvAllowed(crmd: CRMD, dmw: DMW): Bool = MuxLookup(
    crmd.plv,
    false.B
  )(Seq(
    0.U -> dmw.plv0.asBool,
    3.U -> dmw.plv3.asBool
  ))

  /** Predecodes the rare CSR address-mode state into a small VSEG table. */
  def build(crmd: CRMD, dmw0: DMW, dmw1: DMW): AddressFastMap = {
    val result = Wire(new AddressFastMap)
    val pageMode = !crmd.da.asBool && crmd.pg.asBool
    val directUncache =
      crmd.da.asBool && !crmd.pg.asBool && crmd.datm === 0.U

    for (segment <- 0 until 8) {
      val segmentValue = segment.U(3.W)
      val hit0 = pageMode &&
        plvAllowed(crmd, dmw0) && dmw0.vseg === segmentValue
      val hit1 = pageMode && !hit0 &&
        plvAllowed(crmd, dmw1) && dmw1.vseg === segmentValue
      val translationMiss = pageMode && !hit0 && !hit1
      val selectedMat = Mux(hit0, dmw0.mat, dmw1.mat)
      val mappedUncache = (hit0 || hit1) && selectedMat === 0.U

      result.byVseg(segment).resolved := !translationMiss
      result.byVseg(segment).cacheable :=
        !translationMiss && !directUncache && !mappedUncache
      result.byVseg(segment).pseg := Mux(
        hit0,
        dmw0.pseg,
        Mux(hit1, dmw1.pseg, segmentValue)
      )
    }

    result
  }
}

class AddressTranslationState extends Bundle {
  val crmd = new CRMD
  val asid = UInt(10.W)
  val dmw0 = new DMW
  val dmw1 = new DMW
}

class AddressModeTranslatorIO extends Bundle {
  val vaddr = Input(UInt(32.W))
  val state = Input(new AddressTranslationState)

  val paddr = Output(UInt(32.W))
  val mat = Output(UInt(2.W))
  val dmwHit = Output(Bool())
  val translationMiss = Output(Bool())
}

/** Selects direct-address or DMW translation. Page-table lookup belongs to the
  * later full MMU wrapper; a mapped-mode DMW miss is reported to that boundary.
  */
class AddressModeTranslator extends Module {
  val io = IO(new AddressModeTranslatorIO)

  private val pageMode = !io.state.crmd.da.asBool && io.state.crmd.pg.asBool

  private def plvAllowed(dmw: DMW): Bool = MuxLookup(
    io.state.crmd.plv,
    false.B
  )(Seq(
    0.U -> dmw.plv0.asBool,
    3.U -> dmw.plv3.asBool
  ))

  private def hits(dmw: DMW): Bool =
    pageMode &&
      plvAllowed(dmw) &&
      io.vaddr(31, 29) === dmw.vseg

  val hit0 = hits(io.state.dmw0)
  val hit1 = !hit0 && hits(io.state.dmw1)

  io.paddr := MuxCase(io.vaddr, Seq(
    hit0 -> Cat(io.state.dmw0.pseg, io.vaddr(28, 0)),
    hit1 -> Cat(io.state.dmw1.pseg, io.vaddr(28, 0))
  ))
  io.mat := Mux(hit0, io.state.dmw0.mat, io.state.dmw1.mat)
  io.dmwHit := hit0 || hit1
  io.translationMiss := pageMode && !io.dmwHit
}
