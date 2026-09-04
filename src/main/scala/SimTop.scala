package CPUSTC

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Path, Paths}
import scala.annotation.tailrec
import scala.io.StdIn

import CPUSTC.config.Commit._
import CPUSTC.config.Decode._
import CPUSTC.config.EXEOp._
import CPUSTC.config.RegisterFile._
import CPUSTC.backend.CSRDebugState
import CPUSTC.config.Consts.CSR_ESTAT
import CPUSTC.backend.execute.CounterDebugEvent
import CPUSTC.backend.rob.RobPtr
import CPUSTC.memory.LoadDebugEvent
import CPUSTC.memory.StoreCommitTrace
import CPUSTC.memory.TlbFillDebugEvent
import CPUSTC.memory.backend.DCache
import CPUSTC.memory.MemSysConfig

class SimCommitPort extends Bundle {
    val valid = Bool()
    val pc = UInt(dataWidth.W)
    val inst = UInt(dataWidth.W)
    val data = UInt(dataWidth.W)
    val rd_valid = Bool()
    val rd = UInt(5.W)
    val exception = Bool()
    val exception_code = UInt(6.W)
    val is_tlbfill = Bool()
    val tlbfill_index = UInt(5.W)
    val is_cnt = Bool()
    val timer_64_value = UInt(64.W)
    val csr_rstat = Bool()
    val csr_data = UInt(32.W)
}

class SimExceptionPort extends Bundle {
    val valid = Bool()
    val pc = UInt(dataWidth.W)
    val code = UInt(6.W)
    val ertn = Bool()
    val interrupt = UInt(11.W)
    val inst = UInt(32.W)
    val subcode = UInt(9.W)
    val badv_valid = Bool()
    val badv = UInt(32.W)
}

class SimLoadPort extends Bundle {
    val valid = UInt(8.W)
    val paddr = UInt(32.W)
    val vaddr = UInt(32.W)
}

class SimStorePort extends Bundle {
    val valid = UInt(8.W)
    val paddr = UInt(32.W)
    val vaddr = UInt(32.W)
    val data = UInt(32.W)
}

class SimAXIReadAddr extends Bundle {
    val valid = Output(Bool())
    val ready = Input(Bool())
    val addr = Output(UInt(32.W))
    val len = Output(UInt(8.W))
    val size = Output(UInt(3.W))
    val burst = Output(UInt(2.W))
    val id = Output(UInt(4.W))
}

class SimAXIReadData extends Bundle {
    val valid = Input(Bool())
    val ready = Output(Bool())
    val data = Input(UInt(32.W))
    val last = Input(Bool())
    val resp = Input(UInt(2.W))
    val id = Input(UInt(4.W))
}

class SimAXIWriteAddr extends Bundle {
    val valid = Output(Bool())
    val ready = Input(Bool())
    val addr = Output(UInt(32.W))
    val len = Output(UInt(8.W))
    val size = Output(UInt(3.W))
    val burst = Output(UInt(2.W))
    val id = Output(UInt(4.W))
}

class SimAXIWriteData extends Bundle {
    val valid = Output(Bool())
    val ready = Input(Bool())
    val data = Output(UInt(32.W))
    val strb = Output(UInt(4.W))
    val last = Output(Bool())
}

class SimAXIWriteResp extends Bundle {
    val valid = Input(Bool())
    val ready = Output(Bool())
    val resp = Input(UInt(2.W))
    val id = Input(UInt(4.W))
}

class SimAXI extends Bundle {
    val ar = new SimAXIReadAddr
    val r = new SimAXIReadData
    val aw = new SimAXIWriteAddr
    val w = new SimAXIWriteData
    val b = new SimAXIWriteResp
}

class SimTopIO extends Bundle {
    val hardwareInterrupt = Input(UInt(8.W))
    val axi = new SimAXI
    val cmt = Output(Vec(4, new SimCommitPort))
    val excp = Output(new SimExceptionPort)
    val cmt_rf = Output(Vec(32, UInt(dataWidth.W)))
    val csr = Output(new CSRDebugState)
    val load = Output(Vec(4, new SimLoadPort))
    val store = Output(Vec(4, new SimStorePort))
    val cmt_tlbfill_valid = Output(Bool())
    val cmt_tlbfill_idx = Output(UInt(5.W))
}

class CPU(
    maxCommitPerCycle: Int = ncmt,
    useBlackBoxRam: Boolean = false,
    enablePerfCounters: Boolean = false,
    perfMeasurementPcs: Option[(BigInt, BigInt)] = None,
    perfMeasurementByTimer: Boolean = false,
    perfMeasurementPcInstrs: Option[((BigInt, BigInt), (BigInt, BigInt))] = None,
    memSysConfig: MemSysConfig = MemSysConfig()
) extends Module {
    val io = IO(new SimTopIO)

    val core = Module(new CPUSTCore(
        useBlackBoxRam = useBlackBoxRam,
        enableCommitDebug = true,
        enablePerfCounters = enablePerfCounters,
        maxCommitPerCycle = maxCommitPerCycle,
        perfMeasurementPcs = perfMeasurementPcs,
        perfMeasurementByTimer = perfMeasurementByTimer,
        perfMeasurementPcInstrs = perfMeasurementPcInstrs,
        memSysConfig = memSysConfig
    ))

    core.io.hardRedirect.valid := false.B
    core.io.hardRedirect.bits := 0.U
    core.io.hardwareInterrupt := io.hardwareInterrupt

    io.axi.ar.valid := core.io.axi.arvalid
    io.axi.ar.addr := core.io.axi.araddr
    io.axi.ar.len := core.io.axi.arlen
    io.axi.ar.size := core.io.axi.arsize
    io.axi.ar.burst := core.io.axi.arburst
    io.axi.ar.id := core.io.axi.arid
    core.io.axi.arready := io.axi.ar.ready

    core.io.axi.rvalid := io.axi.r.valid
    io.axi.r.ready := core.io.axi.rready
    core.io.axi.rdata := io.axi.r.data
    core.io.axi.rlast := io.axi.r.last
    core.io.axi.rresp := io.axi.r.resp
    core.io.axi.rid := io.axi.r.id

    io.axi.aw.valid := core.io.axi.awvalid
    io.axi.aw.addr := core.io.axi.awaddr
    io.axi.aw.len := core.io.axi.awlen
    io.axi.aw.size := core.io.axi.awsize
    io.axi.aw.burst := core.io.axi.awburst
    io.axi.aw.id := core.io.axi.awid
    core.io.axi.awready := io.axi.aw.ready

    io.axi.w.valid := core.io.axi.wvalid
    io.axi.w.data := core.io.axi.wdata
    io.axi.w.strb := core.io.axi.wstrb
    io.axi.w.last := core.io.axi.wlast
    core.io.axi.wready := io.axi.w.ready

    core.io.axi.bvalid := io.axi.b.valid
    io.axi.b.ready := core.io.axi.bready
    core.io.axi.bresp := io.axi.b.resp
    core.io.axi.bid := io.axi.b.id

    io.cmt := 0.U.asTypeOf(io.cmt)
    io.load := 0.U.asTypeOf(io.load)
    io.store := 0.U.asTypeOf(io.store)
    io.csr := core.io.csrDebugState
    io.excp.valid := core.io.exceptionTrace.valid
    io.excp.pc := core.io.exceptionTrace.bits.err_pc
    io.excp.code := core.io.exceptionTrace.bits.ecode
    io.excp.ertn := core.io.csrDebugErtn
    io.excp.interrupt := Mux(
        core.io.exceptionTrace.valid && core.io.exceptionTrace.bits.ecode === 0.U,
        core.io.csrDebugInterrupt,
        0.U
    )
    io.excp.inst := core.io.exceptionTrace.bits.instr
    io.excp.subcode := core.io.exceptionTrace.bits.esubcode
    io.excp.badv_valid := core.io.exceptionTrace.bits.badvValid
    io.excp.badv := core.io.exceptionTrace.bits.badv

    val loadValid = RegInit(VecInit(Seq.fill(nrobQ)(
        VecInit(Seq.fill(ndcd)(false.B))
    )))
    val loadEvent = Reg(Vec(nrobQ, Vec(ndcd, new LoadDebugEvent)))
    for (port <- 0 until core.io.loadDebug.length) {
        val event = core.io.loadDebug(port)
        when(event.valid) {
            loadValid(event.bits.robPtr.offset)(event.bits.robPtr.qidx) := true.B
            loadEvent(event.bits.robPtr.offset)(event.bits.robPtr.qidx) := event.bits
        }
    }

    val counterValid = RegInit(VecInit(Seq.fill(nrobQ)(
        VecInit(Seq.fill(ndcd)(false.B))
    )))
    val counterEvent = Reg(Vec(nrobQ, Vec(ndcd, new CounterDebugEvent)))
    for (port <- 0 until core.io.counterDebug.length) {
        val event = core.io.counterDebug(port)
        when(event.valid) {
            counterValid(event.bits.robPtr.offset)(event.bits.robPtr.qidx) := true.B
            counterEvent(event.bits.robPtr.offset)(event.bits.robPtr.qidx) := event.bits
        }
    }

    val tlbFillValid = RegInit(VecInit(Seq.fill(nrobQ)(
        VecInit(Seq.fill(ndcd)(false.B))
    )))
    val tlbFillEvent = Reg(Vec(nrobQ, Vec(ndcd, new TlbFillDebugEvent)))
    when(core.io.tlbFillDebug.valid) {
        tlbFillValid(core.io.tlbFillDebug.bits.robPtr.offset)(
            core.io.tlbFillDebug.bits.robPtr.qidx
        ) := true.B
        tlbFillEvent(core.io.tlbFillDebug.bits.robPtr.offset)(
            core.io.tlbFillDebug.bits.robPtr.qidx
        ) := core.io.tlbFillDebug.bits
    }

    val storeValid = RegInit(VecInit(Seq.fill(nrobQ)(
        VecInit(Seq.fill(ndcd)(false.B))
    )))
    val storeEvent = Reg(Vec(nrobQ, Vec(ndcd, new StoreCommitTrace)))
    when(core.io.storeDebug.valid) {
        storeValid(core.io.storeDebug.bits.robPtr.offset)(
            core.io.storeDebug.bits.robPtr.qidx
        ) := true.B
        storeEvent(core.io.storeDebug.bits.robPtr.offset)(
            core.io.storeDebug.bits.robPtr.qidx
        ) := core.io.storeDebug.bits
    }
    for (i <- 0 until ncmt) {
        val commit = core.io.commitTrace(i)
        io.cmt(i).valid := commit.valid
        io.cmt(i).pc := commit.bits.pc
        io.cmt(i).inst := commit.bits.instr
        io.cmt(i).data := core.io.commitData.get(i)
        io.cmt(i).rd_valid :=
            commit.valid &&
            commit.bits.ldestValid &&
            commit.bits.rfWen &&
            commit.bits.ldest =/= 0.U
        io.cmt(i).rd := commit.bits.ldest
        io.cmt(i).exception := false.B
        io.cmt(i).exception_code := 0.U
        val tlbFillSlotValid = tlbFillValid(commit.bits.robPtr.offset)(commit.bits.robPtr.qidx)
        val tlbFillSlot = tlbFillEvent(commit.bits.robPtr.offset)(commit.bits.robPtr.qidx)
        val tlbFillHit = commit.valid && commit.bits.uop === opTLBFILL &&
            tlbFillSlotValid && tlbFillSlot.robPtr.asUInt === commit.bits.robPtr.asUInt
        io.cmt(i).is_tlbfill := tlbFillHit
        io.cmt(i).tlbfill_index := Mux(tlbFillHit, tlbFillSlot.index, 0.U)
        io.cmt(i).is_cnt := commit.bits.uop === opRDTIMELW ||
            commit.bits.uop === opRDCNTIDW ||
            commit.bits.uop === opRDCNTVLW ||
            commit.bits.uop === opRDCNTVHW
        val counterSlotValid = counterValid(commit.bits.robPtr.offset)(commit.bits.robPtr.qidx)
        val counterSlot = counterEvent(commit.bits.robPtr.offset)(commit.bits.robPtr.qidx)
        val counterHit = commit.valid && io.cmt(i).is_cnt && counterSlotValid &&
            counterSlot.robPtr.asUInt === commit.bits.robPtr.asUInt
        io.cmt(i).timer_64_value := Mux(counterHit, counterSlot.value, 0.U)
        io.cmt(i).csr_rstat := (commit.bits.uop === opCSRRD ||
            commit.bits.uop === opCSRWR ||
            commit.bits.uop === opCSRXCHG) &&
            commit.bits.instr(23, 10) === CSR_ESTAT
        io.cmt(i).csr_data := core.io.commitData.get(i)

        val loadSlotValid = loadValid(commit.bits.robPtr.offset)(commit.bits.robPtr.qidx)
        val loadSlot = loadEvent(commit.bits.robPtr.offset)(commit.bits.robPtr.qidx)
        val loadHit = commit.valid && commit.bits.isLoad && loadSlotValid &&
            loadSlot.robPtr.asUInt === commit.bits.robPtr.asUInt
        val loadIsWord = loadSlot.mask === "b1111".U
        val loadIsHalf = PopCount(loadSlot.mask) === 2.U
        io.load(i).valid := Mux(!loadHit, 0.U, Mux(
            commit.bits.uop === opLL,
            "h20".U,
            Mux(loadIsWord, "h10".U,
                Mux(loadIsHalf,
                    Mux(loadSlot.signed, "h04".U, "h08".U),
                    Mux(loadSlot.signed, "h01".U, "h02".U)))
        ))
        io.load(i).paddr := loadSlot.paddr
        io.load(i).vaddr := loadSlot.vaddr

        val storeSlotValid = storeValid(commit.bits.robPtr.offset)(commit.bits.robPtr.qidx)
        val storeSlot = storeEvent(commit.bits.robPtr.offset)(commit.bits.robPtr.qidx)
        val storeHit = commit.valid &&
            (commit.bits.isStore || commit.bits.uop === opSC) &&
            storeSlotValid && storeSlot.robPtr.asUInt === commit.bits.robPtr.asUInt
        val storeIsWord = storeSlot.mask === "b1111".U
        val storeIsHalf = PopCount(storeSlot.mask) === 2.U
        io.store(i).valid := Mux(!storeHit, 0.U,
            Mux(commit.bits.uop === opSC, "h08".U,
                Mux(storeIsWord, "h04".U, Mux(storeIsHalf, "h02".U, "h01".U))))
        io.store(i).paddr := storeSlot.paddr
        io.store(i).vaddr := storeSlot.vaddr
        val storeByteShift = storeSlot.paddr(1, 0)
        val alignedStoreData = (storeSlot.data << (storeByteShift << 3))(dataWidth - 1, 0)
        val alignedStoreMask = (storeSlot.mask << storeByteShift)(dataWidth / 8 - 1, 0)
        val storeBitMask = Cat((0 until dataWidth / 8).reverse.map { byte =>
            Fill(8, alignedStoreMask(byte))
        })
        io.store(i).data := alignedStoreData & storeBitMask
        when(commit.valid) {
            loadValid(commit.bits.robPtr.offset)(commit.bits.robPtr.qidx) := false.B
            counterValid(commit.bits.robPtr.offset)(commit.bits.robPtr.qidx) := false.B
            tlbFillValid(commit.bits.robPtr.offset)(commit.bits.robPtr.qidx) := false.B
            storeValid(commit.bits.robPtr.offset)(commit.bits.robPtr.qidx) := false.B
        }
    }

    val debugRecovery =
        (core.io.redirectTrace.valid &&
            core.io.redirectTrace.bits.kind ===
                CPUSTC.backend.control.RedirectKind.HARD) ||
        core.io.exceptionTrace.valid ||
        core.io.hardRedirect.valid
    when(debugRecovery) {
        for (offset <- 0 until nrobQ) {
            for (bank <- 0 until ndcd) {
                loadValid(offset)(bank) := false.B
                counterValid(offset)(bank) := false.B
                tlbFillValid(offset)(bank) := false.B
                storeValid(offset)(bank) := false.B
            }
        }
    }

    // CSRFile sees the reservation before a retiring LL updates the memory-side
    // LLBit register. Fold retirement effects in program order so Difftest sees
    // the architectural post-state for the complete commit group.
    var commitLlbit = core.io.csrDebugState.llbctl(0)
    for (i <- 0 until ncmt) {
        val commit = core.io.commitTrace(i)
        val setByLl = commit.valid && commit.bits.uop === opLL
        val clearBySc = commit.valid && commit.bits.uop === opSC
        val clearByCsr =
            commit.valid &&
            commit.bits.commitBoundary &&
            core.io.llbitDebugClear
        commitLlbit = Mux(
            setByLl,
            true.B,
            Mux(clearBySc || clearByCsr, false.B, commitLlbit)
        )
    }
    io.csr.llbctl := Cat(core.io.csrDebugState.llbctl(31, 1), commitLlbit)

    val committedRf = RegInit(VecInit.fill(32)(0.U(dataWidth.W)))
    val committedRfNext = WireInit(committedRf)

    for (i <- 0 until ncmt) {
        val commit = core.io.commitTrace(i)
        when(
            commit.valid &&
            commit.bits.ldestValid &&
            commit.bits.rfWen &&
            commit.bits.ldest =/= 0.U
        ) {
            committedRfNext(commit.bits.ldest) := core.io.commitData.get(i)
        }
    }

    committedRfNext(0) := 0.U
    committedRf := committedRfNext
    io.cmt_rf := committedRfNext

    io.cmt_tlbfill_valid := VecInit(io.cmt.map(_.is_tlbfill)).asUInt.orR
    io.cmt_tlbfill_idx := Mux1H(
        io.cmt.map(_.is_tlbfill),
        io.cmt.map(_.tlbfill_index)
    )
}

object GenerateSimTop extends App {
    ChiselStage.emitSystemVerilogFile(
        new CPU(enablePerfCounters = true),
        args = Array("--target-dir", "generated/sim"),
        firtoolOpts = Array(
            "-disable-all-randomization",
            "-strip-debug-info"
        )
    )
}

object GenerateLA32RSimTop extends App {
    ChiselStage.emitSystemVerilogFile(
        new CPU(
            maxCommitPerCycle = 3,
            useBlackBoxRam = false,
            enablePerfCounters = true
        ),
        args = Array("--target-dir", "generated/la32rsim"),
        firtoolOpts = Array(
            "-disable-all-randomization",
            "-strip-debug-info"
        )
    )
}

private[CPUSTC] object ChipLabTopGenerator {
    private val blackBoxResourceListMarker =
        "// ----- 8< ----- FILE \"firrtl_black_box_resource_files.f\" ----- 8< -----"

    sealed trait OutputLayout
    case object SingleFile extends OutputLayout
    case object SplitModules extends OutputLayout

    def parseLayout(value: String): Option[OutputLayout] = {
        value.trim.toLowerCase match {
            case "1" | "single" => Some(SingleFile)
            case "" | "2" | "split" => Some(SplitModules)
            case _ => None
        }
    }

    def layoutFromArgs(args: Seq[String]): OutputLayout = {
        args match {
            case Seq() => promptForLayout()
            case Seq(value) =>
                parseLayout(value).getOrElse {
                    throw new IllegalArgumentException(
                        s"Unknown output layout '$value'. Use 'single' or 'split'."
                    )
                }
            case _ =>
                throw new IllegalArgumentException(
                    "Expected at most one argument: 'single' or 'split'."
                )
        }
    }

    @tailrec
    private def promptForLayout(): OutputLayout = {
        println("Select ChipLab SystemVerilog output layout:")
        println("  1) Generate all RTL in one CPU.sv file")
        println("  2) Split RTL into one file per module [default]")
        print("Choice [1/2]: ")
        Console.flush()

        val input = Option(StdIn.readLine()).getOrElse("")
        parseLayout(input) match {
            case Some(layout) => layout
            case None =>
                println("Invalid choice. Enter 1 or 2.")
                promptForLayout()
        }
    }

    def cpu(): CPU = {
        new CPU(
            maxCommitPerCycle = 3,
            useBlackBoxRam = true,
            enablePerfCounters = false
        )
    }

    val commonFirtoolOptions: Array[String] = Array(
        "-disable-all-randomization",
        "-strip-debug-info",
        "--lowering-options=disallowLocalVariables"
    )

    def removeBlackBoxResourceList(systemVerilog: String): String = {
        val markerIndex = systemVerilog.indexOf(blackBoxResourceListMarker)
        if (markerIndex < 0) {
            systemVerilog
        } else {
            systemVerilog.substring(0, markerIndex)
        }
    }

    def removeBlackBoxResourceList(output: Path): Unit = {
        val systemVerilog = Files.readString(output, StandardCharsets.UTF_8)
        val cleaned = removeBlackBoxResourceList(systemVerilog)
        if (cleaned.length != systemVerilog.length) {
            Files.writeString(output, cleaned, StandardCharsets.UTF_8)
        }
    }
}

object GenerateChipLabTop extends App {
    import ChipLabTopGenerator._

    layoutFromArgs(args.toSeq) match {
        case SingleFile =>
            val output = Paths.get("generated/chiplab-single/CPU.sv")
            ChiselStage.emitSystemVerilogFile(
                cpu(),
                args = Array("--target-dir", "generated/chiplab-single"),
                firtoolOpts = commonFirtoolOptions
            )
            removeBlackBoxResourceList(output)
        case SplitModules =>
            ChiselStage.emitSystemVerilog(
                cpu(),
                firtoolOpts = commonFirtoolOptions ++ Array(
                    "--split-verilog",
                    "-o=generated/chiplab"
                )
            )
    }
}

object GenerateChipLabSimTop extends App {
    ChiselStage.emitSystemVerilog(
        new CPU(
            maxCommitPerCycle = 3,
            useBlackBoxRam = true,
            enablePerfCounters = false
        ),
        firtoolOpts = Array(
            "-disable-all-randomization",
            "-strip-debug-info",
            "--split-verilog",
            "-o=generated/chiplab-sim"
        )
    )
}

object GenerateChipLabDCache extends App {
    ChiselStage.emitSystemVerilog(
        new DCache(useBlackBoxRam = true),
        firtoolOpts = Array(
            "-disable-all-randomization",
            "-strip-debug-info",
            "--lowering-options=disallowLocalVariables",
            "--split-verilog",
            "-o=generated/chiplab-dcache64-nos3"
        )
    )
}
