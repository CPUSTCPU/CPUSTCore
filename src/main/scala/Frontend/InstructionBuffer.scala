package CPUSTC.frontend

import chisel3._
import chisel3.util._

import CPUSTC.config.Decode._
import CPUSTC.config.Fetch._

private class IBufferAsyncRamBlackBox(width: Int, depth: Int)
    extends BlackBox(Map("RAM_WIDTH" -> width, "RAM_DEPTH" -> depth))
    with HasBlackBoxInline {

    val io = IO(new Bundle {
        val clock = Input(Clock())
        val raddr = Input(UInt(log2Ceil(depth).W))
        val rdata = Output(UInt(width.W))
        val wen   = Input(Bool())
        val waddr = Input(UInt(log2Ceil(depth).W))
        val wdata = Input(UInt(width.W))
    })

    setInline(
        "IBufferAsyncRamBlackBox.sv",
        s"""
           |module IBufferAsyncRamBlackBox #(
           |  parameter RAM_WIDTH = 81,
           |  parameter RAM_DEPTH = 3
           |) (
           |  input  wire                           clock,
           |  input  wire [$$clog2(RAM_DEPTH)-1:0] raddr,
           |  output wire [RAM_WIDTH-1:0]           rdata,
           |  input  wire                           wen,
           |  input  wire [$$clog2(RAM_DEPTH)-1:0] waddr,
           |  input  wire [RAM_WIDTH-1:0]           wdata
           |);
           |  (* ram_style = "distributed" *) reg [RAM_WIDTH-1:0] mem [0:RAM_DEPTH-1];
           |
           |  always @(posedge clock) begin
           |    if (wen)
           |      mem[waddr] <= wdata;
           |  end
           |
           |  assign rdata = mem[raddr];
           |endmodule
           |""".stripMargin
    )
}

private class IBufferAsyncRam(
    width: Int,
    depth: Int,
    useBlackBox: Boolean
) extends Module {
    val io = IO(new Bundle {
        val raddr = Input(UInt(log2Ceil(depth).W))
        val rdata = Output(UInt(width.W))
        val wen   = Input(Bool())
        val waddr = Input(UInt(log2Ceil(depth).W))
        val wdata = Input(UInt(width.W))
    })

    if (useBlackBox) {
        val ram = Module(new IBufferAsyncRamBlackBox(width, depth))
        ram.io.clock := clock
        ram.io.raddr := io.raddr
        ram.io.wen := io.wen
        ram.io.waddr := io.waddr
        ram.io.wdata := io.wdata
        io.rdata := ram.io.rdata
    } else {
        val ram = Mem(depth, UInt(width.W))
        when(io.wen) {
            ram.write(io.waddr, io.wdata)
        }
        io.rdata := ram.read(io.raddr)
    }
}

class IBufferEntry extends Bundle {
    val pc        = UInt(32.W)
    val instr     = UInt(32.W)
    val exception = new FetchException

    val ftqPtr    = new FtqPtr
    val ftqOffset = UInt(log2Ceil(nfch).W)
    val ftqLast   = Bool()
}

private class IBufferPayload extends Bundle {
    val pc             = UInt(32.W)
    val instr          = UInt(32.W)
    val exceptionValid = Bool()
    val exceptionCause = UInt(8.W)

    val ftqPtr    = new FtqPtr
    val ftqOffset = UInt(log2Ceil(nfch).W)
    val ftqLast   = Bool()
}

class InstructionBufferIO extends Bundle {
    val flush = Input(Bool())

    val enq = Flipped(Decoupled(new FetchBundle))
    val deq = Vec(ndcd, Decoupled(new IBufferEntry))

    val empty = Output(Bool())
    val full  = Output(Bool())
}

class InstructionBuffer(useBlackBoxRam: Boolean = false) extends Module {
    val io = IO(new InstructionBufferIO)

    private val numBanks = nfch
    private val rowsPerBank = nib / numBanks
    private val bankWidth = log2Ceil(numBanks)
    private val rowWidth = log2Ceil(rowsPerBank)
    private val countWidth = log2Ceil(nib + 1)
    private val payloadWidth = (new IBufferPayload).getWidth

    require(numBanks == 4)
    require(rowsPerBank == 3)
    require(nib == numBanks * rowsPerBank)
    require(payloadWidth == 81)

    // Four consecutive queue positions always occupy distinct banks. Keeping
    // one binary row pointer per bank removes the generic FIFO's two wide
    // one-hot routing stages while preserving four writes and three reads.
    private val banks = Seq.fill(numBanks)(
        Module(new IBufferAsyncRam(
            payloadWidth,
            rowsPerBank,
            useBlackBoxRam
        ))
    )
    val headBank = RegInit(0.U(bankWidth.W))
    val tailBank = RegInit(0.U(bankWidth.W))
    val headRows = RegInit(VecInit.fill(numBanks)(0.U(rowWidth.W)))
    val tailRows = RegInit(VecInit.fill(numBanks)(0.U(rowWidth.W)))
    val count = RegInit(0.U(countWidth.W))

    private def addBank(base: UInt, amount: UInt): UInt = {
        (base +& amount)(bankWidth - 1, 0)
    }

    private def incrementRow(row: UInt): UInt = {
        Mux(
            row === (rowsPerBank - 1).U,
            0.U(rowWidth.W),
            row + 1.U
        )
    }

    private val contiguousMaskValues = 0 +: (for {
        first <- 0 until nfch
        last <- first until nfch
    } yield (((1 << (last - first + 1)) - 1) << first))
    val maskIsContiguous = contiguousMaskValues.distinct.map { value =>
        io.enq.bits.mask === value.U(nfch.W)
    }.reduce(_ || _)
    assert(
        !io.enq.valid || maskIsContiguous,
        "InstructionBuffer expects contiguous fetch masks"
    )
    for (i <- 1 until ndcd) {
        assert(
            !io.deq(i).ready || io.deq(i - 1).ready,
            "InstructionBuffer expects prefix decode ready"
        )
    }

    private val sourcePayloads = Wire(Vec(nfch, new IBufferPayload))
    for (source <- 0 until nfch) {
        val sourcePc = io.enq.bits.pc + (source * 4).U
        val sourceException = io.enq.bits.exceptions(source)

        sourcePayloads(source).pc             := sourcePc
        sourcePayloads(source).instr          := io.enq.bits.instrs(source).asUInt
        sourcePayloads(source).exceptionValid := sourceException.valid
        sourcePayloads(source).exceptionCause := sourceException.cause
        sourcePayloads(source).ftqPtr          := io.enq.bits.ftqPtr
        sourcePayloads(source).ftqOffset       := source.U
        sourcePayloads(source).ftqLast         := io.enq.bits.mask(source) &&
            (if (source == nfch - 1) true.B else !io.enq.bits.mask(nfch - 1, source + 1).orR)

        when(io.enq.valid && io.enq.bits.mask(source)) {
            assert(
                sourceException.badvValid === sourceException.valid,
                "IBuffer only compresses fetch exceptions whose BADV validity matches exception validity"
            )
            when(sourceException.valid) {
                assert(
                    sourceException.badv === sourcePc,
                    "IBuffer reconstructs fetch exception BADV from the entry PC"
                )
            }
        }
    }

    val sourcePayloadBits = VecInit(sourcePayloads.map(_.asUInt))
    val enqCount = PopCount(io.enq.bits.mask)

    // Preserve the generic cluster FIFO's conservative capacity contract: it
    // accepts a fetch packet only when all four banks have a free row. For a
    // contiguous ring this is exactly count <= nib - nfch, independent of the
    // packet mask and of same-cycle dequeue credit.
    io.enq.ready := count <= (nib - nfch).U
    val enqFire = io.enq.valid && io.enq.ready
    val firstSource = PriorityEncoder(io.enq.bits.mask)

    val bankWriteValid = Wire(Vec(numBanks, Bool()))
    val bankWriteData = Wire(Vec(numBanks, UInt(payloadWidth.W)))
    for (bank <- 0 until numBanks) {
        val relativeBank = (bank.U(bankWidth.W) - tailBank)(bankWidth - 1, 0)
        val sourceIndexWide = firstSource +& relativeBank
        val sourceIndex = sourceIndexWide(bankWidth - 1, 0)

        bankWriteValid(bank) :=
            enqFire && !io.flush && relativeBank < enqCount
        bankWriteData(bank) := sourcePayloadBits(sourceIndex)

        when(bankWriteValid(bank)) {
            assert(sourceIndexWide < nfch.U)
            assert(io.enq.bits.mask(sourceIndex))
        }
    }

    val bankReadData = Wire(Vec(numBanks, UInt(payloadWidth.W)))
    for (bank <- 0 until numBanks) {
        banks(bank).io.raddr := headRows(bank)
        banks(bank).io.wen := bankWriteValid(bank)
        banks(bank).io.waddr := tailRows(bank)
        banks(bank).io.wdata := bankWriteData(bank)
        bankReadData(bank) := banks(bank).io.rdata
    }

    val laneBanks = Wire(Vec(ndcd, UInt(bankWidth.W)))
    val deqFire = Wire(Vec(ndcd, Bool()))
    for (lane <- 0 until ndcd) {
        laneBanks(lane) := addBank(headBank, lane.U)
        val payload = bankReadData(laneBanks(lane))
            .asTypeOf(new IBufferPayload)

        io.deq(lane).valid := count > lane.U
        io.deq(lane).bits.pc := payload.pc
        io.deq(lane).bits.instr := payload.instr
        io.deq(lane).bits.exception.valid := payload.exceptionValid
        io.deq(lane).bits.exception.cause := payload.exceptionCause
        io.deq(lane).bits.exception.badvValid := payload.exceptionValid
        io.deq(lane).bits.exception.badv :=
            Mux(payload.exceptionValid, payload.pc, 0.U)
        io.deq(lane).bits.ftqPtr := payload.ftqPtr
        io.deq(lane).bits.ftqOffset := payload.ftqOffset
        io.deq(lane).bits.ftqLast := payload.ftqLast

        deqFire(lane) := io.deq(lane).valid && io.deq(lane).ready
    }

    val deqCount = PopCount(deqFire)
    val bankDeqFire = Wire(Vec(numBanks, Bool()))
    for (bank <- 0 until numBanks) {
        val laneHits = VecInit.tabulate(ndcd) { lane =>
            deqFire(lane) && laneBanks(lane) === bank.U
        }
        bankDeqFire(bank) := laneHits.asUInt.orR
        assert(PopCount(laneHits) <= 1.U)
    }

    val acceptedEnqCount = Mux(enqFire && !io.flush, enqCount, 0.U)
    val acceptedDeqCount = Mux(io.flush, 0.U, deqCount)
    val countCalcWidth = countWidth + 1
    val nextCountWide =
        count.pad(countCalcWidth) + acceptedEnqCount.pad(countCalcWidth) -
            acceptedDeqCount.pad(countCalcWidth)

    when(io.flush) {
        headBank := 0.U
        tailBank := 0.U
        headRows := VecInit.fill(numBanks)(0.U(rowWidth.W))
        tailRows := VecInit.fill(numBanks)(0.U(rowWidth.W))
        count := 0.U
    }.otherwise {
        for (bank <- 0 until numBanks) {
            when(bankWriteValid(bank)) {
                tailRows(bank) := incrementRow(tailRows(bank))
            }
            when(bankDeqFire(bank)) {
                headRows(bank) := incrementRow(headRows(bank))
            }
        }

        when(enqFire) {
            tailBank := addBank(tailBank, enqCount)
        }
        when(deqCount.orR) {
            headBank := addBank(headBank, deqCount)
        }
        count := nextCountWide(countWidth - 1, 0)
    }

    when(!reset.asBool) {
        assert(count <= nib.U)
        assert(acceptedDeqCount <= count)
        assert(nextCountWide <= nib.U)
        assert(PopCount(bankWriteValid) === acceptedEnqCount)
        for (bank <- 0 until numBanks) {
            assert(headRows(bank) < rowsPerBank.U)
            assert(tailRows(bank) < rowsPerBank.U)
        }
    }

    io.empty := count === 0.U
    io.full  := !io.enq.ready
}
