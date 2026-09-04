package CPUSTC.backend.regfile

import chisel3._
import chisel3.util._

import CPUSTC.config.RegisterFile._

class LegacyPhysicalRegisterFile(
    forwardingWritePorts: Int = nWrite,
    forwardingReadPorts: Int = nRead
) extends Module {
    val io = IO(new PhysicalRegisterFileIO)

    require(forwardingWritePorts > 0)
    require(forwardingWritePorts <= nWrite)
    require(forwardingReadPorts >= 0)
    require(forwardingReadPorts <= nRead)

    val mem = Mem(npreg, UInt(dataWidth.W))

    for (i <- 0 until nWrite) {
        when (io.write(i).valid) {
            assert(io.write(i).bits.addr =/= 0.U)
        }

        when (
            io.write(i).valid &&
            io.write(i).bits.addr =/= 0.U
        ) {
            mem(io.write(i).bits.addr) := io.write(i).bits.data
        }
    }

    for (i <- 0 until nWrite; j <- i + 1 until nWrite) {
        when (io.write(i).valid && io.write(j).valid) {
            assert(io.write(i).bits.addr =/= io.write(j).bits.addr)
        }
    }

    for (r <- 0 until nRead) {
        val addr = io.readReq(r).addr
        val forwardingWrites = io.write.take(forwardingWritePorts)
        val hits = if (r < forwardingReadPorts) {
            VecInit(forwardingWrites.map { w =>
                w.valid &&
                w.bits.addr =/= 0.U &&
                w.bits.addr === addr
            })
        } else {
            VecInit(Seq.fill(forwardingWritePorts)(false.B))
        }
        val forwarded = Mux1H(hits, forwardingWrites.map(_.bits.data))

        io.readData(r) := Mux(
            addr === 0.U,
            0.U,
            Mux(hits.asUInt.orR, forwarded, mem(addr))
        )

        assert(PopCount(hits) <= 1.U)
    }

}

class LvtPhysicalRegisterFile(
    forwardingWritePorts: Int = nWrite,
    forwardingReadPorts: Int = nRead
) extends Module {
    val io = IO(new PhysicalRegisterFileIO)

    require(forwardingWritePorts > 0)
    require(forwardingWritePorts <= nWrite)
    require(forwardingReadPorts >= 0)
    require(forwardingReadPorts <= nRead)

    private val bankWidth = log2Ceil(nWrite)

    // Each write port owns one bank. Replicating that bank once per read port
    // leaves every physical memory with exactly one read and one write port.
    val banks = Seq.tabulate(nWrite, nRead) { case (w, r) =>
        Mem(npreg, UInt(dataWidth.W))
            .suggestName(s"bank_${w}_read_${r}")
    }

    // For each physical register, identify the bank containing its newest value.
    val liveValue = Reg(Vec(npreg, UInt(bankWidth.W)))

    val writeOH = Wire(Vec(nWrite, UInt(npreg.W)))

    for (w <- 0 until nWrite) {
        val write = io.write(w)
        val writeValid = write.valid && write.bits.addr =/= 0.U

        writeOH(w) := Mux(
            writeValid,
            UIntToOH(write.bits.addr, npreg),
            0.U
        )

        when(write.valid) {
            assert(write.bits.addr =/= 0.U)
        }

        when(writeValid) {
            for (r <- 0 until nRead) {
                banks(w)(r)(write.bits.addr) := write.bits.data
            }
            liveValue(write.bits.addr) := w.U
        }
    }

    for (i <- 0 until nWrite; j <- i + 1 until nWrite) {
        when(io.write(i).valid && io.write(j).valid) {
            assert(io.write(i).bits.addr =/= io.write(j).bits.addr)
        }
    }

    val initialized = RegInit(1.U(npreg.W))
    initialized := initialized | writeOH.reduce(_ | _)

    for (r <- 0 until nRead) {
        val req  = io.readReq(r)
        val addr = req.addr

        val forwardingWrites = io.write.take(forwardingWritePorts)
        val dataHits = if (r < forwardingReadPorts) {
            VecInit(forwardingWrites.map { w =>
                w.valid &&
                w.bits.addr =/= 0.U &&
                w.bits.addr === addr
            })
        } else {
            VecInit(Seq.fill(forwardingWritePorts)(false.B))
        }
        val allWriteHits = VecInit(io.write.map { w =>
            req.en &&
            w.valid &&
            w.bits.addr =/= 0.U &&
            w.bits.addr === addr
        })

        val forwarded = Mux1H(dataHits, forwardingWrites.map(_.bits.data))
        val bankData = VecInit((0 until nWrite).map { w =>
            banks(w)(r)(addr)
        })
        val stored = MuxLookup(
            liveValue(addr),
            bankData.head
        )(
            (0 until nWrite).map(w => w.U -> bankData(w))
        )

        io.readData(r) := Mux(
            addr === 0.U,
            0.U,
            Mux(dataHits.asUInt.orR, forwarded, stored)
        )

        assert(PopCount(allWriteHits) <= 1.U)

        when(
            req.en &&
            addr =/= 0.U &&
            !allWriteHits.asUInt.orR &&
            !req.speculative
        ) {
            assert(initialized(addr),
                p"PhysicalRegisterFile: read port $r used uninitialized p${addr}")
            assert(liveValue(addr) < nWrite.U)
        }

        when(req.speculative) {
            assert(req.en)
            assert(addr =/= 0.U)
        }
    }
}

class PhysicalRegisterFile(
    useLvt: Boolean = useLvtPhysicalRegisterFile,
    forwardingWritePorts: Int = nWrite,
    forwardingReadPorts: Int = nRead
) extends Module {
    val io = IO(new PhysicalRegisterFileIO)

    if (useLvt) {
        val impl = Module(new LvtPhysicalRegisterFile(
            forwardingWritePorts,
            forwardingReadPorts
        ))
        impl.io.readReq := io.readReq
        impl.io.write   := io.write
        io.readData     := impl.io.readData
    } else {
        val impl = Module(new LegacyPhysicalRegisterFile(
            forwardingWritePorts,
            forwardingReadPorts
        ))
        impl.io.readReq := io.readReq
        impl.io.write   := io.write
        io.readData     := impl.io.readData
    }
}
