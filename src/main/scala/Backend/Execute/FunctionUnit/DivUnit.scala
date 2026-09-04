package CPUSTC.backend.execute.fu

import chisel3._
import chisel3.util._

import CPUSTC.config._
import CPUSTC.config.DivOp._
import CPUSTC.config.Execute.FU_OP_SZ
import CPUSTC.config.RegisterFile.dataWidth

class DivReq extends Bundle {
    val fn       = UInt(FU_OP_SZ.W)
    val dividend = UInt(dataWidth.W)
    val divisor  = UInt(dataWidth.W)
}

class DivUnitIO extends Bundle {
    val kill = Input(Bool())
    val req  = Flipped(Decoupled(new DivReq))
    val resp = Decoupled(UInt(dataWidth.W))
    val busy = Output(Bool())
    val respPending = Output(Bool())
}

class DivUnit extends Module {
    require(dataWidth == 32)

    val io = IO(new DivUnitIO)

    val sIdle :: sPrepare :: sIterate :: sPost :: sResponse :: Nil = Enum(5)
    val state = RegInit(sIdle)

    val fnReg          = Reg(UInt(FU_OP_SZ.W))
    val dividendMagReg = Reg(UInt(dataWidth.W))
    val divisorMagReg  = Reg(UInt(dataWidth.W))

    val quotientNegReg  = Reg(Bool())
    val remainderNegReg = Reg(Bool())

    val quotientReg       = Reg(UInt(dataWidth.W))
    val remainderReg      = Reg(UInt(dataWidth.W))
    val alignedDivisorReg = Reg(UInt((dataWidth + 1).W))
    val quotBitOHReg       = Reg(UInt(dataWidth.W))

    val resultReg = Reg(UInt(dataWidth.W))

    val selectRemainder = fnReg === MOD || fnReg === MODU

    val reqSignedOp = io.req.bits.fn === DIV || io.req.bits.fn === MOD
    val reqDividendNeg =
        reqSignedOp && io.req.bits.dividend(dataWidth - 1)
    val reqDivisorNeg =
        reqSignedOp && io.req.bits.divisor(dataWidth - 1)
    val reqDividendMag = Mux(
        reqDividendNeg,
        (~io.req.bits.dividend).asUInt + 1.U,
        io.req.bits.dividend
    )
    val reqDivisorMag = Mux(
        reqDivisorNeg,
        (~io.req.bits.divisor).asUInt + 1.U,
        io.req.bits.divisor
    )
    val dividendMag = dividendMagReg
    val divisorMag = divisorMagReg

    val lzcDividend = Log2Rev(Reverse(dividendMag))
    val lzcDivisor  = Log2Rev(Reverse(divisorMag))

    val magnitudeDiff = Cat(0.U(1.W), dividendMag) - Cat(0.U(1.W), divisorMag)
    val dividendLess = magnitudeDiff(dataWidth)
    val alignShift = lzcDivisor - lzcDividend

    val alignedDivisor =
        (Cat(0.U(1.W), divisorMag) << alignShift)(dataWidth, 0)
    val initialQuotBitOH =
        (1.U(dataWidth.W) << alignShift)(dataWidth - 1, 0)

    def negateIf(value: UInt, negate: Bool): UInt =
        Mux(negate, (~value).asUInt + 1.U, value)

    val directQuotientMag = Mux(dividendLess, 0.U, dividendMag)
    val directRemainderMag = Mux(dividendLess, dividendMag, 0.U)
    val directQuotient = negateIf(directQuotientMag, quotientNegReg)
    val directRemainder = negateIf(directRemainderMag, remainderNegReg)
    val directResult = Mux(selectRemainder, directRemainder, directQuotient)

    val diff0 =
        Cat(0.U(1.W), remainderReg) - alignedDivisorReg
    val take0 = !diff0(dataWidth)
    val remainderAfter0 = Mux(take0, diff0(dataWidth - 1, 0), remainderReg)
    val quotientAfter0 = quotientReg | Mux(take0, quotBitOHReg, 0.U)

    val secondDivisor = alignedDivisorReg >> 1
    val hasSecondStep = !quotBitOHReg(0)
    val diff1 =
        Cat(0.U(1.W), remainderAfter0) - secondDivisor
    val take1 = hasSecondStep && !diff1(dataWidth)
    val remainderAfter1 = Mux(take1, diff1(dataWidth - 1, 0), remainderAfter0)
    val quotientAfter1 = quotientAfter0 |
        Mux(take1, quotBitOHReg >> 1, 0.U)

    val lastIteration = quotBitOHReg(1, 0).orR

    val correctedQuotient = negateIf(quotientReg, quotientNegReg)
    val correctedRemainder = negateIf(remainderReg, remainderNegReg)

    io.req.ready  := state === sIdle && !io.kill
    io.resp.valid := state === sResponse && !io.kill
    io.resp.bits  := resultReg
    io.busy       := state =/= sIdle
    io.respPending := state === sResponse

    when(io.kill) {
        state := sIdle
    }.otherwise {
        switch(state) {
            is(sIdle) {
                when(io.req.fire) {
                    fnReg          := io.req.bits.fn
                    dividendMagReg := reqDividendMag
                    divisorMagReg  := reqDivisorMag
                    quotientNegReg := reqDividendNeg ^ reqDivisorNeg
                    remainderNegReg := reqDividendNeg
                    state := sPrepare
                }
            }

            is(sPrepare) {
                when(divisorMag === 0.U || dividendMag === 0.U) {
                    resultReg := 0.U
                    state := sResponse
                }.elsewhen(dividendLess || divisorMag === 1.U) {
                    resultReg := directResult
                    state := sResponse
                }.otherwise {
                    quotientReg       := 0.U
                    remainderReg      := dividendMag
                    alignedDivisorReg := alignedDivisor
                    quotBitOHReg       := initialQuotBitOH
                    state              := sIterate
                }
            }

            is(sIterate) {
                quotientReg  := quotientAfter1
                remainderReg := remainderAfter1

                when(lastIteration) {
                    state := sPost
                }.otherwise {
                    alignedDivisorReg := alignedDivisorReg >> 2
                    quotBitOHReg       := quotBitOHReg >> 2
                }
            }

            is(sPost) {
                resultReg := Mux(
                    selectRemainder,
                    correctedRemainder,
                    correctedQuotient
                )
                state := sResponse
            }

            is(sResponse) {
                when(io.resp.fire) {
                    state := sIdle
                }
            }
        }
    }

    when(io.req.fire) {
        assert(
            io.req.bits.fn === DIV ||
            io.req.bits.fn === MOD ||
            io.req.bits.fn === DIVU ||
            io.req.bits.fn === MODU
        )
    }

    when(state === sIterate) {
        assert(quotBitOHReg.orR)
        assert(PopCount(quotBitOHReg) === 1.U)
        assert(alignedDivisorReg.orR)
    }

    val stalledLastCycle = RegNext(io.resp.valid && !io.resp.ready, false.B)
    val resultLastCycle  = RegNext(io.resp.bits)

    when(stalledLastCycle && !io.kill) {
        assert(io.resp.valid)
        assert(io.resp.bits === resultLastCycle)
    }
}
