package CPUSTC.config

import chisel3._
import chisel3.util._
import Consts._
import Imm._
import Branch._

import scala.language.implicitConversions

// 判断两个 UInt 是否相等，常用于译码表或旁路比较。
object IsEqual {
    def apply(a: UInt, b: UInt): Bool = a === b
}

// 从高位向低位传播 1：某一位及其更高位中只要出现 1，该位输出 1。
// 例：00101000 -> 11111000，常用于生成“低位屏蔽/优先级范围”。
object MaskLower {
    def apply(in: UInt): UInt = {
        val width = in.getWidth
        VecInit((0 until width).map(i => in(width - 1, i).orR)).asUInt
    }
}

// 从低位向高位传播 1：某一位及其更低位中只要出现 1，该位输出 1。
// 例：00101000 -> 00111111，常用于生成“高位屏蔽/优先级范围”。
object MaskUpper {
    def apply(in: UInt): UInt = {
        val width = in.getWidth
        VecInit((0 until width).map(i => in(i, 0).orR)).asUInt
    }
}

// 把二进制下标转换成 one-hot mask。
// 例：idx=2,width=4 -> 0100。
object UIntToMask {
    def apply(idx: UInt, width: Int): UInt = {
        (1.U(width.W) << idx)(width - 1, 0)
    }
}

// 把物理寄存器编号转换成 one-hot mask，并过滤 p0。
// 常用于 FreeList / BusyTable 生成分配、释放、唤醒 mask。
object PregMask {
    def apply(valid: Bool, preg: UInt, width: Int): UInt = {
        Mux(valid && preg =/= 0.U, UIntToOH(preg)(width - 1, 0), 0.U(width.W))
    }
}

// 把多个 Valid[物理寄存器编号] 合并成一个 one-hot mask，并过滤 p0。
object PregMaskOr {
    def apply(reqs: Seq[Valid[UInt]], width: Int): UInt = {
        require(reqs.nonEmpty)
        reqs.map(req => PregMask(req.valid, req.bits, width)).reduce(_ | _)
    }
}

// 判断两个 mask 是否有交集。
object MaskMatch {
    def apply(a: UInt, b: UInt): Bool = (a & b).orR
}

// 环形加 1，用于 FIFO/ROB/FTQ 等循环队列指针。
object WrapInc {
    def apply(value: UInt, n: Int): UInt = {
        require(n > 0)
        if (n == 1) {
            0.U(1.W)
        } else if (isPow2(n)) {
            (value + 1.U)(log2Ceil(n) - 1, 0)
        } else {
            Mux(value === (n - 1).U, 0.U, value + 1.U)
        }
    }
}

// 环形减 1，用于循环队列指针回退。
object WrapDec {
    def apply(value: UInt, n: Int): UInt = {
        require(n > 0)
        if (n == 1) {
            0.U(1.W)
        } else if (isPow2(n)) {
            (value - 1.U)(log2Ceil(n) - 1, 0)
        } else {
            Mux(value === 0.U, (n - 1).U, value - 1.U)
        }
    }
}

// 环形加 inc，用于循环队列指针一次前进多个位置。
object WrapAdd {
    def apply(value: UInt, inc: UInt, n: Int): UInt = {
        require(n > 0)
        if (n == 1) {
            0.U(1.W)
        } else if (isPow2(n)) {
            (value + inc)(log2Ceil(n) - 1, 0)
        } else {
            val result = value.pad(value.getWidth + 1) + inc
            Mux(result >= n.U, result - n.U, result)(log2Ceil(n) - 1, 0)
        }
    }
}

// 在以 head 为队头的环形队列里，判断 i0 是否比 i1 更老。
object IsOlder {
    def apply(i0: UInt, i1: UInt, head: UInt): Bool = {
        (i0 < i1) ^ (i0 < head) ^ (i1 < head)
    }
}

// enable 为真时更新并输出新数据，否则保持上一次 enable 时的数据。
object HoldUnless {
    def apply[T <: Data](data: T, enable: Bool): T = {
        val reg = RegEnable(data, enable)
        Mux(enable, data, reg)
    }
}

// 把 data 连续打一串 n 级寄存器，n=0 时直接返回原数据。
object RegNextN {
    def apply[T <: Data](data: T, n: Int): T = {
        require(n >= 0)
        if (n == 0) data else (0 until n).foldLeft(data)((x, _) => RegNext(x))
    }
}

// 带 enable 的 valid 寄存器，复位值为 false。
object GatedValidRegNext {
    def apply(valid: Bool, enable: Bool): Bool = {
        RegEnable(valid, false.B, enable)
    }
}

// 符号扩展到 targetWidth 位。
object SignExt {
    def apply(data: UInt, targetWidth: Int): UInt = {
        require(targetWidth >= data.getWidth)
        if (targetWidth == data.getWidth) data
        else Cat(Fill(targetWidth - data.getWidth, data(data.getWidth - 1)), data)
    }
}

// 零扩展到 targetWidth 位。
object ZeroExt {
    def apply(data: UInt, targetWidth: Int): UInt = {
        require(targetWidth >= data.getWidth)
        if (targetWidth == data.getWidth) data
        else Cat(0.U((targetWidth - data.getWidth).W), data)
    }
}

// 地址向下对齐到 bytes 边界，bytes 必须是 2 的幂。
object AlignDown {
    def apply(addr: UInt, bytes: Int): UInt = {
        require(isPow2(bytes))
        addr & (~(bytes - 1).U(addr.getWidth.W)).asUInt
    }
}

// 地址向上对齐到 bytes 边界，bytes 必须是 2 的幂。
object AlignUp {
    def apply(addr: UInt, bytes: Int): UInt = {
        require(isPow2(bytes))
        AlignDown(addr + (bytes - 1).U, bytes)
    }
}

// one-hot 查表；要求 key 一定命中 mapping 中的某一项。
object LookupTree {
    def apply[T <: Data](key: UInt, mapping: Iterable[(UInt, T)]): T = {
        val mapSeq = mapping.toSeq
        require(mapSeq.nonEmpty)
        Mux1H(mapSeq.map { case (k, v) => (key === k) -> v })
    }
}

// 带默认值的 one-hot 查表；key 不命中时返回 default。
object LookupTreeDefault {
    def apply[T <: Data](key: UInt, default: T, mapping: Iterable[(UInt, T)]): T = {
        val mapSeq = mapping.toSeq
        val hits = mapSeq.map { case (k, _) => key === k }
        val pairs = mapSeq.map { case (k, v) => (key === k) -> v }
        val defaultPair = (!hits.foldLeft(false.B)(_ || _)) -> default
        Mux1H(defaultPair +: pairs)
    }
}

// 优先级选择器封装，选择第一个条件为真的输入。
object ParallelPriorityMux {
    def apply[T <: Data](in: Seq[(Bool, T)]): T = {
        PriorityMux(in)
    }
}

// LA32 指令立即数生成器，根据 immType 从指令中取位并做符号/零扩展。
object ImmGen {
    def apply(inst: UInt, immType: UInt): UInt = {
        MuxLookup(immType, 0.U(32.W))(Seq(
            immU5   -> Cat(0.U(27.W), inst(14, 10)),
            immU12  -> Cat(0.U(20.W), inst(21, 10)),
            immS12  -> Cat(Fill(20, inst(21)), inst(21, 10)),
            immS14  -> Cat(Fill(16, inst(23)), inst(23, 10), 0.U(2.W)),
            immS16  -> Cat(Fill(14, inst(25)), inst(25, 10), 0.U(2.W)),
            immU20  -> Cat(inst(24, 5), 0.U(12.W)),
            immS20  -> Cat(Fill(10, inst(24)), inst(24, 5), 0.U(2.W)),
            immS26  -> Cat(Fill(4, inst(9)), inst(9, 0), inst(25, 10), 0.U(2.W)),
            immCSR  -> Cat(0.U(18.W), inst(23, 10)),
            immCID  -> Cat(0.U(27.W), inst(4, 0))
        ))
    }
}

// 根据分支类型和两个源操作数，计算分支是否实际跳转。
object BranchTaken {
    def apply(brType: UInt, src1: UInt, src2: UInt): Bool = {
        val eq = src1 === src2
        val lt = src1.asSInt < src2.asSInt
        val ltu = src1 < src2

        MuxLookup(brType, false.B)(Seq(
            BR_NE  -> !eq,
            BR_EQ  -> eq,
            BR_GE  -> !lt,
            BR_GEU -> !ltu,
            BR_LT  -> lt,
            BR_LTU -> ltu,
            BR_J   -> true.B,
            BR_JR  -> true.B
        ))
    }
}

// 允许把 UInt 常量隐式转成 BitPat，方便写 DecodeLogic/译码表。
object ImplicitCast {
    implicit def uintToBitPat(x: UInt): BitPat = BitPat(x)
}

// 循环左移 1 位。
object ShiftAdd1 {
    def apply(x: UInt): UInt = {
        val n = x.getWidth
        x(n-2, 0) ## x(n-1)
    }
}

// 循环右移 1 位。
object ShiftSub1 {
    def apply(x: UInt): UInt = {
        val n = x.getWidth
        x(0) ## x(n-1, 1)
    }
}

// 循环左移 k 位。
object ShiftAddN {
    def apply(x: UInt, k: Int): UInt = {
        val n = x.getWidth
        if (k == 0) x
        else x(n-k-1, 0) ## x(n-1, n-k)
    }
}

// 循环右移 k 位。
object ShiftSubN {
    def apply(x: UInt, k: Int): UInt = {
        val n = x.getWidth
        if (k == 0) x
        else x(k-1, 0) ## x(n-1, k)
    }
}

// 扩展形式的无符号小于比较，适合某些把最高位当扩展位/环形位的比较。
object ESltu {  
    def apply(src1: UInt, src2: UInt): Bool = {
        val n = src1.getWidth
        assert(n == src2.getWidth, "src1 and src2 must have the same width")
        val signNeq = src1(n-1) ^ src2(n-1)
        val src1LtSrc2 = src1(n-2, 0) < src2(n-2, 0)
        // Mux(signNeq, !src1LtSrc2, src1LtSrc2)
        signNeq ^ src1LtSrc2
    }
}

// one-hot 编码数值的小于比较。
object Slt1H {
    def apply(src1: UInt, src2: UInt): Bool = {
        val n = src1.getWidth
        assert(n == src2.getWidth, "src1 and src2 must have the same width")
        val src1Acc = VecInit.tabulate(n)(i => src1.take(i).orR)
        val src2Acc = VecInit.tabulate(n)(i => src2.take(i).orR)
        val diff = src1Acc.zip(src2Acc).map{ case (s1, s2) => s1 & !s2 }
        diff.reduce(_ | _)
    }
}

// 符号扩展到 n 位，Zircon 风格命名。
object SE {
    def apply(x: UInt, n: Int = 32): UInt = {
        val len = x.getWidth
        assert(len <= n, "x must have less than n bits")
        val sign = x(len-1)
        Fill(n-len, sign) ## x
    }
}

// 零扩展到 n 位，Zircon 风格命名。
object ZE {
    def apply(x: UInt, n: Int = 32): UInt = {
        val len = x.getWidth
        assert(len <= n, "x must have less than n bits")
        Fill(n-len, 0.U) ## x
    }
}

// 把 x 调整到 n 位：位宽不足补 0，位宽过长截低 n 位。
object BitAlign {
    def apply(x: UInt, n: Int): UInt = {
        val len = x.getWidth
        if(len == n) x
        else if (len < n) ZE(x, n)
        else x(n-1, 0)
    }
}

// one-hot Mux，sel 可以是 Bool 序列或 UInt one-hot。
object MuxOH {
    def apply[T <: Data](sel: Seq[Bool], in: Seq[T]): T = {
        val n = in.size
        assert(n > 0, "in must have at least one element")
        VecInit(in.zip(sel).map{
            case(i, s) => i.asUInt & Fill(i.getWidth, s)
        }).reduceTree((a: UInt, b: UInt) => (a | b)).asTypeOf(in(0))
    }
    
    def apply[T <: Data](sel: UInt, in:Seq[T]): T = {
        apply(sel.asBools, in)
        // Mux1H
    }
}

// 写优先读：同周期读写同一地址时，返回写入数据而不是旧的 rdata。
object WFirstRead {
    def apply[T <: Data](rdata: T, ridx: UInt, widx: Seq[UInt], wdata: Seq[T], wen: Seq[Bool]): T = {
        assert(widx.size == wdata.size && widx.size == wen.size, "widx, wdata and wen must have the same size")
        val n = wdata.size
        val whit = VecInit.tabulate(n)(i => (ridx === widx(i)) && wen(i))
        Mux(whit.asUInt.orR, Mux1H(whit, wdata), rdata)
    }
}

// 访存类型解码：把 byte/half/word 类型编码展开成字节写使能 mask。
object MTypeDecode {
    def apply(mtype: UInt, n: Int = 4): UInt = {
        val res = Wire(UInt(n.W))
        res := MuxLookup(mtype, 1.U(n.W))(Seq(
            0.U -> 0x1.U(n.W),
            1.U -> 0x3.U(n.W),
            2.U -> 0xf.U(n.W),
        ))
        res
    }
}

// 访存类型编码：把字节写使能 mask 压回 mem type 编码。
object MTypeEncode {
    def apply(mtype: UInt, n: Int = 2): UInt = {
        val res = Wire(UInt(n.W))
        res := MuxLookup(mtype, 0.U(n.W))(Seq(
            0x1.U -> 0.U(n.W),
            0x3.U -> 1.U(n.W),
            0xf.U -> 2.U(n.W),
        ))
        res
    }
}

// 按字段名把 parent 中同名字段赋给 child，常用于 Bundle 之间继承公共字段。
object InheritFields {
    def apply[T <: Bundle, P <: Bundle](child: T, parent: P): Unit = {
        parent.elements.foreach { case (name, data) =>
            if (child.elements.contains(name)) {
                child.elements(name) := data
            }
        }
    }
}

// 按 one-hot 位数循环右移 x。
object RotateRightOH {
    def apply(x: UInt, nOH: UInt): UInt = {
        val width = x.getWidth
        assert(width == nOH.getWidth, "two operators must have the same width")
        val xShifts = VecInit.tabulate(width)(i => ShiftSubN(x, i))
        Mux1H(nOH, xShifts)
    }
}

// 按 one-hot 位数循环左移 x。
object RotateLeftOH {
    def apply(x: UInt, nOH: UInt): UInt = {  
        val width = x.getWidth
        assert(width == nOH.getWidth, "two operators must have the same width")
        val xShifts = VecInit.tabulate(width)(i => ShiftAddN(x, i))
        Mux1H(nOH, xShifts)
    }
}

// 位矩阵转置：输入 Vec[UInt]，输出按 bit 位置重新分组后的 Vec[UInt]。
object Transpose {
    def apply(x: Vec[UInt]): Vec[UInt] = {
        val n = x(0).getWidth
        VecInit.tabulate(n)(i => VecInit(x.map(_(i))).asUInt)
    }
}

// 按 one-hot 位数逻辑左移 x。
object Lshift1H {
    def apply(x: UInt, nOH: UInt): UInt = {
        val width = nOH.getWidth
        val xShifts = VecInit.tabulate(width)(i => x << i)
        Mux1H(nOH, xShifts)
    }
}

// 按 one-hot 位数逻辑右移 x。
object Rshift1H {
    def apply(x: UInt, nOH: UInt): UInt = {
        val width = nOH.getWidth
        val xShifts = VecInit.tabulate(width)(i => x >> i)
        Mux1H(nOH, xShifts)
    }
}

// 从低位开始做优先编码，返回最低有效 1 的下标。
object Log2Rev {

  def apply(x: Bits, width: Int): UInt = {
    if (width < 2) {
      0.U
    } else if (width == 2) {
      x(1) && !x(0)
    } else if( width <= divideAndConquerThreshold){
        PriorityEncoder(x)
    } else {
      val mid = 1 << (log2Ceil(width) - 1)
      val hi = x(width - 1, mid)
      val lo = x(mid - 1, 0)
      val useLo = lo.orR
      Cat(!useLo, Mux(useLo, Log2Rev(lo, mid), Log2Rev(hi, width - mid)))
    }
  }

  def apply(x: Bits): UInt = apply(x, x.getWidth)

  private def divideAndConquerThreshold = 4
}

// 只保留输入中的最高有效 1，输出 one-hot。
object Log2OH {
    def apply(x: Bits, width: Int): UInt = {
        if(width < 2) {
            x(0)
        } else if(width == 2) {
            Cat(x(1), (!x(1) && x(0)))
        } else if(width <= divideAndConquerThreshold) {
            Mux(x(width - 1), Cat(1.U(1.W), 0.U((width - 1).W)), Cat(0.U(1.W), apply(x(width - 2, 0), width - 1)))
        } else {
            val mid = 1 << (log2Ceil(width) - 1)
            val hi = x(width - 1, mid)
            val lo = x(mid - 1, 0)
            val usehi = hi.orR
            // Cat(usehi, Mux(usehi, apply(hi, width - mid), apply(lo, mid)))
            Mux(usehi, Cat(apply(hi, width - mid), 0.U(mid.W)), Cat(0.U((width - mid).W), apply(lo, mid)))
        }
    }
    def apply(x: Bits): UInt = apply(x, x.getWidth)
    def apply(x: Seq[Bool]): UInt = apply(VecInit(x).asUInt, x.size)
    private def divideAndConquerThreshold = 4
}

// 只保留输入中的最低有效 1，输出 one-hot。
object Log2OHRev {
    def apply(x: Bits): UInt = {
        Reverse(Log2OH(Reverse(x.asUInt)))
    }
    def apply(x: Seq[Bool]): UInt = {
        apply(VecInit(x).asUInt)
    }
}

// 从 baseOH 指定的位置开始，环形选择 req 中第一个有效位。
object PickRotOH {
    def apply(req: UInt, baseOH: UInt): UInt = {
        require(req.getWidth == baseOH.getWidth)

        val fromBaseMask = MaskUpper(baseOH)
        val fromBaseReq  = req & fromBaseMask

        Mux(
            fromBaseReq.orR,
            Log2OHRev(fromBaseReq),
            Log2OHRev(req)
        )
    }
}

// 从 req 中按轮转顺序选择最多 count 个互不重复的 one-hot。
object PickNRotOH {
    def apply(req: UInt, baseOH: UInt, count: Int): Vec[UInt] = {
        require(count > 0)
        require(req.getWidth == baseOH.getWidth)

        val result = Wire(Vec(count, UInt(req.getWidth.W)))

        var remain = req
        for (i <- 0 until count) {
            result(i) := PickRotOH(remain, baseOH)
            remain = remain & (~result(i)).asUInt
        }

        result
    }
}

// 把请求旋转到 baseOH 后并行计算各请求的排名，再旋回原位置。
// 与串行删除前一授予的 PickNRotOH 相比，多个 grant 之间没有优先编码器依赖。
object PickNRotOHParallel {
    def apply(req: UInt, baseOH: UInt, count: Int): Vec[UInt] = {
        require(count > 0)
        require(count <= req.getWidth)
        require(req.getWidth == baseOH.getWidth)

        val width = req.getWidth
        val countWidth = log2Ceil(width + 1)
        val rotatedReq = RotateRightOH(req, baseOH)

        val rankBefore = Wire(Vec(width, UInt(countWidth.W)))
        rankBefore(0) := 0.U
        for (i <- 1 until width) {
            rankBefore(i) := PopCount(rotatedReq(i - 1, 0))
        }

        val result = Wire(Vec(count, UInt(width.W)))
        for (rank <- 0 until count) {
            val rotatedGrant = VecInit((0 until width).map { i =>
                rotatedReq(i) && rankBefore(i) === rank.U
            }).asUInt

            result(rank) := RotateLeftOH(rotatedGrant, baseOH)
        }

        result
    }
}

// reqCountOH(k)=1 表示需要 k 个槽位。
object CanAcceptByOH {
    def apply(reqCountOH: UInt, selected: Vec[UInt]): Bool = {
        require(reqCountOH.getWidth == selected.length + 1)

        val enough = Wire(Vec(selected.length + 1, Bool()))
        enough(0) := true.B

        for (i <- 1 to selected.length) {
            enough(i) := selected.take(i).map(_.orR).reduce(_ && _)
        }

        Mux1H(reqCountOH, enough)
    }
}
