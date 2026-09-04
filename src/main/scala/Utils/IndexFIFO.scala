package CPUSTC.utils

import chisel3._
import chisel3.util._
import CPUSTC.config._
import scala.reflect.runtime.universe._
import scala.reflect.ClassTag

// isFlst: The FIFO is a free list for reg rename
class IndexTailRestore(n: Int) extends Bundle {
	val ptrOH = UInt(n.W)
	val high  = Bool()
}

class IndexFIFOIO[T <: Data](gen: T, n: Int, rw: Int, ww: Int, isFlst: Boolean, supportTailRestore: Boolean) extends Bundle {
    val enq 	= Flipped(Decoupled(gen))
	val enqCapacity = Output(Bool())
	val enqIdx  = Output(UInt(n.W))
	val enqHigh = Output(Bool())
	    val deq 	= Decoupled(gen)
		val deqPresent = Output(Bool())
	val deqIdx  = Output(UInt(n.W))
	val deqHigh = Output(Bool())
	// read port
	val ridx    = Input(Vec(rw, UInt(n.W)))
	val rdata   = Output(Vec(rw, gen))
	// write port
	val widx    = Input(Vec(ww, UInt(n.W)))
	val wen     = Input(Vec(ww, Bool()))
	val wdata   = Input(Vec(ww, gen))

	val flush 	= Input(Bool())
	val tailRestore = if (supportTailRestore) {
		Some(Flipped(Valid(new IndexTailRestore(n))))
	} else {
		None
	}
	val dbgFIFO = Output(Vec(n, gen))
}

class IndexFIFO[T <: Data : TypeTag : ClassTag](gen: T, n: Int, rw: Int, ww: Int, isFlst: Boolean = false, rstVal: Option[Seq[T]] = None, supportTailRestore: Boolean = false) extends Module {
		val io = IO(new IndexFIFOIO(gen, n, rw, ww, isFlst, supportTailRestore))

		require(!supportTailRestore || !isFlst)

	def hasFunc(funcName: String): Boolean = {
		try {
			val mirror = runtimeMirror(getClass.getClassLoader)
			val instanceMirror = mirror.reflect(gen)
			val methodSymbol = typeOf[T].member(TermName(funcName)).asMethod
			val methodMirror = instanceMirror.reflectMethod(methodSymbol)
			true
		} catch {
			case _: Exception => false
		}
	}

	val hasEnqueueFunc = hasFunc("enqueue")
	val hasWriteFunc = hasFunc("write")

	val q = RegInit(
		if(isFlst && rstVal.isDefined) VecInit(rstVal.get)
		else VecInit.fill(n)(0.U.asTypeOf(gen))
	)

	// full and empty flags
	val fulln = RegInit(true.B)
	val eptyn = RegInit(if(isFlst) true.B else false.B)

	// pointers
	val hptr = RegInit(1.U(n.W))
	val tptr = RegInit(1.U(n.W))
		val hptrHigh = RegInit(0.U(1.W))
		val tptrHigh = RegInit(0.U(1.W))

		val tailRestoreValid = io.tailRestore.map(_.valid).getOrElse(false.B)

		// pointer update logic
		val hptrInc = ShiftAdd1(hptr)
		val tptrInc = ShiftAdd1(tptr)
		val deqFire = io.deq.ready && eptyn && !tailRestoreValid
		val enqFire = io.enq.valid && fulln && !tailRestoreValid
		val hptrNxt = Mux(deqFire, hptrInc, hptr)
		val tptrNxt = Mux(enqFire, tptrInc, tptr)
		if (isFlst) {
			hptr := Mux(io.flush, tptrNxt, hptrNxt)
			tptr := tptrNxt
		} else {
			// ClusterIndexFIFO resets its bank selector on a hard flush. Reset
			// every bank's local row/generation as well so the global ring keeps
			// one canonical pointer geometry for later tail restoration.
			hptr := Mux(io.flush, 1.U(n.W), hptrNxt)
			tptr := Mux(io.flush, 1.U(n.W), tptrNxt)
		}


	val hptrHighNxt = Mux(hptrNxt(0) && hptr(n-1), ~hptrHigh, hptrHigh)
	val tptrHighNxt = Mux(tptrNxt(0) && tptr(n-1), ~tptrHigh, tptrHigh)
		if (isFlst) {
			hptrHigh := Mux(io.flush, tptrHighNxt, hptrHighNxt)
			tptrHigh := tptrHighNxt
		} else {
			hptrHigh := Mux(io.flush, 0.U, hptrHighNxt)
			tptrHigh := Mux(io.flush, 0.U, tptrHighNxt)
		}

	// full and empty flag update logic
		if (!isFlst && supportTailRestore) {
			val oneEntry = (hptrInc & tptr).orR
			val oneFree = (hptr & tptrInc).orR
			val nonEmptyNext =
				enqFire || (eptyn && !(deqFire && oneEntry))
			val notFullNext =
				deqFire || (fulln && !(enqFire && oneFree))

			when(io.flush) {
				fulln := true.B
				eptyn := false.B
			}.elsewhen(!tailRestoreValid) {
				fulln := notFullNext
				eptyn := nonEmptyNext
			}
		} else {
			if(!isFlst){
				when(io.flush){ fulln := true.B }
				.elsewhen(io.enq.valid && !tailRestoreValid) { fulln := !(hptrNxt & tptrNxt) }
				.elsewhen(io.deq.ready && !tailRestoreValid) { fulln := true.B }
			}

			when(io.flush){ eptyn := (if(isFlst) true.B else false.B) }
			.elsewhen(io.deq.ready && !tailRestoreValid) { eptyn := !(hptrNxt & tptrNxt) }
			.elsewhen(io.enq.valid && !tailRestoreValid) { eptyn := true.B }
		}

		io.tailRestore.foreach { restore =>
			when(restore.valid && !io.flush) {
				val samePtr  = hptr === restore.bits.ptrOH
				val sameHigh = hptrHigh === restore.bits.high

				tptr     := restore.bits.ptrOH
				tptrHigh := restore.bits.high
				eptyn    := !(samePtr && sameHigh)
				fulln    := !(samePtr && !sameHigh)
			}
		}

	io.enqIdx  := tptr
	io.enqHigh := tptrHigh
	io.deqIdx  := hptr
	io.deqHigh := hptrHigh
	// random access logic
	for(i <- 0 until rw){
		io.rdata(i) := Mux1H(io.ridx(i), q)
	}
	for(i <- 0 until ww){
		when(io.wen(i)){
			q.zipWithIndex.foreach{ case (qq, j) =>
				when(io.widx(i)(j)){
					if(hasWriteFunc) {
						qq.asInstanceOf[{ def write(data: T): Unit }].write(io.wdata(i))
					} else {
						qq := io.wdata(i)
					}
				}
			}
		}
	}
	// Enqueue has final priority over a random write to the slot being allocated.
	q.zipWithIndex.foreach{ case (qq, i) =>
		when(tptr(i) && io.enq.valid && fulln && !tailRestoreValid) {
			if(hasEnqueueFunc) {
				qq.asInstanceOf[{ def enqueue(data: T): Unit }].enqueue(io.enq.bits)
			} else {
				qq := io.enq.bits
			}
		}
	}
	// q.zipWithIndex.foreach{ case (qq, i) =>
	// 	if(qq.isInstanceOf[ROBEntry]){
	// 		when(io.flush){
	// 			qq.asInstanceOf[ROBEntry].bke.complete := false.B
	// 		}
	// 	}
	// }

	// read logic
	io.deq.bits  := Mux1H(hptr, q)

		io.enqCapacity := fulln
		io.enq.ready := fulln && !tailRestoreValid
			io.deqPresent := eptyn
			io.deq.valid := io.deqPresent && !tailRestoreValid

		io.dbgFIFO   := q

		io.tailRestore.foreach { restore =>
			when(restore.valid) {
				assert(PopCount(restore.bits.ptrOH) === 1.U)
				assert(!io.enq.fire)
				assert(!io.deq.fire)
			}
		}
}
