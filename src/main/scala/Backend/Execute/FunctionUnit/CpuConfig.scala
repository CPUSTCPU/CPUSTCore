package CPUSTC.backend.execute.fu

import chisel3._
import chisel3.util._

import CPUSTC.memory.{DcacheConfig, IcacheConfig, MemSysConfig}

class CpuConfig(memSysConfig: MemSysConfig) {
  private def cacheGeometry(ways: Int, sets: Int, lineBytes: Int): BigInt = {
    require(isPow2(ways) && isPow2(sets) && isPow2(lineBytes))
    BigInt(ways - 1) |
      (BigInt(log2Ceil(sets)) << 16) |
      (BigInt(log2Ceil(lineBytes)) << 24)
  }

  val Architecture: BigInt =
    (BigInt(1) << 2) | (BigInt(31) << 4) | (BigInt(31) << 12)
  val CachePresence: BigInt =
    BigInt(0x5) | (if (memSysConfig.enableL2) BigInt(0x38) else BigInt(0))
  val ICacheGeometry: BigInt = cacheGeometry(
    IcacheConfig.IcacheWay,
    IcacheConfig.IcacheSet,
    IcacheConfig.IcacheLineBytes
  )
  val DCacheGeometry: BigInt = cacheGeometry(
    DcacheConfig.DcacheWay,
    DcacheConfig.DcacheSet,
    DcacheConfig.DcacheLineBytes
  )
  val L2CacheGeometry: BigInt = if (memSysConfig.enableL2) {
    cacheGeometry(
      memSysConfig.l2Ways,
      memSysConfig.l2Sets,
      memSysConfig.l2LineBytes
    )
  } else {
    BigInt(0)
  }

  def read(index: UInt): UInt = MuxLookup(index, 0.U(32.W))(Seq(
    "h00".U -> 0.U(32.W),
    "h01".U -> Architecture.U(32.W),
    "h02".U -> 0.U(32.W),
    "h04".U -> 0.U(32.W),
    "h05".U -> 0.U(32.W),
    "h06".U -> 0.U(32.W),
    "h10".U -> CachePresence.U(32.W),
    "h11".U -> ICacheGeometry.U(32.W),
    "h12".U -> DCacheGeometry.U(32.W),
    "h13".U -> L2CacheGeometry.U(32.W)
  ))
}
