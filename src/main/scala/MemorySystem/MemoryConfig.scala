package CPUSTC.memory
import chisel3._
import chisel3.util._
import CPUSTC.config.MemorySystemConfig

case class MemSysConfig(
    enableL2: Boolean = MemorySystemConfig.L2Cache.enabled,
    l2Sets: Int = MemorySystemConfig.L2Cache.sets,
    l2Ways: Int = MemorySystemConfig.L2Cache.ways,
    l2LineBytes: Int = MemorySystemConfig.L2Cache.lineBytes
)

// Compatibility views keep the memory-system API stable while the canonical
// values are maintained in Config/Parameters.scala.
object IcacheConfig {
    private val p = MemorySystemConfig.ICache
    val nfetch = p.fetchWidth
    val IcacheSet = p.sets
    val IcacheWay = p.ways
    val IcacheLineWord = p.lineWords

    val IcacheLineBits = IcacheLineWord * p.dataBits
    val IcacheLineBytes = IcacheLineBits / 8
    val IcacheFetchBits = nfetch * p.dataBits

    val IcacheWaySize = IcacheSet * IcacheLineBits

    val IcacheOffset = log2Ceil(IcacheLineBytes)
    val IcacheIndex = log2Ceil(IcacheSet)
    val IcacheTag = 32-IcacheIndex-IcacheOffset

    val IcacheLruWidth = IcacheWay
    val IcacheLryHeight = log2Ceil(IcacheWay)
}

object DcacheConfig {
    private val p = MemorySystemConfig.DCache
    val nPorts = p.ports
    val DcacheSet = p.sets
    val DcacheWay = p.ways
    val DcacheLineWord = p.lineWords

    val DcacheDataBits = p.dataBits
    val DcacheMaskBits = DcacheDataBits / 8
    val DcacheLineBits = DcacheLineWord * DcacheDataBits
    val DcacheLineBytes = DcacheLineBits / 8

    val DcacheWaySize = DcacheSet * DcacheLineBits

    val DcacheOffset = log2Ceil(DcacheLineBytes)
    val DcacheIndex = log2Ceil(DcacheSet)
    val DcacheTag = 32 - DcacheIndex - DcacheOffset

    val DcacheLruWidth = DcacheWay
    val DcacheLryHeight = log2Ceil(DcacheWay)
}

object StoreQueueConfig {
    val length = MemorySystemConfig.StoreQueue.entries
    val EnqNum = MemorySystemConfig.StoreQueue.enqueueWidth
}

object LoadQueueConfig {
    val EnqNum = MemorySystemConfig.LoadQueue.enqueueWidth
}

object LoadStateTableConfig {
    val length = MemorySystemConfig.LoadStateTable.entries
}

object MshrConfig {
    val length = MemorySystemConfig.Mshr.entries
}

object WritebackBufferConfig {
    val length = MemorySystemConfig.WritebackBuffer.entries
}

object AXIConfig {
    val DataBits = MemorySystemConfig.AXI.dataBits
    val DataBytes = DataBits / 8
    val IdBits = MemorySystemConfig.AXI.idBits
    val IcacheMshrBypassLimit = MemorySystemConfig.AXI.icacheMshrBypassLimit
}
