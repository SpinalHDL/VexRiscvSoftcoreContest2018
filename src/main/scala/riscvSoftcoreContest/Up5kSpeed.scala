package riscvSoftcoreContest

import spinal.core._
import spinal.lib._
import spinal.lib.bus.avalon.{AvalonMM, AvalonMMSlaveFactory, SYMBOLS, WORDS}
import spinal.lib.bus.misc.{BusSlaveFactoryDelayed, BusSlaveFactoryElement, SingleMapping, SizeMapping}
import spinal.lib.com.spi.SpiMaster
import spinal.lib.fsm.{State, StateMachine}
import vexriscv.{plugin, _}
import vexriscv.demo._
import vexriscv.ip.InstructionCacheConfig
import vexriscv.plugin._

import scala.collection.mutable.ArrayBuffer

object Up5kSpeed {
  def main(args: Array[String]): Unit = {
    SpinalVerilog(Up5kSpeed())
  }

  def core() = new VexRiscv(
    config = VexRiscvConfig(
      List(
        new IBusSimplePlugin(
          resetVector = 0x80000000l,
          cmdForkOnSecondStage = false,
          cmdForkPersistence = false,
          prediction = NONE,
          catchAccessFault = false,
          compressedGen = false,
          injectorStage = true,
          rspHoldValue = false
        ),
//        new IBusCachedPlugin(
//          resetVector = 0x80000000l,
//          config = InstructionCacheConfig(
//            cacheSize = 4096,
//            bytePerLine = 32,
//            wayCount = 1,
//            addressWidth = 32,
//            cpuDataWidth = 32,
//            memDataWidth = 32,
//            catchIllegalAccess = false,
//            catchAccessFault = true,
//            catchMemoryTranslationMiss = false,
//            asyncTagMemory = false,
//            twoCycleRam = true,
//            twoCycleCache = true
//          )
//        ),
        new DBusSimplePlugin(
          catchAddressMisaligned = true,
          catchAccessFault = false
        ),
        new CsrPlugin(
          new CsrPluginConfig(
            catchIllegalAccess = false,
            mvendorid      = null,
            marchid        = null,
            mimpid         = null,
            mhartid        = null,
            misaExtensionsInit = 0,
            misaAccess     = CsrAccess.READ_ONLY,
            mtvecAccess    = CsrAccess.WRITE_ONLY,
            mtvecInit      = 0x80000020l,
            mepcAccess     = CsrAccess.READ_WRITE,
            mscratchGen    = true,
            mcauseAccess   = CsrAccess.READ_ONLY,
            mbadaddrAccess = CsrAccess.READ_ONLY,
            mcycleAccess   = CsrAccess.NONE,
            minstretAccess = CsrAccess.NONE,
            ecallGen       = true,
            ebreakGen      = true,
            wfiGenAsWait   = false,
            wfiGenAsNop    = true,
            ucycleAccess   = CsrAccess.NONE
          )
        ),
        new DecoderSimplePlugin(
          catchIllegalInstruction = false
        ),
        new RegFilePlugin(
          regFileReadyKind = plugin.SYNC,
          zeroBoot = false
        ),
        new DivPlugin,
        new MulPlugin,
        new IntAluPlugin,
        new SrcPlugin(
          separatedAddSub = false,
          executeInsertion = true
        ),
        new FullBarrelShifterPlugin(
          earlyInjection = true
        ),
        new HazardSimplePlugin(
          bypassExecute = true,
          bypassMemory = true,
          bypassWriteBack = true,
          bypassWriteBackBuffer = true
        ),
        new BranchPlugin(
          earlyBranch = false,
          catchAddressMisaligned = true
        )
      )
    )
  )
}



case class SB_SPRAM256KA() extends BlackBox{
  val DATAIN = in Bits(16 bits)
  val ADDRESS = in UInt(14 bits)
  val MASKWREN = in Bits(4 bits)
  val WREN = in Bool()
  val CHIPSELECT = in Bool()
  val CLOCK = in Bool()
  val DATAOUT = out Bits(16 bits)
  val STANDBY = in Bool()
  val SLEEP = in Bool()
  val POWEROFF = in Bool()
  mapCurrentClockDomain(CLOCK)
}



class IRom() extends Component{
  val io = new Bundle{
    val bus = slave(IBusSimpleBus())
  }


}

case class Spram(simpleBusConfig : SimpleBusConfig) extends Component{
  val io = new Bundle{
    val bus = slave(SimpleBus(simpleBusConfig))
  }

  val cmd = Flow(SimpleBusCmd(simpleBusConfig))
  cmd << io.bus.cmd.toFlow

  val rspPending = RegNext(cmd.valid && !cmd.wr) init(False)
  val rspTarget = RegNext(io.bus.cmd.valid)


  val mems = List.fill(2)(SB_SPRAM256KA())
  mems(0).DATAIN := cmd.data(15 downto 0)
  mems(0).MASKWREN := cmd.mask(1) ## cmd.mask(1) ## cmd.mask(0) ## cmd.mask(0)
  mems(1).DATAIN := cmd.data(31 downto 16)
  mems(1).MASKWREN := cmd.mask(3) ## cmd.mask(3) ## cmd.mask(2) ## cmd.mask(2)
  for(mem <- mems){
    mem.CHIPSELECT := cmd.valid
    mem.ADDRESS := (cmd.address >> 2).resized
    mem.WREN := cmd.wr
    mem.STANDBY  := False
    mem.SLEEP    := False
    mem.POWEROFF := True
  }

  val readData = mems(1).DATAOUT ## mems(0).DATAOUT


  io.bus.rsp.valid := rspPending && rspTarget
  io.bus.rsp.data  := readData
}


case class SimpleBusArbiter(simpleBusConfig : SimpleBusConfig) extends Component{
  val io = new Bundle{
    val iBus = slave(SimpleBus(simpleBusConfig))
    val dBus = slave(SimpleBus(simpleBusConfig))
    val masterBus = master(SimpleBus(simpleBusConfig))
  }

  io.masterBus.cmd.valid   := io.iBus.cmd.valid || io.dBus.cmd.valid
  io.masterBus.cmd.wr      := io.dBus.cmd.valid && io.dBus.cmd.wr
  io.masterBus.cmd.address := io.dBus.cmd.valid ? io.dBus.cmd.address | io.iBus.cmd.address
  io.masterBus.cmd.data    := io.dBus.cmd.data
  io.masterBus.cmd.mask    := io.dBus.cmd.mask
  io.iBus.cmd.ready := io.masterBus.cmd.ready && !io.dBus.cmd.valid
  io.dBus.cmd.ready := io.masterBus.cmd.ready


  val rspPending = RegInit(False) clearWhen(io.masterBus.rsp.valid)
  val rspTarget = RegInit(False)
  when(io.masterBus.cmd.fire && !io.masterBus.cmd.wr){
    rspTarget  := io.dBus.cmd.valid
    rspPending := True
  }

  when(rspPending && !io.masterBus.rsp.valid){
    io.iBus.cmd.ready := False
    io.dBus.cmd.ready := False
    io.masterBus.cmd.valid := False
  }

  io.iBus.rsp.valid := io.masterBus.rsp.valid && !rspTarget
  io.iBus.rsp.data  := io.masterBus.rsp.data

  io.dBus.rsp.valid := io.masterBus.rsp.valid && rspTarget
  io.dBus.rsp.data  := io.masterBus.rsp.data
}

class SimpleBusSlaveFactory(bus: SimpleBus) extends BusSlaveFactoryDelayed{
  bus.cmd.ready := True

  val readAtCmd = Flow(Bits(bus.config.dataWidth bits))
  val readAtRsp = readAtCmd.stage()

  val askWrite = (bus.cmd.valid && bus.cmd.wr).allowPruning()
  val askRead  = (bus.cmd.valid && !bus.cmd.wr).allowPruning()
  val doWrite  = (askWrite && bus.cmd.ready).allowPruning()
  val doRead   = (askRead  && bus.cmd.ready).allowPruning()

  bus.rsp.valid := readAtRsp.valid
  bus.rsp.data := readAtRsp.payload

  readAtCmd.valid := doRead
  readAtCmd.payload := 0

  def readAddress() : UInt = bus.cmd.address
  def writeAddress() : UInt = bus.cmd.address

  override def readHalt(): Unit = bus.cmd.ready := False
  override def writeHalt(): Unit = bus.cmd.ready := False

  override def build(): Unit = {
    super.doNonStopWrite(bus.cmd.data)

    def doMappedElements(jobs : Seq[BusSlaveFactoryElement]) = super.doMappedElements(
      jobs = jobs,
      askWrite = askWrite,
      askRead = askRead,
      doWrite = doWrite,
      doRead = doRead,
      writeData = bus.cmd.data,
      readData = readAtCmd.payload
    )

    switch(bus.cmd.address) {
      for ((address, jobs) <- elementsPerAddress if address.isInstanceOf[SingleMapping]) {
        is(address.asInstanceOf[SingleMapping].address) {
          doMappedElements(jobs)
        }
      }
    }

    for ((address, jobs) <- elementsPerAddress if !address.isInstanceOf[SingleMapping]) {
      when(address.hit(bus.cmd.address)){
        doMappedElements(jobs)
      }
    }
  }

  override def busDataWidth: Int = bus.config.dataWidth
  override def wordAddressInc: Int = busDataWidth / 8
}

case class FlashXpi(addressWidth : Int) extends Component{
  val io = new Bundle {
    val bus = slave(SimpleBus(addressWidth, 32))
    val flash = master(SpiMaster())
  }

  io.bus.cmd.ready := False
  io.flash.ss(0) := True
  io.flash.sclk := False
  io.flash.mosi := False


  val buffer = new Area{
    val fill = RegNext(False) init(False)
    val buffer = Reg(Bits(32 bits))
    val counter = Counter(32)
    when(fill){
      counter.increment()
      buffer := buffer(30 downto 0) ## io.flash.miso
    }

    val done = RegNext(counter.willOverflow) init(False)
    io.bus.rsp.valid := done
    io.bus.rsp.data := buffer.subdivideIn(8 bits).reverse.asBits() //little endian conversion
  }

  val fsm = new StateMachine{
    val SETUP, IDLE, CMD, PAYLOAD = State()
    setEntry(SETUP)

    val counter = Reg(UInt(7 bits))
    counter := counter + 1
    SETUP.onEntry(counter := 0)
    SETUP.whenIsActive{
      io.flash.ss(0) := False
      io.flash.sclk := counter.lsb
      val bitstream = B"x8183"
      io.flash.mosi := bitstream.asBools.reverse((counter >> 1).resized)
      when(counter === widthOf(counter >> 1)){
        goto(IDLE)
      }
    }

    IDLE.whenIsActive{
      when(io.bus.cmd.valid){
        goto(CMD)
      }
    }


    CMD.onEntry(counter := 0)
    CMD.whenIsActive{
      io.flash.ss(0) := False
      io.flash.sclk := counter.lsb
      val bitstream = B"x0B" ## io.bus.cmd.address.resize(24 bits) ## B"x00"
      io.flash.mosi := bitstream.asBools.reverse((counter >> 1).resized)
      when(counter === widthOf(bitstream)){
        io.bus.cmd.ready := True
        goto(PAYLOAD)
      }
    }

    PAYLOAD.onEntry(counter := 0)
    PAYLOAD.whenIsActive{
      io.flash.ss(0) := False
      io.flash.sclk := counter.lsb
      buffer.fill setWhen(counter.lsb)
      when(counter === 32*2){
        goto(IDLE)
      }
    }



  }
}

case class Peripherals() extends Component{
  val io = new Bundle{
    val bus = slave(SimpleBus(6, 32))
    val mTimeInterrupt = out Bool()
    val leds = out Bits(3 bits)
    val serialTx = out Bool()
//    val flashSpi = master(SpiMaster())
  }

  val mTime = Reg(UInt(64 bits)) init(0)
  val mTimeCmp = Reg(UInt(64 bits)) init(0)
  mTime := mTime + 1
  io.mTimeInterrupt := mTime > mTimeCmp

  val serialTx = new Area{
    val counter = Counter(12)
    val buffer = Reg(Bits(8 bits))
    val bitstream = buffer ## "0111"
    val busy = counter =/= 0

    io.serialTx := RegNext(bitstream(counter)) init(True)
    val timer = CounterFreeRun(200)
    when(counter =/= 0 && timer.willOverflow){
      counter.increment()
    }
  }


  val mapper = new SimpleBusSlaveFactory(io.bus)
  mapper.write(serialTx.buffer, 0x0, 0)
  when(mapper.isWriting(0x0)) { serialTx.counter.increment() }
  mapper.read(serialTx.busy, 0x0, 0)
  mapper.driveAndRead(io.leds, 0x4, 0) init(0)
//  mapper.drive(io.flashSpi.mosi, 0x4, 0)
//  mapper.drive(io.flashSpi.sclk, 0x4, 1) init(False)
//  mapper.drive(io.flashSpi.ss(0), 0x4, 2) init(True)
//  mapper.read(io.flashSpi.miso, 0x8, 0)
  mapper.readMultiWord(mTime, 0x10)
  mapper.writeMultiWord(mTimeCmp, 0x18)
}


case class Up5kSpeed() extends Component {
  val pipelineDBus = true

  val io = new Bundle {
    val leds = out Bits(3 bits)
    val serialTx = out Bool()
    val flash = master(SpiMaster())
  }

  val cpu = Up5kSpeed.core()
  val mainBusConfig = SimpleBusConfig(
    addressWidth = 32,
    dataWidth = 32
  )

  val slowBusConfig = SimpleBusConfig(
    addressWidth = 21,
    dataWidth = 32
  )

  val dBus = SimpleBus(mainBusConfig)
  val dBusMapping = ArrayBuffer[(SimpleBus,SizeMapping)]()
  val iBus = SimpleBus(mainBusConfig)
  val iBusMapping = ArrayBuffer[(SimpleBus,SizeMapping)]()
  val slowBus = SimpleBus(slowBusConfig)
  val slowMapping = ArrayBuffer[(SimpleBus,SizeMapping)]()


  val iRam = Spram(mainBusConfig)

  val iRamArbiter = SimpleBusArbiter(mainBusConfig)
  iRamArbiter.io.masterBus <> iRam.io.bus
  iBusMapping += iRamArbiter.io.iBus -> (0x80000000l, 64 kB)
  dBusMapping += iRamArbiter.io.dBus -> (0x80000000l, 64 kB)

  val dRam = Spram(mainBusConfig)
  dBusMapping += dRam.io.bus -> (0x90000000l, 64 kB)

  val slowArbiter = SimpleBusArbiter(mainBusConfig)
  slowArbiter.io.masterBus.resizableAddress() <> slowBus
  iBusMapping += slowArbiter.io.iBus -> (0xF0000000l, 2 MB)
  dBusMapping += slowArbiter.io.dBus -> (0xF0000000l, 2 MB)

  val peripherals = Peripherals()
  peripherals.io.serialTx <> io.serialTx
  peripherals.io.leds     <> io.leds
  slowMapping += peripherals.io.bus -> (0x000000, 256 Byte)


  val flashXip = FlashXpi(addressWidth = 20)
  RegNext(flashXip.io.flash.ss).init(1)       <> io.flash.ss
  RegNext(flashXip.io.flash.sclk).init(False) <> io.flash.sclk
  RegNext(flashXip.io.flash.mosi)             <> io.flash.mosi
  flashXip.io.flash.miso                      <> io.flash.miso
  slowMapping += flashXip.io.bus -> (0x100000, 1 MB)



  val iBusDecoder = new MuraxSimpleBusDecoder(
    master = iBus,
    specification = iBusMapping,
    pipelineMaster = false
  )

  val dBusDecoder = new MuraxSimpleBusDecoder(
    master = dBus,
    specification = dBusMapping,
    pipelineMaster = false
  )

  val slowDecoder = new MuraxSimpleBusDecoder(
    master = slowBus,
    specification = slowMapping,
    pipelineMaster = false
  )


  //Map the CPU into the SoC
  for(plugin <- cpu.plugins) plugin match{
    case plugin : IBusSimplePlugin =>
      val cmd = plugin.iBus.cmd.halfPipe() //TODO improve
      val rsp = plugin.iBus.rsp
      iBus.cmd.valid   := cmd.valid
      iBus.cmd.wr      := True
      iBus.cmd.address := cmd.pc
      iBus.cmd.data.assignDontCare()
      iBus.cmd.mask.assignDontCare()
      cmd.ready := iBus.cmd.ready

      rsp.valid := iBus.rsp.valid
      rsp.error := False
      rsp.inst := iBus.rsp.data
    case plugin : DBusSimplePlugin => {
      val cmd = plugin.dBus.cmd.halfPipe() //TODO improve
      val rsp = plugin.dBus.rsp
      dBus.cmd.valid   := cmd.valid
      dBus.cmd.wr      := cmd.wr
      dBus.cmd.address := cmd.address
      dBus.cmd.data    := cmd.data
      dBus.cmd.mask    := cmd.size.mux(
        0 -> B"0001",
        1 -> B"0011",
        default -> B"1111"
      ) |<< cmd.address(1 downto 0)
      cmd.ready := dBus.cmd.ready

      rsp.ready := dBus.rsp.valid
      rsp.data := dBus.rsp.data
    }
    case plugin : CsrPlugin        => {
      plugin.externalInterrupt := False
      plugin.timerInterrupt := peripherals.io.mTimeInterrupt
    }
    case _ =>
  }
}


