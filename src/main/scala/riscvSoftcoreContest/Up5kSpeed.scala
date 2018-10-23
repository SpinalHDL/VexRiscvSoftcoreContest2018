package riscvSoftcoreContest

import spinal.core._
import spinal.lib._
import spinal.lib.bus.avalon.{AvalonMM, AvalonMMSlaveFactory, SYMBOLS, WORDS}
import spinal.lib.bus.misc.{BusSlaveFactoryDelayed, BusSlaveFactoryElement, SingleMapping, SizeMapping}
import spinal.lib.com.spi.SpiMaster
import spinal.lib.fsm.{State, StateMachine}
import vexriscv.{plugin, _}
import vexriscv.demo.{SimpleBus, _}
import vexriscv.ip.InstructionCacheConfig
import vexriscv.plugin._

import scala.collection.mutable
import scala.collection.mutable.ArrayBuffer

object Up5kSpeed {
  def main(args: Array[String]): Unit = {
    SpinalVerilog(Up5kSpeed(Up5kSpeedParameters(
      ioClkFrequency = 12 MHz,
      ioSerialBaudRate = 115200
    )))
  }

  def core() = new VexRiscv(
    config = VexRiscvConfig(
      List(
        new IBusSimplePlugin(
          resetVector = 0xF0120000l,
          cmdForkOnSecondStage = false,
          cmdForkPersistence = true,
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
//        new DivPlugin,
//        new MulPlugin,
        new MulDivIterativePlugin(
          genMul = true,
          genDiv = true,
          mulUnrollFactor = 1,
          divUnrollFactor = 1
        ),
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
          earlyBranch = true,
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


object SimpleBusArbiter{
  def apply(inputs : Seq[SimpleBus], pendingRspMax : Int): SimpleBus = {
    val c = SimpleBusArbiter(inputs.head.config, inputs.size, pendingRspMax)
    (inputs, c.io.inputs).zipped.foreach(_ <> _)
    c.io.output
  }
}

case class SimpleBusArbiter(simpleBusConfig : SimpleBusConfig, portCount : Int, pendingRspMax : Int) extends Component{
  val io = new Bundle{
    val inputs = Vec(slave(SimpleBus(simpleBusConfig)), portCount)
    val output = master(SimpleBus(simpleBusConfig))
  }

  val arbiter = StreamArbiterFactory.lowerFirst.transactionLock.build(SimpleBusCmd(simpleBusConfig), portCount)
  (arbiter.io.inputs, io.inputs).zipped.foreach(_ <> _.cmd)

  val (outputCmdFork, routeCmdFork) = StreamFork2(arbiter.io.output)
  io.output.cmd << outputCmdFork

  val rspRoute = routeCmdFork.translateWith(arbiter.io.chosen).throwWhen(routeCmdFork.wr).queueLowLatency(size = pendingRspMax,latency = 1)
  rspRoute.ready := io.output.rsp.valid

  for((input, id) <- io.inputs.zipWithIndex){
    input.rsp.valid := io.output.rsp.valid && rspRoute.payload === id
    input.rsp.payload := io.output.rsp.payload
  }


//  val inputOH = OHMasking.first(io.inputs.map(_.cmd.valid).asBits)
//  val inputSel = OHToUInt(inputOH)
//
//  val cmd = Stream(SimpleBusCmd(simpleBusConfig))
//  cmd.valid   := io.inputs.map(_.cmd.valid).orR
//  cmd.payload := io.inputs(inputSel).cmd.payload
//  for((input, sel) <- (io.inputs,inputOH.asBools).zipped){
//    input.cmd.ready := cmd.ready && sel
//  }
//
//  val (outputCmdFork, routeCmdFork) = StreamFork2(cmd)
//  io.output.cmd << outputCmdFork
//
//  val rspRoute = routeCmdFork.translateWith(inputSel).throwWhen(routeCmdFork.wr).queueLowLatency(size = pendingRspMax,latency = 1)
//  rspRoute.ready := io.output.rsp.valid
//
//  for((input, id) <- io.inputs.zipWithIndex){
//    input.rsp.valid := io.output.rsp.valid && rspRoute.payload === id
//    input.rsp.payload := io.output.rsp.payload
//  }
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

case class SimpleBusDecoder(busConfig : SimpleBusConfig, mappings : Seq[SizeMapping], pendingMax : Int = 7) extends Component{
  val io = new Bundle {
    val input = slave(SimpleBus(busConfig))
    val outputs = Vec(master(SimpleBus(busConfig)), mappings.size)
  }
  def masterPipelined = io.input

  val slaveBuses = io.outputs
  val memorySpaces = mappings

  val hits = Vec(for((slaveBus, memorySpace) <- (slaveBuses, memorySpaces).zipped) yield {
    val hit = memorySpace.hit(masterPipelined.cmd.address)
    slaveBus.cmd.valid   := masterPipelined.cmd.valid && hit
    slaveBus.cmd.payload := masterPipelined.cmd.payload.resized
    hit
  })
  val noHit = !hits.orR
  masterPipelined.cmd.ready := (hits,slaveBuses).zipped.map(_ && _.cmd.ready).orR || noHit

  val rspPendingCounter = Reg(UInt(log2Up(pendingMax + 1) bits)) init(0)
  rspPendingCounter := rspPendingCounter + U(masterPipelined.cmd.fire && !masterPipelined.cmd.wr) - U(masterPipelined.rsp.valid)
  val rspHits = RegNextWhen(hits, masterPipelined.cmd.fire)
  val rspPending  = rspPendingCounter =/= 0
  val rspNoHit    = !rspHits.orR
  masterPipelined.rsp.valid   := slaveBuses.map(_.rsp.valid).orR || (rspPending && rspNoHit)
  masterPipelined.rsp.payload := slaveBuses.map(_.rsp.payload).read(OHToUInt(rspHits))

  val cmdWait = masterPipelined.cmd.valid && rspPending && hits =/= rspHits
  when(cmdWait){
    masterPipelined.cmd.ready := False
    slaveBuses.foreach(_.cmd.valid := False)
  }
}

object SimpleBusConnectors{
  def direct(m : SimpleBus, s : SimpleBus) : Unit = m >> s
}

case class SimpleBusInterconnect(){
  case class MasterModel(var connector : (SimpleBus,SimpleBus) => Unit = SimpleBusConnectors.direct)
  case class SlaveModel(mapping : SizeMapping, var connector : (SimpleBus,SimpleBus) => Unit = SimpleBusConnectors.direct)
  case class ConnectionModel(m : SimpleBus, s : SimpleBus, var connector : (SimpleBus,SimpleBus) => Unit = SimpleBusConnectors.direct)
  val masters = mutable.LinkedHashMap[SimpleBus, MasterModel]()
  val slaves = mutable.LinkedHashMap[SimpleBus, SlaveModel]()
  val connections = ArrayBuffer[ConnectionModel]()
  var pendingRspMax = 2


  def setConnector(bus : SimpleBus, connector : (SimpleBus,SimpleBus) => Unit): Unit = (masters.get(bus), slaves.get(bus)) match {
    case (Some(m), None) => m.connector = connector
    case (None, Some(s)) => s.connector = connector
  }

  def setConnector(m : SimpleBus, s : SimpleBus, connector : (SimpleBus,SimpleBus) => Unit): Unit = connections.find(e => e.m == m && e.s == s) match {
    case Some(c) => c.connector = connector
  }

  def addSlave(bus: SimpleBus,mapping: SizeMapping) : this.type = {
    slaves(bus) = SlaveModel(mapping)
    this
  }

  def addSlaves(orders : (SimpleBus,SizeMapping)*) : this.type = {
    orders.foreach(order => addSlave(order._1,order._2))
    this
  }

  def addMaster(bus : SimpleBus, accesses : Seq[SimpleBus]) : this.type = {
    masters(bus) = MasterModel()
    for(s <- accesses) connections += ConnectionModel(bus, s)
    this
  }

  def addMasters(specs : (SimpleBus,Seq[SimpleBus])*) : this.type = {
    specs.foreach(spec => addMaster(spec._1,spec._2))
    this
  }

  def build(): Unit ={
    def applyName(bus : Bundle,name : String, onThat : Nameable) : Unit = {
      if(bus.component == Component.current)
        onThat.setCompositeName(bus,name)
      else if(bus.isNamed)
        onThat.setCompositeName(bus.component,bus.getName() + "_" + name)
    }

    val connectionsInput  = mutable.HashMap[ConnectionModel,SimpleBus]()
    val connectionsOutput = mutable.HashMap[ConnectionModel,SimpleBus]()
    for((bus, model) <- masters){
      val busConnections = connections.filter(_.m == bus)
      val busSlaves = busConnections.map(c => slaves(c.s))
      val decoder = new SimpleBusDecoder(bus.config, busSlaves.map(_.mapping))
      applyName(bus,"decoder",decoder)
      model.connector(bus, decoder.io.input)
      for((connection, decoderOutput) <- (busConnections, decoder.io.outputs).zipped) {
        connectionsInput(connection) = decoderOutput
      }
    }

    for((bus, model) <- slaves){
      val busConnections = connections.filter(_.s == bus)
      val busMasters = busConnections.map(c => masters(c.m))
      val arbiter = new SimpleBusArbiter(bus.config, busMasters.size, pendingRspMax)
      applyName(bus,"arbiter",arbiter)
      model.connector(arbiter.io.output, bus)
      for((connection, arbiterInput) <- (busConnections, arbiter.io.inputs).zipped) {
        connectionsOutput(connection) = arbiterInput
      }
    }

    for(connection <- connections){
      connection.connector(connectionsInput(connection), connectionsOutput(connection))
    }
  }

  //Will make SpinalHDL calling the build function at the end of the current component elaboration
  Component.current.addPrePopTask(build)
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
      when(counter === widthOf(bitstream)*2-1){
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
      when(counter === widthOf(bitstream)*2-1){
        io.bus.cmd.ready := True
        goto(PAYLOAD)
      }
    }

    PAYLOAD.onEntry(counter := 0)
    PAYLOAD.whenIsActive{
      io.flash.ss(0) := False
      io.flash.sclk := counter.lsb
      buffer.fill setWhen(counter.lsb)
      when(counter === 32*2-1){
        goto(IDLE)
      }
    }
  }
}

case class Peripherals(serialBaudRate : Int) extends Component{
  val io = new Bundle{
    val bus = slave(SimpleBus(6, 32))
    val mTimeInterrupt = out Bool()
    val leds = out Bits(3 bits)
    val serialTx = out Bool()
  }

  val mapper = new SimpleBusSlaveFactory(io.bus)
  mapper.driveAndRead(io.leds, 0x4, 0) init(0)

  val mTime = new Area {
    val counter = Reg(UInt(32 bits)) init(0)
    val cmp = Reg(UInt(32 bits)) init(0)
    counter := counter + 1
    io.mTimeInterrupt := (cmp - counter).msb
    mapper.readMultiWord(counter, 0x10)
    mapper.writeMultiWord(cmp, 0x18)
  }

  val serialTx = new Area{
    val counter = Counter(12)
    val buffer = Reg(Bits(8 bits))
    val bitstream = buffer ## "0111"
    val busy = counter =/= 0

    io.serialTx := RegNext(bitstream(counter)) init(True)
    val timer = CounterFreeRun(ClockDomain.current.frequency.getValue.toInt/serialBaudRate)
    when(counter =/= 0 && timer.willOverflow){
      counter.increment()
    }

    mapper.write(buffer, 0x0, 0)
    when(mapper.isWriting(0x0)) { counter.increment() }
    mapper.read(busy, 0x0, 0)
  }
}



case class Up5kSpeedParameters(ioClkFrequency : HertzNumber,
                               ioSerialBaudRate : Int)



case class Up5kSpeed(p : Up5kSpeedParameters) extends Component {
  val io = new Bundle {
    val clk, reset = in Bool()
    val leds = out Bits(3 bits)
    val serialTx = out Bool()
    val flash = master(SpiMaster())
  }

  val resetCtrlClockDomain = ClockDomain(
    clock = io.clk,
    config = ClockDomainConfig(
      resetKind = BOOT
    )
  )


  val resetCtrl = new ClockingArea(resetCtrlClockDomain) {
    val mainClkResetUnbuffered  = False

    //Implement an counter to keep the reset mainClkResetUnbuffered high 64 cycles
    // Also this counter will automatically do a reset when the system boot.
    val systemClkResetCounter = Reg(UInt(6 bits)) init(0)
    when(systemClkResetCounter =/= U(systemClkResetCounter.range -> true)){
      systemClkResetCounter := systemClkResetCounter + 1
      mainClkResetUnbuffered := True
    }
    when(BufferCC(io.reset)){
      systemClkResetCounter := 0
    }

    //Create all reset used later in the design
    val systemReset  = RegNext(mainClkResetUnbuffered)
  }


  val systemClockDomain = ClockDomain(
    clock = io.clk,
    reset = resetCtrl.systemReset,
    frequency = FixedFrequency(p.ioClkFrequency)
  )

  val system = new ClockingArea(systemClockDomain) {

    val mainBusConfig = SimpleBusConfig(
      addressWidth = 32,
      dataWidth = 32
    )

    val slowBusConfig = SimpleBusConfig(
      addressWidth = 21,
      dataWidth = 32
    )

    val dBus = SimpleBus(mainBusConfig)
    val iBus = SimpleBus(mainBusConfig)
    val slowBus = SimpleBus(slowBusConfig)
    val fastInterconnect = SimpleBusInterconnect()
    val slowInterconnect = SimpleBusInterconnect()

    val iRam = Spram(mainBusConfig)
    val dRam = Spram(mainBusConfig)

    val peripherals = Peripherals(serialBaudRate = p.ioSerialBaudRate)
    peripherals.io.serialTx <> io.serialTx
    peripherals.io.leds <> io.leds


    val flashXip = FlashXpi(addressWidth = 20)
    RegNext(flashXip.io.flash.ss).init(1) <> io.flash.ss
    RegNext(flashXip.io.flash.sclk).init(False) <> io.flash.sclk
    RegNext(flashXip.io.flash.mosi) <> io.flash.mosi
    flashXip.io.flash.miso <> io.flash.miso

    fastInterconnect.addSlave(iRam.io.bus, (0x80000000l, 64 kB))
    fastInterconnect.addSlave(dRam.io.bus, (0x90000000l, 64 kB))
    fastInterconnect.addSlave(slowBus    , (0xF0000000l,  2 MB))
    fastInterconnect.addMaster(dBus, accesses = List(iRam.io.bus, dRam.io.bus, slowBus))
    fastInterconnect.addMaster(iBus, accesses = List(iRam.io.bus,              slowBus))

    slowInterconnect.addSlave(peripherals.io.bus, (0x000000, 256 Byte))
    slowInterconnect.addSlave(flashXip.io.bus, (0x100000, 1 MB))
    slowInterconnect.addMaster(slowBus,accesses =  List(peripherals.io.bus, flashXip.io.bus))





    //Map the CPU into the SoC
    val cpu = Up5kSpeed.core()
    for (plugin <- cpu.plugins) plugin match {
      case plugin: IBusSimplePlugin =>
        val cmd = plugin.iBus.cmd //TODO improve
        val rsp = plugin.iBus.rsp
        iBus.cmd.valid := cmd.valid
        iBus.cmd.wr := False
        iBus.cmd.address := cmd.pc
        iBus.cmd.data.assignDontCare()
        iBus.cmd.mask.assignDontCare()
        cmd.ready := iBus.cmd.ready

        rsp.valid := iBus.rsp.valid
        rsp.error := False
        rsp.inst := iBus.rsp.data
      case plugin: DBusSimplePlugin => {
        val cmd = plugin.dBus.cmd.s2mPipe() //TODO improve
        val rsp = plugin.dBus.rsp
        dBus.cmd.valid := cmd.valid
        dBus.cmd.wr := cmd.wr
        dBus.cmd.address := cmd.address
        dBus.cmd.data := cmd.data
        dBus.cmd.mask := cmd.size.mux(
          0 -> B"0001",
          1 -> B"0011",
          default -> B"1111"
        ) |<< cmd.address(1 downto 0)
        cmd.ready := dBus.cmd.ready

        rsp.ready := dBus.rsp.valid
        rsp.data := dBus.rsp.data
      }
      case plugin: CsrPlugin => {
        plugin.externalInterrupt := False
        plugin.timerInterrupt := peripherals.io.mTimeInterrupt
      }
      case _ =>
    }
  }
}

case class SB_GB() extends BlackBox{
  val USER_SIGNAL_TO_GLOBAL_BUFFER = in Bool()
  val GLOBAL_BUFFER_OUTPUT = out Bool()
}


object Up5kSpeedEvaluationBoard{
  case class Up5kSpeedEvaluationBoard() extends Component{
    val io = new Bundle {
      val iceClk  = in  Bool() //35

//      val serialTx  = out  Bool() //

      val flashSpi  = master(SpiMaster()) //16 15 14 17

      val leds = new Bundle {
        val r,g,b = out Bool() //41 40 39
      }
    }

    val mainClkBuffer = SB_GB()
    mainClkBuffer.USER_SIGNAL_TO_GLOBAL_BUFFER <> io.iceClk

    val soc = Up5kSpeed(Up5kSpeedParameters(
      ioClkFrequency = 12 MHz,
      ioSerialBaudRate = 115200
    ))

    soc.io.clk      <> mainClkBuffer.GLOBAL_BUFFER_OUTPUT
    soc.io.reset    <> False
    soc.io.flash    <> io.flashSpi
    //    soc.io.serialTx <> io.serialTx
    soc.io.serialTx <> io.leds.b
//    soc.io.leds(0)  <> io.leds.r
    False  <> io.leds.r
//    soc.io.leds(1)  <> io.leds.g
    True  <> io.leds.g
//    soc.io.leds(2)  <> io.leds.b
  }

  def main(args: Array[String]) {
    SpinalVerilog(Up5kSpeedEvaluationBoard())
  }
}
