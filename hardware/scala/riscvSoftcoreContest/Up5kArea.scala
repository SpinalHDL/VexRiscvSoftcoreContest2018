package riscvSoftcoreContest

import spinal.core._
import spinal.lib._
import spinal.lib.bus.avalon.{AvalonMM, AvalonMMSlaveFactory, SYMBOLS, WORDS}
import spinal.lib.bus.misc._
import spinal.lib.com.spi.SpiMaster
import spinal.lib.fsm.{State, StateMachine}
import vexriscv.{plugin, _}
import vexriscv.demo.{SimpleBus, _}
import vexriscv.ip.InstructionCacheConfig
import vexriscv.plugin._

import scala.collection.mutable
import scala.collection.mutable.ArrayBuffer


case class Up5kAreaParameters(ioClkFrequency : HertzNumber,
                              ioSerialBaudRate : Int,
                              noComplianceOverhead : Boolean = false,
                              withMemoryStage : Boolean = false,
                              withRfBypass : Boolean = false,
                              withPipelining : Boolean = false,
                              withCsr : Boolean = true){
  def toVexRiscvConfig() = {
    val config = VexRiscvConfig(
      withMemoryStage = withMemoryStage,
      withWriteBackStage = false,
      List(
        new IBusSimplePlugin(
          resetVector = 0x000A0000l,
          cmdForkOnSecondStage = false,
          cmdForkPersistence = false,
          prediction = NONE,
          catchAccessFault = false,
          compressedGen = false,
          injectorStage = false,
          rspHoldValue = !withPipelining,
          singleInstructionPipeline = !withPipelining,
          busLatencyMin = 1,
          pendingMax = if(withPipelining) 3 else 1
        ),
        new DBusSimplePlugin(
          catchAddressMisaligned = withCsr && !noComplianceOverhead,
          catchAccessFault = false
        ),
        new DecoderSimplePlugin(
          catchIllegalInstruction = false
        ),
        new RegFilePlugin(
          regFileReadyKind = plugin.SYNC,
          zeroBoot = true,
          x0Init = false,
          readInExecute = true,
          syncUpdateOnStall = withPipelining
        ),
        new IntAluPlugin,
        new SrcPlugin(
          separatedAddSub = false,
          executeInsertion = true,
          decodeAddSub = false
        ),
        new LightShifterPlugin(),
        new BranchPlugin(
          earlyBranch = !withMemoryStage,
          catchAddressMisaligned = withCsr && !noComplianceOverhead,
          fenceiGenAsAJump = withPipelining,
          fenceiGenAsANop = !withPipelining
        )
      )
    )
    if(withPipelining){
      config.plugins += new HazardSimplePlugin(
        bypassExecute = withRfBypass,
        bypassWriteBackBuffer = withRfBypass
      )
    }
    if(withCsr) config.plugins += new CsrPlugin(
      if (noComplianceOverhead) new CsrPluginConfig(
        catchIllegalAccess = true,
        mvendorid = null,
        marchid = null,
        mimpid = null,
        mhartid = null,
        misaExtensionsInit = 0,
        misaAccess = CsrAccess.NONE,
        mtvecAccess = CsrAccess.WRITE_ONLY,
        mtvecInit = null,
        mepcAccess = CsrAccess.READ_WRITE,
        mscratchGen = false,
        mcauseAccess = CsrAccess.READ_ONLY,
        mbadaddrAccess = CsrAccess.NONE,
        mcycleAccess = CsrAccess.NONE,
        minstretAccess = CsrAccess.NONE,
        ecallGen = true,
        ebreakGen = false,
        wfiGenAsWait = false,
        wfiGenAsNop = true,
        ucycleAccess = CsrAccess.NONE
      )
      else new CsrPluginConfig(
        catchIllegalAccess = false,
        mvendorid = null,
        marchid = null,
        mimpid = null,
        mhartid = null,
        misaExtensionsInit = 0,
        misaAccess = CsrAccess.NONE,
        mtvecAccess = CsrAccess.WRITE_ONLY,
        mtvecInit = null,
        mepcAccess = CsrAccess.READ_WRITE,
        mscratchGen = true,
        mcauseAccess = CsrAccess.READ_ONLY,
        mbadaddrAccess = CsrAccess.READ_ONLY,
        mcycleAccess = CsrAccess.NONE,
        minstretAccess = CsrAccess.NONE,
        ecallGen = true,
        ebreakGen = true,
        wfiGenAsWait = false,
        wfiGenAsNop = true,
        ucycleAccess = CsrAccess.NONE
      )
    )
    config
  }
}


case class Up5kArea(p : Up5kAreaParameters) extends Component {
  val io = new Bundle {
    val clk, reset = in Bool()
    val leds = out Bits(3 bits)
    val serialTx = out Bool()
    val flash = master(SpiMaster())
  }

  val resetCtrlClockDomain = ClockDomain(
    clock = io.clk,
    config = ClockDomainConfig(
      resetKind = BOOT //Bitstream loaded FF
    )
  )


  val resetCtrl = new ClockingArea(resetCtrlClockDomain) {
    val resetUnbuffered  = False

    //Power on reset counter
    val resetCounter = Reg(UInt(6 bits)) init(0)
    when(!resetCounter.andR){
      resetCounter := resetCounter + 1
      resetUnbuffered := True
    }
    when(BufferCC(io.reset)){
      resetCounter := 0
    }

    //Create all reset used later in the design
    val systemResetBuffered  = RegNext(resetUnbuffered)
    val systemReset = SB_GB(systemResetBuffered)
  }


  val systemClockDomain = ClockDomain(
    clock = io.clk,
    reset = resetCtrl.systemReset,
    frequency = FixedFrequency(p.ioClkFrequency),
    config = ClockDomainConfig(
      resetKind = spinal.core.SYNC
    )
  )

  val system = new ClockingArea(systemClockDomain) {

    val mainBusConfig = SimpleBusConfig(
      addressWidth = 20,
      dataWidth = 32
    )


    val dBus = SimpleBus(mainBusConfig)
    val iBus = SimpleBus(mainBusConfig)
    val mainBus = SimpleBus(mainBusConfig)
    val interconnect = SimpleBusInterconnect()

    val ram = Spram()

    val peripherals = Peripherals(
      serialBaudRate = p.ioSerialBaudRate,
      smallTimer = true
    )
    peripherals.io.serialTx <> io.serialTx
    peripherals.io.leds <> io.leds


    val flashXip = FlashXpi(addressWidth = 19)
    RegNext(flashXip.io.flash.ss).init(1) <> io.flash.ss
    RegNext(flashXip.io.flash.sclk).init(False) <> io.flash.sclk
    RegNext(flashXip.io.flash.mosi) <> io.flash.mosi
    flashXip.io.flash.miso <> io.flash.miso

    interconnect.addSlaves(
      ram.io.bus          -> SizeMapping(0x00000,  64 kB),
      peripherals.io.bus  -> SizeMapping(0x70000,  64 kB),
      flashXip.io.bus     -> SizeMapping(0x80000,  512 kB),
      mainBus             -> DefaultMapping
    )
    interconnect.addMasters(
      dBus   -> List(mainBus),
      iBus   -> List(mainBus),
      mainBus-> List(ram.io.bus, peripherals.io.bus, flashXip.io.bus)
    )

    if(p.withPipelining) {
      interconnect.setConnector(mainBus) { (m, s) =>
        m.cmd.s2mPipe() >> s.cmd
        m.rsp << s.rsp
      }
    }


    //Map the CPU into the SoC
    val cpu = new VexRiscv(p.toVexRiscvConfig())
    for (plugin <- cpu.plugins) plugin match {
      case plugin : IBusSimplePlugin => iBus << plugin.iBus.toSimpleBus()
      case plugin : IBusCachedPlugin => iBus << plugin.iBus.toSimpleBus()
      case plugin : DBusSimplePlugin => dBus << plugin.dBus.toSimpleBus()
      case plugin : CsrPlugin => {
        plugin.externalInterrupt := False //Not used
        plugin.timerInterrupt := peripherals.io.mTimeInterrupt
      }
      case _ =>
    }
  }
}


case class Up5kAreaEvn() extends Component{
  val io = new Bundle {
    val iceClk  = in  Bool()
    val serialTx  = out  Bool()
    val flashSpi  = master(SpiMaster())
    val leds = new Bundle {
      val r,g,b = out Bool()
    }
  }

  val clkBuffer = SB_GB()
  clkBuffer.USER_SIGNAL_TO_GLOBAL_BUFFER <> io.iceClk

  val soc = Up5kArea(Up5kAreaParameters(
    ioClkFrequency = 12 MHz,
    ioSerialBaudRate = 115200
  ))

  soc.io.clk      <> clkBuffer.GLOBAL_BUFFER_OUTPUT
  soc.io.reset    <> False
  soc.io.flash    <> io.flashSpi
  soc.io.serialTx <> io.serialTx


  val ledDriver = SB_RGBA_DRV()
  ledDriver.CURREN   := True
  ledDriver.RGBLEDEN := True
  ledDriver.RGB0PWM  := !io.serialTx
  ledDriver.RGB1PWM  := soc.io.leds(0)
  ledDriver.RGB2PWM  := soc.io.leds(1)

  ledDriver.RGB0 <> io.leds.b
  ledDriver.RGB1 <> io.leds.g
  ledDriver.RGB2 <> io.leds.r
}


//Main used to generate the SoC design
object Up5kArea {
  def main(args: Array[String]): Unit = {
    SpinalRtlConfig().generateVerilog(Up5kArea(Up5kAreaParameters(
      ioClkFrequency = 12 MHz,
      ioSerialBaudRate = 115200
    )))
  }
}


object Up5kAreaEvn{
  def main(args: Array[String]) {
    SpinalRtlConfig().generateVerilog(Up5kAreaEvn())
  }
}
