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


case class Up5kPerfParameters(ioClkFrequency : HertzNumber,
                              ioSerialBaudRate : Int){
  def toVexRiscvConfig() = {
    val config = VexRiscvConfig(
      withMemoryStage = true,
      withWriteBackStage = true,
      List(
        new IBusSimplePlugin(
          resetVector = 0x000A0000l,
          cmdForkOnSecondStage = true,
          cmdForkPersistence = true,
          prediction = DYNAMIC_TARGET,
          catchAccessFault = false,
          compressedGen = false,
          injectorStage = true,
          rspHoldValue = false,
          historyRamSizeLog2 = 8
        ),
        new DBusSimplePlugin(
          catchAddressMisaligned = true,
          catchAccessFault = false,
          emitCmdInMemoryStage = true
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
            mtvecInit      = null,
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
        new MulPlugin,
        new MulDivIterativePlugin(
          genMul = false,
          genDiv = true,
          divUnrollFactor = 1,
          dhrystoneOpt = true
        ),
        new IntAluPlugin,
        new SrcPlugin(
          separatedAddSub = true,
          executeInsertion = false,
          decodeAddSub = false
        ),
        new FullBarrelShifterPlugin(
          earlyInjection = false
        ),
        new HazardSimplePlugin(
          bypassExecute = true,
          bypassMemory = true,
          bypassWriteBack = true,
          bypassWriteBackBuffer = true
        ),
        new BranchPlugin(
          earlyBranch = false,
          catchAddressMisaligned = true,
          fenceiGenAsAJump = true
        )
      )
    )
    config
  }
}



case class Up5kPerf(p : Up5kPerfParameters) extends Component {
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
    val resetUnbuffered  = False

    //Power on reset counter
    val resetCounter = Reg(UInt(6 bits)) init(0)
    when(resetCounter =/= U(resetCounter.range -> true)){
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
    val busConfig = SimpleBusConfig(
      addressWidth = 20,
      dataWidth = 32
    )

    val dBus = SimpleBus(busConfig)
    val iBus = SimpleBus(busConfig)
    val slowBus = SimpleBus(busConfig)
    val interconnect = SimpleBusInterconnect()

    val iRam = Spram()
    val dRam = Spram()

    val peripherals = Peripherals(serialBaudRate = p.ioSerialBaudRate)
    peripherals.io.serialTx <> io.serialTx
    peripherals.io.leds <> io.leds


    val flashXip = FlashXpi(addressWidth = 19)
    RegNext(flashXip.io.flash.ss).init(1) <> io.flash.ss
    RegNext(flashXip.io.flash.sclk).init(False) <> io.flash.sclk
    RegNext(flashXip.io.flash.mosi) <> io.flash.mosi
    flashXip.io.flash.miso <> io.flash.miso

    interconnect.addSlaves(
      dRam.io.bus         -> SizeMapping(0x00000,  64 kB),
      iRam.io.bus         -> SizeMapping(0x10000,  64 kB),
      peripherals.io.bus  -> SizeMapping(0x70000, 256 Byte),
      flashXip.io.bus     -> SizeMapping(0x80000, 512 kB),
      slowBus             -> DefaultMapping
    )
    interconnect.addMasters(
      dBus   -> List(             dRam.io.bus, slowBus),
      iBus   -> List(iRam.io.bus,              slowBus),
      slowBus-> List(iRam.io.bus, dRam.io.bus,           peripherals.io.bus, flashXip.io.bus)
    )

    interconnect.setConnector(dBus, slowBus){(m, s) =>
      m.cmd.halfPipe() >> s.cmd
      m.rsp            << s.rsp
    }
    interconnect.setConnector(iBus, slowBus){(m, s) =>
      m.cmd.halfPipe() >> s.cmd
      m.rsp            << s.rsp
    }

    interconnect.setConnector(slowBus){(m, s) =>
      m.cmd >> s.cmd
      m.rsp << s.rsp.stage()
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


case class Up5kPerfEvn() extends Component{
  val io = new Bundle {
    val iceClk  = in  Bool()
    val serialTx  = out  Bool()
    val flashSpi  = master(SpiMaster())
    val leds = new Bundle {
      val r,g,b = out Bool()
    }
  }

  val pll = SB_PLL40_PAD()
  pll.PACKAGEPIN := io.iceClk
  pll.RESETB := True
  pll.BYPASS := False

  val soc = Up5kPerf(Up5kPerfParameters(
    ioClkFrequency = 24 MHz,
    ioSerialBaudRate = 115200
  ))

  soc.io.clk      <> pll.PLLOUTCORE
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
object Up5kPerf {
  def main(args: Array[String]): Unit = {
    SpinalRtlConfig().generateVerilog(Up5kPerf(Up5kPerfParameters(
      ioClkFrequency = 12 MHz,
      ioSerialBaudRate = 115200
    )))
  }
}

object Up5kPerfEvn{
  def main(args: Array[String]) {
    SpinalRtlConfig().generateVerilog(Up5kPerfEvn())
  }
}
