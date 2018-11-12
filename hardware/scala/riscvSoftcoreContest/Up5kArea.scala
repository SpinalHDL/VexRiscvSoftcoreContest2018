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


object Up5kAreaCore {
  def main(args: Array[String]): Unit = {
    SpinalRtlConfig().generateVerilog(Up5kArea.core(Up5kAreaParameters(1 MHz, 115200)))
  }
}

case class Up5kAreaParameters(ioClkFrequency : HertzNumber,
                              ioSerialBaudRate : Int,
                              withMemoryStage : Boolean = false,
                              withEmulation : Boolean = true,
                              withRfBypass : Boolean = false,
                              withPessimisticInterlock : Boolean = true,
                              withPipelining : Boolean = false,
                              withCsr : Boolean = true)



object Up5kArea {
  def main(args: Array[String]): Unit = {
    SpinalRtlConfig().generateVerilog(Up5kArea(Up5kAreaParameters(
      ioClkFrequency = 12 MHz,
      ioSerialBaudRate = 115200
    )))
  }

  def core(p : Up5kAreaParameters) = {
    import p._
    assert(!(withRfBypass && withPessimisticInterlock))
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
          catchAddressMisaligned = withCsr,
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
          catchAddressMisaligned = withCsr,
          fenceiGenAsAJump = withPipelining,
          fenceiGenAsANop = !withPipelining
        )
      )
    )
    if(withPipelining){
      config.plugins += (if(withPessimisticInterlock) {
        new HazardPessimisticPlugin
      } else {
        new HazardSimplePlugin(
          bypassExecute = withRfBypass,
          bypassWriteBackBuffer = withRfBypass
        )
      })
    }
    if(withCsr) config.plugins += new CsrPlugin(
      if (withEmulation) new CsrPluginConfig(
        catchIllegalAccess = true,
        mvendorid = null,
        marchid = null,
        mimpid = null,
        mhartid = null,
        misaExtensionsInit = 0,
        misaAccess = CsrAccess.NONE,
        mtvecAccess = CsrAccess.NONE,
        mtvecInit = 0x00008l,
        mepcAccess = CsrAccess.READ_WRITE,
        mscratchGen = false,
        mcauseAccess = CsrAccess.READ_ONLY,
        mbadaddrAccess = CsrAccess.NONE,
        mcycleAccess = CsrAccess.NONE,
        minstretAccess = CsrAccess.NONE,
        ecallGen = true,
        ebreakGen = true,
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
    new VexRiscv(config)
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
    val systemResetBuffered  = RegNext(mainClkResetUnbuffered)
    val systemReset = SB_GB(systemResetBuffered)
  }


  val systemClockDomain = ClockDomain(
    clock = io.clk,
    reset = resetCtrl.systemReset,
    frequency = FixedFrequency(p.ioClkFrequency)
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
      interconnect.setConnector(mainBus) { (i, b) =>
        i.cmd.stage() >> b.cmd
        i.rsp << b.rsp
      }
    }

//    interconnect.setConnector(dBus) { (i, b) =>
////      i.cmd >> b.cmd
//      i.cmd.valid <> (b.cmd.valid)
//      i.cmd.payload <> (b.cmd.payload)
//      (i.cmd.ready):= RegNext(b.cmd.ready)
//
//      i.rsp << b.rsp
//    }


//    interconnect.setConnector(iBus) { (i, b) =>
//      i.cmd.stage >> b.cmd
//
//      i.rsp << b.rsp.stage
//    }


    //Map the CPU into the SoC
    val cpu = Up5kArea.core(p)
    for (plugin <- cpu.plugins) plugin match {
      case plugin: IBusSimplePlugin =>
        val cmd = plugin.iBus.cmd
        val rsp = plugin.iBus.rsp
        iBus.cmd.valid := cmd.valid
        iBus.cmd.wr := False
        iBus.cmd.address := cmd.pc.resized
        iBus.cmd.data.assignDontCare()
        iBus.cmd.mask.assignDontCare()
        cmd.ready := iBus.cmd.ready

        rsp.valid := iBus.rsp.valid
        rsp.error := False
        rsp.inst := iBus.rsp.data

//        rsp.valid := RegNext(iBus.rsp.valid) init(False)
//        rsp.error := False
//        rsp.inst := RegNextWhen(iBus.rsp.data, iBus.rsp.valid)
      case plugin: DBusSimplePlugin => {
        val cmd = plugin.dBus.cmd
        val rsp = plugin.dBus.rsp
        dBus.cmd.valid := cmd.valid
        dBus.cmd.wr := cmd.wr
        dBus.cmd.address := cmd.address.resized
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



object Up5kAreaEvn{
  def main(args: Array[String]) {
    SpinalRtlConfig().generateVerilog(Up5kAreaEvn())
  }
}

case class Up5kAreaEvn() extends Component{
  val io = new Bundle {
    val iceClk  = in  Bool() //35

    val serialTx  = out  Bool() //

    val flashSpi  = master(SpiMaster()) //16 15 14 17

    val leds = new Bundle {
      val r,g,b = out Bool() //41 40 39
    }
  }

  val mainClkBuffer = SB_GB()
  mainClkBuffer.USER_SIGNAL_TO_GLOBAL_BUFFER <> io.iceClk

  val soc = Up5kArea(Up5kAreaParameters(
    ioClkFrequency = 12 MHz,
    ioSerialBaudRate = 115200
  ))

  soc.io.clk      <> mainClkBuffer.GLOBAL_BUFFER_OUTPUT
  soc.io.reset    <> False
  soc.io.flash    <> io.flashSpi
  soc.io.serialTx <> io.serialTx
  //    soc.io.serialTx <> io.leds.b
  //    soc.io.leds(0)  <> io.leds.r
  //    soc.io.leds(1)  <> io.leds.g
  //    True <> io.leds.r
  //    True <> io.leds.g


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

