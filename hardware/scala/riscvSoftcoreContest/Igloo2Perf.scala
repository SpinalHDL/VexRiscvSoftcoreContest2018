package riscvSoftcoreContest

import spinal.core._
import spinal.lib._
import spinal.lib.bus.avalon.{AvalonMM, AvalonMMSlaveFactory, SYMBOLS, WORDS}
import spinal.lib.bus.misc._
import spinal.lib.com.spi.SpiMaster
import spinal.lib.eda.bench.{AlteraStdTargets, Bench, Rtl, XilinxStdTargets}
import spinal.lib.eda.icestorm.IcestormStdTargets
import spinal.lib.fsm.{State, StateMachine}
import vexriscv.{plugin, _}
import vexriscv.demo.{SimpleBus, _}
import vexriscv.ip.{DataCacheConfig, InstructionCacheConfig}
import vexriscv.plugin._

import scala.collection.mutable
import scala.collection.mutable.ArrayBuffer




case class Igloo2PerfParameters(ioClkFrequency : HertzNumber,
                                ioSerialBaudRate : Int) {
  def toVexRiscvConfig() = {
    val config = VexRiscvConfig(
      List(
        new IBusSimplePlugin(
          resetVector = 0xA0000l,
          cmdForkOnSecondStage = true,
          cmdForkPersistence = false, //Don't need it, as the ibus memory has it's dedicated port and there is a halfpipe to the slowBus
          prediction = DYNAMIC_TARGET,
          catchAccessFault = false,
          compressedGen = false,
          injectorStage = true,
          rspHoldValue = false,
          historyRamSizeLog2 = 9
        ),
//      Could have been used as a instruction cache
//        new IBusCachedPlugin(
//          resetVector = 0xA0000l,
//          prediction = DYNAMIC_TARGET,
//          relaxedPcCalculation = true,
//          config = InstructionCacheConfig(
//            cacheSize = 4096,
//            bytePerLine = 32,
//            wayCount = 1,
//            addressWidth = 32,
//            cpuDataWidth = 32,
//            memDataWidth = 32,
//            catchIllegalAccess = false,
//            catchAccessFault = false,
//            catchMemoryTranslationMiss = false,
//            asyncTagMemory = false,
//            twoCycleRam = true,
//            twoCycleCache = true
//          )
//        ),
        new DBusSimplePlugin(
          catchAddressMisaligned = true,
          catchAccessFault = false,
          earlyInjection = false,
          emitCmdInMemoryStage = true
        ),
//      Could have been used as a data cache
//        new DBusCachedPlugin(
//          config = new DataCacheConfig(
//            cacheSize         = 4096,
//            bytePerLine       = 32,
//            wayCount          = 1,
//            addressWidth      = 32,
//            cpuDataWidth      = 32,
//            memDataWidth      = 32,
//            catchAccessError  = false,
//            catchIllegal      = false,
//            catchUnaligned    = true,
//            catchMemoryTranslationMiss = false
//          ),
//          memoryTranslatorPortConfig = null
//        ),
//        new StaticMemoryTranslatorPlugin(
//          ioRange      = _(19 downto 16) === 0x7
//        ),
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
            ucycleAccess   = CsrAccess.NONE,
            pipelineCsrRead = true
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


case class Igloo2Perf(p : Igloo2PerfParameters) extends Component {
  val io = new Bundle {
    val clk, reset = in Bool()
    val leds = out Bits(3 bits)
    val serialTx = out Bool()
    val serialRx = in Bool()
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

    // Keep the reset active for a while
    val bootTime = if(GenerationFlags.simulation.isEnabled) 1 ms else 100 ms
    val resetCounter = Reg(UInt(log2Up((p.ioClkFrequency*bootTime).toBigInt()) bits)) init(0)
    when(resetCounter =/= U(resetCounter.range -> true)){
      resetCounter := resetCounter + 1
      resetUnbuffered := True
    }
    when(BufferCC(io.reset)){
      resetCounter := 0
    }

    //Create all reset used later in the design
    val systemResetBuffered  = RegNext(resetUnbuffered)
    val systemReset = CombInit(systemResetBuffered)

    val progResetBuffered  = RegNext(resetUnbuffered)
    val progReset = CombInit(progResetBuffered)
  }

  //Clock domain used to program the flash via the rxSerial
  val progClockDomain = ClockDomain(
    clock = io.clk,
    reset = resetCtrl.progReset,
    frequency = FixedFrequency(p.ioClkFrequency)
  )

  val systemClockDomain = ClockDomain(
    clock = io.clk,
    reset = resetCtrl.systemReset,
    frequency = FixedFrequency(p.ioClkFrequency),
    config = ClockDomainConfig(
      resetKind = spinal.core.ASYNC
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


    val ram = SimpleBusMultiPortRam(32 kB, portCount = 2)

    val peripherals = Peripherals(serialBaudRate = p.ioSerialBaudRate)
    peripherals.io.serialTx <> io.serialTx
    peripherals.io.leds <> io.leds

    val flashXip = FlashXpi(addressWidth = 19, slowDownFactor = 3)

    interconnect.addSlaves(
      ram.io.buses(0)         -> SizeMapping(0x00000,  64 kB),
      ram.io.buses(1)         -> SizeMapping(0x00000,  64 kB),
      peripherals.io.bus  -> SizeMapping(0x70000,  64 Byte),
      flashXip.io.bus     -> SizeMapping(0x80000, 512 kB),
      slowBus             -> DefaultMapping
    )
    interconnect.addMasters(
      dBus   -> List(ram.io.buses(0), slowBus),
      iBus   -> List(ram.io.buses(1), slowBus),
      slowBus-> List(peripherals.io.bus, flashXip.io.bus)
    )

    interconnect.noTransactionLockOn(ram.io.buses)

    //Add pipelining
    interconnect.setConnector(dBus, slowBus){(m,s) =>
      m.cmd.halfPipe() >> s.cmd
      m.rsp            << s.rsp
    }
    interconnect.setConnector(iBus, slowBus){(m,s) =>
      m.cmd.halfPipe() >> s.cmd
      m.rsp            << s.rsp
    }
    interconnect.setConnector(slowBus){(m,s) =>
      m.cmd >> s.cmd
      m.rsp << s.rsp.stage()
    }
    interconnect.setConnector(dBus){(m,s) =>
      m.cmd.s2mPipe() >> s.cmd  //Cut dBus cmd ready path
      m.rsp << s.rsp
    }

    //Map the CPU into the SoC
    val cpu = new VexRiscv(p.toVexRiscvConfig())
    for (plugin <- cpu.plugins) plugin match {
      case plugin : IBusSimplePlugin => iBus << plugin.iBus.toSimpleBus()
      case plugin : IBusCachedPlugin => iBus << plugin.iBus.toSimpleBus()
      case plugin : DBusSimplePlugin => dBus << plugin.dBus.toSimpleBus()
      case plugin : DBusCachedPlugin => dBus << plugin.dBus.toSimpleBus()
      case plugin : CsrPlugin => {
        plugin.externalInterrupt := False //Not used
        plugin.timerInterrupt := peripherals.io.mTimeInterrupt
      }
      case _ =>
    }
  }

  val prog = new ClockingArea(progClockDomain){
    val ctrl = SerialRxOutput(p.ioSerialBaudRate, 0x07)
    ctrl.io.serialRx := io.serialRx
    resetCtrl.systemResetBuffered setWhen(ctrl.io.output(7))

    val ssReg, sclkReg, mosiReg = Reg(Bool) init(True)
    ssReg <> io.flash.ss(0)
    sclkReg <> io.flash.sclk
    mosiReg <> io.flash.mosi
    io.flash.miso <> system.flashXip.io.flash.miso

    when(ctrl.io.output(6)){
      ssReg := ctrl.io.output(0)
      sclkReg := ctrl.io.output(1)
      mosiReg := ctrl.io.output(2)
    } otherwise {
      ssReg := system.flashXip.io.flash.ss(0)
      sclkReg := system.flashXip.io.flash.sclk
      mosiReg := system.flashXip.io.flash.mosi
    }
  }

}

case class Igloo2PerfCreative(p : Igloo2PerfParameters) extends Component{
  val io = new Bundle {
    val serialTx  = out  Bool()
    val serialRx  = in  Bool()
    val flashSpi  = master(SpiMaster())
    val probes  = out(Bits(8 bits)) //Used to probe the spiFlash signals
    val leds = out Bits(3 bits)
  }

  val oscInst = osc1()
  val cccInst = ccc1()
  cccInst.RCOSC_25_50MHZ <> oscInst.RCOSC_25_50MHZ_CCC

  val DEVRST_N = in Bool()
  val por = SYSRESET()
  por.DEVRST_N := DEVRST_N

  val soc = Igloo2Perf(p)
  soc.io.clk      <> cccInst.GL0
  soc.io.reset    <> !por.POWER_ON_RESET_N
  soc.io.flash    <> io.flashSpi
  soc.io.leds     <> io.leds
  soc.io.serialTx <> io.serialTx
  soc.io.serialRx <> io.serialRx
  io.probes(3 downto 0) := io.flashSpi.asBits
  io.probes(4) := soc.io.reset
  io.probes(5) := soc.io.clk
  io.probes(6) := cccInst.LOCK
  io.probes(7) := True
}

object Igloo2Perf {
  def main(args: Array[String]): Unit = {
    SpinalRtlConfig().includeSimulation.generateVerilog(Igloo2Perf(Igloo2PerfParameters(
      ioClkFrequency = 114 MHz,
      ioSerialBaudRate = 11400000 //For faster simulation
    )))
  }
}

object Igloo2PerfCreative{
  def main(args: Array[String]) {
    SpinalRtlConfig().generateVerilog(Igloo2PerfCreative(Igloo2PerfParameters(
      ioClkFrequency = 114 MHz,
      ioSerialBaudRate = 115200
    )))
  }
}