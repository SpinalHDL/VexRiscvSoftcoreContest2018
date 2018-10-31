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

object Igloo2Speed {
  def main(args: Array[String]): Unit = {
    SpinalRtlConfig().includeSimulation.generateVerilog(Igloo2Speed(Igloo2SpeedParameters(
      ioClkFrequency = 25 MHz,
      ioSerialBaudRate = 115200
    )))
  }

  def core() = new VexRiscv(
    config = VexRiscvConfig(
      List(
        new IBusSimplePlugin(
          resetVector = 0x00020000l,
          cmdForkOnSecondStage = false,
          cmdForkPersistence = true,
          prediction = NONE,
          catchAccessFault = false,
          compressedGen = false,
          injectorStage = true,
          rspHoldValue = false,
          historyRamSizeLog2 = 10
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
        new MulPlugin,
        new MulDivIterativePlugin(
          genMul = false,
          genDiv = true,
          divUnrollFactor = 1
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
          catchAddressMisaligned = true
        )
      )
    )
  )
}



case class SimpleBusRam(onChipRamSize : BigInt) extends Component{
  val io = new Bundle{
    val bus = slave(SimpleBus(log2Up(onChipRamSize), 32))
  }

  val ram = Mem(Bits(32 bits), onChipRamSize / 4).addTag(Verilator.public)
  io.bus.rsp.valid := RegNext(io.bus.cmd.fire && !io.bus.cmd.wr) init(False)
  io.bus.rsp.data := ram.readWriteSync(
    address = (io.bus.cmd.address >> 2).resized,
    data  = io.bus.cmd.data,
    enable  = io.bus.cmd.valid,
    write  = io.bus.cmd.wr,
    mask  = io.bus.cmd.mask
  )
  io.bus.cmd.ready := True
}




case class Igloo2SpeedParameters(ioClkFrequency : HertzNumber,
                                 ioSerialBaudRate : Int)



case class Igloo2Speed(p : Igloo2SpeedParameters) extends Component {
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
    val mainClkResetUnbuffered  = False

    //Implement an counter to keep the reset mainClkResetUnbuffered high 64 cycles
    // Also this counter will automatically do a reset when the system boot.
    val bootTime = if(GenerationFlags.simulation.isEnabled) 1 ms else 100 ms
    val systemClkResetCounter = Reg(UInt(log2Up((p.ioClkFrequency*bootTime).toBigInt()) bits)) init(0)
    when(systemClkResetCounter =/= U(systemClkResetCounter.range -> true)){
      systemClkResetCounter := systemClkResetCounter + 1
      mainClkResetUnbuffered := True
    }
    when(BufferCC(io.reset)){
      systemClkResetCounter := 0
    }

    //Create all reset used later in the design
    val systemResetBuffered  = RegNext(mainClkResetUnbuffered)
    val systemReset = CombInit(systemResetBuffered)

    val progResetBuffered  = RegNext(mainClkResetUnbuffered)
    val progReset = CombInit(progResetBuffered)
  }


  val progClockDomain = ClockDomain(
    clock = io.clk,
    reset = resetCtrl.progReset,
    frequency = FixedFrequency(p.ioClkFrequency)
  )

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
    val slowBus = SimpleBus(mainBusConfig)
    val interconnect = SimpleBusInterconnect()

    val iRam = SimpleBusRam(20 kB)
    val dRam = SimpleBusRam(16 kB)

    val peripherals = Peripherals(serialBaudRate = p.ioSerialBaudRate)
    peripherals.io.serialTx <> io.serialTx
    peripherals.io.leds <> io.leds


    val flashXip = FlashXpi(addressWidth = 20, slowDownFactor = 3)
    interconnect.addSlaves(
      iRam.io.bus         -> SizeMapping(0x80000,  32 kB),
      dRam.io.bus         -> SizeMapping(0x90000,  16 kB),
      peripherals.io.bus  -> SizeMapping(0xF0000, 256 Byte),
      flashXip.io.bus     -> SizeMapping(0x00000, 512 kB),
      slowBus             -> DefaultMapping
    )
    interconnect.addMasters(
      dBus   -> List(             dRam.io.bus, slowBus),
      iBus   -> List(iRam.io.bus,              slowBus),
      slowBus-> List(iRam.io.bus, dRam.io.bus,           peripherals.io.bus, flashXip.io.bus)
    )

    interconnect.setConnector(dBus, slowBus){(i,b) =>
      i.cmd.halfPipe() >> b.cmd
      i.rsp            << b.rsp
    }
    interconnect.setConnector(iBus, slowBus){(i,b) =>
      i.cmd.halfPipe() >> b.cmd
      i.rsp            << b.rsp
    }
    interconnect.setConnector(slowBus){(i,b) =>
      i.cmd >> b.cmd
      i.rsp << b.rsp.stage()
    }
    interconnect.setConnector(slowBus, iRam.io.bus){(i,b) =>
      i.cmd.halfPipe() >> b.cmd
      i.rsp            << b.rsp
    }
    interconnect.setConnector(slowBus, dRam.io.bus){(i,b) =>
      i.cmd.halfPipe() >> b.cmd
      i.rsp            << b.rsp
    }


//    interconnect.setConnector(dBus){(i,b) =>
//      i.cmd.halfPipe() >> b.cmd
//      i.rsp << b.rsp
//    }



    //Map the CPU into the SoC
    val cpu = Igloo2Speed.core()
    for (plugin <- cpu.plugins) plugin match {
      case plugin: IBusSimplePlugin =>
        val cmd = plugin.iBus.cmd //TODO improve
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
      case plugin: DBusSimplePlugin => {
        val cmd = plugin.dBus.cmd.s2mPipe() //TODO improve
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

  val prog = new ClockingArea(progClockDomain){
    val ctrl = SerialRxOutput(115200,0x07)
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



object Igloo2SpeedEvaluationBoard{
  case class Igloo2SpeedEvaluationBoard() extends Component{
    val io = new Bundle {
      val serialTx  = out  Bool()
      val serialRx  = in  Bool()

//      val button = in Bool()

      val flashSpi  = master(SpiMaster())
      val probes  = out(Bits(8 bits))

      val leds = out Bits(3 bits)
    }

    val oscInst = osc1()
    val cccInst = ccc1()
    cccInst.RCOSC_25_50MHZ <> oscInst.RCOSC_25_50MHZ_CCC

    val DEVRST_N = in Bool()
    val por = SYSRESET()
    por.DEVRST_N := DEVRST_N

    val soc = Igloo2Speed(Igloo2SpeedParameters(
      ioClkFrequency = 25 MHz,
      ioSerialBaudRate = 115200
    ))
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

  def main(args: Array[String]) {
    SpinalRtlConfig().generateVerilog(Igloo2SpeedEvaluationBoard())
  }
}
