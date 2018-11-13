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

object Up5kPerf {
  def main(args: Array[String]): Unit = {
    SpinalRtlConfig().generateVerilog(Up5kPerf(Up5kPerfParameters(
      ioClkFrequency = 12 MHz,
      ioSerialBaudRate = 115200
    )))
  }

  def up5kCustomMul(rs1 : UInt, rs2 : UInt, stage : Stage, vexriscv : VexRiscv) = new Area{
    import MulDivIterativePlugin._
    import stage._
    import Riscv._
    import vexriscv.config._
//    val mul = S(rs1)*S(False ## rs2)
//    val mul = S(rs1)*S(False ## rs2(15 downto 0)) + (S(rs1)*S(False ## rs2(31 downto 16)) << 16)
    val rs2LowS = S(False ## rs2(0, 16 bits))
    val rs2HighS = S(False ## rs2(16, 16 bits))
    val mulA = S(0, 90 bits) +  S(False ## (rs1(0, 16 bits) * rs2(0, 16 bits)))   +  (S(False ## (rs1(16, 16 bits) * rs2(0, 16 bits)))  << 16) + ((S(rs1(32, 1 bits)) * S(False ## rs2( 0, 16 bits))) << (32))
    val mulB = S(0, 90 bits) + (S(False ## (rs1(0, 16 bits) * rs2(16, 16 bits)))) +  (S(False ## (rs1(16, 16 bits) * rs2(16, 16 bits))) << 16) +  (S(rs1(32, 1 bits)) * S(False ## rs2(16, 16 bits))  << (32))

    //    val mulA = S(0, 90 bits) +  S(False ## (rs1(0, 16 bits) * rs2(0, 16 bits)))   + ((S(rs1(16, 17 bits)) * S(False ## rs2( 0, 16 bits))) << (16))
//    val mulB = S(0, 90 bits) + (S(False ## (rs1(0, 16 bits) * rs2(16, 16 bits)))) +  (S(rs1(16, 17 bits)) * S(False ## rs2(16, 16 bits))  << (16))
//    val mulA = S(0, 90 bits) + S(rs1(0, 16 bits)*rs2(0, 16 bits) + U(S(rs1(0, 16 bits))*rs2HighS << (16)))
//    val mulB = S(0, 90 bits) + (S(rs1(16, 17 bits))*rs2LowS << 16) + S(rs1(16, 17 bits))*rs2HighS << (16+16)
    val mul = mulA + (mulB << 16)
    when(arbitration.isValid && input(IS_MUL)){
      output(REGFILE_WRITE_DATA) := ((input(INSTRUCTION)(13 downto 12) === B"00") ? mul(31 downto 0) | mul(63 downto 32)).asBits
    }
  }

  def core() = new VexRiscv(
    config = VexRiscvConfig(
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
//        new DivPlugin,
//        new MulPlugin,
        new MulDivIterativePlugin(
          genMul = true,
          genDiv = true,
          mulUnrollFactor = 4,
          divUnrollFactor = 1//,
//          customMul = up5kCustomMul
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
  )
}




case class Up5kPerfParameters(ioClkFrequency : HertzNumber,
                               ioSerialBaudRate : Int)



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
    val slowBus = SimpleBus(mainBusConfig)
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

    //interconnect.noTransactionLockOn(List(iRam.io.bus, dRam.io.bus))

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
//    interconnect.setConnector(slowBus, iRam.io.bus){(i,b) =>
//      i.cmd.halfPipe() >> b.cmd
//      i.rsp            << b.rsp
//    }
//    interconnect.setConnector(slowBus, dRam.io.bus){(i,b) =>
//      i.cmd.halfPipe() >> b.cmd
//      i.rsp            << b.rsp
//    }

//      interconnect.setConnector(dBus){(i,b) =>
//        i.cmd.halfPipe() >> b.cmd
//        i.rsp            << b.rsp.stage()
//      }
//      interconnect.setConnector(iBus){(i,b) =>
//        i.cmd.halfPipe() >> b.cmd
//        i.rsp            << b.rsp.stage()
//      }


    interconnect.setConnector(dBus){(i,b) =>
      i.cmd.s2mPipe() >> b.cmd
      i.rsp << b.rsp
    }




    //Map the CPU into the SoC
    val cpu = Up5kPerf.core()
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


object Up5kPerfEvn{
  def main(args: Array[String]) {
    SpinalRtlConfig().generateVerilog(Up5kPerfEvn())
  }
}


case class Up5kPerfEvn() extends Component{
  val io = new Bundle {
    val iceClk  = in  Bool() //35

    val serialTx  = out  Bool() //

    val flashSpi  = master(SpiMaster()) //16 15 14 17

    val leds = new Bundle {
      val r,g,b = out Bool() //41 40 39
    }
  }

  //    val mainClkBuffer = SB_GB()
  //    mainClkBuffer.USER_SIGNAL_TO_GLOBAL_BUFFER <> io.iceClk
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