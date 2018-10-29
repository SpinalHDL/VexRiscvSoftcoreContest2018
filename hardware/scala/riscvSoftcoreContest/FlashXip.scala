package riscvSoftcoreContest

import spinal.core._
import spinal.lib.com.spi.SpiMaster
import spinal.lib.fsm.{State, StateMachine}
import spinal.lib._
import vexriscv.demo.SimpleBus

case class FlashXpi(addressWidth : Int, slowDownFactor : Int = 0) extends Component{
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

    val counter = Reg(UInt(7 + slowDownFactor bits))
    counter := counter + 1
    SETUP.onEntry(counter := 0)
    SETUP.whenIsActive{
      io.flash.ss(0) := False
      io.flash.sclk := counter(slowDownFactor)
      val bitstream = B"x8183"
      io.flash.mosi := bitstream.asBools.reverse((counter >> 1+slowDownFactor).resized)
      when(counter === (widthOf(bitstream)*2 << slowDownFactor)-1){
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
      io.flash.sclk := counter(slowDownFactor)
      val bitstream = B"x0B" ## io.bus.cmd.address.resize(24 bits) ## B"x00"
      io.flash.mosi := bitstream.asBools.reverse((counter >> 1+slowDownFactor).resized)
      when(counter === (widthOf(bitstream)*2 << slowDownFactor)-1){
        io.bus.cmd.ready := True
        goto(PAYLOAD)
      }
    }

    PAYLOAD.onEntry(counter := 0)
    PAYLOAD.whenIsActive{
      io.flash.ss(0) := False
      io.flash.sclk := counter(slowDownFactor)
      buffer.fill setWhen(counter(slowDownFactor downto 0).andR)
      when(counter === (32*2 << slowDownFactor)-1){
        goto(IDLE)
      }
    }
  }
}
