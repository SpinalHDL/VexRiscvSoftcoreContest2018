package riscvSoftcoreContest

import spinal.core._
import spinal.lib._
import vexriscv.demo.{SimpleBus, SimpleBusSlaveFactory}

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
    mapper.read(counter, 0x10)
    mapper.write(cmp, 0x18)
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
