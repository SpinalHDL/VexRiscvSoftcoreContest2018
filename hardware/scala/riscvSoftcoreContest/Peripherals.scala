package riscvSoftcoreContest

import spinal.core._
import spinal.lib._
import vexriscv.demo.{SimpleBus, SimpleBusSlaveFactory}

case class Peripherals(serialBaudRate : Int, smallTimer : Boolean = false) extends Component{
  val io = new Bundle{
    val bus = slave(SimpleBus(6, 32))
    val mTimeInterrupt = out Bool()
    val leds = out Bits(3 bits)
    val serialTx = out Bool()
  }

  val mapper = new SimpleBusSlaveFactory(io.bus)
  mapper.driveAndRead(io.leds, 0x4, 0) init(0)

  val serialTx = new Area{
    val counter = Counter(12)
    val buffer = Reg(Bits(8 bits))
    val bitstream = buffer ## "0111"
    val busy = counter =/= 0

    io.serialTx := RegNext(bitstream(counter)) init(True)
    val timer = CounterFreeRun(ClockDomain.current.frequency.getValue.toInt/serialBaudRate)
    when(counter =/= 0 && timer.willOverflowIfInc){
      counter.increment()
    }

    mapper.write(buffer, 0x0, 0)
    when(mapper.isWriting(0x0)) { counter.increment() }
    mapper.read(busy, 0x0, 0)
  }

  val timer = if(!smallTimer) new Area {
    val counter = Reg(UInt(32 bits)) init(0)
    val cmp = Reg(UInt(32 bits)) init(0)
    val interrupt = RegInit(False) setWhen(!(counter - cmp).msb) clearWhen(mapper.isWriting(0x18))
    counter := counter + 1
    io.mTimeInterrupt := interrupt
    mapper.read(counter, 0x10)
    mapper.write(cmp, 0x18)
  } else new Area {
    val width = Math.max(20, log2Up(ClockDomain.current.frequency.getValue.toInt/100)) //downto to 100 hz
    val counter, cmp = Reg(UInt(width bits))
    val hit = counter === cmp
    val interrupt = RegInit(False) setWhen(hit) clearWhen(mapper.isWriting(0x10))
    counter := counter + 1
    when(hit || mapper.isWriting(0x18)){
      counter := 0
    }
    io.mTimeInterrupt := interrupt
    mapper.read(counter, 0x10)
    mapper.write(cmp, 0x18)

//    val counter = CounterFreeRun(ClockDomain.current.frequency.getValue.toInt/100)
//    val interrupt = RegInit(False) setWhen(counter.willOverflowIfInc) clearWhen(mapper.isWriting(0x10))
//    io.mTimeInterrupt := interrupt
//    mapper.read(counter.value, 0x10)

//    val counter = Reg(UInt(16 bits))
//    val cmp = Reg(UInt(16 bits))
//    val hit = counter === cmp
//    val interrupt = RegInit(False) setWhen(hit) clearWhen(mapper.isWriting(0x10))
//    counter := counter + U(serialTx.timer.willOverflowIfInc)
//    when(hit || mapper.isWriting(0x18)){
//      counter := 0
//    }
//    io.mTimeInterrupt := interrupt
//    mapper.read(counter, 0x10)
//    mapper.write(cmp, 0x18)
  }
}
