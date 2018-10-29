package riscvSoftcoreContest

import spinal.core._
import spinal.lib._

case class SerialRxOutput(serialBaudRate : Int, outputInit : Int) extends Component{
  val io = new Bundle {
    val serialRx = in Bool()
    val output = out Bits (8 bits)
  }

  val timerBaud = ClockDomain.current.frequency.getValue.toInt / serialBaudRate
  val timerBaudPlusHalf = (timerBaud*1.5).toInt
  val timer = Reg(UInt(log2Up(timerBaudPlusHalf) bits))
  val timerTick = timer === 0
  timer := timer -1

  val state = RegInit(U"00")
  val bitCounter = Reg(UInt(3 bits))

  val serialRx = BufferCC(io.serialRx)
  val outputReg = Reg(Bits(8 bits))  init(outputInit)
  io.output := outputReg

  switch(state) {
    is(0) {
      when(!serialRx){
        timer := timerBaudPlusHalf-1
        state := 1
      }
    }
    is(1) {
      when(timerTick) {
        outputReg(bitCounter) := serialRx
        timer := timerBaud - 1
        bitCounter := bitCounter + 1
        when(bitCounter === 7) {
          state := 2
        }
      }
    }
    is(2) {
      when(timerTick) {
        state := 0
      }
    }
  }
}
