package riscvSoftcoreContest

import spinal.core._
import spinal.lib.slave
import vexriscv.demo.SimpleBus

object SpinalRtlConfig {
  def apply() = SpinalConfig(targetDirectory = "hardware/netlist")
}


case class SimpleBusRam(onChipRamSize : BigInt, relaxedCmd : Boolean = false, relaxedRsp : Boolean = false) extends Component{
  val io = new Bundle{
    val bus = slave(SimpleBus(log2Up(onChipRamSize), 32))
  }
  io.bus.cmd.ready := True

  val ram = Mem(Bits(32 bits), onChipRamSize / 4).addTag(Verilator.public)
  io.bus.rsp.valid := RegNext(io.bus.cmd.fire && !io.bus.cmd.wr) init(False)
  val readLogic = (relaxedCmd, relaxedRsp) match {
    case (false, false) => new Area {
      io.bus.rsp.data := ram.readWriteSync(
        address = (io.bus.cmd.address >> 2).resized,
        data = io.bus.cmd.data,
        enable = io.bus.cmd.valid,
        write = io.bus.cmd.wr,
        mask = io.bus.cmd.mask
      )
    }
    case (true, false) => new Area {
      val cmd = RegNext(io.bus.cmd.asFlow)
      ClockDomain.current.withRevertedClockEdge() {
        io.bus.rsp.data := ram.readWriteSync(
          address = (cmd.address >> 2).resized,
          data = cmd.data,
          enable = cmd.valid,
          write = cmd.wr,
          mask = cmd.mask
        )
      }
    }
    case (false, true) => new Area {
      val rsp = ClockDomain.current.withRevertedClockEdge() (
        ram.readWriteSync(
          address = (io.bus.cmd.address >> 2).resized,
          data = io.bus.cmd.data,
          enable = io.bus.cmd.valid,
          write = io.bus.cmd.wr,
          mask = io.bus.cmd.mask
        )
      )
      io.bus.rsp.data := RegNext(rsp)
    }
  }
}




case class SimpleBusMultiPortRam(onChipRamSize : BigInt, portCount : Int) extends Component{
  val io = new Bundle{
    val buses = Vec(slave(SimpleBus(log2Up(onChipRamSize), 32)), portCount)
  }
  val ram = Mem(Bits(32 bits), onChipRamSize / 4).addTag(Verilator.public)
  
  for(bus <- io.buses) {
    bus.cmd.ready := True
    bus.rsp.valid := RegNext(bus.cmd.fire && !bus.cmd.wr) init (False)
    bus.rsp.data := ram.readWriteSync(
      address = (bus.cmd.address >> 2).resized,
      data = bus.cmd.data,
      enable = bus.cmd.valid,
      write = bus.cmd.wr,
      mask = bus.cmd.mask
    )
  }
}
