package riscvSoftcoreContest

import spinal.core.SpinalConfig

object SpinalRtlConfig {
  def apply() = SpinalConfig(targetDirectory = "hardware/netlist")
}
