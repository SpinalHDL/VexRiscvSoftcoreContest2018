# Microsemi Corp.
# Date: 2018-Nov-22 01:09:18
# This file was generated based on the following SDC source files:
#   D:/pro/hdl/riscvSoftcoreContest/hardware/synthesis/igloo2PerfCreative/libero/constraint/final.sdc
#

create_clock -name {oscToCcc} -period 20 -waveform {0 10 } [ get_pins { oscInst/osc1_0/I_RCOSC_25_50MHZ/CLKOUT } ]
create_generated_clock -name {cccGL0} -multiply_by 57 -divide_by 25 -source [ get_pins { cccInst/ccc1_0/CCC_INST/RCOSC_25_50MHZ } ] -phase 0 [ get_pins { cccInst/ccc1_0/CCC_INST/GL0 } ]
