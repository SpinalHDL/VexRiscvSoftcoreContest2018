set_component ccc1_ccc1_0_FCCC
# Microsemi Corp.
# Date: 2018-Nov-19 15:05:17
#

create_clock -period 20 [ get_pins { CCC_INST/RCOSC_25_50MHZ } ]
create_generated_clock -multiply_by 57 -divide_by 25 -source [ get_pins { CCC_INST/RCOSC_25_50MHZ } ] -phase 0 [ get_pins { CCC_INST/GL0 } ]
