ZEPHYR=ext/zephyr
SHELL=/bin/bash
NETLIST_DEPENDENCIES=$(shell find hardware/scala -type f)
.ONESHELL:
ROOT=$(shell pwd)

##########################################
# Netlist generation commande (SpinalHDL)
##########################################

# Simulation netlist
igloo2Perf.v : 
	rm -rf hardware/netlist/Igloo2Perf.v
	make hardware/netlist/Igloo2Perf.v ARGS="${ARGS}"

up5kPerf.v :  
	rm -rf hardware/netlist/Up5kPerf.v
	make hardware/netlist/Up5kPerf.v ARGS="${ARGS}"

up5kArea.v :  
	rm -rf hardware/netlist/Up5kArea.v
	make hardware/netlist/Up5kArea.v ARGS="${ARGS}"

# Synthesis netlist
igloo2PerfCreative.v :  
	rm -rf hardware/netlist/Igloo2PerfCreative.v
	make hardware/netlist/Igloo2PerfCreative.v ARGS="${ARGS}"

up5kPerfEvn.v :  
	rm -rf hardware/netlist/Up5kPerfEvn.v
	make hardware/netlist/Up5kPerfEvn.v ARGS="${ARGS}"

up5kAreaEvn.v :  
	rm -rf hardware/netlist/Up5kAreaEvn.v
	make hardware/netlist/Up5kAreaEvn.v ARGS="${ARGS}"



####################################
# up5kPerf simulation commands
####################################

up5kPerf_sim_compliance_rv32i:  software/bootloader
	make -C ext/riscv-compliance variant RISCV_TARGET=vexriscv_contest RISCV_DEVICE=up5kPerf RISCV_ISA=rv32i TRACE=${TRACE}

up5kPerf_sim_compliance_rv32im: software/bootloader
	make -C ext/riscv-compliance variant RISCV_TARGET=vexriscv_contest RISCV_DEVICE=up5kPerf RISCV_ISA=rv32im

up5kPerf_sim_dhrystone: software/dhrystone/up5kPerf/build/dhrystone.bin software/bootloader
	make -C test/up5kPerf run ARGS='--iramBin ${ROOT}/software/dhrystone/up5kPerf/build/dhrystone.bin --bootloader ${ROOT}/software/bootloader/up5kPerf/noFlash.bin'

up5kPerf_sim_synchronization: ext/zephyr/samples/synchronization/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin software/bootloader
	make -C test/up5kPerf run ARGS='--iramBin ${ROOT}/ext/zephyr/samples/synchronization/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin --bootloader ${ROOT}/software/bootloader/up5kPerf/noFlash.bin'

up5kPerf_sim_philosophers: ext/zephyr/samples/philosophers/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin software/bootloader
	make -C test/up5kPerf run ARGS='--iramBin ${ROOT}/ext/zephyr/samples/philosophers/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin --bootloader ${ROOT}/software/bootloader/up5kPerf/noFlash.bin'




##############################################
# up5kPerf evn board programmation commands
##############################################

up5kPerf_evn_prog_icecube2:
	iceprog -o 0x00000 hardware/synthesis/up5kPerfEvn/icecube2/icecube2_Implmnt/sbt/outputs/bitmap/Up5kPerfEvn_bitmap.bin

up5kPerf_evn_prog_bootloader: software/bootloader
	iceprog -o 0x20000 software/bootloader/up5kPerf/copyFlash.bin

up5kPerf_evn_prog_dhrystone: software/dhrystone/up5kPerf/build/dhrystone.bin
	iceprog -o 0x30000 software/dhrystone/up5kPerf/build/dhrystone.bin

up5kPerf_evn_prog_all_dhrystone: up5kPerf_evn_prog_icecube2 up5kPerf_evn_prog_bootloader up5kPerf_evn_prog_dhrystone

up5kPerf_evn_prog_syncronization: ext/zephyr/samples/synchronization/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin
	iceprog -o 0x30000 ext/zephyr/samples/synchronization/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin

up5kPerf_evn_prog_philosophers: ext/zephyr/samples/philosophers/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin
	iceprog -o 0x30000 ext/zephyr/samples/philosophers/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin




##############################################
# igloo2Perf simulation commands
##############################################

igloo2Perf_sim_compliance_rv32i: software/bootloader
	make -C ext/riscv-compliance variant RISCV_TARGET=vexriscv_contest RISCV_DEVICE=igloo2Perf RISCV_ISA=rv32i

igloo2Perf_sim_compliance_rv32im: software/bootloader
	make -C ext/riscv-compliance variant RISCV_TARGET=vexriscv_contest RISCV_DEVICE=igloo2Perf RISCV_ISA=rv32im

igloo2Perf_sim_dhrystone: software/dhrystone/igloo2Perf/build/dhrystone.bin software/bootloader
	make -C test/igloo2Perf run ARGS='--iramBin ${ROOT}/software/dhrystone/igloo2Perf/build/dhrystone.bin --bootloader ${ROOT}/software/bootloader/igloo2Perf/noFlash.bin'

igloo2Perf_sim_synchronization: ext/zephyr/samples/synchronization/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin software/bootloader
	make -C test/igloo2Perf run ARGS='--iramBin ${ROOT}/ext/zephyr/samples/synchronization/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin --bootloader ${ROOT}/software/bootloader/igloo2Perf/noFlash.bin'

igloo2Perf_sim_philosophers: ext/zephyr/samples/philosophers/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin software/bootloader
	make -C test/igloo2Perf run ARGS='--iramBin ${ROOT}/ext/zephyr/samples/philosophers/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin --bootloader ${ROOT}/software/bootloader/igloo2Perf/noFlash.bin'

igloo2Perf_sim_dhrystone_with_preloaded_flash: software/dhrystone/igloo2Perf/build/dhrystone.bin software/bootloader
	make -C test/igloo2Perf run ARGS='--flashBin ${ROOT}/software/dhrystone/igloo2Perf/build/dhrystone.bin --bootloader ${ROOT}/software/bootloader/igloo2Perf/copyFlash.bin'

igloo2Perf_sim_dhrystone_without_preloaded_flash: software/dhrystone/igloo2Perf/build/dhrystone.bin software/bootloader
	rm -rf ${ROOT}/tmp
	mkdir tmp
	python scripts/binToFlash.py software/bootloader/igloo2Perf/copyFlash.bin 0x20000 11400000 ${ROOT}/tmp/bootloader.bin
	python scripts/binToFlash.py software/dhrystone/igloo2Perf/build/dhrystone.bin 0x30000 11400000 ${ROOT}/tmp/app.bin
	cat ${ROOT}/tmp/bootloader.bin >> ${ROOT}/tmp/serial.bin
	cat ${ROOT}/tmp/app.bin >> ${ROOT}/tmp/serial.bin
	make -C test/igloo2Perf run ARGS='--serialLoad ${ROOT}/tmp/serial.bin'


######################################################################################
# igloo2Perf creative board commands to generate the programmation files
######################################################################################

igloo2Perf_creative_serial_bootloader: software/bootloader
	python scripts/binToFlash.py software/bootloader/igloo2Perf/copyFlash.bin 0x20000 115000 igloo2Perf_creative_serial_bootloader.bin

igloo2Perf_creative_serial_dhrystone: software/dhrystone/igloo2Perf/build/dhrystone.bin
	python scripts/binToFlash.py software/dhrystone/igloo2Perf/build/dhrystone.bin 0x30000 115000 igloo2Perf_creative_serial_dhrystone.bin

igloo2Perf_creative_serial_synchronization: ext/zephyr/samples/synchronization/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin
	python scripts/binToFlash.py ext/zephyr/samples/synchronization/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin 0x30000 115000 igloo2Perf_creative_serial_synchronization.bin

igloo2Perf_creative_serial_philosophers: ext/zephyr/samples/philosophers/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin
	python scripts/binToFlash.py ext/zephyr/samples/philosophers/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin 0x30000 115000 igloo2Perf_creative_serial_philosophers.bin



##############################################
# up5kArea simulation commands
##############################################

up5kArea_sim_compliance_rv32i: software/bootloader
	make -C ext/riscv-compliance variant RISCV_TARGET=vexriscv_contest RISCV_DEVICE=up5kArea RISCV_ISA=rv32i

up5kArea_sim_dhrystone: software/dhrystone/up5kArea/build/dhrystone.bin software/bootloader
	make -C test/up5kArea run ARGS='--iramBin ${ROOT}/software/dhrystone/up5kArea/build/dhrystone.bin --bootloader ${ROOT}/software/bootloader/up5kArea/noFlash.bin'

up5kArea_sim_synchronization: ext/zephyr/samples/synchronization/vexriscv_contest_up5karea_evn/zephyr/zephyr.bin software/bootloader
	make -C test/up5kArea run ARGS='--iramBin ${ROOT}/ext/zephyr/samples/synchronization/vexriscv_contest_up5karea_evn/zephyr/zephyr.bin --bootloader ${ROOT}/software/bootloader/up5kArea/noFlash.bin'

up5kArea_sim_philosophers: ext/zephyr/samples/philosophers/vexriscv_contest_up5karea_evn/zephyr/zephyr.bin software/bootloader
	make -C test/up5kArea run ARGS='--iramBin ${ROOT}/ext/zephyr/samples/philosophers/vexriscv_contest_up5karea_evn/zephyr/zephyr.bin --bootloader ${ROOT}/software/bootloader/up5kArea/noFlash.bin'



##############################################
# up5kArea evn board programmation commands
##############################################

up5kArea_evn_prog_icecube2:
	iceprog -o 0x00000 hardware/synthesis/up5kAreaEvn/icecube2/icecube2_Implmnt/sbt/outputs/bitmap/Up5kAreaEvn_bitmap.bin

up5kArea_evn_prog_bootloader: software/bootloader
	iceprog -o 0x20000 software/bootloader/up5kArea/copyFlash.bin

up5kArea_evn_prog_dhrystone: software/dhrystone/up5kArea/build/dhrystone.bin
	iceprog -o 0x30000 software/dhrystone/up5kArea/build/dhrystone.bin

up5kArea_evn_prog_all_dhrystone: up5kArea_evn_prog_icecube2 up5kArea_evn_prog_bootloader up5kArea_evn_prog_dhrystone

up5kArea_evn_prog_syncronization: ext/zephyr/samples/synchronization/vexriscv_contest_up5karea_evn/zephyr/zephyr.bin
	iceprog -o 0x30000 ext/zephyr/samples/synchronization/vexriscv_contest_up5karea_evn/zephyr/zephyr.bin

up5kArea_evn_prog_philosophers: ext/zephyr/samples/philosophers/vexriscv_contest_up5karea_evn/zephyr/zephyr.bin
	iceprog -o 0x30000 ext/zephyr/samples/philosophers/vexriscv_contest_up5karea_evn/zephyr/zephyr.bin




##############################################
# Internals
##############################################

clean:
	rm -rf ext/zephyr/samples/synchronization/vexriscv_*
	rm -rf ext/zephyr/samples/philosophers/vexriscv_*
	make -C software/bootloader/up5kPerf clean
	make -C software/bootloader/up5kArea clean
	make -C software/bootloader/igloo2Perf clean
	make -C software/dhrystone/igloo2Perf clean
	make -C software/dhrystone/up5kPerf clean
	make -C software/dhrystone/up5kArea clean
	make -C test/up5kPerf clean
	make -C test/up5kArea clean
	make -C test/igloo2Perf clean

.PHONY: software/bootloader
software/bootloader:
	source ${ZEPHYR}/zephyr-env.sh
	make -C software/bootloader/up5kPerf all
	make -C software/bootloader/up5kArea all
	make -C software/bootloader/igloo2Perf all


${ZEPHYR}/samples/%/zephyr/zephyr.bin:
	cd ${ZEPHYR}
	source zephyr-env.sh
	rm -rf samples/$(*)
	mkdir samples/$(*)
	cd samples/$(*)
	cmake -DBOARD=$(lastword $(subst /, ,$(*))) ..
	make -j


.PHONY: software/dhrystone/up5kPerf/build/dhrystone.bin
software/dhrystone/up5kPerf/build/dhrystone.bin:
	source ${ZEPHYR}/zephyr-env.sh
	make -C software/dhrystone/up5kPerf

.PHONY: software/dhrystone/igloo2Perf/build/dhrystone.bin
software/dhrystone/igloo2Perf/build/dhrystone.bin:
	source ${ZEPHYR}/zephyr-env.sh
	make -C software/dhrystone/igloo2Perf


.PHONY: software/dhrystone/up5kArea/build/dhrystone.bin
software/dhrystone/up5kArea/build/dhrystone.bin:
	source ${ZEPHYR}/zephyr-env.sh
	make -C software/dhrystone/up5kArea



hardware/netlist/%.v: ${NETLIST_DEPENDENCIES}
	sbt "run-main riscvSoftcoreContest.$(subst hardware/netlist/,,$(subst .v,,$@)) ${ARGS}"

