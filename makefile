ZEPHYR=ext/zephyr
SHELL=/bin/bash
NETLIST_DEPENDENCIES=$(shell find hardware/scala -type f)
.ONESHELL:
ROOT=$(shell pwd)

clean:
	rm -rf ext/zephyr/samples/synchronization/vexriscv_*
	rm -rf ext/zephyr/samples/philosophers/vexriscv_*
	make -C software/bootloader/up5k clean
	make -C software/bootloader/igloo2 clean


.PHONY: software/bootloader
software/bootloader:
	source ${ZEPHYR}/zephyr-env.sh
	make -C software/bootloader/up5k all
	make -C software/bootloader/igloo2 all

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



hardware/netlist/%.v: ${NETLIST_DEPENDENCIES}
	sbt "run-main riscvSoftcoreContest.$(subst hardware/netlist/,,$(subst .v,,$@))"


up5kPerf_sim_compliance_rv32i: software/dhrystone/up5kPerf/build/dhrystone.bin software/bootloader
	make -C ext/riscv-compliance variant RISCV_TARGET=vexriscv_contest RISCV_DEVICE=up5kPerf RISCV_ISA=rv32i

up5kPerf_sim_compliance_rv32im: software/dhrystone/up5kPerf/build/dhrystone.bin software/bootloader
	make -C ext/riscv-compliance variant RISCV_TARGET=vexriscv_contest RISCV_DEVICE=up5kPerf RISCV_ISA=rv32im

up5kPerf_sim_dhrystone: software/dhrystone/up5kPerf/build/dhrystone.bin software/bootloader
	make -C test/up5kPerf run ARGS='--iramBin ${ROOT}/software/dhrystone/up5kPerf/build/dhrystone.bin --bootloader ${ROOT}/software/bootloader/up5k/noFlash.bin'

up5kPerf_sim_synchronization: ext/zephyr/samples/synchronization/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin software/bootloader
	make -C test/up5kPerf run ARGS='--iramBin ${ROOT}/ext/zephyr/samples/synchronization/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin --bootloader ${ROOT}/software/bootloader/up5k/noFlash.bin'

up5kPerf_sim_philosophers: ext/zephyr/samples/philosophers/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin software/bootloader
	make -C test/up5kPerf run ARGS='--iramBin ${ROOT}/ext/zephyr/samples/philosophers/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin --bootloader ${ROOT}/software/bootloader/up5k/noFlash.bin'


up5kPerf_evn_prog_icecube2:
	iceprog -o 0x00000 hardware/synthesis/up5kDmipsEvaluationBoard/icecube2/icecube2_Implmnt/sbt/outputs/bitmap/Up5kPerfEvaluationBoard_bitmap.bin

up5kPerf_evn_prog_bootloader:
	iceprog -o 0x20000 software/bootloader/up5k/copyFlash.bin

up5kPerf_evn_prog_dhrystone: software/dhrystone/up5kPerf/build/dhrystone.bin
	iceprog -o 0x30000 software/dhrystone/up5kPerf/build/dhrystone.bin

up5kPerf_evn_prog_all_dhrystone: up5kPerf_evn_prog_icecube2 up5kPerf_evn_prog_bootloader up5kPerf_evn_prog_dhrystone

up5kPerf_evn_prog_syncronization: ext/zephyr/samples/synchronization/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin
	iceprog -o 0x30000 ext/zephyr/samples/synchronization/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin

up5kPerf_evn_prog_philosophers: ext/zephyr/samples/philosophers/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin
	iceprog -o 0x30000 ext/zephyr/samples/philosophers/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin




igloo2Perf_sim_compliance_rv32i: software/dhrystone/igloo2Perf/build/dhrystone.bin software/bootloader
	make -C ext/riscv-compliance variant RISCV_TARGET=vexriscv_contest RISCV_DEVICE=igloo2Perf RISCV_ISA=rv32i

igloo2Perf_sim_compliance_rv32im: software/dhrystone/igloo2Perf/build/dhrystone.bin software/bootloader
	make -C ext/riscv-compliance variant RISCV_TARGET=vexriscv_contest RISCV_DEVICE=igloo2Perf RISCV_ISA=rv32im

igloo2Perf_sim_dhrystone: software/dhrystone/igloo2Perf/build/dhrystone.bin software/bootloader
	make -C test/igloo2Perf run ARGS='--iramBin ${ROOT}/software/dhrystone/igloo2Perf/build/dhrystone.bin --bootloader ${ROOT}/software/bootloader/igloo2/noFlash.bin'

igloo2Perf_sim_synchronization: ext/zephyr/samples/synchronization/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin software/bootloader
	make -C test/igloo2Perf run ARGS='--iramBin ${ROOT}/ext/zephyr/samples/synchronization/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin --bootloader ${ROOT}/software/bootloader/igloo2/noFlash.bin'

igloo2Perf_sim_philosophers: ext/zephyr/samples/philosophers/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin software/bootloader
	make -C test/igloo2Perf run ARGS='--iramBin ${ROOT}/ext/zephyr/samples/philosophers/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin --bootloader ${ROOT}/software/bootloader/igloo2/noFlash.bin'
