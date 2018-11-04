ZEPHYR=ext/zephyr
SHELL=/bin/bash
NETLIST_DEPENDENCIES=$(shell find hardware/scala -type f)
.ONESHELL:

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



software/dhrystone/up5kPerf/build/dhrystone.bin:
	source ${ZEPHYR}/zephyr-env.sh
	make -C software/dhrystone/up5kPerf

software/dhrystone/igloo2Perf/build/dhrystone.bin:
	source ${ZEPHYR}/zephyr-env.sh
	make -C software/dhrystone/igloo2Perf



hardware/netlist/%.v: ${NETLIST_DEPENDENCIES}
	sbt "run-main riscvSoftcoreContest.$(subst hardware/netlist/,,$(subst .v,,$@))"



up5kPerf_sim: hardware/netlist/Up5kPerf.v test/up5kPerf/main.cpp
	make -C test/up5kPerf clean run ${ARGS}

up5kPerf_sim_dhrystone: software/dhrystone/up5kPerf/build/dhrystone.bin software/bootloader hardware/netlist/Up5kPerf.v test/up5kPerf/main.cpp
	make -C test/up5kPerf clean run IRAM_BIN=../../software/dhrystone/up5kPerf/build/dhrystone.bin

up5kPerf_sim_synchronization: ext/zephyr/samples/synchronization/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin software/bootloader hardware/netlist/Up5kPerf.v test/up5kPerf/main.cpp
	make -C test/up5kPerf clean run IRAM_BIN=../../ext/zephyr/samples/synchronization/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin

up5kPerf_sim_philosophers: ext/zephyr/samples/philosophers/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin software/bootloader hardware/netlist/Up5kPerf.v test/up5kPerf/main.cpp
	make -C test/up5kPerf clean run IRAM_BIN=../../ext/zephyr/samples/philosophers/vexriscv_contest_up5kperf_evn/zephyr/zephyr.bin


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





igloo2Perf_sim: hardware/netlist/Igloo2Perf.v test/igloo2Perf/main.cpp
	make -C test/igloo2Perf clean run ${ARGS}

igloo2Perf_sim_dhrystone: software/dhrystone/igloo2Perf/build/dhrystone.bin software/bootloader hardware/netlist/Igloo2Perf.v test/igloo2Perf/main.cpp
	make -C test/igloo2Perf clean run IRAM_BIN=../../software/dhrystone/igloo2Perf/build/dhrystone.bin

igloo2Perf_sim_synchronization: ext/zephyr/samples/synchronization/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin software/bootloader hardware/netlist/Igloo2Perf.v test/igloo2Perf/main.cpp
	make -C test/igloo2Perf clean run IRAM_BIN=../../ext/zephyr/samples/synchronization/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin

igloo2Perf_sim_philosophers: ext/zephyr/samples/philosophers/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin software/bootloader hardware/netlist/Igloo2Perf.v test/igloo2Perf/main.cpp
	make -C test/igloo2Perf clean run IRAM_BIN=../../ext/zephyr/samples/philosophers/vexriscv_contest_igloo2perf_creative/zephyr/zephyr.bin
