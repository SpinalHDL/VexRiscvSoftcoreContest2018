ZEPHYR=ext/zephyr
SHELL=/bin/bash
NETLIST_DEPENDENCIES=$(shell find hardware/scala -type f)
.ONESHELL:

clean:
	rm -rf ext/zephyr/samples/synchronization/vexriscv_*
	rm -rf ext/zephyr/samples/philosophers/vexriscv_*
	make -C software/bootloader/up5k clean
	make -C software/bootloader/igloo2 clean



software/bootloader/%.bin:
	source ${ZEPHYR}/zephyr-env.sh
	make -C software/bootloader/$(*D) $(*F).bin

.PHONY: bootloader
bootloader:
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



.PHONY: software/dhrystone/build/dhrystone.bin
software/dhrystone/build/dhrystone.bin:
	source ${ZEPHYR}/zephyr-env.sh
	make -C software/dhrystone



hardware/netlist/%.v: ${NETLIST_DEPENDENCIES}
	sbt "run-main riscvSoftcoreContest.$(subst hardware/netlist/,,$(subst .v,,$@))"



simUp5kSpeed: hardware/netlist/Up5kSpeed.v test/up5kSpeed/main.cpp
	make -C test/up5kSpeed clean run ${ARGS}

simUp5kSpeedDhrystone: software/dhrystone/build/dhrystone.bin bootloader hardware/netlist/Up5kSpeed.v test/up5kSpeed/main.cpp
	make -C test/up5kSpeed clean run IRAM_BIN=../../software/dhrystone/build/dhrystone.bin

simUp5kSpeedSynchronization: ext/zephyr/samples/synchronization/vexriscv_contest_fast_up5kev/zephyr/zephyr.bin bootloader hardware/netlist/Up5kSpeed.v test/up5kSpeed/main.cpp
	make -C test/up5kSpeed clean run IRAM_BIN=../../ext/zephyr/samples/synchronization/vexriscv_contest_fast_up5kev/zephyr/zephyr.bin

