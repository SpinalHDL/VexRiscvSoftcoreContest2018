ZEPHYR=ext/zephyr
SHELL=/bin/bash
NETLIST_DEPENDENCIES=$(shell find hardware/scala -type f)
.ONESHELL:

clean:
	rm -rf ext/zephyr/samples/synchronization/build_*
	rm -rf ext/zephyr/samples/philosophers/build_*


${ZEPHYR}/samples/%/zephyr/zephyr.bin:
	cd ${ZEPHYR};
	source zephyr-env.sh
	rm -rf $(subst ${ZEPHYR}/,,$(subst /zephyr/zephyr.bin,,$@))
	mkdir $(subst ${ZEPHYR}/,,$(subst /zephyr/zephyr.bin,,$@))
	cd $(subst ${ZEPHYR}/,,$(subst /zephyr/zephyr.bin,,$@))
	cmake -DBOARD=$(lastword $(subst /, ,$(subst /zephyr/zephyr.bin,,$@))) ..
	make -j


hardware/netlist/%.v: ${NETLIST_DEPENDENCIES}
	sbt "run-main riscvSoftcoreContest.$(subst hardware/netlist/,,$(subst .v,,$@))"

simUp5kSpeed: hardware/netlist/Up5kSpeed.v test/up5kSpeed/main.cpp
	make -C test/up5kSpeed clean run ${ARGS}

simUp5kSpeedDhrystone: hardware/netlist/Up5kSpeed.v test/up5kSpeed/main.cpp
	make -C test/up5kSpeed clean run IRAM_BIN=../../software/dhrystone/build/dhrystone.bin