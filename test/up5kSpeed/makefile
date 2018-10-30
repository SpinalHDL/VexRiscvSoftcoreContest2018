TOPLEVEL=Up5kSpeed
ROOT=  ../..
VERILOGS=  ../common/up5k_cells_sim.v ${ROOT}/hardware/netlist/${TOPLEVEL}.v

TRACE ?= no
DEBUG ?= no
IRAM_BIN ?= no
FLASH_BIN ?= no

ADDCFLAGS += -CFLAGS -I../../common


ifeq ($(FLASH_BIN),no)
	FLASH_BOOTLOADER ?= ${ROOT}/software/bootloader/up5k/noFlash.bin
else
	FLASH_BOOTLOADER ?= ${ROOT}/software/bootloader/up5k/copyFlash.bin
endif


ifeq ($(TRACE),yes)
	VERILATOR_ARGS += --trace
	ADDCFLAGS += -CFLAGS -DTRACE
endif


ifeq ($(DEBUG),yes)
	ADDCFLAGS += -CFLAGS -O0 -CFLAGS -g
else
	ADDCFLAGS += -CFLAGS -O3
endif


ifneq ($(IRAM_BIN),no)
	ADDCFLAGS += -CFLAGS -DIRAM_BIN='\"$(IRAM_BIN)\"'
endif

ADDCFLAGS += -CFLAGS -DFLASH_BOOTLOADER='\"$(FLASH_BOOTLOADER)\"'
ifneq ($(FLASH_BIN),no)
	ADDCFLAGS += -CFLAGS -DFLASH_BIN='\"$(FLASH_BIN)\"'
endif

all: clean compile

run: compile
	./obj_dir/V${TOPLEVEL}

verilate: ${VERILOGS}
	rm -f ${TOPLEVEL}.v*.bin
	cp -f ../../../../${TOPLEVEL}.v*.bin . | true
	verilator --cc --top-module  ${TOPLEVEL}   ${VERILOGS}  -O3 -CFLAGS -std=c++11  ${ADDCFLAGS} --gdbbt ${VERILATOR_ARGS} -Wno-UNOPTFLAT -Wno-WIDTH --x-assign 0 --exe main.cpp
 	
compile: verilate
	make  -j${THREAD_COUNT} -C obj_dir/ -f V${TOPLEVEL}.mk V${TOPLEVEL}
 	
clean:
	rm -rf obj_dir
	rm -f ${TOPLEVEL}.v*.bin
 	
