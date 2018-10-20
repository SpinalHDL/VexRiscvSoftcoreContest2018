TOPLEVEL=Up5kSpeed
VERILOGS=  ../up5k_cells_sim.v ../../../../../${TOPLEVEL}.v

TRACE ?= no
DEBUG ?= yes
IRAM_BIN ?= no

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

all: clean compile

run: compile
	./obj_dir/V${TOPLEVEL}

verilate: ${VERILOGS}
	rm -f ${TOPLEVEL}.v*.bin
	cp -f ../../../../../${TOPLEVEL}.v*.bin . | true
	verilator --cc --top-module  ${TOPLEVEL}   ${VERILOGS}  -O3 -CFLAGS -std=c++11  ${ADDCFLAGS} --gdbbt ${VERILATOR_ARGS} -Wno-UNOPTFLAT -Wno-WIDTH --x-assign 0 --exe main.cpp
 	
compile: verilate
	make  -j${THREAD_COUNT} -C obj_dir/ -f V${TOPLEVEL}.mk V${TOPLEVEL}
 	
clean:
	rm -rf obj_dir
	rm -f ${TOPLEVEL}.v*.bin
 	
