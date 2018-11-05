TOPLEVEL=Igloo2Perf
ROOT=  ../..
VERILOGS = ${ROOT}/hardware/netlist/${TOPLEVEL}.v
HEADERS  = ../common/misc.h ../common/testbench.h
SRCS     = main.cpp

TRACE ?= no
DEBUG ?= no


ADDCFLAGS += -CFLAGS -I../../common

ifeq ($(TRACE),yes)
	VERILATOR_ARGS += --trace
	ADDCFLAGS += -CFLAGS -DTRACE
endif

ifeq ($(DEBUG),yes)
	ADDCFLAGS += -CFLAGS -O0 -CFLAGS -g
else
	ADDCFLAGS += -CFLAGS -O3
endif

run: ./obj_dir/V${TOPLEVEL}
	./obj_dir/V${TOPLEVEL} ${ARGS}

./obj_dir: ${VERILOGS}
	rm -rf ./obj_dir
	rm -f ${TOPLEVEL}.v*.bin
	cp -f ../../../../${TOPLEVEL}.v*.bin . | true
	verilator --cc --top-module  ${TOPLEVEL}   ${VERILOGS}  -O3 -CFLAGS -std=c++11  ${ADDCFLAGS} --gdbbt ${VERILATOR_ARGS} -Wno-UNOPTFLAT -Wno-WIDTH --x-assign 0 --exe ${SRCS}
 	
./obj_dir/V${TOPLEVEL}: ./obj_dir ${SRCS} ${HEADERS}
	make  -j${THREAD_COUNT} -C obj_dir/ -f V${TOPLEVEL}.mk V${TOPLEVEL}
 	
compile: ./obj_dir/V${TOPLEVEL}

clean:
	rm -rf obj_dir
	rm -f ${TOPLEVEL}.v*.bin

all: clean compile

 	
