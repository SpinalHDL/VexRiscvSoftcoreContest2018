CFLAGS= -march=RV32IM -g -O3 -MD
LFLAGS= -nostdlib  -mcmodel=medany -nostartfiles -ffreestanding -fPIC -fPIE

RISCV_BIN ?= ${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/bin/riscv32-zephyr-elf/riscv32-zephyr-elf-
RISCV_GCC=${RISCV_BIN}gcc
RISCV_OBJCOPY=${RISCV_BIN}objcopy
RISCV_OBJDUMP=${RISCV_BIN}objdump

%.o: %.S
	$(RISCV_GCC) -c $(CFLAGS) -o $@ $^

%.elf: %.S
	$(RISCV_GCC)  $(CFLAGS) -o $@ $^ $(LFLAGS) -Wl,-Bstatic,-T,mapping.ld,-Map,$@.map,--print-memory-usage
	$(RISCV_OBJDUMP) -S -d $@ > $@.asm

%.bin: %.elf
	$(RISCV_OBJCOPY) -O binary $^ $@

all: copyFlash.bin noFlash.bin

clean:
	rm -f *.o
	rm -f *.elf
	rm -f *.bin