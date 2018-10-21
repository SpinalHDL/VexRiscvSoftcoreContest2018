CFLAGS= -march=rv32i  -mabi=ilp32 -g -O3 -MD
LFLAGS= -nostdlib  -mcmodel=medany -nostartfiles -ffreestanding -fPIC -fPIE

RISCV_GCC=riscv64-unknown-elf-gcc
RISCV_OBJCOPY=riscv64-unknown-elf-objcopy
RISCV_OBJDUMP=riscv64-unknown-elf-objdump

%.o: %.S
	$(RISCV_GCC) -c $(CFLAGS) -o $@ $^

%.elf: %.S
	$(RISCV_GCC)  $(CFLAGS) -o $@ $^ $(LFLAGS) -Wl,-Bstatic,-T,mapping.ld,-Map,$@.map,--print-memory-usage
	$(RISCV_OBJDUMP) -S -d $@ > $@.asm

%.bin: %.elf
	$(RISCV_OBJCOPY) -O binary $^ $@

all: copyFlash.bin