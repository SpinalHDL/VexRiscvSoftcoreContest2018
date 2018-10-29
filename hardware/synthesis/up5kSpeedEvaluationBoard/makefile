TOPLEVEL = Up5kSpeedEvaluationBoard

PIN_DEF = mapping.pcf
VERILOGS = ../../../../../${TOPLEVEL}.v
DEVICE = up5k


all: $(TOPLEVEL).rpt $(TOPLEVEL).bin

%.blif: ${VERILOGS}
	yosys -p 'synth_ice40 -top ${TOPLEVEL} -blif $@' $<

%.asc: $(PIN_DEF) %.blif
	arachne-pnr -d $(subst up,,$(subst hx,,$(subst lp,,$(DEVICE)))) -o $@ -p $^

%.bin: %.asc
	icepack $< $@

%.rpt: %.asc
	icetime -d $(DEVICE) -mtr $@ $<

%_tb: %_tb.v %.v
	iverilog -o $@ $^

%_tb.vcd: %_tb
	vvp -N $< +vcd=$@

%_syn.v: %.blif
	yosys -p 'read_blif -wideports $^; write_verilog $@'

%_syntb: %_tb.v %_syn.v
	iverilog -o $@ $^ `yosys-config --datdir/ice40/cells_sim.v`

%_syntb.vcd: %_syntb
	vvp -N $< +vcd=$@

time: ${TOPLEVEL}.rpt
	cat ${TOPLEVEL}.rpt


prog: $(TOPLEVEL).bin
	iceprog $<

progBoot:
	iceprog -o 0x20000  ../../../asm/up5kBootFlash/copyFlash.bin
	iceprog -o 0x30000  /home/spinalvm/hdl/zephyr/zephyrSpinalHdl/samples/synchronization/build_up5kspeed/zephyr/zephyr.bin


sudo-prog: $(TOPLEVEL).bin
	@echo 'Executing prog as root!!!'
	sudo iceprog $<

clean:
	rm -f $(TOPLEVEL).blif $(TOPLEVEL).asc $(TOPLEVEL).rpt $(TOPLEVEL).bin

.SECONDARY:
.PHONY: all prog clean