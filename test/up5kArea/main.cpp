#include "testbench.h"
#include "misc.h"
#include "VUp5kArea.h"
#include "VUp5kArea_Up5kArea.h"
#include "VUp5kArea_Spram.h"
#include "VUp5kArea_SB_SPRAM256KA.h"

#include <fstream>
#include <iostream>
#include <functional>
#include <sstream>

using namespace std;



#define SYSTEM_CLK_HZ 12000000
#define SERIAL_BAUDRATE 115200
#define TIMESCALE uint64_t(1e12)
int main(int argc, char **argv) {
	cout << "Simulation start" << endl;
	Verilated::commandArgs(argc, argv);
	TESTBENCH<VUp5kArea> *tb = new TESTBENCH<VUp5kArea>(TIMESCALE/SYSTEM_CLK_HZ);
	auto serialTx = new SerialTx([=]() {return tb->dut->io_serialTx;}, TIMESCALE/SERIAL_BAUDRATE);
	tb->add(serialTx);
	auto flash = new N25Q032(&tb->dut->io_flash_ss, &tb->dut->io_flash_sclk, &tb->dut->io_flash_mosi, &tb->dut->io_flash_miso);
	flash->loadBin(0x020000, argString("--bootloader", argc, argv));
    char *flashBin = argString("--flashBin", argc, argv);
	if(flashBin) flash->loadBin(0x030000, flashBin);
	tb->add(flash);


    char *iramBin = argString("--iramBin", argc, argv);
    if(iramBin){
        assert(access( iramBin, F_OK ) != -1);
        FILE *ram_binFile = fopen(iramBin, "r");
        fseek(ram_binFile, 0, SEEK_END);
        uint32_t ram_binSize = ftell(ram_binFile);
        fseek(ram_binFile, 0, SEEK_SET);
        uint8_t * ram_bin = new uint8_t[ram_binSize];
        fread(ram_bin, 1, ram_binSize, ram_binFile);

        uint8_t *ram0 = (uint8_t*)tb->dut->Up5kArea->system_ram->mems_0->mem;
        uint8_t *ram1 = (uint8_t*)tb->dut->Up5kArea->system_ram->mems_1->mem;
        for(int i = 0;i < ram_binSize;i++){
            switch(i&3){
                case 0: ram0[(i + 0x10)/4*2 + 0] = ram_bin[i]; break;
                case 1: ram0[(i + 0x10)/4*2 + 1] = ram_bin[i]; break;
                case 2: ram1[(i + 0x10)/4*2 + 0] = ram_bin[i]; break;
                case 3: ram1[(i + 0x10)/4*2 + 1] = ram_bin[i]; break;
            }
        }
    }

    uint64_t timeout = -1;
    char *timeoutStr = argString("--timeout", argc, argv);
    if(timeoutStr) {   
        std::istringstream iss(timeoutStr);
        iss >> timeout;
    }

    tb->reset();

	while(!tb->done()) {
		tb->tick();
        if(tb->tickCount > timeout) break;
	} exit(EXIT_SUCCESS);

	cout << "Simulation end" << endl;
}
