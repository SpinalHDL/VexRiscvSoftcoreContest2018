#include "testbench.h"
#include "misc.h"
#include "VIgloo2Perf.h"
#include "VIgloo2Perf_Igloo2Perf.h"
#include "VIgloo2Perf_SimpleBusMultiPortRam.h"

#include <fstream>
#include <iostream>
#include <functional>
#include <sstream>

using namespace std;



class SpiFlash : public Agent {
	CData *ss, *sclk, *mosi, *miso;
    enum  { IDLE, INSTRUCTION, CONFIG, READ_ADDRESS, READ_DUMMY, READ_DATA, WRITE_ADDRESS, WRITE_DATA} state = IDLE;
    uint32_t counter;
    uint32_t address;
    uint32_t buffer;
    CData sclkOld;
public:
    uint8_t rom[1 << 24];
    SpiFlash(CData* ss, CData* sclk, CData* mosi, CData* miso) : ss(ss), sclk(sclk), mosi(mosi), miso(miso){
        for(int idx = 0;idx < (1 << 24);idx++){
            rom[idx] = rand();
        }
    }
    void loadBin(uint32_t address, const char* path){
        FILE *f = fopen(path, "r");
        fseek(f, 0, SEEK_END);
        uint32_t binSize = ftell(f);
        fseek(f, 0, SEEK_SET);
        uint8_t * bin = new uint8_t[binSize];
        fread(bin, 1, binSize, f);

        for(int i = 0;i < binSize;i++){
        	rom[address + i] = bin[i];
        }
    }
    virtual void preCycle(uint64_t time) {}
    virtual void postCycle(uint64_t time) {
		if(*ss){
			state = IDLE;
		} else {
	    	bool risingEdge =   *sclk && !sclkOld;
	    	bool fallingEdge = !*sclk && sclkOld;
			switch(state){
			case IDLE:
				state = INSTRUCTION;
				counter = 0;
				buffer = 0;
				break;
			case INSTRUCTION:
				if(risingEdge){
					buffer |= *mosi << (7-counter);
					counter++;
					if(counter == 8){
						switch(buffer){
						case 0x81:
							state = CONFIG;
							buffer = 0;
							counter = 0;
							break;
						case 0x0B:
							state = READ_ADDRESS;
							buffer = 0;
							counter = 0;
							break;
						case 0x02:
							state = WRITE_ADDRESS;
							buffer = 0;
							counter = 0;
							break;
						}
					}
				}
				break;
			case CONFIG:
				if(risingEdge){
					buffer |= *mosi << (7-counter);
					counter++;
					if(counter == 8){
						assert(buffer == 0x83);
					}
				}
				break;
			case READ_ADDRESS:
				if(risingEdge){
					buffer |= *mosi << (23-counter);
					counter++;
					if(counter == 24){
						state = READ_DUMMY;
						address = buffer;
						counter = 0;
					}
				}
				break;
			case WRITE_ADDRESS:
				if(risingEdge){
					buffer |= *mosi << (23-counter);
					counter++;
					if(counter == 24){
						state = WRITE_DATA;
						address = buffer;
                        cout << "Flash write at " << address << endl;
						counter = 0;
						buffer = 0;
					}
				}
				break;
			case READ_DUMMY:
				if(risingEdge){
					counter++;
					if(counter == 8){
						state = READ_DATA;
						counter = 0;
					}
				}
				break;
			case READ_DATA:
				if(fallingEdge){
					*miso = (rom[address] >> (7-counter)) & 1;
					counter++;
					if(counter == 8){
						address++;
						counter = 0;
					}
				}
				break;
			case WRITE_DATA:
				if(risingEdge){
					buffer |=  (*mosi) << (7-counter);
					counter++;
					if(counter == 8){
					    rom[address] = buffer;
						address++;
						counter = 0;
						buffer = 0;
					}
				}
				break;
			}

		}

		sclkOld = *sclk;
    }
};

#define SYSTEM_CLK_HZ 114000000
#define SERIAL_BAUDRATE 11400000
#define SERIAL_LOAD_BAUDRATE 11400000
#define TIMESCALE uint64_t(1e12)

int main(int argc, char **argv) {
	cout << "Simulation start" << endl;
	Verilated::commandArgs(argc, argv);
	TESTBENCH<VIgloo2Perf> *tb = new TESTBENCH<VIgloo2Perf>(TIMESCALE/SYSTEM_CLK_HZ);
	auto serialTx = new SerialTx([=]() {return tb->dut->io_serialTx;}, TIMESCALE/SERIAL_BAUDRATE);
	tb->add(serialTx);
	auto spiFlash = new SpiFlash(&tb->dut->io_flash_ss, &tb->dut->io_flash_sclk, &tb->dut->io_flash_mosi, &tb->dut->io_flash_miso);
	if(argString("--bootloader", argc, argv)){
	    spiFlash->loadBin(0x020000, argString("--bootloader", argc, argv));
	}
    char *flashBin = argString("--flashBin", argc, argv);
	if(flashBin) spiFlash->loadBin(0x030000, flashBin);
	tb->add(spiFlash);


    char *serialLoad = argString("--serialLoad", argc, argv);
    if(serialLoad){
        auto serialRx = SerialRx(&tb->dut->io_serialRx, TIMESCALE/SERIAL_LOAD_BAUDRATE, serialLoad);
        tb->add(&serialRx);
    }


    char *iramBin = argString("--iramBin", argc, argv);
    if(iramBin){
        FILE *ram_binFile = fopen(iramBin, "r");
        fseek(ram_binFile, 0, SEEK_END);
        uint32_t ram_binSize = ftell(ram_binFile);
        fseek(ram_binFile, 0, SEEK_SET);
        uint8_t * ram_bin = new uint8_t[ram_binSize];
        fread(ram_bin, 1, ram_binSize, ram_binFile);


        for(int i = 0;i < ram_binSize;i++){
            switch(i&3){
                case 0: tb->dut->Igloo2Perf->system_ram->ram_symbol0[(i + 0x10)/4] = ram_bin[i]; break;
                case 1: tb->dut->Igloo2Perf->system_ram->ram_symbol1[(i + 0x10)/4] = ram_bin[i]; break;
                case 2: tb->dut->Igloo2Perf->system_ram->ram_symbol2[(i + 0x10)/4] = ram_bin[i]; break;
                case 3: tb->dut->Igloo2Perf->system_ram->ram_symbol3[(i + 0x10)/4] = ram_bin[i]; break;
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
