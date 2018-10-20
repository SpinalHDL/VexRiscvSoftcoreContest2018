#include "../testbench.h"
#include "VUp5kSpeed.h"
#include "VUp5kSpeed_Up5kSpeed.h"
#include "VUp5kSpeed_Spram.h"
#include "VUp5kSpeed_SB_SPRAM256KA.h"
#include <iostream>
#include <functional>

using namespace std;

class SerialTx : public Agent {
    std::function<bool()> pin;
    int baudPeriod;
public:
    SerialTx(std::function<bool()> pin, int baudPeriod) : pin(pin), baudPeriod(baudPeriod){}
    int inFrame = false;
    int bitId = 8;
    uint64_t baudTimeout = 0;
    char buffer;
    virtual void preCycle(uint64_t time) {}
    virtual void postCycle(uint64_t time) {
        if(time < baudTimeout) return;
        if(bitId == 8){
            if(!pin()){
                bitId = 0;
                baudTimeout = time + baudPeriod*1.5;
                buffer = 0;
            }
        } else {
            buffer |= pin() << bitId;
            bitId++;
            baudTimeout = time + baudPeriod;
            if(bitId == 8) {
            	cout << buffer;
                cout.flush();
            }
        }
    }
};


int main(int argc, char **argv) {
	cout << "Simulation start" << endl;
	Verilated::commandArgs(argc, argv);
	TESTBENCH<VUp5kSpeed> *tb = new TESTBENCH<VUp5kSpeed>();
	tb->add(new SerialTx([=]() {return tb->dut->io_serialTx;}, 2000));


    #ifdef IRAM_BIN
    FILE *ram_binFile = fopen(IRAM_BIN, "r");
    fseek(ram_binFile, 0, SEEK_END);
    uint32_t ram_binSize = ftell(ram_binFile);
    fseek(ram_binFile, 0, SEEK_SET);
    uint8_t * ram_bin = new uint8_t[ram_binSize];
    fread(ram_bin, 1, ram_binSize, ram_binFile);

    uint8_t *ram0 = (uint8_t*)tb->dut->Up5kSpeed->iRam->mems_0->mem;
    uint8_t *ram1 = (uint8_t*)tb->dut->Up5kSpeed->iRam->mems_1->mem;
    for(int i = 0;i < ram_binSize;i++){
        switch(i&3){
            case 0: ram0[i/4*2 + 0] = ram_bin[i]; break;
            case 1: ram0[i/4*2 + 1] = ram_bin[i]; break;
            case 2: ram1[i/4*2 + 0] = ram_bin[i]; break;
            case 3: ram1[i/4*2 + 1] = ram_bin[i]; break;
        }
    }
    #endif

    tb->reset();
	while(!tb->done()) {
		tb->tick();
	} exit(EXIT_SUCCESS);

	cout << "Simulation end" << endl;
}
