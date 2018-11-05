#include <verilated_vcd_c.h>
#include <fstream>
#include <iostream>
#include <functional>
#include "verilated_heavy.h"
using namespace std;

class SerialTx : public Agent {
    std::function<bool()> pin;
    int baudPeriod;
    ofstream logTraces;
public:
    SerialTx(std::function<bool()> pin, int baudPeriod) : pin(pin), baudPeriod(baudPeriod){
        logTraces.open ("log.txt");
    }
    int inFrame = false;
    int bitId = 8;
    uint64_t baudTimeout = 0;
    char buffer;
    uint64_t flushTimout = 0;
    virtual void preCycle(uint64_t time) {}
    virtual void postCycle(uint64_t time) {
        if(flushTimout == 0){
            flushTimout = 100000;
            logTraces.flush();
        } else {
            flushTimout--;
        }
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
                if(buffer == (char)0xFF) {
				    logTraces.flush();
                    cout << endl <<  "Simulation done" << endl;
                    cout.flush();
                    exit(EXIT_SUCCESS);
                }
            	cout << buffer;
                cout.flush();
				logTraces << buffer;
            }
        }
    }
};



class SerialRx : public Agent {
    CData *pin;
    int baudPeriod;
    uint8_t * bin;
    uint32_t binSize;
public:
    SerialRx(CData * pin, int baudPeriod, const char* filePath) : pin(pin), baudPeriod(baudPeriod) {
        FILE *ram_binFile = fopen(filePath, "r");
        fseek(ram_binFile, 0, SEEK_END);
        binSize = ftell(ram_binFile);
        fseek(ram_binFile, 0, SEEK_SET);
        bin = new uint8_t[binSize];
        fread(bin, 1, binSize, ram_binFile);
        baudTimeout = baudPeriod * 2;
        *pin = 1;
    }
    int inFrame = false;
    int bitId = 8;
    int byteId = 0;
    int state = 0;
    uint64_t baudTimeout;
    virtual void preCycle(uint64_t time) {}
    virtual void postCycle(uint64_t time) {
        if(byteId == binSize) return;
        if(time < baudTimeout) return;
        baudTimeout += baudPeriod;
        switch(state){
            case 0: *pin = 0; state++; bitId=0; break;
            case 1:
                *pin = (bin[byteId] >> bitId)&1;
                 bitId++;
                 if(bitId == 8) {
                    state++;
                 }
                 break;
            case 2: *pin=1; state=0; byteId++; break;
        }
    }
};

char* argString(char *key, int argc, char **argv){
    for(int idx = 0;idx < argc;idx++){
        if(!strcmp(argv[idx], key)){
            return argv[idx + 1];
        }
    }
    return NULL;
}


