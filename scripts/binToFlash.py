#!/usr/bin/env python

import sys

filePath = sys.argv[1]
targetBase = int(sys.argv[2], 16)
baudperiod = 1.0/int(sys.argv[3], 10)
outputPath = sys.argv[4]

binary =  [ord(i) for i in open(filePath, 'rb').read()]
targetEnd = targetBase + len(binary)

f = open(outputPath, 'wb')

RESET = 1 << 7
SPI_ENABLE= 1 << 6
SS = 1 << 0
SCLK = 1 << 1
MOSI = 1 << 2

currentValue = 0
def setOutput(mask,value):
    global currentValue
    global f
    currentValue &= ~mask
    currentValue |= value
    f.write(bytearray([currentValue]))

def init():
    setOutput(RESET | SPI_ENABLE | SS | SCLK, RESET | SPI_ENABLE | SS)


def start():
    setOutput(SS, 0x00)


def stop():
    setOutput(SCLK, 0x00)
    setOutput(SS, SS)

def frame(bytes):
    start()
    for b in bytes:
        write(b)
    stop()

def write(value):
    for i in xrange(8):
        setOutput(SCLK | MOSI, (((value >> 7-i) & 1) * MOSI))
        setOutput(SCLK, SCLK)

def release():
    setOutput(SPI_ENABLE, 0x00)
    setOutput(RESET, 0x00)

def sleep(sec):
    for i in xrange(int(sec/(10*baudperiod))+1):
        setOutput(0x00, 0x00)


def addressToBytes(address):
    return [(address >> 16) & 0xFF, (address >> 8) & 0xFF, (address >> 0) & 0xFF]

init()

frame([0x9F]+[0]*3) #JEDEC-ID SEQUENCE

frame([0x98]) # GLOBAL BLOCK-PROTECTION UNLOCK

flashSize = 64*1024*1024/8
sectors = [0x2000]*4 +[0x8000] + [0x10000]*(flashSize/0x10000-2) + [0x8000] + [0x2000]*4
sectorStart = 0
for s in sectors:
    sectorEnd = sectorStart + s
    if targetBase >= sectorStart and targetEnd <= sectorEnd:
        #print("Erase " + hex(sectorStart) + "-" + hex(sectorEnd-1))
        frame([0x05] + [0] * 1)  # RDSR
        frame([0x06] + [0] * 1)  # WREN
        frame([0xD8] + addressToBytes(sectorStart))  # EREASE
        sleep(100e-3)
    sectorStart += s

address = targetBase
while address != targetEnd:
    frame([0x05]+[0]*1) # RDSR
    frame([0x06]+[0]*1) # WREN
    offset = address-targetBase
    burstSize = min(0x100 - (offset & 0xFF), targetEnd-address)
    data = binary[offset:offset+burstSize]
    frame([0x02]+addressToBytes(address)+data)#PROGRAM
    sleep(3e-3)
    #frame([0x0B]+addressToBytes(address)+[0]+data)  # READ
    #print("Program " + hex(address) + "-" + hex(address+burstSize-1))
    address += burstSize

release()

f.close()


#./binToFlash.py ../main/asm/igloo2BootFlash/copyFlash.bin 0x20000 115200 copyFlash.bin.serial; cp copyFlash.bin.serial /media/sf_share
#./binToFlash.py ../../dhrystone/build/dhrystone.bin 0x30000 115200 dhrystone.bin.serial; cp dhrystone.bin.serial /media/sf_share
###################################
# frame([0x9F]+[0]*3) #JEDEC-ID SEQUENCE
#
# frame([0x98]) # GLOBAL BLOCK-PROTECTION UNLOCK
#
# frame([0x05]+[0]*1) # RDSR
# frame([0x06]+[0]*1) # WREN
# frame([0xD8]+[0,0,0]) #EREASE
# sleep(100e-3)
#
# frame([0x05]+[0]*1) # RDSR
# frame([0x06]+[0]*1) # WREN
# frame([0x02]+[0,0,0]+[1,2,3,4,5])#PROGRAM
# sleep(3e-3)
#
# frame([0x05]+[0]*1) # RDSR
# frame([0x06]+[0]*1) # WREN
# frame([0x02]+[0,1,0]+[0x11,0x22,0x33,0x44]) #PROGRAM
# sleep(3e-3)
#
#
# frame([0x0B]+[0,0,0]+[0]*10)#READ
# frame([0x0B]+[0,1,0]+[0]*10)#READ
###################################