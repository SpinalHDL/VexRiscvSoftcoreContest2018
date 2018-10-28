https://github.com/cliffordwolf/icestorm/blob/master/icefuzz/icecube.sh
->
sudo apt-get install libc6-i386 zlib1g:i386 libxext6:i386 libpng12-0:i386 libsm6:i386 -y
sudo apt-get install libxi6:i386 libxrender1:i386 libxrandr2:i386 libxfixes3:i386 -y
sudo apt-get install libxcursor1:i386 libxinerama1:i386 libfreetype6:i386 -y
sudo apt-get install libfontconfig1:i386 libglib2.0-0:i386 libstdc++6:i386 libelf1:i386 -y

sudo /sbin/modprobe dummy
sudo /sbin/ip link set name eth0 dev dummy0
sudo /sbin/ifconfig eth0 hw ether 08:00:27:79:ed:b1

https://electronics.stackexchange.com/questions/327527/lattice-icecube2-error-synplify-pro-321

make clean run IRAM_BIN=/home/spinalvm/hdl/riscvSoftcoreContest/dhrystone/build/dhrystone.bin TRACE=y
make clean run IRAM_BIN=/home/spinalvm/hdl/zephyr/zephyrSpinalHdl/samples/synchronization/build_up5kspeed/zephyr/zephyr.bin TRACE=no

