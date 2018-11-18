================================================
Overview
================================================

This repository is a RISC-V SoftCPU Contest entry. It implement 3 SoC :

- Igloo2Perf : Performant Microsemi IGLOO®2 implementation
- Up5kPerf : Performance Lattice iCE40 UltraPlus™ implementation
- Up5kArea : Small Lattice iCE40 UltraPlus™ implementation

There are some general informations :

- Hardware description made in SpinalHDL/Scala
- CPU used in SoCs is VexRiscv
- Netlist exported in Verilog
- Simulations made with Verilator
- Pass all RV32I compliance tests
- Zephyr OS ready

There is some informations about VexRiscv (CPU used) :

- Hosted on https://github.com/SpinalHDL/VexRiscv
- Implement RV32I[M][C]
- Optimized for FPGA
- Pipelined CPU with a parametrable number of stages : (Fetch -> 0 \.\. x stages) Decode Execute [Memory] [Writeback]
- Deeply parametrable via a system of plugins and a dataflow hardware description layer implemented on the top of SpinalHDL/Scala


================================================
Up5kPerf / Igloo2Perf
================================================

Those two SoC (Up5kPerf and Igloo2Perf) are very similar and try to get the maximal dhrystone score.

There is some characteristics of the VexRiscv configuration used :

- RV32IM
- 6 stages : 2xFetch, Decode, Execute, Memory, Writeback
- Bypassed register file
- Branch condition/target processing in the execute stage, jump in the Memory stage
- 1 way branch target predictor
- 1 cycle barrel shifter (result in Memory stage)
- 1 cycles multiplication using FPGA DSP blocks (result in the writeback stage)
- 34 cycles iterative division, with a lookup table to single cycle division that have small arguements

  - The lookup table save about 33 cycles per dhrystone iteration.
  - The lookup table can be disable by setting dhrystoneOpt of the hardware generation to false.
  - The lookup table purpose is only to boost the dhrystone result
  - the lookup table is a 16x16 table of 4 bits
  - This lookup table optimisation isn't really a fair thing :)
- Uncached fetch/load/store buses
- load command emitted in the Memory stage
- Load result in the Writeback stage
- No emulation

There is some comments about the design :

- In both SoC, the CPU boot on the SPI flash
- on-chip-ram organisation :

  - For the Up5k, there is two 64 KB ram based on SPRAM blocks. One for the instruction bus, one for the data bus
  - For the Igloo2, there is one 32 KB true dual port ram with one port for the instruction bus and one port for the data bus.
- No cache were used for the following reasons :

  - There was enough on chip ram to host the instruction and the data
  - The contest requirements were initaly asking to support fence-i instruction, which aren't supported by the VexRiscv caches (line management is done by another way)
  - Even if using an instruction cache and a data cache allow to have a better decoupling between the CPU and the memory system, it wasn't providing frequancy gain in the implemented SoC.
- This SPI flash contain the following partitions :

  - [0x00000 => FPGA bitstream for the Up5k]
  -  0x20000 => CPU bootloader which copy the 0x30000 partition into the instruction ram
  -  0x30000 => Application that the cpu should run
- The reasons why the VexRiscv is configured with 2 fetch stages instead of 1 are :

  - It relax the branch prediction path
  - It relax the instruction bus to ram path
  - The performance/mhz degradation is mostly absorbed by the branch predictor
- The load command are emitted in the Memory stage instead of the Execute stage to relax the address calculation timings
- The data ram was mapped on purpose at the address 0x00000 for the following reasons :

  - The dhrystone benchmark use many global variables, and by mapping the ram this way, they can be accessed at any time via a x0 relative load/store
  - The RISC-V compiler provided by the zephyr compiler don't use the 'gp' register to access global variables
- The spi flash is programmed by the following way :

  - Up5k -> by using the FTDI and iceprog
  - Igloo2 -> by using the FTDI to Up5k serial link

There is a block diagram explaining the SoCs memory system :

.. |up5kPerfDiagram| image:: doc/assets/up5kPerfDiagram.png
   :width: 400

.. |igloo2PerfDiagram| image:: doc/assets/igloo2PerfDiagram.png
   :width: 400

+--------------------+-----------------------+
| Up5kPerf           +  Igloo2Perf           +
+====================+=======================+
| |up5kPerfDiagram|  +  |igloo2PerfDiagram|  +
+--------------------+-----------------------+


================================================
Up5kArea
================================================

This SoC try to use the least LC possible.

There is some characteristics of the VexRiscv configuration used :

- RV32I
- 2 stages : (Fetch_Decode), Execute
- Hazard resolution choices :

  - Single instruction scheduling (smallest)
  - interlocked
  - bypassed (faster)
- No branch prediction
- Iterative shifter, up to 31 cycles
- Uncached fetch/load/store buses
- No emulation

There is some comments about the design :

- It does not try to get the absolute minimal LC usage as it still keep an traditional pipelined approach.
- This design mainly tried to expand the usage scope of VexRiscv by reducing it's LC usage.
- It provide the occupancy of a regular 2 stages pipelined RISC-V, which could serve as a baseline from which, to reduce the area, "major" architecture changes are required.
- VexRiscv was designed as a 5 stages CPU, but by using its dataflow hardware description paradigm, it was quite easy to retarget it into a 2 stages CPU
- The CPU boot on the SPI flash
- The instruction bus and data bus have share the same memory (64 KB SPRAM)
- This SPRAM memory is only used for the software application.
- This SPI flash contain the following partitions :

  - 0x00000 => FPGA bitstream
  - 0x20000 => CPU bootloader which copy the 0x30000 partition into the SPRAM
  - 0x30000 => Application that the cpu should run
- The spi flash is programmed by using the FTDI and iceprog

There is a block diagram explaining the memory system :

.. image:: doc/assets/xAreaDiagram.png
  :width: 400


================================================
Cool things (SpinalHDL)
================================================

The SpinalHDL hardware description is `there <https://github.com/SpinalHDL/riscvSoftcoreContest/tree/master/hardware/scala/riscvSoftcoreContest>`_.


***************
Interconnect
***************

mapping
===============

The following code come from the Up5kPerf toplevel and generate the whole interconnect :

.. code-block:: scala

    val interconnect = SimpleBusInterconnect()
    interconnect.addSlaves(
      dRam.io.bus         -> SizeMapping(0x00000,  64 kB),
      iRam.io.bus         -> SizeMapping(0x10000,  64 kB),
      peripherals.io.bus  -> SizeMapping(0x70000, 256 Byte),
      flashXip.io.bus     -> SizeMapping(0x80000, 512 kB),
      slowBus             -> DefaultMapping
    )
    interconnect.addMasters(
      dBus   -> List(             dRam.io.bus, slowBus),
      iBus   -> List(iRam.io.bus,              slowBus),
      slowBus-> List(iRam.io.bus, dRam.io.bus,           peripherals.io.bus, flashXip.io.bus)
    )

.. image:: doc/assets/up5kPerfDiagram.png
  :width: 400

pipelining
===========================

To improve the interconnect performance, the following code add pipelining stages between some nodes of the interconnect (Up5kPerf toplevel code sample):

.. code-block:: scala

    interconnect.setConnector(dBus, slowBus){(m, s) =>
      m.cmd.halfPipe() >> s.cmd
      m.rsp            << s.rsp
    }
    interconnect.setConnector(iBus, slowBus){(m, s) =>
      m.cmd.halfPipe() >> s.cmd
      m.rsp            << s.rsp
    }

will produce add the following pipelining stages :

.. image:: doc/assets/interconnectPipelining.png
  :width: 400

To explaine a bit the scala syntax, we call the interconnect's setConnector function,

- as first set of arguements we specify that the connector is between dBus et slowBus,
- as second set of arguments we give a lambda function which can be called by the interconnect to connect m (master) to s (slave).

When called, this lambda function connect the m.cmd stream to the s.cmd stream via an halfPipe stage (bandwidth divided by two but all combinatorial path are cuted),
and directly connect the s.rsp flow to m.rsp flow.

