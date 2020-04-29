

# Version 3.0a

 - Added
	 - Support for separate RISC-V Bios and Kernel
	 - Wrap-around cache line read logic in memory controller for base memory model
	 - Command line option `flush-sim-mem` to flush simulator memory hierarchy on every fresh simulation run
	 - Command line option `sim-trace` to generate instruction commit trace during simulation
	 - Distinct configurable read-hit and write-hit latency for all the caches
	 - Return address stack (RAS)
	 - Branch prediction and speculative execution support for out of order core
	 - Print performance counters on terminal when the simulation completes
	 - More performance counters:
		 - Instruction types
		 - ecall
		 - page walks for loads, stores and instructions
		 - memory controller delay for data and instructions
		 - hardware interrupts
 - Changed
	 - Port to TinyEMU version `2019-12-21`
	 - For bimodal branch predictor, store prediction bits in a separate Branch history table (BHT)
	 - For in-order core, non-memory instructions can forward their result from MEM stage in addition to EX stage
	 - For in-order core, relaxed interlocking on WAW data hazard
	 - Simplified out of order core design, ROB slots are now used as physical registers along with a single rename table and a single global issue queue
 - Fixed
	 - Correctly calculated the rounding mode for floating pointing instruction decoding
	 - Converted `c.addiw` result buffer into `int32_t` on 64-bit simulation
	 - Set the data type to `unint64_t` for 64-bit simulation, for the buffer which holds the memory address for atomic instructions
	 - Pass the correct physical address to check the status of wrap-around read completion during instruction fetch
	 - Issue #13 and #14 (thanks to Okhotnikov Grigory)

# Version 2.0a

 - Added
	 - Added [DRAMSim2](https://github.com/umd-memsys/DRAMSim2) support
 - Changed
	 - Flush all the CPU caches and DRAM models for every new simulation run
 - Fixed
	 - Issue #8: useless cleaning of local variables

# Version 1.1a
 - Added
	- Add 16550A UART support (thanks to Marc Gauthier)
	- Add a timestamp suffix to the stats file
 - Changed
	- Reworked the dram latency parameters to match the Sifive HiFive U540 Board
	- Increased the dram dispatch queue size from 32 to 64
 - Fixed
	- Calculation of hardware page walk latency
	- Miscalculation in page fault counters
	- Issue #2: memory leaks in copy_file
	- Issue #3: 'log' instead 'log2'
