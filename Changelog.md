# Version 2.0a
- Added [DRAMSim2](https://github.com/umd-memsys/DRAMSim2) support
- Flush all the CPU caches and DRAM models for every new simulation run
- Fixed Issue #8: useless cleaning of local variables

# Version 1.1a
- Added 16550A UART support (thanks to Marc Gauthier)
- Reworked the dram latency parameters to match the Sifive HiFive U540 Board
- Increased the dram dispatch queue size from 32 to 64
- Add a timestamp suffix to the stats file
- Bug fix: fixed the calculation of hardware page walk latency
- Bug fix: fixed the miscalculation in page fault counters
- Fixed Issue #2: memory leaks in copy_file
- Fixed Issue #3: 'log' instead 'log2'
