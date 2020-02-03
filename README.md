## MARSS-RISCV: Micro-Architectural System Simulator for RISC-V

[![MIT license](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/bucaps/marss-riscv/blob/master/src/MIT-LICENSE.txt) [![GitHub issues](https://img.shields.io/github/issues/Naereen/StrapDown.js.svg)](https://github.com/bucaps/marss-riscv/issues/)

MARSS-RISCV (Micro-ARchitectural System Simulator - RISCV) is a open source, **cycle-accurate single core full-system (Linux) micro-architectural simulator** for the [RISC-V](https://riscv.org/specifications/) ISA built upon [TinyEMU emulator (https://bellard.org/tinyemu)](https://bellard.org/tinyemu) by Fabrice Bellard and uses its code for all the device emulation and configuration. It consists of detailed cycle accurate models of a modern RISC-V In-order and Out-of-order processor with branch prediction unit and a complete memory hierarchy including TLBs, caches and DRAM. It comes integrated with [DRAMSim2](https://github.com/umd-memsys/DRAMSim2), a cycle accurate memory system simulator. It is currently being developed and maintained by [CAPS](https://github.com/bucaps/) (Computer Architecture and Power Aware Systems Research Group) at the State University of New York at Binghamton. Being a true full system simulator, MARSS-RISCV can simulate all of the system in a cycle accurate fashion including OS code, libraries, interrupt handlers etc.

Currently, our simulator is in alpha status as we are validating the cycle accuracy using various development boards. The simulated in-order core is tested and operational, however, the simulated out-of-order core is in micro-architectural testing phase.

## Table of contents
- [Features](#features)
- [Getting started with the simulator running a Linux guest](#getting-started-with-the-simulator-running-a-linux-guest)
- [Running full system simulations](#running-full-system-simulations)
- [Viewing live simulation stats](#viewing-live-simulation-stats)
- [Generating simulation trace](#generating-simulation-trace)
- [Future work](#future-work)
- [Technical notes](#technical-notes)
- [Authors](#authors)
- [Acknowledgment](#acknowledgment)
- [License](#license)

## Features 
- True full system simulator: simulates all of the system in a cycle accurate fashion including the bootloader, OS code, libraries, interrupt handlers, user level applications etc
- Fully configurable, cycle-accurate, in-order and out-of-order single-core RISC-V CPU
- Multiple execution units with configurable latencies (execution units can be configured to be pipelined)
- 2-level cache hierarchy with various allocation and miss handling policies
- 2 DRAM memory models: Simple DIMM based basic DRAM model that simulates row-buffer (open-page) hits and [DRAMSim2](https://github.com/umd-memsys/DRAMSim2)
- Bi-modal and 2-level adaptive (Gshare, Gselect, GAg, GAp, PAg, PAp) branch prediction support
- Supports `RV32GC` and `RV64GC` (user level ISA version `2.2`, privileged architecture version `1.10`)
- VirtIO console, network, block device, input and 9P filesystem
- JSON configuration file
- Easy to install, use and modify

For internal micro-architectural details, see [MARSS-RISCV Micro-architecture Documentation](https://marss-riscv-docs.readthedocs.io/en/latest/).

## Getting started with the simulator running a Linux guest

### System requirements
* 32-bit or 64-bit Linux machine
* Libcurl, OpenSSL and SDL Libraries
* Standard C compiler

### Installing the dependencies
Make sure that you have all the dependencies (`ssl`, `sdl`, and `curl` libraries) installed on the system. For Debian-based (including Ubuntu) systems, the packages are: `build-essential`, `libssl-dev`, `libsdl1.2-dev`, `libcurl4-openssl-dev`. 
```console
$ sudo apt-get update
$ sudo apt-get install build-essential
$ sudo apt-get install libssl-dev
$ sudo apt-get install libsdl1.2-dev
$ sudo apt-get install libcurl4-openssl-dev
```

### Compiling the simulator

First, clone the simulator repository:

```console
$ git clone https://github.com/bucaps/marss-riscv
```
Then, `cd` into the simulator source directory:

```console
$ cd marss-riscv/src/
$ git submodule update --init --recursive
```
Set the `CONFIG_XLEN` variable in the Makefile to the desired `XLEN` as required. Supported `XLEN` values are `32` and `64`.

Then, compile the simulator using:

```console
$ make
```

### Preparing the bootloader, kernel and userland image

Using pre-built bootloader, kernel and userland images is the easiest way to start. The pre-built images are located in the [marss-riscv-images](https://github.com/bucaps/marss-riscv-images) repository. To clone it, make sure you have [Git LFS](https://git-lfs.github.com/) installed on your system, and type:

```console
$ git clone https://github.com/bucaps/marss-riscv-images
```

The userland image needs to be decompressed before running the simulator:

```console
$ cd marss-riscv-images/riscv32-unknown-linux-gnu/
$ xz -d -k -T 0 riscv32.img.xz
```

When decompression finishes, launch the simulator with:

```console
$ ../../marss-riscv -mem-model base riscvemu.cfg
```

Simulation parameters can be configured using `riscvemu.cfg`, RISCVEMU JSON configuration file. 

By default, the simulator will boot in "snapshot" mode, meaning it will **not** retain the file system changes after it is shut down. In order to persist the changes, pass `-rw` command line argument to the simulator. MARSS-RISCV comes with 2 DRAM memory models: Basic and DRAMSim2. To specify which memory model to use, run MARSS-RISCV with command line option `-mem-model` and specify either `base` or `dramsim2`. For DRAMSim2, the paths to `ini` and `system ini file` can be specified in `riscvemu.cfg` file.

It may also be desirable to grow the userland image (has roughly 200MB of available free space by default). More information about how to grow it can be found [here](https://github.com/bucaps/marss-riscv-images#how-to-use).

By default, guest boots in emulation mode. To start in simulation mode run with `-simstart` command line option.

Once the guest boots, we need to initialize the environment. Normally, this should happen automatically but due to an unresolved bug it needs to done explicitly:

```console
# export PYTHONPATH=/usr/lib64/python2.7/site-packages/
# env-update
```

The system is ready for use. It has a working GCC compiler, ssh, git, and [more](https://github.com/bucaps/marss-riscv-images/blob/master/riscv32-unknown-linux-gnu/PACKAGES). It has a virtual network interface. However, there's no host-to-guest routing, so it's not possible to ssh into the guest.

By default, `Ctrl-C` will not kill the simulator. The command `halt` will cleanly shutdown the guest. Alternatively, you can pass the `-ctrlc` command line argument to the simulator, which will allow it to be killed using `Ctrl-C`.

Once you have access to the guest machine terminal, see next section for running simulations.

## Running full system simulations

### Using checkpoints
We have provided two special checkpoint instructions, `SIM_START()` and `SIM_STOP()`, which inform MARSS-RISCV to enable and disable simulation mode respectively, when they are encountered during instruction processing. When the simulation is disabled, MARSS-RISCV runs in the emulation mode. For this purpose, we have used two special unused registers from user mode CSR address space, `0x800` and `0x801`. Using these two special markers, it is possible to simulate any section in the source code. However, after inserting these markers, the source code must be recompiled inside the guest OS using the installed gcc toolchain. Below code shows a simple hello world program with checkpoints. On encountering `SIM_STOP()` checkpoint during the simulation, all the performance stats are saved in the file `sim_stats_file` as configured in the simulator configuration file.

```c
/* hello_world.c */

#include <stdio.h>

#define SIM_START() asm("csrs 0x800,zero")
#define SIM_STOP() asm("csrs  0x801,zero")

int main()
{
  SIM_START();
  printf("Hello World\n");
  SIM_STOP();
  return 0;
}
```

### Using simulate script
Alternatively, you can use the provided simulation script (`simulate.c`) provided [here](https://github.com/bucaps/marss-riscv/tree/master/scripts) which forks a child process. Child enters the simulation mode and execs the command. Parent process waits for the child to complete and then switches MARSS-RISCV back to emulation mode. Using this script it is possible to simulate any program without need to modify and re-compile the source code. Since child switches to simulation mode before calling `exec()`, `exec()` also runs in the simulation mode. Hence, performance statistics generated at the end of the simulation will also include stats for `exec()`.

### Running benchmarks
We have provided a detailed step-by-step comprehensive tutorial [here](https://marss-riscv-docs.readthedocs.io/en/latest/sections/running-full-system.html) which configures MARSS-RISCV to simulate a simple 5-stage 32-bit in-order RISC-V processor and run [CoreMark](https://github.com/eembc/coremark), an industry-standard benchmark that measures the performance of central processing units (CPU) and embedded microcrontrollers (MCU).

## Viewing live simulation stats
You can view live simulation stats using the provided `stats-display` tool. First, open a new terminal before executing the simulator and launch `stats-display`:

```console
$ ./stats-display
```
Then launch the simulator on a different terminal with `-stats-display` command-line option.

## Generating simulation trace
To generate instruction commit trace of the programs running in the simulation mode, add `CONFIG_SIM_TRACE` flag to the CFLAGS variable in the makefile and recompile. Generated trace is saved in the file `sim_trace_file` as configured in the simulator configuration file.

Sample trace is shown below:
```bash
cycle = 112  pc = 0x6aaaadf8  insn = 0x85be863a  c.mv a2,a4    mode = PRV_U  status = simulated
cycle = 113  pc = 0x6aaaadfa  insn = 0x250385be  c.mv a1,a5    mode = PRV_U  status = simulated
cycle = 115  pc = 0x6aaaadfc  insn = 0xfa842503  lw a0,s0,-88  mode = PRV_U  status = simulated

```

## Future Work
* Support for 3-stage in-order pipeline
* Support for return address stack
* Support for branch prediction unit, speculative execution and age-ordered instruction issue logic in the out-of-order core
* Cycle accuracy validation using various RISC-V development boards

## Technical notes

### Floating point emulation

The floating point emulation is bit exact and supports all the
specified instructions for 32 and 64 bit floating point numbers. It
uses the new SoftFP library.

### HTIF console

The standard HTIF console uses registers at variable addresses which
are deduced by loading specific ELF symbols. TinyEMU does not rely on
an ELF loader, so it is much simpler to use registers at fixed
addresses (0x40008000). A small modification was made in the
"riscv-pk" boot loader to support it. The HTIF console is only used to
display boot messages and to power off the virtual system. The OS
should use the VirtIO console.

### Network usage

The easiest way is to use the "user" mode network driver. No specific
configuration is necessary.

MARSS-RISCV also supports a "tap" network driver to redirect the network
traffic from a VirtIO network adapter.

You can look at the netinit.sh script to create the tap network
interface and to redirect the virtual traffic to Internet thru a
NAT. The exact configuration may depend on the Linux distribution and
local firewall configuration.

The VM configuration file must include:

eth0: { driver: "tap", ifname: "tap0" }

and configure the network in the guest system with:

ifconfig eth0 192.168.3.2
route add -net 0.0.0.0 gw 192.168.3.1 eth0

### Network filesystem

MARSS-RISCV supports the VirtIO 9P filesystem to access local or remote
filesystems. For remote filesystems, it does HTTP requests to download
the files. The protocol is compatible with the vfsync utility. In the
"mount" command, "/dev/rootN" must be used as device name where N is
the index of the filesystem. When N=0 it is omitted.

The build_filelist tool builds the file list from a root directory. A
simple web server is enough to serve the files.

The '.preload' file gives a list of files to preload when opening a
given file.

### Network block device

MARSS-RISCV supports an HTTP block device. The disk image is split into
small files. Use the 'splitimg' utility to generate images. The URL of
the JSON blk.txt file must be provided as disk image filename.

## Authors
* Copyright (c) 2017-2019 Gaurav Kothari {gkothar1@binghamton.edu}
* Copyright (c) 2018-2019 Parikshit Sarnaik {psarnai1@binghamton.edu}

## Acknowledgment
This work was supported in part by DARPA through an award from the SSITH program. We would like to thank Gokturk Yuksek, Ravi Theja Gollapudi and Kanad Ghose for assistance with the internal details of TinyEMU and the development of the MARSS-RISCV.

For DRAMSim2, see [here](https://github.com/umd-memsys/DRAMSim2).

## License
This project is licensed under the MIT License - see the
src/MIT-LICENSE.txt file for details.

The SLIRP library has its own license (two clause BSD license).
DRAMSim2 has its own license (two clause BSD license).
