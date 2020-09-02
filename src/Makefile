#
# TinyEMU
# 
# Copyright (c) 2016-2018 Fabrice Bellard
#
# MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
#
# Copyright (c) 2017-2020 Gaurav Kothari {gkothar1@binghamton.edu}
# State University of New York at Binghamton
#
# Copyright (c) 2018-2019 Parikshit Sarnaik {psarnai1@binghamton.edu}
# State University of New York at Binghamton
#
# Copyright (c) 2019 Göktürk Yüksek {gokturk@binghamton.edu}
# State University of New York at Binghamton
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

# Set the XLEN value
CONFIG_XLEN=64

# if set, network filesystem is enabled. libcurl and libcrypto
# (openssl) must be installed.
CONFIG_FS_NET=y

# SDL support (optional)
CONFIG_SDL=y

# user space network redirector
CONFIG_SLIRP=y

CROSS_PREFIX=
EXE=
CC=$(CROSS_PREFIX)gcc
STRIP=$(CROSS_PREFIX)strip
CFLAGS=-O2 -Wall -g -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -MMD
CFLAGS+=-D_GNU_SOURCE -DCONFIG_VERSION=\"$(shell cat VERSION)\"
CFLAGS+=-DMAX_XLEN=$(CONFIG_XLEN)
LDFLAGS=

PROGS+= marss-riscv$(EXE) sim-stats-display
ifdef CONFIG_FS_NET
PROGS+=build_filelist splitimg
endif

# DRAMSIM3 connections for the simulator
DRAMSIM3_WRAPPER_C_CONNECTOR_LIB+=libdramsim_wrapper_c_connector.so
DRAMSIM3_WRAPPER_C_CONNECTOR_OBJ:=$(addprefix riscvsim/memory_hierarchy/, dramsim_wrapper_c_connector.o)
DRAMSIM3_WRAPPER_LIB+=libdramsim_wrapper_lib.so
DRAMSIM3_WRAPPER_OBJ:=$(addprefix riscvsim/memory_hierarchy/, dramsim_wrapper.o)
DRAMSIM3_LIB_SO+=libdramsim3.so
DRAMSIM3_FMT_LIB_DIR=DRAMsim3/ext/fmt/include
DRAMSIM3_INI_LIB_DIR=DRAMsim3/ext/headers
DRAMSIM3_JSON_LIB_DIR=DRAMsim3/ext/headers
DRAMSIM3_ARGS_LIB_DIR=DRAMsim3/ext/headers
DRAMSIM3_INC=-IDRAMsim3/src/ -I$(DRAMSIM3_FMT_LIB_DIR) -I$(DRAMSIM3_INI_LIB_DIR) -I$(DRAMSIM3_ARGS_LIB_DIR) -I$(DRAMSIM3_JSON_LIB_DIR)

# Ramulator connections for the simulator
RAMULATOR_WRAPPER_C_CONNECTOR_LIB+=libramulator_wrapper_c_connector.so
RAMULATOR_WRAPPER_C_CONNECTOR_OBJ:=$(addprefix riscvsim/memory_hierarchy/, ramulator_wrapper_c_connector.o)
RAMULATOR_WRAPPER_LIB+=libramulator_wrapper_lib.so
RAMULATOR_WRAPPER_OBJ:=$(addprefix riscvsim/memory_hierarchy/, ramulator_wrapper.o)
RAMULATOR_LIB_SO+=libramulator.so
RAMULATOR_INC+=-I./ramulator/src

# Top-level simulator object file
SIM_OBJ_FILE=riscvsim.o

# Simulator object files for each module
SIM_UTILS:=$(addprefix riscvsim/utils/, sim_exception.o sim_trace.o cpu_latches.o evict_policy.o circular_queue.o sim_params.o sim_stats.o sim_log.o)
SIM_DECODER_OBJS:=$(addprefix riscvsim/decoder/, riscv_isa_string_generator.o riscv_isa_decoder.o riscv_isa_execute.o)
SIM_BPU_OBJS:=$(addprefix riscvsim/bpu/, ras.o bht.o btb.o adaptive_predictor.o bpu.o)
SIM_MEM_HY_OBJS:=$(addprefix riscvsim/memory_hierarchy/, temu_mem_map_wrapper.o dram.o memory_hierarchy.o memory_controller.o cache.o )
SIM_IN_CORE_OBJS:=$(addprefix riscvsim/core/, inorder_frontend.o inorder_backend.o inorder.o)
SIM_CORE_OBJS:=$(addprefix riscvsim/core/, riscv_sim_cpu.o)
SIM_OO_CORE_OBJS:=$(addprefix riscvsim/core/, ooo_frontend.o ooo_branch.o ooo_lsu.o ooo_backend.o ooo.o)
SIM_OBJS:=$(SIM_UTILS) $(SIM_DECODER_OBJS) $(SIM_BPU_OBJS) $(SIM_MEM_HY_OBJS) $(SIM_CORE_OBJS) $(SIM_IN_CORE_OBJS) $(SIM_OO_CORE_OBJS)

all: $(PROGS)

EMU_OBJS:=virtio.o pci.o fs.o cutils.o iomem.o simplefb.o \
    json.o machine.o rtc_timer.o temu.o

ifdef CONFIG_SLIRP
CFLAGS+=-DCONFIG_SLIRP
EMU_OBJS+=$(addprefix slirp/, bootp.o ip_icmp.o mbuf.o slirp.o tcp_output.o cksum.o ip_input.o misc.o socket.o tcp_subr.o udp.o if.o ip_output.o sbuf.o tcp_input.o tcp_timer.o)
endif

EMU_OBJS+=fs_disk.o
EMU_LIBS=-lrt -lpthread -lm

ifdef CONFIG_FS_NET
CFLAGS+=-DCONFIG_FS_NET
EMU_OBJS+=fs_net.o fs_wget.o fs_utils.o block_net.o
EMU_LIBS+=-lcurl -lcrypto
endif # CONFIG_FS_NET

ifdef CONFIG_SDL
EMU_LIBS+=-lSDL
EMU_OBJS+=sdl.o
CFLAGS+=-DCONFIG_SDL
endif

EMU_OBJS+=riscv_machine.o softfp.o riscv_cpu.o
CFLAGS+=-DCONFIG_RISCV_MAX_XLEN=64

libdramsim_wrapper_c_connector.so: $(DRAMSIM3_WRAPPER_LIB) $(DRAMSIM3_WRAPPER_C_CONNECTOR_OBJ)
	$(CXX) -shared -o $(DRAMSIM3_WRAPPER_C_CONNECTOR_LIB) $(DRAMSIM3_WRAPPER_C_CONNECTOR_OBJ) -L. -ldramsim_wrapper_lib -Wl,-rpath=.

libdramsim_wrapper_lib.so: $(DRAMSIM3_LIB_SO) $(DRAMSIM3_WRAPPER_OBJ)
	$(CXX) -shared -o $(DRAMSIM3_WRAPPER_LIB) $(DRAMSIM3_WRAPPER_OBJ) -L./DRAMsim3 -ldramsim3 -Wl,-rpath=./DRAMsim3

libdramsim3.so:
	cd DRAMsim3/ && $(MAKE) $(DRAMSIM3_LIB_SO) && cd ..

libramulator_wrapper_c_connector.so: $(RAMULATOR_WRAPPER_LIB) $(RAMULATOR_WRAPPER_C_CONNECTOR_OBJ)
	$(CXX) -shared -o $(RAMULATOR_WRAPPER_C_CONNECTOR_LIB) $(RAMULATOR_WRAPPER_C_CONNECTOR_OBJ) -L. -lramulator_wrapper_lib -Wl,-rpath=.

libramulator_wrapper_lib.so: $(RAMULATOR_LIB_SO) $(RAMULATOR_WRAPPER_OBJ)
	$(CXX) -shared -o $(RAMULATOR_WRAPPER_LIB) $(RAMULATOR_WRAPPER_OBJ) -L./ramulator -lramulator -Wl,-rpath=./ramulator

libramulator.so:
	cd ramulator && $(MAKE) $(RAMULATOR_LIB_SO) && cd ..

riscvsim.o: $(SIM_OBJS)
	$(LD) -r -o $@ $^ $(SIM_LIBS)

sim-stats-display: stats_display.o
	$(CC) -o sim-stats-display stats_display.o -lrt

marss-riscv$(EXE): $(SIM_OBJ_FILE) $(DRAMSIM3_WRAPPER_C_CONNECTOR_LIB) $(RAMULATOR_WRAPPER_C_CONNECTOR_LIB) $(EMU_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(EMU_LIBS) -L. -ldramsim_wrapper_c_connector -Wl,-rpath=. -L. -lramulator_wrapper_c_connector -Wl,-rpath=.

riscv_cpu.o: riscv_cpu.c
	$(CC) $(CFLAGS) -DMAX_XLEN=$(CONFIG_XLEN) -c -o $@ $<

build_filelist: build_filelist.o fs_utils.o cutils.o
	$(CC) $(LDFLAGS) -o $@ $^ -lm

splitimg: splitimg.o
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.cpp
	$(CXX) -g -DMAX_XLEN=$(CONFIG_XLEN) $(DRAMSIM3_INC) -I./ramulator/src -fpic -shared -c -std=c++11 -o $@ $<

clean:
	rm -f *.o *.so *.d *~ $(PROGS) slirp/*.o slirp/*.d slirp/*~ riscvsim/bpu/*.d riscvsim/core/*.d  riscvsim/decoder/*.d riscvsim/utils/*.d  riscvsim/memory_hierarchy/*.d $(SIM_OBJS)
	rm -f $(RAMULATOR_WRAPPER_C_CONNECTOR_OBJ) $(RAMULATOR_WRAPPER_OBJ)
	cd ramulator && $(MAKE) clean && cd ..
	rm -f $(DRAMSIM3_WRAPPER_C_CONNECTOR_OBJ) $(DRAMSIM3_WRAPPER_OBJ)
	cd DRAMsim3 && $(MAKE) clean && cd ..

-include $(wildcard *.d)
-include $(wildcard slirp/*.d)
