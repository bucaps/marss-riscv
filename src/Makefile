#
# TinyEMU
# 
# Copyright (c) 2016-2018 Fabrice Bellard
#
# MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
#
# Copyright (c) 2017-2019 Gaurav Kothari {gkothar1@binghamton.edu}
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
LDFLAGS=

bindir=/usr/local/bin
INSTALL=install

PROGS+= marss-riscv$(EXE) stats-display
ifdef CONFIG_FS_NET
PROGS+=build_filelist splitimg
endif

# ===============================================================================================
# Top-level object variables needed by the simulator

# DRAMSim2 library for simulator
DRAMSIM2_OBJS+= libdramsim.so libdramsim_wrapper_cpp_lib.so libdramsim_wrapper_c_connector.so

# Top-level simulator object file
SIM_OBJ_FILE=riscvsim.o
CFLAGS+=-DMAX_XLEN=$(CONFIG_XLEN)

# Simulator Object Files
SIM_UTILS:=$(addprefix riscvsim/, circular_queue.o common_core_utils.o sim_params_stats.o)
SIM_DECODER_OBJS:=$(addprefix riscvsim/, riscv_ins_str_creator.o riscv_isa_decoder.o riscv_ins_execute.o)
SIM_BPU_OBJS:=$(addprefix riscvsim/, ras.o bht.o btb.o adaptive_predictor.o bpu.o)
SIM_CACHE_OBJS:=$(addprefix riscvsim/, mmu.o memory_controller.o cache.o )
SIM_IN_CORE_OBJS:=$(addprefix riscvsim/, inorder_frontend.o inorder_backend.o inorder.o)
SIM_CORE_OBJS:=$(addprefix riscvsim/, riscv_sim_cpu.o)
SIM_OO_CORE_OBJS:=$(addprefix riscvsim/, ooo_frontend.o ooo_branch.o ooo_lsu.o ooo_backend.o ooo.o)
SIM_OBJS:=$(SIM_UTILS) $(SIM_DECODER_OBJS) $(SIM_BPU_OBJS) $(SIM_CACHE_OBJS) $(SIM_CORE_OBJS) $(SIM_IN_CORE_OBJS) $(SIM_OO_CORE_OBJS)

# ===============================================================================================

all: $(DRAMSIM2_OBJS) $(PROGS)

EMU_OBJS:=virtio.o pci.o fs.o cutils.o iomem.o simplefb.o \
    json.o machine.o temu.o

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

# ===============================================================================================
# Objects needed by the simulator

# For DRAMSim2
libdramsim_wrapper_c_connector.so:
	$(CXX) -g -DMAX_XLEN=$(CONFIG_XLEN) -fpic -shared -o libdramsim_wrapper_c_connector.so riscvsim/dramsim_wrapper_c_connector.cpp -I./DRAMSim2 -L. -ldramsim_wrapper_cpp_lib -Wl,-rpath=. -std=c++11

libdramsim_wrapper_cpp_lib.so:
	$(CXX) -g -DMAX_XLEN=$(CONFIG_XLEN) -fpic -shared -o libdramsim_wrapper_cpp_lib.so riscvsim/dramsim_wrapper.cpp -I./DRAMSim2 -L./DRAMSim2 -ldramsim -Wl,-rpath=./DRAMSim2 -std=c++11

libdramsim.so:
	@echo "Building DRAMSim2 shared library:"
	@cd DRAMSim2/ && $(MAKE) libdramsim.so
	@cd ..
	@echo "Building MARSS-RISCV Simulator:"

# For Simulator
riscvsim.o: $(SIM_OBJS)
	$(LD) -r -o $@ $^ $(SIM_LIBS)

# Simulation statistics display utility
stats-display: stats_display.c
	$(CC) stats_display.c -o stats-display -lrt
# ===============================================================================================

marss-riscv$(EXE): $(SIM_OBJ_FILE) $(EMU_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(EMU_LIBS) -L. -ldramsim_wrapper_c_connector -Wl,-rpath=.

riscv_cpu.o: riscv_cpu.c
	$(CC) $(CFLAGS) -DMAX_XLEN=$(CONFIG_XLEN) -c -o $@ $<

build_filelist: build_filelist.o fs_utils.o cutils.o
	$(CC) $(LDFLAGS) -o $@ $^ -lm

splitimg: splitimg.o
	$(CC) $(LDFLAGS) -o $@ $^

install: $(PROGS)
	$(STRIP) $(PROGS)
	$(INSTALL) -m755 $(PROGS) "$(DESTDIR)$(bindir)"

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f *.o *.d *~ $(PROGS) slirp/*.o slirp/*.d slirp/*~ riscvsim/*.o riscvsim/*.d riscvsim/*~ $(DRAMSIM2_OBJS)

-include $(wildcard *.d)
-include $(wildcard slirp/*.d)
