# RISCV emulator
# 
# Copyright (c) 2016-2017 Fabrice Bellard
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

# Set XLEN and FLEN Size
CONFIG_XLEN=32
CONFIG_FLEN=64

# Keep blank to expand compilation messages else '@' to hide them
COMPILE_DEBUG=@

# if set, network filesystem is enabled. libcurl and libcrypto
# (openssl) must be installed.
CONFIG_FS_NET=y
# SDL support (optional)
CONFIG_SDL=y

# user space network redirector
CONFIG_SLIRP=y

ifdef CONFIG_WIN32
CROSS_PREFIX=i686-w64-mingw32-
EXE=.exe
else
CROSS_PREFIX=
EXE=
endif
CC=$(CROSS_PREFIX)gcc
STRIP=$(CROSS_PREFIX)strip
CFLAGS=-O3 -g -Wall -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -MMD
CFLAGS+=-D_GNU_SOURCE -DCONFIG_VERSION=\"$(shell cat VERSION)\"
LDFLAGS=

bindir=/usr/local/bin
INSTALL=install

# RISCV Core simulator object file
RISCVSIM_OBJ_FILE=

RISCVSIM_OBJ_FILE+=riscvsim.o
CFLAGS+=-DRV$(CONFIG_XLEN) -DSIM_FLEN=$(CONFIG_FLEN)

PROGS+= marss-riscv$(EXE) stats-display

ifndef CONFIG_WIN32
PROGS+=
ifdef CONFIG_FS_NET
PROGS+=build_filelist splitimg
endif
endif

PROGS+=

all: clean $(PROGS) 

stats-display:
	$(CC) stats_display.c -o stats-display -lrt

EMU_OBJS:=virtio.o pci.o fs.o cutils.o iomem.o simplefb.o \
    json.o machine.o sim_params_stats.o

# RISCV simulator object files
SIM_UTILS:=$(addprefix riscvsim/, circular_queue.o common_core_utils.o)
SIM_DECODER_OBJS:=$(addprefix riscvsim/, riscv_ins_str_creator.o riscv_isa_decoder.o riscv_ins_execute.o)
SIM_BPU_OBJS:=$(addprefix riscvsim/, btb.o adaptive_predictor.o bpu.o)
SIM_CACHE_OBJS:=$(addprefix riscvsim/, mmu.o dram.o memory_controller.o cache.o )
SIM_IN_CORE_OBJS:=$(addprefix riscvsim/, inorder_frontend.o inorder_backend.o inorder.o)
SIM_CORE_OBJS:=$(addprefix riscvsim/, riscv_sim_cpu.o)
SIM_OO_CORE_OBJS:=$(addprefix riscvsim/, ooo_frontend.o ooo_branch.o ooo_lsu.o ooo_backend.o ooo.o)
RISCV_SIM_OBJS:=$(SIM_UTILS) $(SIM_DECODER_OBJS) $(SIM_BPU_OBJS) $(SIM_CACHE_OBJS) $(SIM_CORE_OBJS) $(SIM_IN_CORE_OBJS) $(SIM_OO_CORE_OBJS)

ifdef CONFIG_SLIRP
CFLAGS+=-DCONFIG_SLIRP
EMU_OBJS+=$(addprefix slirp/, bootp.o ip_icmp.o mbuf.o slirp.o tcp_output.o cksum.o ip_input.o misc.o socket.o tcp_subr.o udp.o if.o ip_output.o sbuf.o tcp_input.o tcp_timer.o)
endif

ifndef CONFIG_WIN32
EMU_OBJS+=fs_disk.o
EMU_LIBS=-lrt
endif
ifdef CONFIG_FS_NET
CFLAGS+=-DCONFIG_FS_NET
EMU_OBJS+=fs_net.o fs_wget.o fs_utils.o block_net.o
EMU_LIBS+=-lcurl -lcrypto
endif # CONFIG_FS_NET
ifdef CONFIG_SDL
EMU_LIBS+=-lSDL
EMU_OBJS+=sdl.o
CFLAGS+=-DCONFIG_SDL
endif # CONFIG_SDL

EMU_LIBS+=-lrt -lpthread -lm

RISCVEMU_OBJS:=$(EMU_OBJS) riscvemu.o riscv_machine.o softfp.o

riscvsim.o: $(RISCV_SIM_OBJS)
	$(LD) -r -o $@ $^ $(SIM_LIBS)

marss-riscv$(EXE): $(RISCVSIM_OBJ_FILE) riscv_cpu.o $(RISCVEMU_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(EMU_LIBS)

riscvemu.o: riscvemu.c
	$(COMPILE_DEBUG)$(CC) $(CFLAGS) -DCONFIG_XLEN=\"$(CONFIG_XLEN)\" -DCONFIG_FLEN=\"$(CONFIG_FLEN)\" -c -o $@ $<
	$(COMPILE_DEBUG)echo "CC $<"

riscv_cpu.o: riscv_cpu.c
	$(COMPILE_DEBUG)$(CC) $(CFLAGS) -c -o $@ $<
	$(COMPILE_DEBUG)echo "CC $<"

build_filelist: build_filelist.o fs_utils.o cutils.o
	$(CC) $(LDFLAGS) -o $@ $^ -lm

splitimg: splitimg.o
	$(CC) $(LDFLAGS) -o $@ $^

install: $(PROGS)
	$(INSTALL) -m755 $(PROGS) "$(DESTDIR)$(bindir)"

%.o: %.c
	$(COMPILE_DEBUG)$(CC) $(CFLAGS) -c -o $@ $<
	$(COMPILE_DEBUG)echo "CC $<"

clean:
	rm -f *.o *.d *~ $(PROGS) slirp/*.o slirp/*.d slirp/*~ riscvsim/*.o riscvsim/*.d riscvsim/*~ 

-include $(wildcard *.d)
-include $(wildcard slirp/*.d)
