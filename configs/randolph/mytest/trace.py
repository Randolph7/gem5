# Copyright (c) 2015, 2023 Arm Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Basic elastic traces replay script that configures a Trace CPU

import argparse

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.params import NULL
from m5.util import (
    addToPath,
    fatal,
    warn,
)

from gem5.isas import ISA

addToPath("../../")

from common import (
    MemConfig,
    Options,
    Simulation,
    CacheConfig,
    CpuConfig,
    ObjectList,
    
)
from common.Caches import *
from common.cpu2000 import *
from common.FileSystemConfig import config_filesystem
from ruby import Ruby

parser = argparse.ArgumentParser()
Options.addCommonOptions(parser)
Options.MyHybridOptions(parser)  

if "--ruby" in sys.argv:
    print(
        "This script does not support Ruby configuration, mainly"
        " because Trace CPU has been tested only with classic memory system"
    )
    sys.exit(1)

args = parser.parse_args()

np = args.num_cpus

(CPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(args)

if args.num_cpus > 1:
    fatal("This script does not support multi-processor trace replay.\n")

if (args.hybrid_type == "DRAMONLY"):
    system = System(cpu=[CPUClass(cpu_id=i) for i in range(np)],
                    mem_mode = test_mem_mode,
                    mem_ranges = [AddrRange(args.mem_size)],
                    cache_line_size = args.cacheline_size)

elif (args.hybrid_type == "NVMONLY"):
    system = System(cpu=[CPUClass(cpu_id=i) for i in range(np)],
                    mem_mode = test_mem_mode,
                    mem_ranges = [AddrRange(args.nvm_size)],
                    cache_line_size = args.cacheline_size)
else :
    system = System(cpu=[CPUClass(cpu_id=i) for i in range(np)],
                    mem_mode = test_mem_mode,
                    mem_ranges = [AddrRange(args.mem_size),AddrRange(Addr(args.mem_size), size =args.nvm_size)],
                    cache_line_size = args.cacheline_size)
    args.hybrid_channel = True

# Generate the TraceCPU
# system.cpu = TraceCPU()

# Create a top-level voltage domain
system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)

# Create a source clock for the system. This is used as the clock period for
# xbar and memory
system.clk_domain = SrcClockDomain(
    clock=args.sys_clock, voltage_domain=system.voltage_domain
)

# Create a CPU voltage domain
system.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs. In case of Trace CPUs this clock
# is actually used only by the caches connected to the CPU.
system.cpu_clk_domain = SrcClockDomain(
    clock=args.cpu_clock, voltage_domain=system.cpu_voltage_domain
)

# All cpus belong to a common cpu_clk_domain, therefore running at a common
# frequency.
for cpu in system.cpu:
    cpu.clk_domain = system.cpu_clk_domain

# Assign input trace files to the Trace CPU
system.cpu.instTraceFile = args.inst_trace_file
system.cpu.dataTraceFile = args.data_trace_file

# Configure the classic memory system args
MemClass = Simulation.setMemClass(args)
system.membus = SystemXBar()
system.system_port = system.membus.cpu_side_ports

# Configure the classic cache hierarchy
CacheConfig.config_cache(args, system)

MemConfig.config_mem(args, system)

if (args.hybrid_type == "HYBRID"):
    system.mem_ctrls[0].dram.addr_mapping = args.addr_map
    system.mem_ctrls[0].nvm.addr_mapping = args.addr_map
elif(args.hybrid_type == "DRAMONLY"):
    system.mem_ctrls[0].dram.addr_mapping = args.addr_map
else:
    system.mem_ctrls[0].nvm.addr_mapping = args.addr_map

root = Root(full_system=False, system=system)
Simulation.run(args, root, system, None)
