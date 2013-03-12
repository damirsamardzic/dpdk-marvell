#   BSD LICENSE
# 
#   Copyright(c) 2010-2013 Intel Corporation. All rights reserved.
#   All rights reserved.
# 
#   Redistribution and use in source and binary forms, with or without 
#   modification, are permitted provided that the following conditions 
#   are met:
# 
#     * Redistributions of source code must retain the above copyright 
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright 
#       notice, this list of conditions and the following disclaimer in 
#       the documentation and/or other materials provided with the 
#       distribution.
#     * Neither the name of Intel Corporation nor the names of its 
#       contributors may be used to endorse or promote products derived 
#       from this software without specific prior written permission.
# 
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 

#
# machine:
#
#   - can define ARCH variable (overriden by cmdline value)
#   - can define CROSS variable (overriden by cmdline value)
#   - define MACHINE_CFLAGS variable (overriden by cmdline value)
#   - define MACHINE_LDFLAGS variable (overriden by cmdline value)
#   - define MACHINE_ASFLAGS variable (overriden by cmdline value)
#   - can define CPU_CFLAGS variable (overriden by cmdline value) that
#     overrides the one defined in arch.
#   - can define CPU_LDFLAGS variable (overriden by cmdline value) that
#     overrides the one defined in arch.
#   - can define CPU_ASFLAGS variable (overriden by cmdline value) that
#     overrides the one defined in arch.
#   - may override any previously defined variable
#

# ARCH =
# CROSS =
# MACHINE_CFLAGS =
# MACHINE_LDFLAGS =
# MACHINE_ASFLAGS =
# CPU_CFLAGS =
# CPU_LDFLAGS =
# CPU_ASFLAGS =

MACHINE_CFLAGS = -march=native
AUTO_CPUFLAGS = $(shell grep -m 1 flags /proc/cpuinfo)

# adding flags to CPUFLAGS

ifneq ($(filter $(AUTO_CPUFLAGS),sse),)
CPUFLAGS += SSE
endif

ifneq ($(filter $(AUTO_CPUFLAGS),sse2),)
CPUFLAGS += SSE2
endif

ifneq ($(filter $(AUTO_CPUFLAGS),sse3),)
CPUFLAGS += SSE3
endif

ifneq ($(filter $(AUTO_CPUFLAGS),ssse3),)
CPUFLAGS += SSSE3
endif

ifneq ($(filter $(AUTO_CPUFLAGS),sse4_1),)
CPUFLAGS += SSE4_1
endif

ifneq ($(filter $(AUTO_CPUFLAGS),sse4_2),)
CPUFLAGS += SSE4_2
endif

ifneq ($(filter $(AUTO_CPUFLAGS),aes),)
CPUFLAGS += AES
endif

ifneq ($(filter $(AUTO_CPUFLAGS),pclmulqdq),)
CPUFLAGS += PCLMULQDQ
endif

ifneq ($(filter $(AUTO_CPUFLAGS),avx),)
CPUFLAGS += AVX
endif

ifneq ($(filter $(AUTO_CPUFLAGS),rdrnd),)
CPUFLAGS += RDRAND
endif

ifneq ($(filter $(AUTO_CPUFLAGS),fsgsbase),)
CPUFLAGS += FSGSBASE
endif

ifneq ($(filter $(AUTO_CPUFLAGS),f16c),)
CPUFLAGS += F16C
endif
