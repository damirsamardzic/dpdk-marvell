/*
 *   BSD LICENSE
 *
 *   Copyright (C) Semihalf 2017.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Semihalf nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RTE_MRVL_COMPAT_H_
#define RTE_MRVL_COMPAT_H_

#define SAM_HW_RING_SIZE       256
#define SAM_SA_DMABUF_SIZE (64 * 4)


typedef uint8_t		u8;
typedef uint16_t	u16;
typedef uint32_t	u32;
typedef uint64_t	u64;
typedef int8_t		s8;
typedef int16_t		s16;
typedef int32_t		s32;
typedef int64_t		s64;

/** parameters for CIO instance */
struct sam_cio_params {
        const char *match; /**< SAM HW string in DTS file. e.g. "cio-0:0" */
        u32 size;          /**< ring size in number of descriptors */
        u32 num_sessions;  /**< number of supported sessions */
        u32 max_buf_size;  /**< maximum buffer size [in bytes] */
};

struct sam_cio {
	u8  id;				/* ring id in SAM HW unit */
	struct sam_cio_params params;
//	struct sam_cio_op *operations;	/* array of operations */
//	struct sam_sa *sessions;	/* array of sessions */
//	struct sam_hw_ring hw_ring;
	u32 next_request;
	u32 next_result;
};

int sam_cio_init(struct sam_cio_params *params, struct sam_cio **cio);

int sam_cio_deinit(struct sam_cio *cio);

int mv_sys_dma_mem_init(u64 size);

#endif /* RTE_MRVL_COMPAT_H_ */
