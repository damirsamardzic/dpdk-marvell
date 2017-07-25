/*
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Semihalf. All rights reserved.
 *   All rights reserved.
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
#ifndef MRVL_QOS_H_
#define MRVL_QOS_H_

#include <rte_common.h>
#include <rte_config.h>

#include "mrvl_ethdev.h"

#define MRVL_PP2_TC_MAX 8
#define MRVL_CP_PER_TC 8

/* stubs */
enum pp2_ppio_outqs_sched_mode_stub {
	PP2_PPIO_SCHED_M_SP = 0,
	PP2_PPIO_SCHED_M_WRR
};

struct pp2_ppio_rate_limit_params {
	uint32_t max_burst_kfps;
	uint32_t max_burst_mbps;
	uint32_t max_throuput_kfps;
	uint32_t max_throuput_mbps;
};

/* QoS config. */
struct mrvl_qos_cfg {
	struct {
		struct pp2_ppio_rate_limit_params rate_limit_params;
		struct {
			uint8_t inq[MRVL_PP2_RXQ_MAX];
			uint8_t dscp[MRVL_CP_PER_TC];
			uint8_t pcp[MRVL_CP_PER_TC];
			uint8_t inqs;
			uint8_t dscps;
			uint8_t pcps;
		} tc[MRVL_PP2_TC_MAX];
		struct {
			struct pp2_ppio_rate_limit_params rate_limit_params;
			enum pp2_ppio_outqs_sched_mode_stub sched_mode;
			uint8_t weight;
			uint8_t rate_limit;
		} outq[MRVL_PP2_RXQ_MAX];
		uint16_t inqs;
		uint16_t outqs;

	} port[RTE_MAX_ETHPORTS];
};

/* Global QoS configuration. */
extern struct mrvl_qos_cfg *mrvl_qos_cfg;


int
mrvl_get_qoscfg(const char *key __rte_unused, const char *path,
		void *extra_args);

#endif /* MRVL_QOS_H_ */
