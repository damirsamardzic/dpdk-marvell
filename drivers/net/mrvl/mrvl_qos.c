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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <rte_common.h>
#include <rte_cfgfile.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_string_fns.h>

#include "mrvl_qos.h"

/* Parsing tokens. */
#define MRVL_TOK_DEFAULT "default"
#define MRVL_TOK_PORT "port"
#define MRVL_TOK_MAX_BURST_KFPS "max_burst_kfps"
#define MRVL_TOK_MAX_BURST_MBPS "max_burst_mbps"
#define MRVL_TOK_MAX_THROUGHPUT_KFPS "max_throughput_kfps"
#define MRVL_TOK_MAX_THROUGHPUT_MBPS "max_throughput_mbps"
#define MRVL_TOK_OUTQ "outq"
#define MRVL_TOK_INQ "inq"
#define MRVL_TOK_QOS_MODE "qos_mode"
#define MRVL_TOK_WRR "WRR"
#define MRVL_TOK_SP "SP"
#define MRVL_TOK_WEIGHT "weight"
#define MRVL_TOK_RATE_LIMIT "rate_limit"
#define MRVL_TOK_TC "tc"
#define MRVL_TOK_PCP "pcp"
#define MRVL_TOK_DSCP "dscp"

#define MAX_TOKENS 3

/* Global QoS configuration. */
struct mrvl_qos_cfg *mrvl_qos_cfg;

static int get_outq_cfg (struct rte_cfgfile *file, int port, int outq,
		struct mrvl_qos_cfg *cfg)
{
	char sec_name[32];
	const char *entry;

	snprintf(sec_name, sizeof(sec_name), "%s %d %s %d",
		MRVL_TOK_PORT, port, MRVL_TOK_OUTQ, outq);

	/* Skip non-existing */
	if (rte_cfgfile_num_sections(file, sec_name, strlen(sec_name)) <= 0)
		return 0;

	entry = rte_cfgfile_get_entry(file, sec_name,
			MRVL_TOK_MAX_BURST_KFPS);
	if (entry) {
		cfg->port[port].outq[outq].rate_limit_params.max_burst_kfps =
			(uint32_t)strtoul(entry, NULL, 0);
		cfg->port[port].outq[outq].rate_limit = 1;
	}

	entry = rte_cfgfile_get_entry(file, sec_name,
			MRVL_TOK_MAX_BURST_MBPS);
	if (entry) {
		cfg->port[port].outq[outq].rate_limit_params.max_burst_mbps =
			(uint32_t)strtoul(entry, NULL, 0);
		cfg->port[port].outq[outq].rate_limit = 1;
	}

	entry = rte_cfgfile_get_entry(file, sec_name,
			MRVL_TOK_MAX_THROUGHPUT_KFPS);
	if (entry) {
		cfg->port[port].outq[outq].rate_limit_params.max_throuput_kfps =
			(uint32_t)strtoul(entry, NULL, 0);
		cfg->port[port].outq[outq].rate_limit = 1;
	}

	entry = rte_cfgfile_get_entry(file, sec_name,
			MRVL_TOK_MAX_THROUGHPUT_MBPS);
	if (entry) {
		cfg->port[port].outq[outq].rate_limit_params.max_throuput_mbps =
			(uint32_t)strtoul(entry, NULL, 0);
		cfg->port[port].outq[outq].rate_limit = 1;
	}

	entry = rte_cfgfile_get_entry(file, sec_name,
			MRVL_TOK_QOS_MODE);
	if (entry) {
		if (strncmp(entry, MRVL_TOK_WRR, sizeof(MRVL_TOK_WRR)) == 0)
			cfg->port[port].outq[outq].sched_mode = PP2_PPIO_SCHED_M_WRR;
		else if (strncmp(entry, MRVL_TOK_SP, sizeof(MRVL_TOK_SP)) == 0)
			cfg->port[port].outq[outq].sched_mode = PP2_PPIO_SCHED_M_SP;
		else
			return -1;
	} else {
		;// Use some default?
	}

	entry = rte_cfgfile_get_entry(file, sec_name,
			MRVL_TOK_WEIGHT);
	if (entry)
		cfg->port[port].outq[outq].weight =
			(uint8_t)strtoul(entry, NULL, 0);

	if (rte_cfgfile_has_entry(file, sec_name,
			MRVL_TOK_RATE_LIMIT)) {
		cfg->port[port].outq[outq].rate_limit = 1;
	}

	if (cfg->port[port].outq[outq].rate_limit == 1) {
		/* Setup default values for unset entries. We can do this only now,
		 * because rate_limiting is finally known here for sure. */
		if (cfg->port[port].outq[outq]
		                          .rate_limit_params.max_burst_kfps == 0)
			cfg->port[port].outq[outq]
			                      .rate_limit_params.max_burst_kfps =
				cfg->port[port].rate_limit_params.max_burst_kfps;

		if (cfg->port[port].outq[outq]
		                          .rate_limit_params.max_burst_mbps == 0)
			cfg->port[port].outq[outq]
			                      .rate_limit_params.max_burst_mbps =
				cfg->port[port].rate_limit_params.max_burst_mbps;

		if (cfg->port[port].outq[outq]
		                          .rate_limit_params.max_throuput_kfps == 0)
			cfg->port[port].outq[outq]
			                      .rate_limit_params.max_throuput_kfps =
				cfg->port[port].rate_limit_params.max_throuput_kfps;

		if (cfg->port[port].outq[outq]
		                          .rate_limit_params.max_throuput_mbps == 0)
			cfg->port[port].outq[outq]
			                      .rate_limit_params.max_throuput_mbps =
				cfg->port[port].rate_limit_params.max_throuput_mbps;
		if ((cfg->port[port].outq[outq]
		                       .rate_limit_params.max_burst_kfps == 0) ||
			(cfg->port[port].outq[outq]
			                   .rate_limit_params.max_burst_mbps == 0) ||
			(cfg->port[port].outq[outq]
			                   .rate_limit_params.max_throuput_kfps == 0) ||
			(cfg->port[port].outq[outq]
			                   .rate_limit_params.max_throuput_mbps == 0))
			//Enabling rate limiting needs setting all values
			rte_exit(EXIT_FAILURE,
					"Default values for port %d needed but not set!\n", port);
	}

	return 0;
}

/**
 * Gets entry values and places them in table.
 *
 * Entry can be anything, e.g. "1 2-3 5 6 7-9". This needs to be converted to
 * table entries, respectively: {1,2,3,5,6,7,8,9}.
 * As all result tables are always 1-byte long, we won't overcomplicate
 * the function, but we'll keep API generic, check if someone hasn't changed
 * element size and make it simple to extend to other sizes.
 */
static int
get_entry_values(const char *entry, uint8_t *tab,
	size_t elem_sz, uint8_t max_elems)
{
	/* There should not be more tokens than max elements.
	 * Add 1 for error trap. */
	char *tokens[max_elems + 1];

	/* Begin, End + error trap = 3. */
	char *rng_tokens[MAX_TOKENS];
	int nb_tokens, nb_rng_tokens;
	int i;
	long beg, end;
	char val;
	int values = 0;

	if (elem_sz != 1) {
		return -1;
	}

	/* If there are more tokens than array size, rte_strsplit will
	 * not return error, just array size. */
	nb_tokens = rte_strsplit((char*)(uintptr_t)entry, strlen(entry), tokens, max_elems + 1, ' ');
	if (nb_tokens > max_elems)
		return -2;

	for (i = 0; i < nb_tokens; ++i) {
		if (strchr(tokens[i], '-') != NULL) {
			/* Split to begin and end tokens tokens.
			 * We want to catch error cases too, thus we leave option
			 * for number of tokens to be more than 2. */
			nb_rng_tokens = rte_strsplit(tokens[i], strlen(entry),
					rng_tokens, MAX_TOKENS, '-');
			if (nb_rng_tokens != 2)
				return -3;

			/* Range and sanity checks. */
			beg = (char)strtol(rng_tokens[0], NULL, 0);
			end = (char)strtol(rng_tokens[1], NULL, 0);
			if ((beg < 0 )||(beg > UCHAR_MAX)||(end < 0 )||(end > UCHAR_MAX)
					|| (end < beg ))
				return -4;

			for (val = beg; val <= end; ++val) {
				*tab = val;
				tab = RTE_PTR_ADD(tab, elem_sz);
				++values;
				if (values >= max_elems)
					return -5;
			}
		} else {
			val = (char)strtol(tokens[i], NULL, 0);
			*tab = val;
			tab = RTE_PTR_ADD(tab, elem_sz);
			++values;
			if (values >= max_elems)
				return -6;
		}
	}

	return values;
}

static int get_tc_cfg (struct rte_cfgfile *file, int port, int tc,
		struct mrvl_qos_cfg *cfg)
{
	char sec_name[32];
	const char *entry;
	int n;

	snprintf(sec_name, sizeof(sec_name), "%s %d %s %d",
		MRVL_TOK_PORT, port, MRVL_TOK_TC, tc);

	/* Skip non-existing */
	if (rte_cfgfile_num_sections(file, sec_name, strlen(sec_name)) <= 0)
		return 0;

	entry = rte_cfgfile_get_entry(file, sec_name, MRVL_TOK_INQ);
	if (entry) {
		n = get_entry_values(entry,
			cfg->port[port].tc[tc].inq,
			sizeof(cfg->port[port].tc[tc].inq[0]),
			RTE_DIM(cfg->port[port].tc[tc].inq));
		if (n < 0)
			return n;
		cfg->port[port].tc[tc].inqs = n;
	}

	entry = rte_cfgfile_get_entry(file, sec_name, MRVL_TOK_PCP);
	if (entry) {
		n = get_entry_values(entry,
			cfg->port[port].tc[tc].pcp,
			sizeof(cfg->port[port].tc[tc].pcp[0]),
			RTE_DIM(cfg->port[port].tc[tc].pcp));
		if (n < 0)
			return n;
		cfg->port[port].tc[tc].pcps = n;
	}

	entry = rte_cfgfile_get_entry(file, sec_name, MRVL_TOK_DSCP);
	if (entry) {
		n = get_entry_values(entry,
			cfg->port[port].tc[tc].dscp,
			sizeof(cfg->port[port].tc[tc].dscp[0]),
			RTE_DIM(cfg->port[port].tc[tc].dscp));
		if (n < 0)
			return n;
		cfg->port[port].tc[tc].dscps = n;
	}
	return 0;
}

int
mrvl_get_qoscfg(const char *key __rte_unused, const char *path,
		void *extra_args)
{
	struct mrvl_qos_cfg **cfg = extra_args;
	struct rte_cfgfile *file = rte_cfgfile_load(path, 0);
	int n, i;
	const char *entry;
	char sec_name[32];
	int rate_limit, ret;

	if (file == NULL)
		rte_exit(EXIT_FAILURE, "Cannot load configuration %s\n", path);

	/* This is not accessed on the fast path, so we can ignore socket. */
	*cfg = rte_zmalloc("mrvl_qos_cfg", sizeof(struct mrvl_qos_cfg), 0);
	if (*cfg == NULL)
		rte_exit(EXIT_FAILURE, "Cannot allocate configuration %s\n", path);

	n = rte_cfgfile_num_sections(file, MRVL_TOK_PORT,
		sizeof(MRVL_TOK_PORT) - 1);

	if (n == 0) {
		/* This is weird, but not bad. */
		RTE_LOG(WARNING, PMD, "Empty configuration file?\n");
		return 0;
	}

	for (n = 0; n < ports_nb; ++n) {
		snprintf(sec_name, sizeof(sec_name), "%s %d %s",
			MRVL_TOK_PORT, n, MRVL_TOK_DEFAULT);

		/* Skip non-existing */
		if (rte_cfgfile_num_sections(file, sec_name, strlen(sec_name)) <= 0)
			continue;

		rate_limit = 0;

		entry = rte_cfgfile_get_entry(file, sec_name, MRVL_TOK_MAX_BURST_KFPS);
		if (entry) {
			(*cfg)->port[n].rate_limit_params.max_burst_kfps =
				(uint32_t)strtoul(entry, NULL, 0);
			rate_limit = 1;
		}

		entry = rte_cfgfile_get_entry(file, sec_name, MRVL_TOK_MAX_BURST_MBPS);
		if (entry) {
			(*cfg)->port[n].rate_limit_params.max_burst_mbps =
				(uint32_t)strtoul(entry, NULL, 0);
			rate_limit = 1;
		}

		entry = rte_cfgfile_get_entry(file, sec_name,
				MRVL_TOK_MAX_THROUGHPUT_KFPS);
		if (entry) {
			(*cfg)->port[n].rate_limit_params.max_throuput_kfps =
				(uint32_t)strtoul(entry, NULL, 0);
			rate_limit = 1;
		}

		entry = rte_cfgfile_get_entry(file, sec_name,
				MRVL_TOK_MAX_THROUGHPUT_MBPS);
		if (entry) {
			(*cfg)->port[n].rate_limit_params.max_throuput_mbps =
				(uint32_t)strtoul(entry, NULL, 0);
			rate_limit = 1;
		}

		if ((rate_limit == 1) &&
			(((*cfg)->port[n].rate_limit_params.max_burst_kfps == 0) ||
				((*cfg)->port[n].rate_limit_params.max_burst_mbps == 0) ||
				((*cfg)->port[n].rate_limit_params.max_throuput_kfps == 0) ||
				((*cfg)->port[n].rate_limit_params.max_throuput_mbps == 0)))
			//Enabling rate limiting needs setting all values
			rte_exit(EXIT_FAILURE, "Default values for port %d not set!\n", n);

		for (i = 0; i < MRVL_PP2_RXQ_MAX; ++i)
			if ((ret = get_outq_cfg(file, n, i, *cfg)) < 0)
				rte_exit(EXIT_FAILURE, "Error %d parsing port %d outq %d!\n",
					ret, n, i);

		for (i = 0; i < MRVL_PP2_TC_MAX; ++i)
			if ((ret = get_tc_cfg(file, n, i, *cfg)) < 0)
				rte_exit(EXIT_FAILURE, "Error %d parsing port %d tc %d!\n",
					ret, n, i);
	}

	return 0;
}

