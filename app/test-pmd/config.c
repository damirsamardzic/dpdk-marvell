/*-
 *   BSD LICENSE
 * 
 *   Copyright(c) 2010-2012 Intel Corporation. All rights reserved.
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
 *     * Neither the name of Intel Corporation nor the names of its 
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
 * 
 *  version: DPDK.L.1.2.3-3
 */

#include <stdarg.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <inttypes.h>

#include <sys/queue.h>

#include <rte_common.h>
#include <rte_byteorder.h>
#include <rte_debug.h>
#include <rte_log.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_launch.h>
#include <rte_tailq.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_lcore.h>
#include <rte_atomic.h>
#include <rte_branch_prediction.h>
#include <rte_ring.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_interrupts.h>
#include <rte_pci.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_string_fns.h>

#include "testpmd.h"

static void
print_ethaddr(const char *name, struct ether_addr *eth_addr)
{
	printf("%s%02X:%02X:%02X:%02X:%02X:%02X", name,
	       eth_addr->addr_bytes[0],
	       eth_addr->addr_bytes[1],
	       eth_addr->addr_bytes[2],
	       eth_addr->addr_bytes[3],
	       eth_addr->addr_bytes[4],
	       eth_addr->addr_bytes[5]);
}

void
nic_stats_display(portid_t port_id)
{
	struct rte_eth_stats stats;

	static const char *nic_stats_border = "########################";

	if (port_id >= nb_ports) {
		printf("Invalid port, range is [0, %d]\n", nb_ports - 1);
		return;
	}
	rte_eth_stats_get(port_id, &stats);
	printf("\n  %s NIC statistics for port %-2d %s\n",
	       nic_stats_border, port_id, nic_stats_border);
	printf("  RX-packets: %-10"PRIu64" RX-errors: %-10"PRIu64"RX-bytes: "
	       "%-"PRIu64"\n"
	       "  TX-packets: %-10"PRIu64" TX-errors: %-10"PRIu64"TX-bytes: "
	       "%-"PRIu64"\n",
	       stats.ipackets, stats.ierrors, stats.ibytes,
	       stats.opackets, stats.oerrors, stats.obytes);

	/* stats fdir */
	if (fdir_conf.mode != RTE_FDIR_MODE_NONE)
		printf("  Fdirmiss:   %-10"PRIu64" Fdirmatch: %-10"PRIu64"\n",
		       stats.fdirmiss,
		       stats.fdirmatch);

	printf("  %s############################%s\n",
	       nic_stats_border, nic_stats_border);
}

void
nic_stats_clear(portid_t port_id)
{
	if (port_id >= nb_ports) {
		printf("Invalid port, range is [0, %d]\n", nb_ports - 1);
		return;
	}
	rte_eth_stats_reset(port_id);
	printf("\n  NIC statistics for port %d cleared\n", port_id);
}

void
port_infos_display(portid_t port_id)
{
	struct rte_port *port;
	struct rte_eth_link link;
	static const char *info_border = "*********************";

	if (port_id >= nb_ports) {
		printf("Invalid port, range is [0, %d]\n", nb_ports - 1);
		return;
	}
	port = &ports[port_id];
	rte_eth_link_get(port_id, &link);
	printf("\n%s Infos for port %-2d %s\n",
	       info_border, port_id, info_border);
	print_ethaddr("MAC address: ", &port->eth_addr);
	printf("\nLink status: %s\n", (link.link_status) ? ("up") : ("down"));
	printf("Link speed: %u Mbps\n", (unsigned) link.link_speed);
	printf("Link duplex: %s\n", (link.link_duplex == ETH_LINK_FULL_DUPLEX) ?
	       ("full-duplex") : ("half-duplex"));
	printf("Promiscuous mode: %s\n",
	       rte_eth_promiscuous_get(port_id) ? "enabled" : "disabled");
	printf("Allmulticast mode: %s\n",
	       rte_eth_allmulticast_get(port_id) ? "enabled" : "disabled");
	printf("Maximum number of MAC addresses: %u\n",
	       (unsigned int)(port->dev_info.max_mac_addrs));
}

static int
port_id_is_invalid(portid_t port_id)
{
	if (port_id < nb_ports)
		return 0;
	printf("Invalid port %d (must be < nb_ports=%d)\n", port_id, nb_ports);
	return 1;
}

static int
vlan_id_is_invalid(uint16_t vlan_id)
{
	if (vlan_id < 4096)
		return 0;
	printf("Invalid vlan_id %d (must be < 4096)\n", vlan_id);
	return 1;
}

static int
port_reg_off_is_invalid(portid_t port_id, uint32_t reg_off)
{
	uint64_t pci_len;

	if (reg_off & 0x3) {
		printf("Port register offset 0x%X not aligned on a 4-byte "
		       "boundary\n",
		       (unsigned)reg_off);
		return 1;
	}
	pci_len = ports[port_id].dev_info.pci_dev->mem_resource.len;
	if (reg_off >= pci_len) {
		printf("Port %d: register offset %u (0x%X) out of port PCI "
		       "resource (length=%"PRIu64")\n",
		       port_id, (unsigned)reg_off, (unsigned)reg_off,  pci_len);
		return 1;
	}
	return 0;
}

static int
reg_bit_pos_is_invalid(uint8_t bit_pos)
{
	if (bit_pos <= 31)
		return 0;
	printf("Invalid bit position %d (must be <= 31)\n", bit_pos);
	return 1;
}

#define display_port_and_reg_off(port_id, reg_off) \
	printf("port %d PCI register at offset 0x%X: ", (port_id), (reg_off))

static inline void
display_port_reg_value(portid_t port_id, uint32_t reg_off, uint32_t reg_v)
{
	display_port_and_reg_off(port_id, (unsigned)reg_off);
	printf("0x%08X (%u)\n", (unsigned)reg_v, (unsigned)reg_v);
}

void
port_reg_bit_display(portid_t port_id, uint32_t reg_off, uint8_t bit_x)
{
	uint32_t reg_v;


	if (port_id_is_invalid(port_id))
		return;
	if (port_reg_off_is_invalid(port_id, reg_off))
		return;
	if (reg_bit_pos_is_invalid(bit_x))
		return;
	reg_v = port_id_pci_reg_read(port_id, reg_off);
	display_port_and_reg_off(port_id, (unsigned)reg_off);
	printf("bit %d=%d\n", bit_x, (int) ((reg_v & (1 << bit_x)) >> bit_x));
}

void
port_reg_bit_field_display(portid_t port_id, uint32_t reg_off,
			   uint8_t bit1_pos, uint8_t bit2_pos)
{
	uint32_t reg_v;
	uint8_t  l_bit;
	uint8_t  h_bit;

	if (port_id_is_invalid(port_id))
		return;
	if (port_reg_off_is_invalid(port_id, reg_off))
		return;
	if (reg_bit_pos_is_invalid(bit1_pos))
		return;
	if (reg_bit_pos_is_invalid(bit2_pos))
		return;
	if (bit1_pos > bit2_pos)
		l_bit = bit2_pos, h_bit = bit1_pos;
	else
		l_bit = bit1_pos, h_bit = bit2_pos;

	reg_v = port_id_pci_reg_read(port_id, reg_off);
	reg_v >>= l_bit;
	if (h_bit < 31)
		reg_v &= ((1 << (h_bit - l_bit + 1)) - 1);
	display_port_and_reg_off(port_id, (unsigned)reg_off);
	printf("bits[%d, %d]=0x%0*X (%u)\n", l_bit, h_bit,
	       ((h_bit - l_bit) / 4) + 1, (unsigned)reg_v, (unsigned)reg_v);
}

void
port_reg_display(portid_t port_id, uint32_t reg_off)
{
	uint32_t reg_v;

	if (port_id_is_invalid(port_id))
		return;
	if (port_reg_off_is_invalid(port_id, reg_off))
		return;
	reg_v = port_id_pci_reg_read(port_id, reg_off);
	display_port_reg_value(port_id, reg_off, reg_v);
}

void
port_reg_bit_set(portid_t port_id, uint32_t reg_off, uint8_t bit_pos,
		 uint8_t bit_v)
{
	uint32_t reg_v;

	if (port_id_is_invalid(port_id))
		return;
	if (port_reg_off_is_invalid(port_id, reg_off))
		return;
	if (reg_bit_pos_is_invalid(bit_pos))
		return;
	if (bit_v > 1) {
		printf("Invalid bit value %d (must be 0 or 1)\n", (int) bit_v);
		return;
	}
	reg_v = port_id_pci_reg_read(port_id, reg_off);
	if (bit_v == 0)
		reg_v &= ~(1 << bit_pos);
	else
		reg_v |= (1 << bit_pos);
	port_id_pci_reg_write(port_id, reg_off, reg_v);
	display_port_reg_value(port_id, reg_off, reg_v);
}

void
port_reg_bit_field_set(portid_t port_id, uint32_t reg_off,
		       uint8_t bit1_pos, uint8_t bit2_pos, uint32_t value)
{
	uint32_t max_v;
	uint32_t reg_v;
	uint8_t  l_bit;
	uint8_t  h_bit;

	if (port_id_is_invalid(port_id))
		return;
	if (port_reg_off_is_invalid(port_id, reg_off))
		return;
	if (reg_bit_pos_is_invalid(bit1_pos))
		return;
	if (reg_bit_pos_is_invalid(bit2_pos))
		return;
	if (bit1_pos > bit2_pos)
		l_bit = bit2_pos, h_bit = bit1_pos;
	else
		l_bit = bit1_pos, h_bit = bit2_pos;

	if ((h_bit - l_bit) < 31)
		max_v = (1 << (h_bit - l_bit + 1)) - 1;
	else
		max_v = 0xFFFFFFFF;

	if (value > max_v) {
		printf("Invalid value %u (0x%x) must be < %u (0x%x)\n",
				(unsigned)value, (unsigned)value,
				(unsigned)max_v, (unsigned)max_v);
		return;
	}
	reg_v = port_id_pci_reg_read(port_id, reg_off);
	reg_v &= ~(max_v << l_bit); /* Keep unchanged bits */
	reg_v |= (value << l_bit); /* Set changed bits */
	port_id_pci_reg_write(port_id, reg_off, reg_v);
	display_port_reg_value(port_id, reg_off, reg_v);
}

void
port_reg_set(portid_t port_id, uint32_t reg_off, uint32_t reg_v)
{
	if (port_id_is_invalid(port_id))
		return;
	if (port_reg_off_is_invalid(port_id, reg_off))
		return;
	port_id_pci_reg_write(port_id, reg_off, reg_v);
	display_port_reg_value(port_id, reg_off, reg_v);
}

/*
 * RX/TX ring descriptors display functions.
 */
static int
rx_queue_id_is_invalid(queueid_t rxq_id)
{
	if (rxq_id < nb_rxq)
		return 0;
	printf("Invalid RX queue %d (must be < nb_rxq=%d)\n", rxq_id, nb_rxq);
	return 1;
}

static int
tx_queue_id_is_invalid(queueid_t txq_id)
{
	if (txq_id < nb_txq)
		return 0;
	printf("Invalid TX queue %d (must be < nb_rxq=%d)\n", txq_id, nb_txq);
	return 1;
}

static int
rx_desc_id_is_invalid(uint16_t rxdesc_id)
{
	if (rxdesc_id < nb_rxd)
		return 0;
	printf("Invalid RX descriptor %d (must be < nb_rxd=%d)\n",
	       rxdesc_id, nb_rxd);
	return 1;
}

static int
tx_desc_id_is_invalid(uint16_t txdesc_id)
{
	if (txdesc_id < nb_txd)
		return 0;
	printf("Invalid TX descriptor %d (must be < nb_txd=%d)\n",
	       txdesc_id, nb_txd);
	return 1;
}

static const struct rte_memzone *
ring_dma_zone_lookup(const char *ring_name, uint8_t port_id, uint16_t q_id)
{
	char mz_name[RTE_MEMZONE_NAMESIZE];
	const struct rte_memzone *mz;

	rte_snprintf(mz_name, sizeof(mz_name), "%s_%s_%d_%d",
		 ports[port_id].dev_info.driver_name, ring_name, port_id, q_id);
	mz = rte_memzone_lookup(mz_name);
	if (mz == NULL)
		printf("%s ring memory zoneof (port %d, queue %d) not"
		       "found (zone name = %s\n",
		       ring_name, port_id, q_id, mz_name);
	return (mz);
}

union igb_ring_dword {
	uint64_t dword;
	struct {
		uint32_t hi;
		uint32_t lo;
	} words;
};

struct igb_ring_desc {
	union igb_ring_dword lo_dword;
	union igb_ring_dword hi_dword;
};

static void
ring_descriptor_display(const struct rte_memzone *ring_mz, uint16_t desc_id)
{
	struct igb_ring_desc *ring;
	struct igb_ring_desc rd;

	ring = (struct igb_ring_desc *) ring_mz->addr;
	rd.lo_dword = rte_le_to_cpu_64(ring[desc_id].lo_dword);
	rd.hi_dword = rte_le_to_cpu_64(ring[desc_id].hi_dword);
	printf("    0x%08X - 0x%08X / 0x%08X - 0x%08X\n",
		(unsigned)rd.lo_dword.words.lo, (unsigned)rd.lo_dword.words.hi,
		(unsigned)rd.hi_dword.words.lo, (unsigned)rd.hi_dword.words.hi);
}

void
rx_ring_desc_display(portid_t port_id, queueid_t rxq_id, uint16_t rxd_id)
{
	const struct rte_memzone *rx_mz;

	if (port_id_is_invalid(port_id))
		return;
	if (rx_queue_id_is_invalid(rxq_id))
		return;
	if (rx_desc_id_is_invalid(rxd_id))
		return;
	rx_mz = ring_dma_zone_lookup("rx_ring", port_id, rxq_id);
	if (rx_mz == NULL)
		return;
	ring_descriptor_display(rx_mz, rxd_id);
}

void
tx_ring_desc_display(portid_t port_id, queueid_t txq_id, uint16_t txd_id)
{
	const struct rte_memzone *tx_mz;

	if (port_id_is_invalid(port_id))
		return;
	if (tx_queue_id_is_invalid(txq_id))
		return;
	if (tx_desc_id_is_invalid(txd_id))
		return;
	tx_mz = ring_dma_zone_lookup("tx_ring", port_id, txq_id);
	if (tx_mz == NULL)
		return;
	ring_descriptor_display(tx_mz, txd_id);
}

void
fwd_lcores_config_display(void)
{
	lcoreid_t lc_id;

	printf("List of forwarding lcores:");
	for (lc_id = 0; lc_id < nb_cfg_lcores; lc_id++)
		printf(" %2u", fwd_lcores_cpuids[lc_id]);
	printf("\n");
}
void
rxtx_config_display(void)
{
	printf("  %s packet forwarding - CRC stripping %s - "
	       "packets/burst=%d\n", cur_fwd_eng->fwd_mode_name,
	       rx_mode.hw_strip_crc ? "enabled" : "disabled",
	       nb_pkt_per_burst);

	if (cur_fwd_eng == &tx_only_engine)
		printf("  packet len=%u - nb packet segments=%d\n",
				(unsigned)tx_pkt_length, (int) tx_pkt_nb_segs);

	printf("  nb forwarding cores=%d - nb forwarding ports=%d\n",
	       nb_fwd_lcores, nb_fwd_ports);
	printf("  RX queues=%d - RX desc=%d - RX free threshold=%d\n",
	       nb_rxq, nb_rxd, rx_free_thresh);
	printf("  RX threshold registers: pthresh=%d hthresh=%d wthresh=%d\n",
	       rx_thresh.pthresh, rx_thresh.hthresh, rx_thresh.wthresh);
	printf("  TX queues=%d - TX desc=%d - TX free threshold=%d\n",
	       nb_txq, nb_txd, tx_free_thresh);
	printf("  TX threshold registers: pthresh=%d hthresh=%d wthresh=%d\n",
	       tx_thresh.pthresh, tx_thresh.hthresh, tx_thresh.wthresh);
	printf("  TX RS bit threshold=%d\n", tx_rs_thresh);
}

/*
 * Setup forwarding configuration for each logical core.
 */
static void
setup_fwd_config_of_each_lcore(struct fwd_config *cfg)
{
	streamid_t nb_fs_per_lcore;
	streamid_t nb_fs;
	streamid_t sm_id;
	lcoreid_t  nb_extra;
	lcoreid_t  nb_fc;
	lcoreid_t  nb_lc;
	lcoreid_t  lc_id;

	nb_fs = cfg->nb_fwd_streams;
	nb_fc = cfg->nb_fwd_lcores;
	if (nb_fs <= nb_fc) {
		nb_fs_per_lcore = 1;
		nb_extra = 0;
	} else {
		nb_fs_per_lcore = (streamid_t) (nb_fs / nb_fc);
		nb_extra = (lcoreid_t) (nb_fs % nb_fc);
	}
	nb_extra = (lcoreid_t) (nb_fs % nb_fc);

	nb_lc = (lcoreid_t) (nb_fc - nb_extra);
	sm_id = 0;
	for (lc_id = 0; lc_id < nb_lc; lc_id++) {
		fwd_lcores[lc_id]->stream_idx = sm_id;
		fwd_lcores[lc_id]->stream_nb = nb_fs_per_lcore;
		sm_id = (streamid_t) (sm_id + nb_fs_per_lcore);
	}

	/*
	 * Assign extra remaining streams, if any.
	 */
	nb_fs_per_lcore = (streamid_t) (nb_fs_per_lcore + 1);
	for (lc_id = 0; lc_id < nb_extra; lc_id++) {
		fwd_lcores[nb_lc + lc_id]->stream_idx = sm_id;
		fwd_lcores[nb_lc + lc_id]->stream_nb = nb_fs_per_lcore;
		sm_id = (streamid_t) (sm_id + nb_fs_per_lcore);
	}
}

static void
simple_fwd_config_setup(void)
{
	portid_t i;
	portid_t j;
	portid_t inc = 2;

	if (nb_fwd_ports % 2) {
		if (port_topology == PORT_TOPOLOGY_CHAINED) {
			inc = 1;
		}
		else {
			printf("\nWarning! Cannot handle an odd number of ports "
			       "with the current port topology. Configuration "
			       "must be changed to have an even number of ports, "
			       "or relaunch application with "
			       "--port-topology=chained\n\n");
		}
	}

	cur_fwd_config.nb_fwd_ports = (portid_t) nb_fwd_ports;
	cur_fwd_config.nb_fwd_streams =
		(streamid_t) cur_fwd_config.nb_fwd_ports;

	/*
	 * In the simple forwarding test, the number of forwarding cores
	 * must be lower or equal to the number of forwarding ports.
	 */
	cur_fwd_config.nb_fwd_lcores = (lcoreid_t) nb_fwd_lcores;
	if (cur_fwd_config.nb_fwd_lcores > cur_fwd_config.nb_fwd_ports)
		cur_fwd_config.nb_fwd_lcores =
			(lcoreid_t) cur_fwd_config.nb_fwd_ports;
	setup_fwd_config_of_each_lcore(&cur_fwd_config);

	for (i = 0; i < cur_fwd_config.nb_fwd_ports; i = (portid_t) (i + inc)) {
		j = (portid_t) ((i + 1) % cur_fwd_config.nb_fwd_ports);
		fwd_streams[i]->rx_port   = fwd_ports_ids[i];
		fwd_streams[i]->rx_queue  = 0;
		fwd_streams[i]->tx_port   = fwd_ports_ids[j];
		fwd_streams[i]->tx_queue  = 0;
		fwd_streams[i]->peer_addr = j;

		if (port_topology == PORT_TOPOLOGY_PAIRED) {
			fwd_streams[j]->rx_port   = fwd_ports_ids[j];
			fwd_streams[j]->rx_queue  = 0;
			fwd_streams[j]->tx_port   = fwd_ports_ids[i];
			fwd_streams[j]->tx_queue  = 0;
			fwd_streams[j]->peer_addr = i;
		}
	}
}

/**
 * For the RSS forwarding test, each core is assigned on every port a transmit
 * queue whose index is the index of the core itself. This approach limits the
 * maximumm number of processing cores of the RSS test to the maximum number of
 * TX queues supported by the devices.
 *
 * Each core is assigned a single stream, each stream being composed of
 * a RX queue to poll on a RX port for input messages, associated with
 * a TX queue of a TX port where to send forwarded packets.
 * All packets received on the RX queue of index "RxQj" of the RX port "RxPi"
 * are sent on the TX queue "TxQl" of the TX port "TxPk" according to the two
 * following rules:
 *    - TxPk = (RxPi + 1) if RxPi is even, (RxPi - 1) if RxPi is odd
 *    - TxQl = RxQj
 */
static void
rss_fwd_config_setup(void)
{
	portid_t   rxp;
	portid_t   txp;
	queueid_t  rxq;
	queueid_t  nb_q;
	lcoreid_t  lc_id;

	nb_q = nb_rxq;
	if (nb_q > nb_txq)
		nb_q = nb_txq;
	cur_fwd_config.nb_fwd_lcores = (lcoreid_t) nb_fwd_lcores;
	cur_fwd_config.nb_fwd_ports = nb_fwd_ports;
	cur_fwd_config.nb_fwd_streams =
		(streamid_t) (nb_q * cur_fwd_config.nb_fwd_ports);
	if (cur_fwd_config.nb_fwd_streams > cur_fwd_config.nb_fwd_lcores)
		cur_fwd_config.nb_fwd_streams =
			(streamid_t)cur_fwd_config.nb_fwd_lcores;
	else
		cur_fwd_config.nb_fwd_lcores =
			(lcoreid_t)cur_fwd_config.nb_fwd_streams;
	setup_fwd_config_of_each_lcore(&cur_fwd_config);
	rxp = 0; rxq = 0;
	for (lc_id = 0; lc_id < cur_fwd_config.nb_fwd_lcores; lc_id++) {
		struct fwd_stream *fs;

		fs = fwd_streams[lc_id];
		if ((rxp & 0x1) == 0)
			txp = (portid_t) (rxp + 1);
		else
			txp = (portid_t) (rxp - 1);
		fs->rx_port = fwd_ports_ids[rxp];
		fs->rx_queue = rxq;
		fs->tx_port = fwd_ports_ids[txp];
		fs->tx_queue = rxq;
		fs->peer_addr = fs->tx_port;
		rxq = (queueid_t) (rxq + 1);
		if (rxq < nb_q)
			continue;
		/*
		 * rxq == nb_q
		 * Restart from RX queue 0 on next RX port
		 */
		rxq = 0;
		if (numa_support && (nb_fwd_ports <= (nb_ports >> 1)))
			rxp = (portid_t)
				(rxp + ((nb_ports >> 1) / nb_fwd_ports));
		else
			rxp = (portid_t) (rxp + 1);
	}
}

void
fwd_config_setup(void)
{
	cur_fwd_config.fwd_eng = cur_fwd_eng;
	if ((nb_rxq > 1) && (nb_txq > 1))
		rss_fwd_config_setup();
	else
		simple_fwd_config_setup();
}

static void
pkt_fwd_config_display(struct fwd_config *cfg)
{
	struct fwd_stream *fs;
	lcoreid_t  lc_id;
	streamid_t sm_id;

	printf("%s packet forwarding - ports=%d - cores=%d - streams=%d - "
	       "NUMA support %s\n",
	       cfg->fwd_eng->fwd_mode_name,
	       cfg->nb_fwd_ports, cfg->nb_fwd_lcores, cfg->nb_fwd_streams,
	       numa_support == 1 ? "enabled" : "disabled");
	for (lc_id = 0; lc_id < cfg->nb_fwd_lcores; lc_id++) {
		printf("Logical Core %u (socket %u) forwards packets on "
		       "%d streams:",
		       fwd_lcores_cpuids[lc_id],
		       rte_lcore_to_socket_id(fwd_lcores_cpuids[lc_id]),
		       fwd_lcores[lc_id]->stream_nb);
		for (sm_id = 0; sm_id < fwd_lcores[lc_id]->stream_nb; sm_id++) {
			fs = fwd_streams[fwd_lcores[lc_id]->stream_idx + sm_id];
			printf("\n  RX P=%d/Q=%d (socket %u) -> TX "
			       "P=%d/Q=%d (socket %u) ",
			       fs->rx_port, fs->rx_queue,
			       ports[fs->rx_port].socket_id,
			       fs->tx_port, fs->tx_queue,
			       ports[fs->tx_port].socket_id);
			print_ethaddr("peer=",
				      &peer_eth_addrs[fs->peer_addr]);
		}
		printf("\n");
	}
	printf("\n");
}


void
fwd_config_display(void)
{
	fwd_config_setup();
	pkt_fwd_config_display(&cur_fwd_config);
}

void
set_fwd_lcores_list(unsigned int *lcorelist, unsigned int nb_lc)
{
	unsigned int i;
	unsigned int lcore_cpuid;
	int record_now;

	record_now = 0;
 again:
	for (i = 0; i < nb_lc; i++) {
		lcore_cpuid = lcorelist[i];
		if (! rte_lcore_is_enabled(lcore_cpuid)) {
			printf("Logical core %u not enabled\n", lcore_cpuid);
			return;
		}
		if (lcore_cpuid == rte_get_master_lcore()) {
			printf("Master core %u cannot forward packets\n",
			       lcore_cpuid);
			return;
		}
		if (record_now)
			fwd_lcores_cpuids[i] = lcore_cpuid;
	}
	if (record_now == 0) {
		record_now = 1;
		goto again;
	}
	nb_cfg_lcores = (lcoreid_t) nb_lc;
	if (nb_fwd_lcores != (lcoreid_t) nb_lc) {
		printf("previous number of forwarding cores %u - changed to "
		       "number of configured cores %u\n",
		       (unsigned int) nb_fwd_lcores, nb_lc);
		nb_fwd_lcores = (lcoreid_t) nb_lc;
	}
}

void
set_fwd_lcores_mask(uint64_t lcoremask)
{
	unsigned int lcorelist[64];
	unsigned int nb_lc;
	unsigned int i;

	if (lcoremask == 0) {
		printf("Invalid NULL mask of cores\n");
		return;
	}
	nb_lc = 0;
	for (i = 0; i < 64; i++) {
		if (! ((uint64_t)(1ULL << i) & lcoremask))
			continue;
		lcorelist[nb_lc++] = i;
	}
	set_fwd_lcores_list(lcorelist, nb_lc);
}

void
set_fwd_lcores_number(uint16_t nb_lc)
{
	if (nb_lc > nb_cfg_lcores) {
		printf("nb fwd cores %u > %u (max. number of configured "
		       "lcores) - ignored\n",
		       (unsigned int) nb_lc, (unsigned int) nb_cfg_lcores);
		return;
	}
	nb_fwd_lcores = (lcoreid_t) nb_lc;
	printf("Number of forwarding cores set to %u\n",
	       (unsigned int) nb_fwd_lcores);
}

void
set_fwd_ports_list(unsigned int *portlist, unsigned int nb_pt)
{
	unsigned int i;
	portid_t port_id;
	int record_now;

	record_now = 0;
 again:
	for (i = 0; i < nb_pt; i++) {
		port_id = (portid_t) portlist[i];
		if (port_id >= nb_ports) {
			printf("Invalid port id %u > %u\n",
			       (unsigned int) port_id,
			       (unsigned int) nb_ports);
			return;
		}
		if (record_now)
			fwd_ports_ids[i] = port_id;
	}
	if (record_now == 0) {
		record_now = 1;
		goto again;
	}
	nb_cfg_ports = (portid_t) nb_pt;
	if (nb_fwd_ports != (portid_t) nb_pt) {
		printf("previous number of forwarding ports %u - changed to "
		       "number of configured ports %u\n",
		       (unsigned int) nb_fwd_ports, nb_pt);
		nb_fwd_ports = (portid_t) nb_pt;
	}
}

void
set_fwd_ports_mask(uint64_t portmask)
{
	unsigned int portlist[64];
	unsigned int nb_pt;
	unsigned int i;

	if (portmask == 0) {
		printf("Invalid NULL mask of ports\n");
		return;
	}
	nb_pt = 0;
	for (i = 0; i < 64; i++) {
		if (! ((uint64_t)(1ULL << i) & portmask))
			continue;
		portlist[nb_pt++] = i;
	}
	set_fwd_ports_list(portlist, nb_pt);
}

void
set_fwd_ports_number(uint16_t nb_pt)
{
	if (nb_pt > nb_cfg_ports) {
		printf("nb fwd ports %u > %u (number of configured "
		       "ports) - ignored\n",
		       (unsigned int) nb_pt, (unsigned int) nb_cfg_ports);
		return;
	}
	nb_fwd_ports = (portid_t) nb_pt;
	printf("Number of forwarding ports set to %u\n",
	       (unsigned int) nb_fwd_ports);
}

void
set_nb_pkt_per_burst(uint16_t nb)
{
	if (nb > MAX_PKT_BURST) {
		printf("nb pkt per burst: %u > %u (maximum packet per burst) "
		       " ignored\n",
		       (unsigned int) nb, (unsigned int) MAX_PKT_BURST);
		return;
	}
	nb_pkt_per_burst = nb;
	printf("Number of packets per burst set to %u\n",
	       (unsigned int) nb_pkt_per_burst);
}

void
set_tx_pkt_segments(unsigned *seg_lengths, unsigned nb_segs)
{
	uint16_t tx_pkt_len;
	unsigned i;

	if (nb_segs >= (unsigned) nb_txd) {
		printf("nb segments per TX packets=%u >= nb_txd=%u - ignored\n",
		       nb_segs, (unsigned int) nb_txd);
		return;
	}

	/*
	 * Check that each segment length is greater or equal than
	 * the mbuf data sise.
	 * Check also that the total packet length is greater or equal than the
	 * size of an empty UDP/IP packet (sizeof(struct ether_hdr) + 20 + 8).
	 */
	tx_pkt_len = 0;
	for (i = 0; i < nb_segs; i++) {
		if (seg_lengths[i] > (unsigned) mbuf_data_size) {
			printf("length[%u]=%u > mbuf_data_size=%u - give up\n",
			       i, seg_lengths[i], (unsigned) mbuf_data_size);
			return;
		}
		tx_pkt_len = (uint16_t)(tx_pkt_len + seg_lengths[i]);
	}
	if (tx_pkt_len < (sizeof(struct ether_hdr) + 20 + 8)) {
		printf("total packet length=%u < %d - give up\n",
				(unsigned) tx_pkt_len,
				(int)(sizeof(struct ether_hdr) + 20 + 8));
		return;
	}

	for (i = 0; i < nb_segs; i++)
		tx_pkt_seg_lengths[i] = (uint16_t) seg_lengths[i];

	tx_pkt_length  = tx_pkt_len;
	tx_pkt_nb_segs = (uint8_t) nb_segs;
}

void
set_pkt_forwarding_mode(const char *fwd_mode_name)
{
	struct fwd_engine *fwd_eng;
	unsigned i;

	i = 0;
	while ((fwd_eng = fwd_engines[i]) != NULL) {
		if (! strcmp(fwd_eng->fwd_mode_name, fwd_mode_name)) {
			printf("Set %s packet forwarding mode\n",
			       fwd_mode_name);
			cur_fwd_eng = fwd_eng;
			return;
		}
		i++;
	}
	printf("Invalid %s packet forwarding mode\n", fwd_mode_name);
}

void
set_verbose_level(uint16_t vb_level)
{
	printf("Change verbose level from %u to %u\n",
	       (unsigned int) verbose_level, (unsigned int) vb_level);
	verbose_level = vb_level;
}

void
rx_vlan_filter_set(portid_t port_id, uint16_t vlan_id, int on)
{
	int diag;

	if (port_id_is_invalid(port_id))
		return;
	if (vlan_id_is_invalid(vlan_id))
		return;
	diag = rte_eth_dev_vlan_filter(port_id, vlan_id, on);
	if (diag == 0)
		return;
	printf("rte_eth_dev_vlan_filter(port_pi=%d, vlan_id=%d, on=%d) failed "
	       "diag=%d\n",
	       port_id, vlan_id, on, diag);
}

void
rx_vlan_all_filter_set(portid_t port_id, int on)
{
	uint16_t vlan_id;

	if (port_id_is_invalid(port_id))
		return;
	for (vlan_id = 0; vlan_id < 4096; vlan_id++)
		rx_vlan_filter_set(port_id, vlan_id, on);
}

void
tx_vlan_set(portid_t port_id, uint16_t vlan_id)
{
	if (port_id_is_invalid(port_id))
		return;
	if (vlan_id_is_invalid(vlan_id))
		return;
	ports[port_id].tx_ol_flags |= PKT_TX_VLAN_PKT;
	ports[port_id].tx_vlan_id = vlan_id;
}

void
tx_vlan_reset(portid_t port_id)
{
	if (port_id_is_invalid(port_id))
		return;
	ports[port_id].tx_ol_flags &= ~PKT_TX_VLAN_PKT;
}

void
tx_cksum_set(portid_t port_id, uint8_t cksum_mask)
{
	uint16_t tx_ol_flags;
	if (port_id_is_invalid(port_id))
		return;
	/* Clear last 4 bits and then set L3/4 checksum mask again */
	tx_ol_flags = (uint16_t) (ports[port_id].tx_ol_flags & 0xFFF0);
	ports[port_id].tx_ol_flags = (uint16_t) ((cksum_mask & 0xf) | tx_ol_flags);
}

void
fdir_add_signature_filter(portid_t port_id, uint8_t queue_id,
			  struct rte_fdir_filter *fdir_filter)
{
	int diag;

	if (port_id_is_invalid(port_id))
		return;

	diag = rte_eth_dev_fdir_add_signature_filter(port_id, fdir_filter,
						     queue_id);
	if (diag == 0)
		return;

	printf("rte_eth_dev_fdir_add_signature_filter for port_id=%d failed "
	       "diag=%d\n", port_id, diag);
}

void
fdir_update_signature_filter(portid_t port_id, uint8_t queue_id,
			     struct rte_fdir_filter *fdir_filter)
{
	int diag;

	if (port_id_is_invalid(port_id))
		return;

	diag = rte_eth_dev_fdir_update_signature_filter(port_id, fdir_filter,
							queue_id);
	if (diag == 0)
		return;

	printf("rte_eth_dev_fdir_update_signature_filter for port_id=%d failed "
	       "diag=%d\n", port_id, diag);
}

void
fdir_remove_signature_filter(portid_t port_id,
			     struct rte_fdir_filter *fdir_filter)
{
	int diag;

	if (port_id_is_invalid(port_id))
		return;

	diag = rte_eth_dev_fdir_remove_signature_filter(port_id, fdir_filter);
	if (diag == 0)
		return;

	printf("rte_eth_dev_fdir_add_signature_filter for port_id=%d failed "
	       "diag=%d\n", port_id, diag);

}

void
fdir_get_infos(portid_t port_id)
{
	struct rte_eth_fdir fdir_infos;

	static const char *fdir_stats_border = "########################";

	if (port_id_is_invalid(port_id))
		return;

	rte_eth_dev_fdir_get_infos(port_id, &fdir_infos);

	printf("\n  %s FDIR infos for port %-2d %s\n",
	       fdir_stats_border, port_id, fdir_stats_border);

	printf("  collision: %-10"PRIu64" free: %-10"PRIu64"\n"
	       "  maxhash: %-10"PRIu64" maxlen: %-10"PRIu64"\n"
	       "  add : %-10"PRIu64"   remove : %-10"PRIu64"\n"
	       "  f_add: %-10"PRIu64" f_remove: %-10"PRIu64"\n",
	       (uint64_t)(fdir_infos.collision), (uint64_t)(fdir_infos.free),
	       (uint64_t)(fdir_infos.maxhash), (uint64_t)(fdir_infos.maxlen),
	       fdir_infos.add, fdir_infos.remove,
	       fdir_infos.f_add, fdir_infos.f_remove);
	printf("  %s############################%s\n",
	       fdir_stats_border, fdir_stats_border);
}

void
fdir_add_perfect_filter(portid_t port_id, uint16_t soft_id, uint8_t queue_id,
			uint8_t drop, struct rte_fdir_filter *fdir_filter)
{
	int diag;

	if (port_id_is_invalid(port_id))
		return;

	diag = rte_eth_dev_fdir_add_perfect_filter(port_id, fdir_filter,
						   soft_id, queue_id, drop);
	if (diag == 0)
		return;

	printf("rte_eth_dev_fdir_add_perfect_filter for port_id=%d failed "
	       "diag=%d\n", port_id, diag);
}

void
fdir_update_perfect_filter(portid_t port_id, uint16_t soft_id, uint8_t queue_id,
			   uint8_t drop, struct rte_fdir_filter *fdir_filter)
{
	int diag;

	if (port_id_is_invalid(port_id))
		return;

	diag = rte_eth_dev_fdir_update_perfect_filter(port_id, fdir_filter,
						      soft_id, queue_id, drop);
	if (diag == 0)
		return;

	printf("rte_eth_dev_fdir_update_perfect_filter for port_id=%d failed "
	       "diag=%d\n", port_id, diag);
}

void
fdir_remove_perfect_filter(portid_t port_id, uint16_t soft_id,
			   struct rte_fdir_filter *fdir_filter)
{
	int diag;

	if (port_id_is_invalid(port_id))
		return;

	diag = rte_eth_dev_fdir_remove_perfect_filter(port_id, fdir_filter,
						      soft_id);
	if (diag == 0)
		return;

	printf("rte_eth_dev_fdir_update_perfect_filter for port_id=%d failed "
	       "diag=%d\n", port_id, diag);
}

void
fdir_set_masks(portid_t port_id, struct rte_fdir_masks *fdir_masks)
{
	int diag;

	if (port_id_is_invalid(port_id))
		return;

	diag = rte_eth_dev_fdir_set_masks(port_id, fdir_masks);
	if (diag == 0)
		return;

	printf("rte_eth_dev_set_masks_filter for port_id=%d failed "
	       "diag=%d\n", port_id, diag);
}
