// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2020, MIPI Alliance, Inc.
 *
 * Author: Nicolas Pitre <npitre@baylibre.com>
 *
 * Note: The I3C HCI v2.0 spec is still in flux. The IBI support is based on
 * v1.x of the spec and v2.0 will likely be split out.
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/i3c/master.h>
#include <linux/io.h>
#include <linux/pci.h>

#include "hci.h"
#include "cmd.h"
#include "ibi.h"

/*
 * Software Parameter Values (somewhat arb itrary for now).
 * Some of them could be determined at run time eventually.
 */

#define XFER_RINGS			1	/* max: 8 */
#define XFER_RING_ENTRIES		16	/* max: 255 */

#define IBI_RINGS			1	/* max: 8 */
#define IBI_STATUS_RING_ENTRIES		32	/* max: 255 */
#define IBI_CHUNK_CACHELINES		1	/* max: 256 bytes equivalent */
#define IBI_CHUNK_POOL_SIZE		128	/* max: 1023 */

/*
 * Ring Header Preamble
 */

#define rhs_reg_read(r)		readl(hci->RHS_regs + (RHS_##r))
#define rhs_reg_write(r, v)	writel(v, hci->RHS_regs + (RHS_##r))

#define RHS_CONTROL			0x00
#define PREAMBLE_SIZE			GENMASK(31, 24)	/* Preamble Section Size */
#define HEADER_SIZE			GENMASK(23, 16)	/* Ring Header Size */
#define MAX_HEADER_COUNT_CAP		GENMASK(7, 4) /* HC Max Header Count */
#define MAX_HEADER_COUNT		GENMASK(3, 0) /* Driver Max Header Count */

#define RHS_RHn_OFFSET(n)		(0x04 + (n)*4)

/*
 * Ring Header (Per-Ring Bundle)
 */

#define rh_reg_read(r)		readl(rh->regs + (RH_##r))
#define rh_reg_write(r, v)	writel(v, rh->regs + (RH_##r))

#define RH_CR_SETUP			0x00	/* Command/Response Ring */
#define CR_XFER_STRUCT_SIZE		GENMASK(31, 24)
#define CR_RESP_STRUCT_SIZE		GENMASK(23, 16)
#define CR_RING_SIZE			GENMASK(8, 0)

#define RH_IBI_SETUP			0x04
#define IBI_STATUS_STRUCT_SIZE		GENMASK(31, 24)
#define IBI_STATUS_RING_SIZE		GENMASK(23, 16)
#define IBI_DATA_CHUNK_SIZE		GENMASK(12, 10)
#define IBI_DATA_CHUNK_COUNT		GENMASK(9, 0)

#define RH_CHUNK_CONTROL			0x08

#define RH_INTR_STATUS			0x10
#define RH_INTR_STATUS_ENABLE		0x14
#define RH_INTR_SIGNAL_ENABLE		0x18
#define RH_INTR_FORCE			0x1c
#define INTR_IBI_READY			BIT(12)
#define INTR_TRANSFER_COMPLETION	BIT(11)
#define INTR_RING_OP			BIT(10)
#define INTR_TRANSFER_ERR		BIT(9)
#define INTR_IBI_RING_FULL		BIT(6)
#define INTR_TRANSFER_ABORT		BIT(5)

#define RH_RING_STATUS			0x20
#define RING_STATUS_LOCKED		BIT(3)
#define RING_STATUS_ABORTED		BIT(2)
#define RING_STATUS_RUNNING		BIT(1)
#define RING_STATUS_ENABLED		BIT(0)

#define RH_RING_CONTROL			0x24
#define RING_CTRL_ABORT			BIT(2)
#define RING_CTRL_RUN_STOP		BIT(1)
#define RING_CTRL_ENABLE		BIT(0)

#define RH_RING_OPERATION1		0x28
#define RING_OP1_IBI_DEQ_PTR		GENMASK(23, 16)
#define RING_OP1_CR_SW_DEQ_PTR		GENMASK(15, 8)
#define RING_OP1_CR_ENQ_PTR		GENMASK(7, 0)

#define RH_RING_OPERATION2		0x2c
#define RING_OP2_IBI_ENQ_PTR		GENMASK(23, 16)
#define RING_OP2_CR_DEQ_PTR		GENMASK(7, 0)

#define RH_CMD_RING_BASE_LO		0x30
#define RH_CMD_RING_BASE_HI		0x34
#define RH_RESP_RING_BASE_LO		0x38
#define RH_RESP_RING_BASE_HI		0x3c
#define RH_IBI_STATUS_RING_BASE_LO	0x40
#define RH_IBI_STATUS_RING_BASE_HI	0x44
#define RH_IBI_DATA_RING_BASE_LO	0x48
#define RH_IBI_DATA_RING_BASE_HI	0x4c

#define RH_CMD_RING_SG			0x50	/* Ring Scatter Gather Support */
#define RH_RESP_RING_SG			0x54
#define RH_IBI_STATUS_RING_SG		0x58
#define RH_IBI_DATA_RING_SG		0x5c
#define RING_SG_BLP			BIT(31)	/* Buffer Vs. List Pointer */
#define RING_SG_LIST_SIZE		GENMASK(15, 0)

/*
 * Data Buffer Descriptor (in memory)
 */

#define DATA_BUF_BLP			BIT(31)	/* Buffer Vs. List Pointer */
#define DATA_BUF_IOC			BIT(30)	/* Interrupt on Completion */
#define DATA_BUF_BLOCK_SIZE		GENMASK(15, 0)

struct hci_rh_data {
	void __iomem *regs;
	void *xfer, *resp, *ibi_status, *ibi_data;
	dma_addr_t xfer_dma, resp_dma, ibi_status_dma, ibi_data_dma;
	unsigned int xfer_entries, ibi_status_entries, ibi_chunks_total;
	unsigned int xfer_struct_sz, resp_struct_sz, ibi_status_sz, ibi_chunk_sz;
	unsigned int done_ptr, ibi_chunk_ptr, xfer_space;
	struct hci_xfer **src_xfers;
	spinlock_t lock;
	struct completion op_done;
};

struct hci_rings_data {
	struct device *sysdev;
	unsigned int total;
	struct hci_rh_data headers[] __counted_by(total);
};

struct hci_dma_dev_ibi_data {
	struct i3c_generic_ibi_pool *pool;
	unsigned int max_len;
};

struct fld_info {
	char reg[33];	// Register name
	char fld[33];	// Bitfield name
	u8 w;		// Bitfield width
	u8 b;		// Bitfield offset
	u16 offs;	// Register offset
};

static const struct fld_info hc_regs[] = {
	{ .reg = "HCI_VERSION", .fld = "VERSION", .w = 32, .b = 0, .offs = 0x0 },
	{ .reg = "HC_CONTROL", .fld = "IBA_INCLUDE", .w = 1, .b = 0, .offs = 0x4 },
	{ .reg = "HC_CONTROL", .fld = "I2C_SLAVE_PRESENT", .w = 1, .b = 7, .offs = 0x4 },
	{ .reg = "HC_CONTROL", .fld = "HOT_JOIN_CTRL", .w = 1, .b = 8, .offs = 0x4 },
	{ .reg = "HC_CONTROL", .fld = "ABORT", .w = 1, .b = 29, .offs = 0x4 },
	{ .reg = "HC_CONTROL", .fld = "RESUME", .w = 1, .b = 30, .offs = 0x4 },
	{ .reg = "HC_CONTROL", .fld = "BUS_ENABLE", .w = 1, .b = 31, .offs = 0x4 },
	{ .reg = "MASTER_DEVICE_ADDR", .fld = "DYNAMIC_ADDR", .w = 7, .b = 16, .offs = 0x8 },
	{ .reg = "MASTER_DEVICE_ADDR", .fld = "DYNAMIC_ADDR_VALID", .w = 1, .b = 31, .offs = 0x8 },
	{ .reg = "HC_CAPABILITIES", .fld = "COMBO_COMMAND", .w = 1, .b = 2, .offs = 0xC },
	{ .reg = "HC_CAPABILITIES", .fld = "AUTO_COMMAND", .w = 1, .b = 3, .offs = 0xC },
	{ .reg = "HC_CAPABILITIES", .fld = "NON_CURRENT_MASTER_CAP", .w = 1, .b = 5, .offs = 0xC },
	{ .reg = "HC_CAPABILITIES", .fld = "HDR_DDR_EN", .w = 1, .b = 6, .offs = 0xC },
	{ .reg = "HC_CAPABILITIES", .fld = "HDR_TS_EN", .w = 1, .b = 7, .offs = 0xC },
	{ .reg = "RESET_CONTROL", .fld = "SOFT_RST", .w = 1, .b = 0, .offs = 0x10 },
	{ .reg = "RESET_CONTROL", .fld = "CMD_QUEUE_RST", .w = 1, .b = 1, .offs = 0x10 },
	{ .reg = "RESET_CONTROL", .fld = "RESP_QUEUE_RST", .w = 1, .b = 2, .offs = 0x10 },
	{ .reg = "RESET_CONTROL", .fld = "TX_FIFO_RST", .w = 1, .b = 3, .offs = 0x10 },
	{ .reg = "RESET_CONTROL", .fld = "RX_FIFO_RST", .w = 1, .b = 4, .offs = 0x10 },
	{ .reg = "RESET_CONTROL", .fld = "IBI_QUEUE_RST", .w = 1, .b = 5, .offs = 0x10 },
	{ .reg = "PRESENT_STATE", .fld = "CURRENT_MASTER", .w = 1, .b = 2, .offs = 0x14 },
	{ .reg = "INTR_STATUS", .fld = "HC_INTERNAL_ERR_STAT", .w = 1, .b = 10, .offs = 0x20 },
	{ .reg = "INTR_STATUS_ENABLE", .fld = "HC_INTERNAL_ERR_STAT_EN", .w = 1, .b = 10, .offs = 0x24 },
	{ .reg = "INTR_SIGNAL_ENABLE", .fld = "HC_INTERNAL_ERR_SIGNAL_EN", .w = 1, .b = 10, .offs = 0x28 },
	{ .reg = "INTR_FORCE", .fld = "HC_INTERNAL_ERR_FORCE", .w = 1, .b = 10, .offs = 0x2C },
	{ .reg = "DAT_SECTION_OFFSET", .fld = "TABLE_OFFSET", .w = 12, .b = 0, .offs = 0x30 },
	{ .reg = "DAT_SECTION_OFFSET", .fld = "TABLE_SIZE", .w = 6, .b = 12, .offs = 0x30 },
	{ .reg = "DCT_SECTION_OFFSET", .fld = "TABLE_OFFSET", .w = 12, .b = 0, .offs = 0x34 },
	{ .reg = "DCT_SECTION_OFFSET", .fld = "TABLE_SIZE", .w = 7, .b = 12, .offs = 0x34 },
	{ .reg = "DCT_SECTION_OFFSET", .fld = "TABLE_INDEX", .w = 3, .b = 19, .offs = 0x34 },
	{ .reg = "RING_HEADERS_SECTION_OFFSET", .fld = "SECTION_OFFSET", .w = 16, .b = 0, .offs = 0x38 },
	{ .reg = "PIO_SECTION_OFFSET", .fld = "SECTION_OFFSET", .w = 16, .b = 0, .offs = 0x3C },
	{ .reg = "EXTCAPS_SECTION_OFFSET", .fld = "SECTION_OFFSET", .w = 16, .b = 0, .offs = 0x40 },
	{ .reg = "IBI_NOTIFY_CTRL", .fld = "NOTIFY_HJ_REJECTED", .w = 1, .b = 0, .offs = 0x58 },
	{ .reg = "IBI_NOTIFY_CTRL", .fld = "NOTIFY_MR_REJECTED", .w = 1, .b = 1, .offs = 0x58 },
	{ .reg = "IBI_NOTIFY_CTRL", .fld = "NOTIFY_SIR_REJECTED", .w = 1, .b = 3, .offs = 0x58 },
#if 0
	{ .reg = "DEV_CTX_BASE_LO", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x60 },
	{ .reg = "DEV_CTX_BASE_HI", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x64 },
	{ .reg = "DEV_ADDR_TABLE1_LOC1", .fld = "STATIC_ADDRESS", .w = 7, .b = 0, .offs = 0x80 },
	{ .reg = "DEV_ADDR_TABLE1_LOC1", .fld = "IBI_WITH_DATA", .w = 1, .b = 12, .offs = 0x80 },
	{ .reg = "DEV_ADDR_TABLE1_LOC1", .fld = "SIR_REJECT", .w = 1, .b = 13, .offs = 0x80 },
	{ .reg = "DEV_ADDR_TABLE1_LOC1", .fld = "MR_REJECT", .w = 1, .b = 14, .offs = 0x80 },
	{ .reg = "DEV_ADDR_TABLE1_LOC1", .fld = "TS", .w = 1, .b = 15, .offs = 0x80 },
	{ .reg = "DEV_ADDR_TABLE1_LOC1", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 16, .offs = 0x80 },
	{ .reg = "DEV_ADDR_TABLE1_LOC1", .fld = "RING_ID", .w = 3, .b = 26, .offs = 0x80 },
	{ .reg = "DEV_ADDR_TABLE1_LOC1", .fld = "DEV_NACK_RETRY_CNT", .w = 2, .b = 29, .offs = 0x80 },
	{ .reg = "DEV_ADDR_TABLE1_LOC1", .fld = "DEVICE", .w = 1, .b = 31, .offs = 0x80 },
	{ .reg = "DEV_ADDR_TABLE1_LOC2", .fld = "AUTOCMD_MASK", .w = 8, .b = 0, .offs = 0x84 },
	{ .reg = "DEV_ADDR_TABLE1_LOC2", .fld = "AUTOCMD_VALUE", .w = 8, .b = 8, .offs = 0x84 },
	{ .reg = "DEV_ADDR_TABLE1_LOC2", .fld = "AUTOCMD_MODE", .w = 3, .b = 16, .offs = 0x84 },
	{ .reg = "DEV_ADDR_TABLE1_LOC2", .fld = "AUTOCMD_HDR_CODE", .w = 8, .b = 19, .offs = 0x84 },
	{ .reg = "DEV_ADDR_TABLE2_LOC1", .fld = "STATIC_ADDRESS", .w = 7, .b = 0, .offs = 0x88 },
	{ .reg = "DEV_ADDR_TABLE2_LOC1", .fld = "IBI_WITH_DATA", .w = 1, .b = 12, .offs = 0x88 },
	{ .reg = "DEV_ADDR_TABLE2_LOC1", .fld = "SIR_REJECT", .w = 1, .b = 13, .offs = 0x88 },
	{ .reg = "DEV_ADDR_TABLE2_LOC1", .fld = "MR_REJECT", .w = 1, .b = 14, .offs = 0x88 },
	{ .reg = "DEV_ADDR_TABLE2_LOC1", .fld = "TS", .w = 1, .b = 15, .offs = 0x88 },
	{ .reg = "DEV_ADDR_TABLE2_LOC1", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 16, .offs = 0x88 },
	{ .reg = "DEV_ADDR_TABLE2_LOC1", .fld = "RING_ID", .w = 3, .b = 26, .offs = 0x88 },
	{ .reg = "DEV_ADDR_TABLE2_LOC1", .fld = "DEV_NACK_RETRY_CNT", .w = 2, .b = 29, .offs = 0x88 },
	{ .reg = "DEV_ADDR_TABLE2_LOC1", .fld = "DEVICE", .w = 1, .b = 31, .offs = 0x88 },
	{ .reg = "DEV_ADDR_TABLE2_LOC2", .fld = "AUTOCMD_MASK", .w = 8, .b = 0, .offs = 0x8C },
	{ .reg = "DEV_ADDR_TABLE2_LOC2", .fld = "AUTOCMD_VALUE", .w = 8, .b = 8, .offs = 0x8C },
	{ .reg = "DEV_ADDR_TABLE2_LOC2", .fld = "AUTOCMD_MODE", .w = 3, .b = 16, .offs = 0x8C },
	{ .reg = "DEV_ADDR_TABLE2_LOC2", .fld = "AUTOCMD_HDR_CODE", .w = 8, .b = 19, .offs = 0x8C },
	{ .reg = "DEV_ADDR_TABLE3_LOC1", .fld = "STATIC_ADDRESS", .w = 7, .b = 0, .offs = 0x90 },
	{ .reg = "DEV_ADDR_TABLE3_LOC1", .fld = "IBI_WITH_DATA", .w = 1, .b = 12, .offs = 0x90 },
	{ .reg = "DEV_ADDR_TABLE3_LOC1", .fld = "SIR_REJECT", .w = 1, .b = 13, .offs = 0x90 },
	{ .reg = "DEV_ADDR_TABLE3_LOC1", .fld = "MR_REJECT", .w = 1, .b = 14, .offs = 0x90 },
	{ .reg = "DEV_ADDR_TABLE3_LOC1", .fld = "TS", .w = 1, .b = 15, .offs = 0x90 },
	{ .reg = "DEV_ADDR_TABLE3_LOC1", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 16, .offs = 0x90 },
	{ .reg = "DEV_ADDR_TABLE3_LOC1", .fld = "RING_ID", .w = 3, .b = 26, .offs = 0x90 },
	{ .reg = "DEV_ADDR_TABLE3_LOC1", .fld = "DEV_NACK_RETRY_CNT", .w = 2, .b = 29, .offs = 0x90 },
	{ .reg = "DEV_ADDR_TABLE3_LOC1", .fld = "DEVICE", .w = 1, .b = 31, .offs = 0x90 },
	{ .reg = "DEV_ADDR_TABLE3_LOC2", .fld = "AUTOCMD_MASK", .w = 8, .b = 0, .offs = 0x94 },
	{ .reg = "DEV_ADDR_TABLE3_LOC2", .fld = "AUTOCMD_VALUE", .w = 8, .b = 8, .offs = 0x94 },
	{ .reg = "DEV_ADDR_TABLE3_LOC2", .fld = "AUTOCMD_MODE", .w = 3, .b = 16, .offs = 0x94 },
	{ .reg = "DEV_ADDR_TABLE3_LOC2", .fld = "AUTOCMD_HDR_CODE", .w = 8, .b = 19, .offs = 0x94 },
	{ .reg = "DEV_ADDR_TABLE4_LOC1", .fld = "STATIC_ADDRESS", .w = 7, .b = 0, .offs = 0x98 },
	{ .reg = "DEV_ADDR_TABLE4_LOC1", .fld = "IBI_WITH_DATA", .w = 1, .b = 12, .offs = 0x98 },
	{ .reg = "DEV_ADDR_TABLE4_LOC1", .fld = "SIR_REJECT", .w = 1, .b = 13, .offs = 0x98 },
	{ .reg = "DEV_ADDR_TABLE4_LOC1", .fld = "MR_REJECT", .w = 1, .b = 14, .offs = 0x98 },
	{ .reg = "DEV_ADDR_TABLE4_LOC1", .fld = "TS", .w = 1, .b = 15, .offs = 0x98 },
	{ .reg = "DEV_ADDR_TABLE4_LOC1", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 16, .offs = 0x98 },
	{ .reg = "DEV_ADDR_TABLE4_LOC1", .fld = "RING_ID", .w = 3, .b = 26, .offs = 0x98 },
	{ .reg = "DEV_ADDR_TABLE4_LOC1", .fld = "DEV_NACK_RETRY_CNT", .w = 2, .b = 29, .offs = 0x98 },
	{ .reg = "DEV_ADDR_TABLE4_LOC1", .fld = "DEVICE", .w = 1, .b = 31, .offs = 0x98 },
	{ .reg = "DEV_ADDR_TABLE4_LOC2", .fld = "AUTOCMD_MASK", .w = 8, .b = 0, .offs = 0x9C },
	{ .reg = "DEV_ADDR_TABLE4_LOC2", .fld = "AUTOCMD_VALUE", .w = 8, .b = 8, .offs = 0x9C },
	{ .reg = "DEV_ADDR_TABLE4_LOC2", .fld = "AUTOCMD_MODE", .w = 3, .b = 16, .offs = 0x9C },
	{ .reg = "DEV_ADDR_TABLE4_LOC2", .fld = "AUTOCMD_HDR_CODE", .w = 8, .b = 19, .offs = 0x9C },
	{ .reg = "DEV_ADDR_TABLE5_LOC1", .fld = "STATIC_ADDRESS", .w = 7, .b = 0, .offs = 0xA0 },
	{ .reg = "DEV_ADDR_TABLE5_LOC1", .fld = "IBI_WITH_DATA", .w = 1, .b = 12, .offs = 0xA0 },
	{ .reg = "DEV_ADDR_TABLE5_LOC1", .fld = "SIR_REJECT", .w = 1, .b = 13, .offs = 0xA0 },
	{ .reg = "DEV_ADDR_TABLE5_LOC1", .fld = "MR_REJECT", .w = 1, .b = 14, .offs = 0xA0 },
	{ .reg = "DEV_ADDR_TABLE5_LOC1", .fld = "TS", .w = 1, .b = 15, .offs = 0xA0 },
	{ .reg = "DEV_ADDR_TABLE5_LOC1", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 16, .offs = 0xA0 },
	{ .reg = "DEV_ADDR_TABLE5_LOC1", .fld = "RING_ID", .w = 3, .b = 26, .offs = 0xA0 },
	{ .reg = "DEV_ADDR_TABLE5_LOC1", .fld = "DEV_NACK_RETRY_CNT", .w = 2, .b = 29, .offs = 0xA0 },
	{ .reg = "DEV_ADDR_TABLE5_LOC1", .fld = "DEVICE", .w = 1, .b = 31, .offs = 0xA0 },
	{ .reg = "DEV_ADDR_TABLE5_LOC2", .fld = "AUTOCMD_MASK", .w = 8, .b = 0, .offs = 0xA4 },
	{ .reg = "DEV_ADDR_TABLE5_LOC2", .fld = "AUTOCMD_VALUE", .w = 8, .b = 8, .offs = 0xA4 },
	{ .reg = "DEV_ADDR_TABLE5_LOC2", .fld = "AUTOCMD_MODE", .w = 3, .b = 16, .offs = 0xA4 },
	{ .reg = "DEV_ADDR_TABLE5_LOC2", .fld = "AUTOCMD_HDR_CODE", .w = 8, .b = 19, .offs = 0xA4 },
	{ .reg = "DEV_ADDR_TABLE6_LOC1", .fld = "STATIC_ADDRESS", .w = 7, .b = 0, .offs = 0xA8 },
	{ .reg = "DEV_ADDR_TABLE6_LOC1", .fld = "IBI_WITH_DATA", .w = 1, .b = 12, .offs = 0xA8 },
	{ .reg = "DEV_ADDR_TABLE6_LOC1", .fld = "SIR_REJECT", .w = 1, .b = 13, .offs = 0xA8 },
	{ .reg = "DEV_ADDR_TABLE6_LOC1", .fld = "MR_REJECT", .w = 1, .b = 14, .offs = 0xA8 },
	{ .reg = "DEV_ADDR_TABLE6_LOC1", .fld = "TS", .w = 1, .b = 15, .offs = 0xA8 },
	{ .reg = "DEV_ADDR_TABLE6_LOC1", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 16, .offs = 0xA8 },
	{ .reg = "DEV_ADDR_TABLE6_LOC1", .fld = "RING_ID", .w = 3, .b = 26, .offs = 0xA8 },
	{ .reg = "DEV_ADDR_TABLE6_LOC1", .fld = "DEV_NACK_RETRY_CNT", .w = 2, .b = 29, .offs = 0xA8 },
	{ .reg = "DEV_ADDR_TABLE6_LOC1", .fld = "DEVICE", .w = 1, .b = 31, .offs = 0xA8 },
	{ .reg = "DEV_ADDR_TABLE6_LOC2", .fld = "AUTOCMD_MASK", .w = 8, .b = 0, .offs = 0xAC },
	{ .reg = "DEV_ADDR_TABLE6_LOC2", .fld = "AUTOCMD_VALUE", .w = 8, .b = 8, .offs = 0xAC },
	{ .reg = "DEV_ADDR_TABLE6_LOC2", .fld = "AUTOCMD_MODE", .w = 3, .b = 16, .offs = 0xAC },
	{ .reg = "DEV_ADDR_TABLE6_LOC2", .fld = "AUTOCMD_HDR_CODE", .w = 8, .b = 19, .offs = 0xAC },
	{ .reg = "DEV_ADDR_TABLE7_LOC1", .fld = "STATIC_ADDRESS", .w = 7, .b = 0, .offs = 0xB0 },
	{ .reg = "DEV_ADDR_TABLE7_LOC1", .fld = "IBI_WITH_DATA", .w = 1, .b = 12, .offs = 0xB0 },
	{ .reg = "DEV_ADDR_TABLE7_LOC1", .fld = "SIR_REJECT", .w = 1, .b = 13, .offs = 0xB0 },
	{ .reg = "DEV_ADDR_TABLE7_LOC1", .fld = "MR_REJECT", .w = 1, .b = 14, .offs = 0xB0 },
	{ .reg = "DEV_ADDR_TABLE7_LOC1", .fld = "TS", .w = 1, .b = 15, .offs = 0xB0 },
	{ .reg = "DEV_ADDR_TABLE7_LOC1", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 16, .offs = 0xB0 },
	{ .reg = "DEV_ADDR_TABLE7_LOC1", .fld = "RING_ID", .w = 3, .b = 26, .offs = 0xB0 },
	{ .reg = "DEV_ADDR_TABLE7_LOC1", .fld = "DEV_NACK_RETRY_CNT", .w = 2, .b = 29, .offs = 0xB0 },
	{ .reg = "DEV_ADDR_TABLE7_LOC1", .fld = "DEVICE", .w = 1, .b = 31, .offs = 0xB0 },
	{ .reg = "DEV_ADDR_TABLE7_LOC2", .fld = "AUTOCMD_MASK", .w = 8, .b = 0, .offs = 0xB4 },
	{ .reg = "DEV_ADDR_TABLE7_LOC2", .fld = "AUTOCMD_VALUE", .w = 8, .b = 8, .offs = 0xB4 },
	{ .reg = "DEV_ADDR_TABLE7_LOC2", .fld = "AUTOCMD_MODE", .w = 3, .b = 16, .offs = 0xB4 },
	{ .reg = "DEV_ADDR_TABLE7_LOC2", .fld = "AUTOCMD_HDR_CODE", .w = 8, .b = 19, .offs = 0xB4 },
	{ .reg = "DEV_ADDR_TABLE8_LOC1", .fld = "STATIC_ADDRESS", .w = 7, .b = 0, .offs = 0xB8 },
	{ .reg = "DEV_ADDR_TABLE8_LOC1", .fld = "IBI_WITH_DATA", .w = 1, .b = 12, .offs = 0xB8 },
	{ .reg = "DEV_ADDR_TABLE8_LOC1", .fld = "SIR_REJECT", .w = 1, .b = 13, .offs = 0xB8 },
	{ .reg = "DEV_ADDR_TABLE8_LOC1", .fld = "MR_REJECT", .w = 1, .b = 14, .offs = 0xB8 },
	{ .reg = "DEV_ADDR_TABLE8_LOC1", .fld = "TS", .w = 1, .b = 15, .offs = 0xB8 },
	{ .reg = "DEV_ADDR_TABLE8_LOC1", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 16, .offs = 0xB8 },
	{ .reg = "DEV_ADDR_TABLE8_LOC1", .fld = "RING_ID", .w = 3, .b = 26, .offs = 0xB8 },
	{ .reg = "DEV_ADDR_TABLE8_LOC1", .fld = "DEV_NACK_RETRY_CNT", .w = 2, .b = 29, .offs = 0xB8 },
	{ .reg = "DEV_ADDR_TABLE8_LOC1", .fld = "DEVICE", .w = 1, .b = 31, .offs = 0xB8 },
	{ .reg = "DEV_ADDR_TABLE8_LOC2", .fld = "AUTOCMD_MASK", .w = 8, .b = 0, .offs = 0xBC },
	{ .reg = "DEV_ADDR_TABLE8_LOC2", .fld = "AUTOCMD_VALUE", .w = 8, .b = 8, .offs = 0xBC },
	{ .reg = "DEV_ADDR_TABLE8_LOC2", .fld = "AUTOCMD_MODE", .w = 3, .b = 16, .offs = 0xBC },
	{ .reg = "DEV_ADDR_TABLE8_LOC2", .fld = "AUTOCMD_HDR_CODE", .w = 8, .b = 19, .offs = 0xBC },
#endif
#if 0
	{ .reg = "COMMAND_QUEUE_PORT", .fld = "COMMAND", .w = 32, .b = 0, .offs = 0xC0 },
	{ .reg = "RESPONSE_QUEUE_PORT", .fld = "DATA_LENGTH", .w = 16, .b = 0, .offs = 0xC4 },
	{ .reg = "RESPONSE_QUEUE_PORT", .fld = "TID", .w = 4, .b = 24, .offs = 0xC4 },
	{ .reg = "RESPONSE_QUEUE_PORT", .fld = "ERR_STATUS", .w = 4, .b = 28, .offs = 0xC4 },
	{ .reg = "TX_DATA_PORT", .fld = "TX_DATA_PORT", .w = 32, .b = 0, .offs = 0xC8 },
	{ .reg = "RX_DATA_PORT", .fld = "RX_DATA_PORT", .w = 32, .b = 0, .offs = 0xC8 },
	{ .reg = "IBI_PORT", .fld = "IBI_DATA", .w = 32, .b = 0, .offs = 0xCC },
	{ .reg = "QUEUE_THLD_CTRL", .fld = "CMD_EMPTY_BUF_THLD", .w = 8, .b = 0, .offs = 0xD0 },
	{ .reg = "QUEUE_THLD_CTRL", .fld = "RESP_BUF_THLD", .w = 8, .b = 8, .offs = 0xD0 },
	{ .reg = "QUEUE_THLD_CTRL", .fld = "IBI_DATA_THLD", .w = 8, .b = 16, .offs = 0xD0 },
	{ .reg = "QUEUE_THLD_CTRL", .fld = "IBI_STATUS_THLD", .w = 8, .b = 24, .offs = 0xD0 },
	{ .reg = "DATA_BUFFER_THLD_CTRL", .fld = "TX_BUF_THLD", .w = 3, .b = 0, .offs = 0xD4 },
	{ .reg = "DATA_BUFFER_THLD_CTRL", .fld = "RX_BUF_THLD", .w = 3, .b = 8, .offs = 0xD4 },
	{ .reg = "DATA_BUFFER_THLD_CTRL", .fld = "TX_START_THLD", .w = 3, .b = 16, .offs = 0xD4 },
	{ .reg = "DATA_BUFFER_THLD_CTRL", .fld = "RX_START_THLD", .w = 3, .b = 24, .offs = 0xD4 },
	{ .reg = "QUEUE_SIZE_CTRL", .fld = "CR_QUEUE_SIZE", .w = 8, .b = 0, .offs = 0xD8 },
	{ .reg = "QUEUE_SIZE_CTRL", .fld = "IBI_STATUS_SIZE", .w = 8, .b = 8, .offs = 0xD8 },
	{ .reg = "QUEUE_SIZE_CTRL", .fld = "RX_DATA_BUFFER_SIZE", .w = 8, .b = 16, .offs = 0xD8 },
	{ .reg = "QUEUE_SIZE_CTRL", .fld = "TX_DATA_BUFFER_SIZE", .w = 8, .b = 24, .offs = 0xD8 },
	{ .reg = "PIO_INTR_STATUS", .fld = "TX_THLD_STAT", .w = 1, .b = 0, .offs = 0xE0 },
	{ .reg = "PIO_INTR_STATUS", .fld = "RX_THLD_STAT", .w = 1, .b = 1, .offs = 0xE0 },
	{ .reg = "PIO_INTR_STATUS", .fld = "IBI_STATUS_THLD_STAT", .w = 1, .b = 2, .offs = 0xE0 },
	{ .reg = "PIO_INTR_STATUS", .fld = "CMD_QUEUE_READY_STAT", .w = 1, .b = 3, .offs = 0xE0 },
	{ .reg = "PIO_INTR_STATUS", .fld = "RESP_READY_STAT", .w = 1, .b = 4, .offs = 0xE0 },
	{ .reg = "PIO_INTR_STATUS", .fld = "TRANSFER_ABORT_STAT", .w = 1, .b = 5, .offs = 0xE0 },
	{ .reg = "PIO_INTR_STATUS", .fld = "TRANSFER_ERR_STAT", .w = 1, .b = 9, .offs = 0xE0 },
	{ .reg = "PIO_INTR_STATUS_ENABLE", .fld = "TX_THLD_STAT_EN", .w = 1, .b = 0, .offs = 0xE4 },
	{ .reg = "PIO_INTR_STATUS_ENABLE", .fld = "RX_THLD_STAT_EN", .w = 1, .b = 1, .offs = 0xE4 },
	{ .reg = "PIO_INTR_STATUS_ENABLE", .fld = "IBI_THLD_STAT_EN", .w = 1, .b = 2, .offs = 0xE4 },
	{ .reg = "PIO_INTR_STATUS_ENABLE", .fld = "CMD_QUEUE_READY_STAT_EN", .w = 1, .b = 3, .offs = 0xE4 },
	{ .reg = "PIO_INTR_STATUS_ENABLE", .fld = "RESP_READY_STAT_INTR_EN", .w = 1, .b = 4, .offs = 0xE4 },
	{ .reg = "PIO_INTR_STATUS_ENABLE", .fld = "TRANSFER_ABORT_STAT_EN", .w = 1, .b = 5, .offs = 0xE4 },
	{ .reg = "PIO_INTR_STATUS_ENABLE", .fld = "TRANSFER_ERR_STAT_EN", .w = 1, .b = 9, .offs = 0xE4 },
	{ .reg = "PIO_INTR_SIGNAL_ENABLE", .fld = "TX_THLD_SIGNAL_EN", .w = 1, .b = 0, .offs = 0xE8 },
	{ .reg = "PIO_INTR_SIGNAL_ENABLE", .fld = "RX_THLD_SIGNAL_EN", .w = 1, .b = 1, .offs = 0xE8 },
	{ .reg = "PIO_INTR_SIGNAL_ENABLE", .fld = "IBI_THLD_SIGNAL_EN", .w = 1, .b = 2, .offs = 0xE8 },
	{ .reg = "PIO_INTR_SIGNAL_ENABLE", .fld = "CMD_QUEUE_READY_SIGNAL_EN", .w = 1, .b = 3, .offs = 0xE8 },
	{ .reg = "PIO_INTR_SIGNAL_ENABLE", .fld = "RESP_READY_SIGNAL_EN", .w = 1, .b = 4, .offs = 0xE8 },
	{ .reg = "PIO_INTR_SIGNAL_ENABLE", .fld = "TRANSFER_ABORT_SIGNAL_EN", .w = 1, .b = 5, .offs = 0xE8 },
	{ .reg = "PIO_INTR_SIGNAL_ENABLE", .fld = "TRANSFER_ERR_SIGNAL_EN", .w = 1, .b = 9, .offs = 0xE8 },
	{ .reg = "PIO_INTR_FORCE", .fld = "TX_THLD_FORCE", .w = 1, .b = 0, .offs = 0xEC },
	{ .reg = "PIO_INTR_FORCE", .fld = "RX_THLD_FORCE", .w = 1, .b = 1, .offs = 0xEC },
	{ .reg = "PIO_INTR_FORCE", .fld = "IBI_THLD_FORCE", .w = 1, .b = 2, .offs = 0xEC },
	{ .reg = "PIO_INTR_FORCE", .fld = "CMD_QUEUE_READY_FORCE", .w = 1, .b = 3, .offs = 0xEC },
	{ .reg = "PIO_INTR_FORCE", .fld = "RESP_READY_FORCE", .w = 1, .b = 4, .offs = 0xEC },
	{ .reg = "PIO_INTR_FORCE", .fld = "TRANSFER_ABORT_FORCE", .w = 1, .b = 5, .offs = 0xEC },
	{ .reg = "PIO_INTR_FORCE", .fld = "TRANSFER_ERR_FORCE", .w = 1, .b = 9, .offs = 0xEC },
#endif
#if 0
	{ .reg = "DEV_CHAR_TABLE1_LOC1", .fld = "MSB_PROVISIONAL_ID", .w = 32, .b = 0, .offs = 0x100 },
	{ .reg = "DEV_CHAR_TABLE1_LOC2", .fld = "LSB_PROVISIONAL_ID", .w = 16, .b = 0, .offs = 0x104 },
	{ .reg = "DEV_CHAR_TABLE1_LOC3", .fld = "DCR", .w = 8, .b = 0, .offs = 0x108 },
	{ .reg = "DEV_CHAR_TABLE1_LOC3", .fld = "BCR", .w = 8, .b = 8, .offs = 0x108 },
	{ .reg = "DEV_CHAR_TABLE1_LOC4", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 0, .offs = 0x10C },
	{ .reg = "DEV_CHAR_TABLE2_LOC1", .fld = "MSB_PROVISIONAL_ID", .w = 32, .b = 0, .offs = 0x110 },
	{ .reg = "DEV_CHAR_TABLE2_LOC2", .fld = "LSB_PROVISIONAL_ID", .w = 16, .b = 0, .offs = 0x114 },
	{ .reg = "DEV_CHAR_TABLE2_LOC3", .fld = "DCR", .w = 8, .b = 0, .offs = 0x118 },
	{ .reg = "DEV_CHAR_TABLE2_LOC3", .fld = "BCR", .w = 8, .b = 8, .offs = 0x118 },
	{ .reg = "DEV_CHAR_TABLE2_LOC4", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 0, .offs = 0x11C },
	{ .reg = "DEV_CHAR_TABLE3_LOC1", .fld = "MSB_PROVISIONAL_ID", .w = 32, .b = 0, .offs = 0x120 },
	{ .reg = "DEV_CHAR_TABLE3_LOC2", .fld = "LSB_PROVISIONAL_ID", .w = 16, .b = 0, .offs = 0x124 },
	{ .reg = "DEV_CHAR_TABLE3_LOC3", .fld = "DCR", .w = 8, .b = 0, .offs = 0x128 },
	{ .reg = "DEV_CHAR_TABLE3_LOC3", .fld = "BCR", .w = 8, .b = 8, .offs = 0x128 },
	{ .reg = "DEV_CHAR_TABLE3_LOC4", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 0, .offs = 0x12C },
	{ .reg = "DEV_CHAR_TABLE4_LOC1", .fld = "MSB_PROVISIONAL_ID", .w = 32, .b = 0, .offs = 0x130 },
	{ .reg = "DEV_CHAR_TABLE4_LOC2", .fld = "LSB_PROVISIONAL_ID", .w = 16, .b = 0, .offs = 0x134 },
	{ .reg = "DEV_CHAR_TABLE4_LOC3", .fld = "DCR", .w = 8, .b = 0, .offs = 0x138 },
	{ .reg = "DEV_CHAR_TABLE4_LOC3", .fld = "BCR", .w = 8, .b = 8, .offs = 0x138 },
	{ .reg = "DEV_CHAR_TABLE4_LOC4", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 0, .offs = 0x13C },
	{ .reg = "DEV_CHAR_TABLE5_LOC1", .fld = "MSB_PROVISIONAL_ID", .w = 32, .b = 0, .offs = 0x140 },
	{ .reg = "DEV_CHAR_TABLE5_LOC2", .fld = "LSB_PROVISIONAL_ID", .w = 16, .b = 0, .offs = 0x144 },
	{ .reg = "DEV_CHAR_TABLE5_LOC3", .fld = "DCR", .w = 8, .b = 0, .offs = 0x148 },
	{ .reg = "DEV_CHAR_TABLE5_LOC3", .fld = "BCR", .w = 8, .b = 8, .offs = 0x148 },
	{ .reg = "DEV_CHAR_TABLE5_LOC4", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 0, .offs = 0x14C },
	{ .reg = "DEV_CHAR_TABLE6_LOC1", .fld = "MSB_PROVISIONAL_ID", .w = 32, .b = 0, .offs = 0x150 },
	{ .reg = "DEV_CHAR_TABLE6_LOC2", .fld = "LSB_PROVISIONAL_ID", .w = 16, .b = 0, .offs = 0x154 },
	{ .reg = "DEV_CHAR_TABLE6_LOC3", .fld = "DCR", .w = 8, .b = 0, .offs = 0x158 },
	{ .reg = "DEV_CHAR_TABLE6_LOC3", .fld = "BCR", .w = 8, .b = 8, .offs = 0x158 },
	{ .reg = "DEV_CHAR_TABLE6_LOC4", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 0, .offs = 0x15C },
	{ .reg = "DEV_CHAR_TABLE7_LOC1", .fld = "MSB_PROVISIONAL_ID", .w = 32, .b = 0, .offs = 0x160 },
	{ .reg = "DEV_CHAR_TABLE7_LOC2", .fld = "LSB_PROVISIONAL_ID", .w = 16, .b = 0, .offs = 0x164 },
	{ .reg = "DEV_CHAR_TABLE7_LOC3", .fld = "DCR", .w = 8, .b = 0, .offs = 0x168 },
	{ .reg = "DEV_CHAR_TABLE7_LOC3", .fld = "BCR", .w = 8, .b = 8, .offs = 0x168 },
	{ .reg = "DEV_CHAR_TABLE7_LOC4", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 0, .offs = 0x16C },
	{ .reg = "DEV_CHAR_TABLE8_LOC1", .fld = "MSB_PROVISIONAL_ID", .w = 32, .b = 0, .offs = 0x170 },
	{ .reg = "DEV_CHAR_TABLE8_LOC2", .fld = "LSB_PROVISIONAL_ID", .w = 16, .b = 0, .offs = 0x174 },
	{ .reg = "DEV_CHAR_TABLE8_LOC3", .fld = "DCR", .w = 8, .b = 0, .offs = 0x178 },
	{ .reg = "DEV_CHAR_TABLE8_LOC3", .fld = "BCR", .w = 8, .b = 8, .offs = 0x178 },
	{ .reg = "DEV_CHAR_TABLE8_LOC4", .fld = "DEV_DYNAMIC_ADDR", .w = 8, .b = 0, .offs = 0x17C },
	{ .reg = "HW_IDENTIFICATION_HEADER", .fld = "CAP_ID", .w = 8, .b = 0, .offs = 0x200 },
	{ .reg = "HW_IDENTIFICATION_HEADER", .fld = "CAP_LEN", .w = 16, .b = 8, .offs = 0x200 },
	{ .reg = "COMP_MANUFACTURER", .fld = "MIPI_VENDOR_ID", .w = 32, .b = 0, .offs = 0x204 },
	{ .reg = "COMP_VERSION", .fld = "I3C_VER_ID", .w = 32, .b = 0, .offs = 0x208 },
	{ .reg = "COMP_TYPE", .fld = "I3C_VER_TYPE", .w = 32, .b = 0, .offs = 0x20C },
	{ .reg = "BUS_TIMING_HEADER", .fld = "CAP_ID", .w = 8, .b = 0, .offs = 0x210 },
	{ .reg = "BUS_TIMING_HEADER", .fld = "CAP_LEN", .w = 16, .b = 8, .offs = 0x210 },
#endif
	{ .reg = "SCL_I3C_OD_TIMING", .fld = "I3C_OD_LCNT", .w = 8, .b = 0, .offs = 0x214 },
	{ .reg = "SCL_I3C_OD_TIMING", .fld = "I3C_OD_HCNT", .w = 8, .b = 16, .offs = 0x214 },
	{ .reg = "SCL_I3C_PP_TIMING", .fld = "I3C_PP_LCNT", .w = 8, .b = 0, .offs = 0x218 },
	{ .reg = "SCL_I3C_PP_TIMING", .fld = "I3C_PP_HCNT", .w = 8, .b = 16, .offs = 0x218 },
	{ .reg = "SCL_I2C_FM_TIMING", .fld = "I2C_FM_LCNT", .w = 16, .b = 0, .offs = 0x21C },
	{ .reg = "SCL_I2C_FM_TIMING", .fld = "I2C_FM_HCNT", .w = 8, .b = 16, .offs = 0x21C },
	{ .reg = "SCL_I2C_FMP_TIMING", .fld = "I2C_FMP_LCNT", .w = 8, .b = 0, .offs = 0x220 },
	{ .reg = "SCL_I2C_FMP_TIMING", .fld = "I2C_FMP_HCNT", .w = 8, .b = 16, .offs = 0x220 },
	{ .reg = "SCL_I2C_SS_TIMING", .fld = "I2C_SS_LCNT", .w = 16, .b = 0, .offs = 0x224 },
	{ .reg = "SCL_I2C_SS_TIMING", .fld = "I2C_SS_HCNT", .w = 16, .b = 16, .offs = 0x224 },
	{ .reg = "SCL_EXT_LCNT_TIMING", .fld = "I3C_EXT_LCNT_1", .w = 8, .b = 0, .offs = 0x228 },
	{ .reg = "SCL_EXT_LCNT_TIMING", .fld = "I3C_EXT_LCNT_2", .w = 8, .b = 8, .offs = 0x228 },
	{ .reg = "SCL_EXT_LCNT_TIMING", .fld = "I3C_EXT_LCNT_3", .w = 8, .b = 16, .offs = 0x228 },
	{ .reg = "SCL_EXT_LCNT_TIMING", .fld = "I3C_EXT_LCNT_4", .w = 8, .b = 24, .offs = 0x228 },
	{ .reg = "SCL_EXT_TERMN_LCNT_TIMING", .fld = "I3C_EXT_TERMN_LCNT", .w = 4, .b = 0, .offs = 0x22C },
	{ .reg = "SCL_EXT_TERMN_LCNT_TIMING", .fld = "I3C_TS_SKEW_CNT", .w = 4, .b = 16, .offs = 0x22C },
	{ .reg = "SDA_HOLD_SWITCH_DLY_TIMING", .fld = "SDA_OD_PP_SWITCH_DLY", .w = 3, .b = 0, .offs = 0x230 },
	{ .reg = "SDA_HOLD_SWITCH_DLY_TIMING", .fld = "SDA_PP_OD_SWITCH_DLY", .w = 3, .b = 8, .offs = 0x230 },
	{ .reg = "SDA_HOLD_SWITCH_DLY_TIMING", .fld = "SDA_TX_HOLD", .w = 3, .b = 16, .offs = 0x230 },
	{ .reg = "BUS_FREE_TIMING", .fld = "I3C_MST_FREE", .w = 16, .b = 0, .offs = 0x234 },
	{ .reg = "DS_EXTCAP_HEADER", .fld = "CAP_ID", .w = 8, .b = 0, .offs = 0x240 },
	{ .reg = "DS_EXTCAP_HEADER", .fld = "CAP_LEN", .w = 16, .b = 8, .offs = 0x240 },
	{ .reg = "QUEUE_STATUS_LEVEL", .fld = "CMD_QUEUE_FREE_LVL", .w = 8, .b = 0, .offs = 0x244 },
	{ .reg = "QUEUE_STATUS_LEVEL", .fld = "RESP_BUF_BLR", .w = 8, .b = 8, .offs = 0x244 },
	{ .reg = "QUEUE_STATUS_LEVEL", .fld = "IBI_BUF_BLR", .w = 8, .b = 16, .offs = 0x244 },
	{ .reg = "QUEUE_STATUS_LEVEL", .fld = "IBI_STATUS_CNT", .w = 5, .b = 24, .offs = 0x244 },
	{ .reg = "DATA_BUFFER_STATUS_LEVEL", .fld = "TX_BUF_FREE_LVL", .w = 8, .b = 0, .offs = 0x248 },
	{ .reg = "DATA_BUFFER_STATUS_LEVEL", .fld = "RX_BUF_LVL", .w = 8, .b = 8, .offs = 0x248 },
	{ .reg = "PRESENT_STATE_DEBUG", .fld = "SCL_LINE_SIGNAL_LEVEL", .w = 1, .b = 0, .offs = 0x24C },
	{ .reg = "PRESENT_STATE_DEBUG", .fld = "SDA_LINE_SIGNAL_LEVEL", .w = 1, .b = 1, .offs = 0x24C },
	{ .reg = "PRESENT_STATE_DEBUG", .fld = "CM_TFR_STATUS", .w = 6, .b = 8, .offs = 0x24C },
	{ .reg = "PRESENT_STATE_DEBUG", .fld = "CM_TFR_ST_STATUS", .w = 6, .b = 16, .offs = 0x24C },
	{ .reg = "PRESENT_STATE_DEBUG", .fld = "CMD_TID", .w = 4, .b = 24, .offs = 0x24C },
	{ .reg = "PRESENT_STATE_DEBUG", .fld = "MASTER_IDLE", .w = 1, .b = 28, .offs = 0x24C },
	{ .reg = "MASTER_EXT_HEADER", .fld = "CAP_ID", .w = 8, .b = 0, .offs = 0x254 },
	{ .reg = "MASTER_EXT_HEADER", .fld = "CAP_LEN", .w = 16, .b = 8, .offs = 0x254 },
	{ .reg = "MASTER_CONFIG", .fld = "APP_IF_MODE", .w = 2, .b = 0, .offs = 0x258 },
	{ .reg = "MASTER_CONFIG", .fld = "APP_IF_DATA_WIDTH", .w = 2, .b = 2, .offs = 0x258 },
	{ .reg = "MASTER_CONFIG", .fld = "OPERATION_MODE", .w = 2, .b = 4, .offs = 0x258 },
};

static const struct fld_info dma_regs_0[] = {
	{ .reg = "CR_SETUP_0", .fld = "RING_SIZE", .w = 8, .b = 0, .offs = 0x180 },
	{ .reg = "CR_SETUP_0", .fld = "Reserved0", .w = 8, .b = 8, .offs = 0x180 },
	{ .reg = "CR_SETUP_0", .fld = "RESP_STRUCT_SIZE", .w = 8, .b = 16, .offs = 0x180 },
	{ .reg = "CR_SETUP_0", .fld = "XFER_STRUCT_SIZE", .w = 8, .b = 24, .offs = 0x180 },
	{ .reg = "IBI_SETUP_0", .fld = "CHUNK_COUNT", .w = 10, .b = 0, .offs = 0x184 },
	{ .reg = "IBI_SETUP_0", .fld = "CHUNK_SIZE", .w = 3, .b = 10, .offs = 0x184 },
	{ .reg = "IBI_SETUP_0", .fld = "Reserved0", .w = 3, .b = 13, .offs = 0x184 },
	{ .reg = "IBI_SETUP_0", .fld = "IBI_STATUS_RING_SIZE", .w = 8, .b = 16, .offs = 0x184 },
	{ .reg = "IBI_SETUP_0", .fld = "IBI_STATUS_STRUCT_SIZE", .w = 8, .b = 24, .offs = 0x184 },
	{ .reg = "CHUNK_CONTROL_0", .fld = "CHUNK_COUNTER", .w = 32, .b = 0, .offs = 0x188 },
	{ .reg = "RH_INTR_STATUS_0", .fld = "SS_RESERVED", .w = 5, .b = 0, .offs = 0x190 },
	{ .reg = "RH_INTR_STATUS_0", .fld = "TRANSFER_ABORT_STAT", .w = 1, .b = 5, .offs = 0x190 },
	{ .reg = "RH_INTR_STATUS_0", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x190 },
	{ .reg = "RH_INTR_STATUS_0", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x190 },
	{ .reg = "RH_INTR_STATUS_0", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x190 },
	{ .reg = "RH_INTR_STATUS_0", .fld = "TRANSFER_ERR_STAT", .w = 1, .b = 9, .offs = 0x190 },
	{ .reg = "RH_INTR_STATUS_0", .fld = "RING_OP_STAT", .w = 1, .b = 10, .offs = 0x190 },
	{ .reg = "RH_INTR_STATUS_0", .fld = "TRANSFER_COMPLETION_STAT", .w = 1, .b = 11, .offs = 0x190 },
	{ .reg = "RH_INTR_STATUS_0", .fld = "IBI_READY_STAT", .w = 1, .b = 12, .offs = 0x190 },
	{ .reg = "RH_INTR_STATUS_0", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x190 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "SS_RESERVED4", .w = 1, .b = 0, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "SS_RESERVED3", .w = 1, .b = 1, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "SS_RESERVED2", .w = 1, .b = 2, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "SS_RESERVED1", .w = 1, .b = 3, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "SS_RESERVED0", .w = 1, .b = 4, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "TRANSFER_ABORT_STAT_EN", .w = 1, .b = 5, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "TRANSFER_ERR_STAT_EN", .w = 1, .b = 9, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "RING_OP_EN", .w = 1, .b = 10, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "TRANSFER_COMPLETION_EN", .w = 1, .b = 11, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "IBI_READY_EN", .w = 1, .b = 12, .offs = 0x194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_0", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x194 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "SS_RESERVED4", .w = 1, .b = 0, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "SS_RESERVED3", .w = 1, .b = 1, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "SS_RESERVED2", .w = 1, .b = 2, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "SS_RESERVED1", .w = 1, .b = 3, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "SS_RESERVED0", .w = 1, .b = 4, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "TRANSFER_ABORT_SIGNAL_EN", .w = 1, .b = 5, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "TRANSFER_ERR_SIGNAL_EN", .w = 1, .b = 9, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "RING_OP_EN", .w = 1, .b = 10, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "TRANSFER_COMPLETION_EN", .w = 1, .b = 11, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "IBI_READY_EN", .w = 1, .b = 12, .offs = 0x198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_0", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x198 },
	{ .reg = "RH_INTR_FORCE_0", .fld = "SS_RESERVED4", .w = 1, .b = 0, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "SS_RESERVED3", .w = 1, .b = 1, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "SS_RESERVED2", .w = 1, .b = 2, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "SS_RESERVED1", .w = 1, .b = 3, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "SS_RESERVED0", .w = 1, .b = 4, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "TRANSFER_ABORT_FORCE", .w = 1, .b = 5, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "TRANSFER_ERR_FORCE", .w = 1, .b = 9, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "RING_OP_FORCE", .w = 1, .b = 10, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "TRANSFER_COMPLETION_FORCE", .w = 1, .b = 11, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "IBI_READY_FORCE", .w = 1, .b = 12, .offs = 0x19C },
	{ .reg = "RH_INTR_FORCE_0", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x19C },
	{ .reg = "RH_STATUS_0", .fld = "ENABLED", .w = 1, .b = 0, .offs = 0x1A0 },
	{ .reg = "RH_STATUS_0", .fld = "RUNNING", .w = 1, .b = 1, .offs = 0x1A0 },
	{ .reg = "RH_STATUS_0", .fld = "ABORTED", .w = 1, .b = 2, .offs = 0x1A0 },
	{ .reg = "RH_STATUS_0", .fld = "LOCKED", .w = 1, .b = 3, .offs = 0x1A0 },
	{ .reg = "RH_STATUS_0", .fld = "Reserved0", .w = 28, .b = 4, .offs = 0x1A0 },
	{ .reg = "RH_CONTROL_0", .fld = "ENABLE", .w = 1, .b = 0, .offs = 0x1A4 },
	{ .reg = "RH_CONTROL_0", .fld = "RS", .w = 1, .b = 1, .offs = 0x1A4 },
	{ .reg = "RH_CONTROL_0", .fld = "ABORT", .w = 1, .b = 2, .offs = 0x1A4 },
	{ .reg = "RH_CONTROL_0", .fld = "Reserved0", .w = 29, .b = 3, .offs = 0x1A4 },
	{ .reg = "RH_OPERATION1_0", .fld = "CR_ENQ_PTR", .w = 8, .b = 0, .offs = 0x1A8 },
	{ .reg = "RH_OPERATION1_0", .fld = "CR_SW_DEQ_PTR", .w = 8, .b = 8, .offs = 0x1A8 },
	{ .reg = "RH_OPERATION1_0", .fld = "IBI_SW_DEQ_PTR", .w = 8, .b = 16, .offs = 0x1A8 },
	{ .reg = "RH_OPERATION1_0", .fld = "Reserved0", .w = 8, .b = 24, .offs = 0x1A8 },
	{ .reg = "RH_OPERATION2_0", .fld = "CR_DEQ_PTR", .w = 8, .b = 0, .offs = 0x1AC },
	{ .reg = "RH_OPERATION2_0", .fld = "Reserved1", .w = 8, .b = 8, .offs = 0x1AC },
	{ .reg = "RH_OPERATION2_0", .fld = "IBI_ENQ_PTR", .w = 8, .b = 16, .offs = 0x1AC },
	{ .reg = "RH_OPERATION2_0", .fld = "Reserved0", .w = 8, .b = 24, .offs = 0x1AC },
	{ .reg = "RH_CMD_RING_BASE_LO_0", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x1B0 },
	{ .reg = "RH_CMD_RING_BASE_HI_0", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x1B4 },
	{ .reg = "RH_RESP_RING_BASE_LO_0", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x1B8 },
	{ .reg = "RH_RESP_RING_BASE_HI_0", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x1BC },
	{ .reg = "RH_IBI_STATUS_RING_BASE_LO_0", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x1C0 },
	{ .reg = "RH_IBI_STATUS_RING_BASE_HI_0", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x1C4 },
	{ .reg = "RH_IBI_DATA_RING_BASE_LO_0", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x1C8 },
	{ .reg = "RH_IBI_DATA_RING_BASE_HI_0", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x1CC },
	{ .reg = "MULT_BUS_INST_EXT_CAP", .fld = "mult_bus_inst_cap_id", .w = 8, .b = 0, .offs = 0x294 },
	{ .reg = "MULT_BUS_INST_EXT_CAP", .fld = "mult_bus_inst_cap_len", .w = 16, .b = 8, .offs = 0x294 },
	{ .reg = "MULT_BUS_INST_EXT_CAP", .fld = "Reserved0", .w = 8, .b = 24, .offs = 0x294 },
	{ .reg = "MULT_BUS_INST_CNT", .fld = "mult_bus_inst_cnt", .w = 4, .b = 0, .offs = 0x298 },
	{ .reg = "MULT_BUS_INST_CNT", .fld = "Reserved0", .w = 28, .b = 4, .offs = 0x298 },
	{ .reg = "MULT_BUS_INST_OFFSET", .fld = "mult_bus_inst_offset", .w = 32, .b = 0, .offs = 0x29C },
	{ .reg = "VND_SPEC_EXT_CAP_2", .fld = "vnd_spc_ext_cap_id", .w = 8, .b = 0, .offs = 0x2B0 },
	{ .reg = "VND_SPEC_EXT_CAP_2", .fld = "vnd_spc_ext_cap_len", .w = 16, .b = 8, .offs = 0x2B0 },
	{ .reg = "VND_SPEC_EXT_CAP_2", .fld = "Reserved0", .w = 8, .b = 24, .offs = 0x2B0 },
	{ .reg = "RESETS", .fld = "i3c_reset", .w = 1, .b = 0, .offs = 0x2B4 },
	{ .reg = "RESETS", .fld = "i3c_reset_done", .w = 1, .b = 1, .offs = 0x2B4 },
	{ .reg = "RESETS", .fld = "Reserved0", .w = 30, .b = 2, .offs = 0x2B4 },
	{ .reg = "GENERAL", .fld = "i3c0_scl_rd_post_drive", .w = 1, .b = 0, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_scl_rd_post_drive", .w = 1, .b = 1, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_scl_rd_pre_drive", .w = 1, .b = 2, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_scl_rd_pre_drive", .w = 1, .b = 3, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_sda_rd_post_drive", .w = 1, .b = 4, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_sda_rd_post_drive", .w = 1, .b = 5, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_sda_rd_pre_drive", .w = 1, .b = 6, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_sda_rd_pre_drive", .w = 1, .b = 7, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_scl_out_rd_drive", .w = 1, .b = 8, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_scl_out_rd_drive", .w = 1, .b = 9, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_sda_out_rd_drive", .w = 1, .b = 10, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_sda_out_rd_drive", .w = 1, .b = 11, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_scl_oe_signal_state", .w = 1, .b = 12, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_scl_oe_signal_state", .w = 1, .b = 13, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_scl_oe_mux_sel", .w = 1, .b = 14, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_scl_oe_mux_sel", .w = 1, .b = 15, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_sda_oe_signal_state", .w = 1, .b = 16, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_sda_oe_signal_state", .w = 1, .b = 17, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_sda_oe_mux_sel", .w = 1, .b = 18, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_sda_oe_mux_sel", .w = 1, .b = 19, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_scl_out_signal_state", .w = 1, .b = 20, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_scl_out_signal_state", .w = 1, .b = 21, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_scl_out_mux_sel", .w = 1, .b = 22, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_scl_out_mux_sel", .w = 1, .b = 23, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_sda_out_signal_state", .w = 1, .b = 24, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_sda_out_signal_state", .w = 1, .b = 25, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c0_sda_out_mux_sel", .w = 1, .b = 26, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "i3c1_sda_out_mux_sel", .w = 1, .b = 27, .offs = 0x2B8 },
	{ .reg = "GENERAL", .fld = "Reserved0", .w = 4, .b = 28, .offs = 0x2B8 },
	{ .reg = "ACTIVELTR", .fld = "snoop_value", .w = 10, .b = 0, .offs = 0x2BC },
	{ .reg = "ACTIVELTR", .fld = "Snoop_latency_scale", .w = 3, .b = 10, .offs = 0x2BC },
	{ .reg = "ACTIVELTR", .fld = "Reserved1", .w = 2, .b = 13, .offs = 0x2BC },
	{ .reg = "ACTIVELTR", .fld = "snoop_requirment", .w = 1, .b = 15, .offs = 0x2BC },
	{ .reg = "ACTIVELTR", .fld = "Non_Snoop_value", .w = 10, .b = 16, .offs = 0x2BC },
	{ .reg = "ACTIVELTR", .fld = "Non_Snoop_latency_scale", .w = 3, .b = 26, .offs = 0x2BC },
	{ .reg = "ACTIVELTR", .fld = "RESERVED0", .w = 2, .b = 29, .offs = 0x2BC },
	{ .reg = "ACTIVELTR", .fld = "Non_Snoop_Requirment", .w = 1, .b = 31, .offs = 0x2BC },
	{ .reg = "IDLELTR", .fld = "snoop_value", .w = 10, .b = 0, .offs = 0x2C0 },
	{ .reg = "IDLELTR", .fld = "Snoop_latency_scale", .w = 3, .b = 10, .offs = 0x2C0 },
	{ .reg = "IDLELTR", .fld = "Reserved1", .w = 2, .b = 13, .offs = 0x2C0 },
	{ .reg = "IDLELTR", .fld = "snoop_requirment", .w = 1, .b = 15, .offs = 0x2C0 },
	{ .reg = "IDLELTR", .fld = "Non_Snoop_value", .w = 10, .b = 16, .offs = 0x2C0 },
	{ .reg = "IDLELTR", .fld = "Non_Snoop_latency_scale", .w = 3, .b = 26, .offs = 0x2C0 },
	{ .reg = "IDLELTR", .fld = "RESERVED0", .w = 2, .b = 29, .offs = 0x2C0 },
	{ .reg = "IDLELTR", .fld = "Non_Snoop_Requirment", .w = 1, .b = 31, .offs = 0x2C0 },
	{ .reg = "SW_SCRATCH_0", .fld = "SW_Scratch_0", .w = 32, .b = 0, .offs = 0x2C4 },
	{ .reg = "CLOCK_GATE", .fld = "Control", .w = 2, .b = 0, .offs = 0x2C8 },
	{ .reg = "CLOCK_GATE", .fld = "reserved0", .w = 30, .b = 2, .offs = 0x2C8 },
	{ .reg = "DEVIDLE_CONTROL", .fld = "cmd_in_progress", .w = 1, .b = 0, .offs = 0x2CC },
	{ .reg = "DEVIDLE_CONTROL", .fld = "intr_req", .w = 1, .b = 1, .offs = 0x2CC },
	{ .reg = "DEVIDLE_CONTROL", .fld = "devidle", .w = 1, .b = 2, .offs = 0x2CC },
	{ .reg = "DEVIDLE_CONTROL", .fld = "restore_required", .w = 1, .b = 3, .offs = 0x2CC },
	{ .reg = "DEVIDLE_CONTROL", .fld = "intr_req_capable", .w = 1, .b = 4, .offs = 0x2CC },
	{ .reg = "DEVIDLE_CONTROL", .fld = "Spare", .w = 27, .b = 5, .offs = 0x2CC },
	{ .reg = "I3C_Threshold", .fld = "i3c0_txd_thld", .w = 3, .b = 0, .offs = 0x2D4 },
	{ .reg = "I3C_Threshold", .fld = "reserved3", .w = 1, .b = 3, .offs = 0x2D4 },
	{ .reg = "I3C_Threshold", .fld = "i3c0_rxd_thld", .w = 3, .b = 4, .offs = 0x2D4 },
	{ .reg = "I3C_Threshold", .fld = "reserved2", .w = 1, .b = 7, .offs = 0x2D4 },
	{ .reg = "I3C_Threshold", .fld = "i3c0_ibi_data_thld", .w = 8, .b = 8, .offs = 0x2D4 },
	{ .reg = "I3C_Threshold", .fld = "i3c1_txd_thld", .w = 3, .b = 16, .offs = 0x2D4 },
	{ .reg = "I3C_Threshold", .fld = "reserved1", .w = 1, .b = 19, .offs = 0x2D4 },
	{ .reg = "I3C_Threshold", .fld = "i3c1_rxd_thld", .w = 3, .b = 20, .offs = 0x2D4 },
	{ .reg = "I3C_Threshold", .fld = "reserved0", .w = 1, .b = 23, .offs = 0x2D4 },
	{ .reg = "I3C_Threshold", .fld = "i3c1_ibi_data_thld", .w = 8, .b = 24, .offs = 0x2D4 },
	{ .reg = "SPARE1", .fld = "SPARE1", .w = 32, .b = 0, .offs = 0x2E4 },
	{ .reg = "SPARE2", .fld = "SPARE2", .w = 32, .b = 0, .offs = 0x2E8 },
	{ .reg = "SPARE3", .fld = "SPARE3", .w = 32, .b = 0, .offs = 0x2EC },
	{ .reg = "DMA_Chkn_Mode", .fld = "DMAC_NO_FLUSH_ON_ABORT", .w = 1, .b = 0, .offs = 0x2F0 },
	{ .reg = "DMA_Chkn_Mode", .fld = "DMAC_NO_CLEAR_CTRL_Q_ON_ABORT", .w = 2, .b = 1, .offs = 0x2F0 },
	{ .reg = "DMA_Chkn_Mode", .fld = "Reserved0", .w = 1, .b = 3, .offs = 0x2F0 },
	{ .reg = "DMA_Chkn_Mode", .fld = "DMAC_ABORT_RING_RESUME", .w = 1, .b = 4, .offs = 0x2F0 },
	{ .reg = "DMA_Chkn_Mode", .fld = "Reserved1", .w = 27, .b = 5, .offs = 0x2F0 },
	{ .reg = "DMA_Debug_Reg0", .fld = "DMAC_FORCE_FLUSH_SIDEQ_HC0", .w = 1, .b = 0, .offs = 0x2F4 },
	{ .reg = "DMA_Debug_Reg0", .fld = "Reserved0", .w = 15, .b = 1, .offs = 0x2F4 },
	{ .reg = "DMA_Debug_Reg0", .fld = "DMAC_ABORT_RING_RESUME", .w = 1, .b = 16, .offs = 0x2F4 },
	{ .reg = "DMA_Debug_Reg0", .fld = "Reserved1", .w = 15, .b = 17, .offs = 0x2F4 },
	{ .reg = "DMA_Debug_Reg1", .fld = "DS_FSM_CH_STATE", .w = 5, .b = 0, .offs = 0x2F8 },
	{ .reg = "DMA_Debug_Reg1", .fld = "DS_FSM_RING_ID_HCI_ID", .w = 2, .b = 5, .offs = 0x2F8 },
	{ .reg = "DMA_Debug_Reg1", .fld = "DS_FSM_WR_STATE", .w = 2, .b = 7, .offs = 0x2F8 },
	{ .reg = "DMA_Debug_Reg1", .fld = "DS_FSM_HW_HS_STATE", .w = 2, .b = 9, .offs = 0x2F8 },
	{ .reg = "DMA_Debug_Reg1", .fld = "DS_FSM_SIDEQ_STATE", .w = 2, .b = 11, .offs = 0x2F8 },
	{ .reg = "DMA_Debug_Reg1", .fld = "Debug_Reg0_RSVD1", .w = 3, .b = 13, .offs = 0x2F8 },
	{ .reg = "DMA_Debug_Reg1", .fld = "US_FSM_CH_STATE", .w = 5, .b = 16, .offs = 0x2F8 },
	{ .reg = "DMA_Debug_Reg1", .fld = "US_FSM_SIDE_IBI_HCI_ID", .w = 2, .b = 21, .offs = 0x2F8 },
	{ .reg = "DMA_Debug_Reg1", .fld = "US_FSM_WR_STATE", .w = 2, .b = 23, .offs = 0x2F8 },
	{ .reg = "DMA_Debug_Reg1", .fld = "US_FSM_HW_HS_STATE", .w = 2, .b = 25, .offs = 0x2F8 },
	{ .reg = "DMA_Debug_Reg1", .fld = "US_FSM_SIDEQ_STATE", .w = 2, .b = 27, .offs = 0x2F8 },
	{ .reg = "DMA_Debug_Reg1", .fld = "Debug_Reg0_RSVD2", .w = 3, .b = 29, .offs = 0x2F8 },
	{ .reg = "DMA_Debug_Reg2", .fld = "DS_CH_R0H0_INT_ENQPTR", .w = 8, .b = 0, .offs = 0x2FC },
	{ .reg = "DMA_Debug_Reg2", .fld = "DS_CH_R1H0_INT_ENQPTR", .w = 8, .b = 8, .offs = 0x2FC },
	{ .reg = "DMA_Debug_Reg2", .fld = "DS_CH_R0H1_INT_ENQPTR", .w = 8, .b = 16, .offs = 0x2FC },
	{ .reg = "DMA_Debug_Reg2", .fld = "DS_CH_R1H1_INT_ENQPTR", .w = 8, .b = 24, .offs = 0x2FC },
	{ .reg = "DMA_Debug_Reg3", .fld = "US_CH_R0H0_INT_CHNK_CNTR", .w = 32, .b = 0, .offs = 0x300 },
	{ .reg = "DMA_Debug_Reg4", .fld = "US_CH_R1H0_INT_CHNK_CNTR", .w = 32, .b = 0, .offs = 0x304 },
	{ .reg = "DMA_Debug_Reg5", .fld = "US_CH_R0H1_INT_CHNK_CNTR", .w = 32, .b = 0, .offs = 0x308 },
	{ .reg = "DMA_Debug_Reg6", .fld = "US_CH_R1H1_INT_CHNK_CNTR", .w = 32, .b = 0, .offs = 0x30C },
	{ .reg = "DMA_Debug_Reg7", .fld = "US_CH_R0H0_RSP_PKT", .w = 32, .b = 0, .offs = 0x310 },
	{ .reg = "DMA_Debug_Reg8", .fld = "US_CH_R1H0_RSP_PKT", .w = 32, .b = 0, .offs = 0x314 },
	{ .reg = "DMA_Debug_Reg9", .fld = "US_CH_R0H1_RSP_PKT", .w = 32, .b = 0, .offs = 0x318 },
	{ .reg = "DMA_Debug_Reg10", .fld = "US_CH_R1H1_RSP_PKT", .w = 32, .b = 0, .offs = 0x31C },
	{ .reg = "DMA_Debug_Reg11", .fld = "IBI_STS_PKT_HCI0", .w = 32, .b = 0, .offs = 0x320 },
	{ .reg = "DMA_Debug_Reg12", .fld = "IBI_STS_PKT_HCI1", .w = 32, .b = 0, .offs = 0x324 },
	{ .reg = "DMA_Debug_Reg13", .fld = "XFER_DESC_BUF_Lo_R0H0", .w = 32, .b = 0, .offs = 0x328 },
	{ .reg = "DMA_Debug_Reg14", .fld = "XFER_DESC_BUF_Hi_R0H0", .w = 32, .b = 0, .offs = 0x32C },
	{ .reg = "DMA_Debug_Reg15", .fld = "XFER_DESC_BUF_Lo_R1H0", .w = 32, .b = 0, .offs = 0x330 },
	{ .reg = "DMA_Debug_Reg16", .fld = "XFER_DESC_BUF_Hi_R1H0", .w = 32, .b = 0, .offs = 0x334 },
	{ .reg = "DMA_Debug_Reg17", .fld = "XFER_DESC_BUF_Lo_R0H1", .w = 32, .b = 0, .offs = 0x338 },
	{ .reg = "DMA_Debug_Reg18", .fld = "XFER_DESC_BUF_Hi_R0H1", .w = 32, .b = 0, .offs = 0x33C },
	{ .reg = "DMA_Debug_Reg19", .fld = "XFER_DESC_BUF_Lo_R1H0", .w = 32, .b = 0, .offs = 0x340 },
	{ .reg = "DMA_Debug_Reg20", .fld = "XFER_DESC_BUF_Hi_R1H0", .w = 32, .b = 0, .offs = 0x344 },
	{ .reg = "DMA_Debug_Reg21", .fld = "XFER_DESC_BUF_Link_Lo_R0H0", .w = 32, .b = 0, .offs = 0x348 },
	{ .reg = "DMA_Debug_Reg22", .fld = "XFER_DESC_BUF_LINK_Hi_R0H0", .w = 32, .b = 0, .offs = 0x34C },
	{ .reg = "DMA_Debug_Reg23", .fld = "XFER_DESC_BUF_LINK_Lo_R1H0", .w = 32, .b = 0, .offs = 0x350 },
	{ .reg = "DMA_Debug_Reg24", .fld = "XFER_DESC_BUF_LINK_Hi_R1H0", .w = 32, .b = 0, .offs = 0x354 },
	{ .reg = "DMA_Debug_Reg25", .fld = "XFER_DESC_BUF_LINK_Lo_R0H1", .w = 32, .b = 0, .offs = 0x358 },
	{ .reg = "DMA_Debug_Reg26", .fld = "XFER_DESC_BUF_LINK_Hi_R0H1", .w = 32, .b = 0, .offs = 0x35C },
	{ .reg = "DMA_Debug_Reg27", .fld = "XFER_DESC_BUF_LINK_Lo_R1H0", .w = 32, .b = 0, .offs = 0x360 },
	{ .reg = "DMA_Debug_Reg28", .fld = "XFER_DESC_BUF_LINK_Hi_R1H0", .w = 32, .b = 0, .offs = 0x364 },
	{ .reg = "DMA_Debug_Reg29", .fld = "XFER_DESC_BLK_SIZE_LIST_CNT_R0H0", .w = 16, .b = 0, .offs = 0x368 },
	{ .reg = "DMA_Debug_Reg29", .fld = "XFER_DESC_CURR_LIST_ENTRY_R0H0", .w = 16, .b = 16, .offs = 0x368 },
	{ .reg = "DMA_Debug_Reg30", .fld = "XFER_DESC_BLK_SIZE_LIST_CNT_R1H0", .w = 16, .b = 0, .offs = 0x36C },
	{ .reg = "DMA_Debug_Reg30", .fld = "XFER_DESC_CURR_LIST_ENTRY_R1H0", .w = 16, .b = 16, .offs = 0x36C },
	{ .reg = "DMA_Debug_Reg31", .fld = "XFER_DESC_BLK_SIZE_LIST_CNT_R0H1", .w = 16, .b = 0, .offs = 0x370 },
	{ .reg = "DMA_Debug_Reg31", .fld = "XFER_DESC_CURR_LIST_ENTRY_R0H1", .w = 16, .b = 16, .offs = 0x370 },
	{ .reg = "DMA_Debug_Reg32", .fld = "XFER_DESC_BLK_SIZE_LIST_CNT_R1H1", .w = 16, .b = 0, .offs = 0x374 },
	{ .reg = "DMA_Debug_Reg32", .fld = "XFER_DESC_CURR_LIST_ENTRY_R1H1", .w = 16, .b = 16, .offs = 0x374 },
	{ .reg = "DMA_Debug_Reg33", .fld = "SIDEQ_PKT_POP_DW0", .w = 32, .b = 0, .offs = 0x378 },
	{ .reg = "DMA_Debug_Reg34", .fld = "SIDEQ_PKT_POP_DW1", .w = 32, .b = 0, .offs = 0x37C },
	{ .reg = "DMA_Debug_Reg35", .fld = "SIDEQ_PKT_POP_DW2", .w = 32, .b = 0, .offs = 0x380 },
	{ .reg = "DMA_Debug_Reg36", .fld = "Reserved0", .w = 32, .b = 0, .offs = 0x384 },
	{ .reg = "DMA_Debug_Reg37", .fld = "Reserved0", .w = 32, .b = 0, .offs = 0x388 },
	{ .reg = "DMA_Debug_Reg38", .fld = "Reserved0", .w = 32, .b = 0, .offs = 0x38C },
	{ .reg = "DMA_Debug_Reg39", .fld = "SIDEQ_PKT_TOQ_DW0_HCI0", .w = 32, .b = 0, .offs = 0x390 },
	{ .reg = "DMA_Debug_Reg40", .fld = "SIDEQ_PKT_TOQ_DW1_HCI0", .w = 32, .b = 0, .offs = 0x394 },
	{ .reg = "DMA_Debug_Reg41", .fld = "SIDEQ_PKT_TOQ_DW2_HCI0", .w = 32, .b = 0, .offs = 0x398 },
	{ .reg = "DMA_Debug_Reg42", .fld = "SIDEQ_PKT_TOQ_DW0_HCI1", .w = 32, .b = 0, .offs = 0x39C },
	{ .reg = "DMA_Debug_Reg43", .fld = "SIDEQ_PKT_TOQ_DW1_HCI1", .w = 32, .b = 0, .offs = 0x3A0 },
	{ .reg = "DMA_Debug_Reg44", .fld = "SIDEQ_PKT_TOQ_DW2_HCI1", .w = 32, .b = 0, .offs = 0x3A4 },
	{ .reg = "DMA_Debug_Reg45", .fld = "Reserved0", .w = 32, .b = 0, .offs = 0x3A8 },
	{ .reg = "DMA_Debug_Reg46", .fld = "Reserved0", .w = 32, .b = 0, .offs = 0x3AC },
	{ .reg = "Ext_CAP_End", .fld = "Ext_Cap_End", .w = 32, .b = 0, .offs = 0x3B0 },
	{ .reg = "RHS_CONTROL_0", .fld = "MAX_HEADER_COUNT", .w = 4, .b = 0, .offs = 0x3C0 },
	{ .reg = "RHS_CONTROL_0", .fld = "MAX_HEADER_COUNT_CAPABILITY", .w = 4, .b = 4, .offs = 0x3C0 },
	{ .reg = "RHS_CONTROL_0", .fld = "Reserved0", .w = 8, .b = 8, .offs = 0x3C0 },
	{ .reg = "RHS_CONTROL_0", .fld = "HEADER_SIZE", .w = 8, .b = 16, .offs = 0x3C0 },
	{ .reg = "RHS_CONTROL_0", .fld = "PREAMBLE_SIZE", .w = 8, .b = 24, .offs = 0x3C0 },
	{ .reg = "RH0_OFFSET_0", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x3C4 },
	{ .reg = "RH1_OFFSET_0", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x3C8 },
	{ .reg = "RH2_OFFSET_0", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x3CC },
	{ .reg = "RH3_OFFSET_0", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x3D0 },
	{ .reg = "RH4_OFFSET_0", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x3D4 },
	{ .reg = "RH5_OFFSET_0", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x3D8 },
	{ .reg = "RH6_OFFSET_0", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x3DC },
	{ .reg = "RH7_OFFSET_0", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x3E0 },
#if 0
	{ .reg = "CR_SETUP_1", .fld = "RING_SIZE", .w = 8, .b = 0, .offs = 0x1180 },
	{ .reg = "CR_SETUP_1", .fld = "Reserved0", .w = 8, .b = 8, .offs = 0x1180 },
	{ .reg = "CR_SETUP_1", .fld = "RESP_STRUCT_SIZE", .w = 8, .b = 16, .offs = 0x1180 },
	{ .reg = "CR_SETUP_1", .fld = "XFER_STRUCT_SIZE", .w = 8, .b = 24, .offs = 0x1180 },
	{ .reg = "IBI_SETUP_1", .fld = "CHUNK_COUNT", .w = 10, .b = 0, .offs = 0x1184 },
	{ .reg = "IBI_SETUP_1", .fld = "CHUNK_SIZE", .w = 3, .b = 10, .offs = 0x1184 },
	{ .reg = "IBI_SETUP_1", .fld = "Reserved0", .w = 3, .b = 13, .offs = 0x1184 },
	{ .reg = "IBI_SETUP_1", .fld = "IBI_STATUS_RING_SIZE", .w = 8, .b = 16, .offs = 0x1184 },
	{ .reg = "IBI_SETUP_1", .fld = "IBI_STATUS_STRUCT_SIZE", .w = 8, .b = 24, .offs = 0x1184 },
	{ .reg = "CHUNK_CONTROL_1", .fld = "CHUNK_COUNTER", .w = 32, .b = 0, .offs = 0x1188 },
	{ .reg = "RH_INTR_STATUS_1", .fld = "SS_RESERVED", .w = 5, .b = 0, .offs = 0x1190 },
	{ .reg = "RH_INTR_STATUS_1", .fld = "TRANSFER_ABORT_STAT", .w = 1, .b = 5, .offs = 0x1190 },
	{ .reg = "RH_INTR_STATUS_1", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x1190 },
	{ .reg = "RH_INTR_STATUS_1", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x1190 },
	{ .reg = "RH_INTR_STATUS_1", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x1190 },
	{ .reg = "RH_INTR_STATUS_1", .fld = "TRANSFER_ERR_STAT", .w = 1, .b = 9, .offs = 0x1190 },
	{ .reg = "RH_INTR_STATUS_1", .fld = "RING_OP_STAT", .w = 1, .b = 10, .offs = 0x1190 },
	{ .reg = "RH_INTR_STATUS_1", .fld = "TRANSFER_COMPLETION_STAT", .w = 1, .b = 11, .offs = 0x1190 },
	{ .reg = "RH_INTR_STATUS_1", .fld = "IBI_READY_STAT", .w = 1, .b = 12, .offs = 0x1190 },
	{ .reg = "RH_INTR_STATUS_1", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x1190 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "SS_RESERVED4", .w = 1, .b = 0, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "SS_RESERVED3", .w = 1, .b = 1, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "SS_RESERVED2", .w = 1, .b = 2, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "SS_RESERVED1", .w = 1, .b = 3, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "SS_RESERVED0", .w = 1, .b = 4, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "TRANSFER_ABORT_STAT_EN", .w = 1, .b = 5, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "TRANSFER_ERR_STAT_EN", .w = 1, .b = 9, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "RING_OP_EN", .w = 1, .b = 10, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "TRANSFER_COMPLETION_EN", .w = 1, .b = 11, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "IBI_READY_EN", .w = 1, .b = 12, .offs = 0x1194 },
	{ .reg = "RH_INTR_STATUS_ENABLE_1", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x1194 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "SS_RESERVED4", .w = 1, .b = 0, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "SS_RESERVED3", .w = 1, .b = 1, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "SS_RESERVED2", .w = 1, .b = 2, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "SS_RESERVED1", .w = 1, .b = 3, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "SS_RESERVED0", .w = 1, .b = 4, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "TRANSFER_ABORT_SIGNAL_EN", .w = 1, .b = 5, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "TRANSFER_ERR_SIGNAL_EN", .w = 1, .b = 9, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "RING_OP_EN", .w = 1, .b = 10, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "TRANSFER_COMPLETION_EN", .w = 1, .b = 11, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "IBI_READY_EN", .w = 1, .b = 12, .offs = 0x1198 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_1", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x1198 },
	{ .reg = "RH_INTR_FORCE_1", .fld = "SS_RESERVED4", .w = 1, .b = 0, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "SS_RESERVED3", .w = 1, .b = 1, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "SS_RESERVED2", .w = 1, .b = 2, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "SS_RESERVED1", .w = 1, .b = 3, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "SS_RESERVED0", .w = 1, .b = 4, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "TRANSFER_ABORT_FORCE", .w = 1, .b = 5, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "TRANSFER_ERR_FORCE", .w = 1, .b = 9, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "RING_OP_FORCE", .w = 1, .b = 10, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "TRANSFER_COMPLETION_FORCE", .w = 1, .b = 11, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "IBI_READY_FORCE", .w = 1, .b = 12, .offs = 0x119C },
	{ .reg = "RH_INTR_FORCE_1", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x119C },
	{ .reg = "RH_STATUS_1", .fld = "ENABLED", .w = 1, .b = 0, .offs = 0x11A0 },
	{ .reg = "RH_STATUS_1", .fld = "RUNNING", .w = 1, .b = 1, .offs = 0x11A0 },
	{ .reg = "RH_STATUS_1", .fld = "ABORTED", .w = 1, .b = 2, .offs = 0x11A0 },
	{ .reg = "RH_STATUS_1", .fld = "LOCKED", .w = 1, .b = 3, .offs = 0x11A0 },
	{ .reg = "RH_STATUS_1", .fld = "Reserved0", .w = 28, .b = 4, .offs = 0x11A0 },
	{ .reg = "RH_CONTROL_1", .fld = "ENABLE", .w = 1, .b = 0, .offs = 0x11A4 },
	{ .reg = "RH_CONTROL_1", .fld = "RS", .w = 1, .b = 1, .offs = 0x11A4 },
	{ .reg = "RH_CONTROL_1", .fld = "ABORT", .w = 1, .b = 2, .offs = 0x11A4 },
	{ .reg = "RH_CONTROL_1", .fld = "Reserved0", .w = 29, .b = 3, .offs = 0x11A4 },
	{ .reg = "RH_OPERATION1_1", .fld = "CR_ENQ_PTR", .w = 8, .b = 0, .offs = 0x11A8 },
	{ .reg = "RH_OPERATION1_1", .fld = "CR_SW_DEQ_PTR", .w = 8, .b = 8, .offs = 0x11A8 },
	{ .reg = "RH_OPERATION1_1", .fld = "IBI_SW_DEQ_PTR", .w = 8, .b = 16, .offs = 0x11A8 },
	{ .reg = "RH_OPERATION1_1", .fld = "Reserved0", .w = 8, .b = 24, .offs = 0x11A8 },
	{ .reg = "RH_OPERATION2_1", .fld = "CR_DEQ_PTR", .w = 8, .b = 0, .offs = 0x11AC },
	{ .reg = "RH_OPERATION2_1", .fld = "Reserved1", .w = 8, .b = 8, .offs = 0x11AC },
	{ .reg = "RH_OPERATION2_1", .fld = "IBI_ENQ_PTR", .w = 8, .b = 16, .offs = 0x11AC },
	{ .reg = "RH_OPERATION2_1", .fld = "Reserved0", .w = 8, .b = 24, .offs = 0x11AC },
	{ .reg = "RH_CMD_RING_BASE_LO_1", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x11B0 },
	{ .reg = "RH_CMD_RING_BASE_HI_1", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x11B4 },
	{ .reg = "RH_RESP_RING_BASE_LO_1", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x11B8 },
	{ .reg = "RH_RESP_RING_BASE_HI_1", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x11BC },
	{ .reg = "RH_IBI_STATUS_RING_BASE_LO_1", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x11C0 },
	{ .reg = "RH_IBI_STATUS_RING_BASE_HI_1", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x11C4 },
	{ .reg = "RH_IBI_DATA_RING_BASE_LO_1", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x11C8 },
	{ .reg = "RH_IBI_DATA_RING_BASE_HI_1", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x11CC },
#endif
};

static const struct fld_info dma_regs_1[] = {
	{ .reg = "CR_SETUP_2", .fld = "RING_SIZE", .w = 8, .b = 0, .offs = 0x580 },
	{ .reg = "CR_SETUP_2", .fld = "Reserved0", .w = 8, .b = 8, .offs = 0x580 },
	{ .reg = "CR_SETUP_2", .fld = "RESP_STRUCT_SIZE", .w = 8, .b = 16, .offs = 0x580 },
	{ .reg = "CR_SETUP_2", .fld = "XFER_STRUCT_SIZE", .w = 8, .b = 24, .offs = 0x580 },
	{ .reg = "IBI_SETUP_2", .fld = "CHUNK_COUNT", .w = 10, .b = 0, .offs = 0x584 },
	{ .reg = "IBI_SETUP_2", .fld = "CHUNK_SIZE", .w = 3, .b = 10, .offs = 0x584 },
	{ .reg = "IBI_SETUP_2", .fld = "Reserved0", .w = 3, .b = 13, .offs = 0x584 },
	{ .reg = "IBI_SETUP_2", .fld = "IBI_STATUS_RING_SIZE", .w = 8, .b = 16, .offs = 0x584 },
	{ .reg = "IBI_SETUP_2", .fld = "IBI_STATUS_STRUCT_SIZE", .w = 8, .b = 24, .offs = 0x584 },
	{ .reg = "CHUNK_CONTROL_2", .fld = "CHUNK_COUNTER", .w = 32, .b = 0, .offs = 0x588 },
	{ .reg = "RH_INTR_STATUS_2", .fld = "SS_RESERVED", .w = 5, .b = 0, .offs = 0x590 },
	{ .reg = "RH_INTR_STATUS_2", .fld = "TRANSFER_ABORT_STAT", .w = 1, .b = 5, .offs = 0x590 },
	{ .reg = "RH_INTR_STATUS_2", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x590 },
	{ .reg = "RH_INTR_STATUS_2", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x590 },
	{ .reg = "RH_INTR_STATUS_2", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x590 },
	{ .reg = "RH_INTR_STATUS_2", .fld = "TRANSFER_ERR_STAT", .w = 1, .b = 9, .offs = 0x590 },
	{ .reg = "RH_INTR_STATUS_2", .fld = "RING_OP_STAT", .w = 1, .b = 10, .offs = 0x590 },
	{ .reg = "RH_INTR_STATUS_2", .fld = "TRANSFER_COMPLETION_STAT", .w = 1, .b = 11, .offs = 0x590 },
	{ .reg = "RH_INTR_STATUS_2", .fld = "IBI_READY_STAT", .w = 1, .b = 12, .offs = 0x590 },
	{ .reg = "RH_INTR_STATUS_2", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x590 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "SS_RESERVED4", .w = 1, .b = 0, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "SS_RESERVED3", .w = 1, .b = 1, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "SS_RESERVED2", .w = 1, .b = 2, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "SS_RESERVED1", .w = 1, .b = 3, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "SS_RESERVED0", .w = 1, .b = 4, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "TRANSFER_ABORT_STAT_EN", .w = 1, .b = 5, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "TRANSFER_ERR_STAT_EN", .w = 1, .b = 9, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "RING_OP_EN", .w = 1, .b = 10, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "TRANSFER_COMPLETION_EN", .w = 1, .b = 11, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "IBI_READY_EN", .w = 1, .b = 12, .offs = 0x594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_2", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x594 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "SS_RESERVED4", .w = 1, .b = 0, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "SS_RESERVED3", .w = 1, .b = 1, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "SS_RESERVED2", .w = 1, .b = 2, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "SS_RESERVED1", .w = 1, .b = 3, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "SS_RESERVED0", .w = 1, .b = 4, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "TRANSFER_ABORT_SIGNAL_EN", .w = 1, .b = 5, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "TRANSFER_ERR_SIGNAL_EN", .w = 1, .b = 9, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "RING_OP_EN", .w = 1, .b = 10, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "TRANSFER_COMPLETION_EN", .w = 1, .b = 11, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "IBI_READY_EN", .w = 1, .b = 12, .offs = 0x598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_2", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x598 },
	{ .reg = "RH_INTR_FORCE_2", .fld = "SS_RESERVED4", .w = 1, .b = 0, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "SS_RESERVED3", .w = 1, .b = 1, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "SS_RESERVED2", .w = 1, .b = 2, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "SS_RESERVED1", .w = 1, .b = 3, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "SS_RESERVED0", .w = 1, .b = 4, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "TRANSFER_ABORT_FORCE", .w = 1, .b = 5, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "TRANSFER_ERR_FORCE", .w = 1, .b = 9, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "RING_OP_FORCE", .w = 1, .b = 10, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "TRANSFER_COMPLETION_FORCE", .w = 1, .b = 11, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "IBI_READY_FORCE", .w = 1, .b = 12, .offs = 0x59C },
	{ .reg = "RH_INTR_FORCE_2", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x59C },
	{ .reg = "RH_STATUS_2", .fld = "ENABLED", .w = 1, .b = 0, .offs = 0x5A0 },
	{ .reg = "RH_STATUS_2", .fld = "RUNNING", .w = 1, .b = 1, .offs = 0x5A0 },
	{ .reg = "RH_STATUS_2", .fld = "ABORTED", .w = 1, .b = 2, .offs = 0x5A0 },
	{ .reg = "RH_STATUS_2", .fld = "LOCKED", .w = 1, .b = 3, .offs = 0x5A0 },
	{ .reg = "RH_STATUS_2", .fld = "Reserved0", .w = 28, .b = 4, .offs = 0x5A0 },
	{ .reg = "RH_CONTROL_2", .fld = "ENABLE", .w = 1, .b = 0, .offs = 0x5A4 },
	{ .reg = "RH_CONTROL_2", .fld = "RS", .w = 1, .b = 1, .offs = 0x5A4 },
	{ .reg = "RH_CONTROL_2", .fld = "ABORT", .w = 1, .b = 2, .offs = 0x5A4 },
	{ .reg = "RH_CONTROL_2", .fld = "Reserved0", .w = 29, .b = 3, .offs = 0x5A4 },
	{ .reg = "RH_OPERATION1_2", .fld = "CR_ENQ_PTR", .w = 8, .b = 0, .offs = 0x5A8 },
	{ .reg = "RH_OPERATION1_2", .fld = "CR_SW_DEQ_PTR", .w = 8, .b = 8, .offs = 0x5A8 },
	{ .reg = "RH_OPERATION1_2", .fld = "IBI_SW_DEQ_PTR", .w = 8, .b = 16, .offs = 0x5A8 },
	{ .reg = "RH_OPERATION1_2", .fld = "Reserved0", .w = 8, .b = 24, .offs = 0x5A8 },
	{ .reg = "RH_OPERATION2_2", .fld = "CR_DEQ_PTR", .w = 8, .b = 0, .offs = 0x5AC },
	{ .reg = "RH_OPERATION2_2", .fld = "Reserved1", .w = 8, .b = 8, .offs = 0x5AC },
	{ .reg = "RH_OPERATION2_2", .fld = "IBI_ENQ_PTR", .w = 8, .b = 16, .offs = 0x5AC },
	{ .reg = "RH_OPERATION2_2", .fld = "Reserved0", .w = 8, .b = 24, .offs = 0x5AC },
	{ .reg = "RH_CMD_RING_BASE_LO_2", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x5B0 },
	{ .reg = "RH_CMD_RING_BASE_HI_2", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x5B4 },
	{ .reg = "RH_RESP_RING_BASE_LO_2", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x5B8 },
	{ .reg = "RH_RESP_RING_BASE_HI_2", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x5BC },
	{ .reg = "RH_IBI_STATUS_RING_BASE_LO_2", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x5C0 },
	{ .reg = "RH_IBI_STATUS_RING_BASE_HI_2", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x5C4 },
	{ .reg = "RH_IBI_DATA_RING_BASE_LO_2", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x5C8 },
	{ .reg = "RH_IBI_DATA_RING_BASE_HI_2", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x5CC },
	{ .reg = "RHS_CONTROL_1", .fld = "MAX_HEADER_COUNT", .w = 4, .b = 0, .offs = 0x7C0 },
	{ .reg = "RHS_CONTROL_1", .fld = "MAX_HEADER_COUNT_CAPABILITY", .w = 4, .b = 4, .offs = 0x7C0 },
	{ .reg = "RHS_CONTROL_1", .fld = "Reserved0", .w = 8, .b = 8, .offs = 0x7C0 },
	{ .reg = "RHS_CONTROL_1", .fld = "HEADER_SIZE", .w = 8, .b = 16, .offs = 0x7C0 },
	{ .reg = "RHS_CONTROL_1", .fld = "PREAMBLE_SIZE", .w = 8, .b = 24, .offs = 0x7C0 },
	{ .reg = "RH0_OFFSET_1", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x7C4 },
	{ .reg = "RH1_OFFSET_1", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x7C8 },
	{ .reg = "RH2_OFFSET_1", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x7CC },
	{ .reg = "RH3_OFFSET_1", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x7D0 },
	{ .reg = "RH4_OFFSET_1", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x7D4 },
	{ .reg = "RH5_OFFSET_1", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x7D8 },
	{ .reg = "RH6_OFFSET_1", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x7DC },
	{ .reg = "RH7_OFFSET_1", .fld = "OFFSET", .w = 32, .b = 0, .offs = 0x7E0 },
	{ .reg = "CR_SETUP_3", .fld = "RING_SIZE", .w = 8, .b = 0, .offs = 0x1580 },
	{ .reg = "CR_SETUP_3", .fld = "Reserved0", .w = 8, .b = 8, .offs = 0x1580 },
	{ .reg = "CR_SETUP_3", .fld = "RESP_STRUCT_SIZE", .w = 8, .b = 16, .offs = 0x1580 },
	{ .reg = "CR_SETUP_3", .fld = "XFER_STRUCT_SIZE", .w = 8, .b = 24, .offs = 0x1580 },
	{ .reg = "IBI_SETUP_3", .fld = "CHUNK_COUNT", .w = 10, .b = 0, .offs = 0x1584 },
	{ .reg = "IBI_SETUP_3", .fld = "CHUNK_SIZE", .w = 3, .b = 10, .offs = 0x1584 },
	{ .reg = "IBI_SETUP_3", .fld = "Reserved0", .w = 3, .b = 13, .offs = 0x1584 },
	{ .reg = "IBI_SETUP_3", .fld = "IBI_STATUS_RING_SIZE", .w = 8, .b = 16, .offs = 0x1584 },
	{ .reg = "IBI_SETUP_3", .fld = "IBI_STATUS_STRUCT_SIZE", .w = 8, .b = 24, .offs = 0x1584 },
	{ .reg = "CHUNK_CONTROL_3", .fld = "CHUNK_COUNTER", .w = 32, .b = 0, .offs = 0x1588 },
	{ .reg = "RH_INTR_STATUS_3", .fld = "SS_RESERVED", .w = 5, .b = 0, .offs = 0x1590 },
	{ .reg = "RH_INTR_STATUS_3", .fld = "TRANSFER_ABORT_STAT", .w = 1, .b = 5, .offs = 0x1590 },
	{ .reg = "RH_INTR_STATUS_3", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x1590 },
	{ .reg = "RH_INTR_STATUS_3", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x1590 },
	{ .reg = "RH_INTR_STATUS_3", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x1590 },
	{ .reg = "RH_INTR_STATUS_3", .fld = "TRANSFER_ERR_STAT", .w = 1, .b = 9, .offs = 0x1590 },
	{ .reg = "RH_INTR_STATUS_3", .fld = "RING_OP_STAT", .w = 1, .b = 10, .offs = 0x1590 },
	{ .reg = "RH_INTR_STATUS_3", .fld = "TRANSFER_COMPLETION_STAT", .w = 1, .b = 11, .offs = 0x1590 },
	{ .reg = "RH_INTR_STATUS_3", .fld = "IBI_READY_STAT", .w = 1, .b = 12, .offs = 0x1590 },
	{ .reg = "RH_INTR_STATUS_3", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x1590 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "SS_RESERVED4", .w = 1, .b = 0, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "SS_RESERVED3", .w = 1, .b = 1, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "SS_RESERVED2", .w = 1, .b = 2, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "SS_RESERVED1", .w = 1, .b = 3, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "SS_RESERVED0", .w = 1, .b = 4, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "TRANSFER_ABORT_STAT_EN", .w = 1, .b = 5, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "TRANSFER_ERR_STAT_EN", .w = 1, .b = 9, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "RING_OP_EN", .w = 1, .b = 10, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "TRANSFER_COMPLETION_EN", .w = 1, .b = 11, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "IBI_READY_EN", .w = 1, .b = 12, .offs = 0x1594 },
	{ .reg = "RH_INTR_STATUS_ENABLE_3", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x1594 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "SS_RESERVED4", .w = 1, .b = 0, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "SS_RESERVED3", .w = 1, .b = 1, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "SS_RESERVED2", .w = 1, .b = 2, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "SS_RESERVED1", .w = 1, .b = 3, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "SS_RESERVED0", .w = 1, .b = 4, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "TRANSFER_ABORT_SIGNAL_EN", .w = 1, .b = 5, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "TRANSFER_ERR_SIGNAL_EN", .w = 1, .b = 9, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "RING_OP_EN", .w = 1, .b = 10, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "TRANSFER_COMPLETION_EN", .w = 1, .b = 11, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "IBI_READY_EN", .w = 1, .b = 12, .offs = 0x1598 },
	{ .reg = "RH_INTR_SIGNAL_ENABLE_3", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x1598 },
	{ .reg = "RH_INTR_FORCE_3", .fld = "SS_RESERVED4", .w = 1, .b = 0, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "SS_RESERVED3", .w = 1, .b = 1, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "SS_RESERVED2", .w = 1, .b = 2, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "SS_RESERVED1", .w = 1, .b = 3, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "SS_RESERVED0", .w = 1, .b = 4, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "TRANSFER_ABORT_FORCE", .w = 1, .b = 5, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "Reserved3", .w = 1, .b = 6, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "Reserved2", .w = 1, .b = 7, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "Reserved1", .w = 1, .b = 8, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "TRANSFER_ERR_FORCE", .w = 1, .b = 9, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "RING_OP_FORCE", .w = 1, .b = 10, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "TRANSFER_COMPLETION_FORCE", .w = 1, .b = 11, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "IBI_READY_FORCE", .w = 1, .b = 12, .offs = 0x159C },
	{ .reg = "RH_INTR_FORCE_3", .fld = "Reserved0", .w = 19, .b = 13, .offs = 0x159C },
	{ .reg = "RH_STATUS_3", .fld = "ENABLED", .w = 1, .b = 0, .offs = 0x15A0 },
	{ .reg = "RH_STATUS_3", .fld = "RUNNING", .w = 1, .b = 1, .offs = 0x15A0 },
	{ .reg = "RH_STATUS_3", .fld = "ABORTED", .w = 1, .b = 2, .offs = 0x15A0 },
	{ .reg = "RH_STATUS_3", .fld = "LOCKED", .w = 1, .b = 3, .offs = 0x15A0 },
	{ .reg = "RH_STATUS_3", .fld = "Reserved0", .w = 28, .b = 4, .offs = 0x15A0 },
	{ .reg = "RH_CONTROL_3", .fld = "ENABLE", .w = 1, .b = 0, .offs = 0x15A4 },
	{ .reg = "RH_CONTROL_3", .fld = "RS", .w = 1, .b = 1, .offs = 0x15A4 },
	{ .reg = "RH_CONTROL_3", .fld = "ABORT", .w = 1, .b = 2, .offs = 0x15A4 },
	{ .reg = "RH_CONTROL_3", .fld = "Reserved0", .w = 29, .b = 3, .offs = 0x15A4 },
	{ .reg = "RH_OPERATION1_3", .fld = "CR_ENQ_PTR", .w = 8, .b = 0, .offs = 0x15A8 },
	{ .reg = "RH_OPERATION1_3", .fld = "CR_SW_DEQ_PTR", .w = 8, .b = 8, .offs = 0x15A8 },
	{ .reg = "RH_OPERATION1_3", .fld = "IBI_SW_DEQ_PTR", .w = 8, .b = 16, .offs = 0x15A8 },
	{ .reg = "RH_OPERATION1_3", .fld = "Reserved0", .w = 8, .b = 24, .offs = 0x15A8 },
	{ .reg = "RH_OPERATION2_3", .fld = "CR_DEQ_PTR", .w = 8, .b = 0, .offs = 0x15AC },
	{ .reg = "RH_OPERATION2_3", .fld = "Reserved1", .w = 8, .b = 8, .offs = 0x15AC },
	{ .reg = "RH_OPERATION2_3", .fld = "IBI_ENQ_PTR", .w = 8, .b = 16, .offs = 0x15AC },
	{ .reg = "RH_OPERATION2_3", .fld = "Reserved0", .w = 8, .b = 24, .offs = 0x15AC },
	{ .reg = "RH_CMD_RING_BASE_LO_3", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x15B0 },
	{ .reg = "RH_CMD_RING_BASE_HI_3", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x15B4 },
	{ .reg = "RH_RESP_RING_BASE_LO_3", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x15B8 },
	{ .reg = "RH_RESP_RING_BASE_HI_3", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x15BC },
	{ .reg = "RH_IBI_STATUS_RING_BASE_LO_3", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x15C0 },
	{ .reg = "RH_IBI_STATUS_RING_BASE_HI_3", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x15C4 },
	{ .reg = "RH_IBI_DATA_RING_BASE_LO_3", .fld = "BASE_LO", .w = 32, .b = 0, .offs = 0x15C8 },
	{ .reg = "RH_IBI_DATA_RING_BASE_HI_3", .fld = "BASE_HI", .w = 32, .b = 0, .offs = 0x15CC },
};

static void dump_regs(struct i3c_hci *hci, void __iomem *base, const struct fld_info *regs, int nr)
{
	int last_offs = INT_MAX;
	u32 reg = 0, mask, val;

	for (int i = 0; i < nr; i++) {
		const struct fld_info *f = regs + i;

		if (f->offs != last_offs) {
			reg = readl(base + f->offs);
			last_offs = f->offs;
		}
		mask = (f->w > 31) ? 0xffffffff : ((1U << f->w) - 1);
		val = (reg >> f->b) & mask;
		dev_info(&hci->master.dev, "%s: %-33s %-33s %#x\n", __func__, f->reg, f->fld, val);
	}
}

static void debug_dump(struct i3c_hci *hci)
{
	void __iomem *base_regs = hci->base_regs;
	void __iomem *base = (void *)((u64)base_regs & ~(u64)0xfff);

	dump_regs(hci, base_regs, hc_regs, ARRAY_SIZE(hc_regs));
	if (base == base_regs)
		dump_regs(hci, base, dma_regs_0, ARRAY_SIZE(dma_regs_0));
	else
		dump_regs(hci, base, dma_regs_1, ARRAY_SIZE(dma_regs_1));
}

static ssize_t do_daa_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct i3c_hci *hci = file->private_data;
	int ret;

	ret = i3c_master_do_daa(&hci->master);
	dev_info(&hci->master.dev, "i3c_master_do_daa ret %d\n", ret);

	return ret ? -EINVAL : count;
}

static const struct file_operations fops_do_daa = {
	.open	= simple_open,
	.write	= do_daa_write,
};

static ssize_t rstdaa_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct i3c_hci *hci = file->private_data;
	int ret;

	ret = i3c_master_rstdaa(&hci->master);
	dev_info(&hci->master.dev, "i3c_master_rstdaa ret %d\n", ret);

	return ret ? -EINVAL : count;
}

static const struct file_operations fops_rstdaa = {
	.open	= simple_open,
	.write	= rstdaa_write,
};

#include <linux/debugfs.h>

void dma_debugfs_init(struct i3c_hci *hci)
{
	struct dentry *dir = debugfs_create_dir(dev_name(&hci->master.dev), NULL);

	debugfs_create_bool("dump_on_dequeue",     0600, dir, &hci->dump_on_dequeue);
	debugfs_create_bool("dump_after_daa",      0600, dir, &hci->dump_after_daa);
	debugfs_create_bool("dump_after_ccc",      0600, dir, &hci->dump_after_ccc);
	debugfs_create_bool("dump_after_i3c_xfer", 0600, dir, &hci->dump_after_i3c_xfer);

	debugfs_create_file("do_daa", 0200, dir, hci, &fops_do_daa);
	debugfs_create_file("rstdaa", 0200, dir, hci, &fops_rstdaa);
}

void dma_debugfs_exit(struct i3c_hci *hci)
{
	debugfs_lookup_and_remove(dev_name(&hci->master.dev), NULL);
}

void dma_dump(struct i3c_hci *hci, int dump_point)
{
	switch (dump_point) {
	case DUMP_POINT_NO_DUMP:
		break;
	case DUMP_POINT_DEQUEUE:
		if (hci->dump_on_dequeue)
			debug_dump(hci);
		break;
	case DUMP_POINT_DAA:
		if (hci->dump_after_daa)
			debug_dump(hci);
		break;
	case DUMP_POINT_CCC:
		if (hci->dump_after_ccc)
			debug_dump(hci);
		break;
	case DUMP_POINT_I3C_XFER:
		if (hci->dump_after_i3c_xfer)
			debug_dump(hci);
		break;
	default:
		break;
	}
}

static void hci_dma_cleanup(struct i3c_hci *hci)
{
	struct hci_rings_data *rings = hci->io_data;
	struct hci_rh_data *rh;
	unsigned int i;

	if (!rings)
		return;

	for (i = 0; i < rings->total; i++) {
		rh = &rings->headers[i];

		rh_reg_write(INTR_SIGNAL_ENABLE, 0);
		rh_reg_write(RING_CONTROL, 0);
	}

	i3c_hci_sync_irq_inactive(hci);

	for (i = 0; i < rings->total; i++) {
		rh = &rings->headers[i];

		rh_reg_write(CR_SETUP, 0);
		rh_reg_write(IBI_SETUP, 0);
	}

	rhs_reg_write(CONTROL, 0);
}

static void hci_dma_free(void *data)
{
	struct i3c_hci *hci = data;
	struct hci_rings_data *rings = hci->io_data;
	struct hci_rh_data *rh;

	if (!rings)
		return;

	for (int i = 0; i < rings->total; i++) {
		rh = &rings->headers[i];

		if (rh->xfer)
			dma_free_coherent(rings->sysdev,
					  rh->xfer_struct_sz * rh->xfer_entries,
					  rh->xfer, rh->xfer_dma);
		if (rh->resp)
			dma_free_coherent(rings->sysdev,
					  rh->resp_struct_sz * rh->xfer_entries,
					  rh->resp, rh->resp_dma);
		kfree(rh->src_xfers);
		if (rh->ibi_status)
			dma_free_coherent(rings->sysdev,
					  rh->ibi_status_sz * rh->ibi_status_entries,
					  rh->ibi_status, rh->ibi_status_dma);
		if (rh->ibi_data_dma)
			dma_unmap_single(rings->sysdev, rh->ibi_data_dma,
					 rh->ibi_chunk_sz * rh->ibi_chunks_total,
					 DMA_FROM_DEVICE);
		kfree(rh->ibi_data);
	}

	kfree(rings);
	hci->io_data = NULL;
}

static void hci_dma_init_rh(struct i3c_hci *hci, struct hci_rh_data *rh, int i)
{
	u32 regval;

	rh_reg_write(CMD_RING_BASE_LO, lower_32_bits(rh->xfer_dma));
	rh_reg_write(CMD_RING_BASE_HI, upper_32_bits(rh->xfer_dma));
	rh_reg_write(RESP_RING_BASE_LO, lower_32_bits(rh->resp_dma));
	rh_reg_write(RESP_RING_BASE_HI, upper_32_bits(rh->resp_dma));

	regval = FIELD_PREP(CR_RING_SIZE, rh->xfer_entries);
	rh_reg_write(CR_SETUP, regval);

	rh_reg_write(INTR_STATUS_ENABLE, 0xffffffff);
	rh_reg_write(INTR_SIGNAL_ENABLE, INTR_IBI_READY |
					 INTR_TRANSFER_COMPLETION |
					 INTR_RING_OP |
					 INTR_TRANSFER_ERR |
					 INTR_IBI_RING_FULL |
					 INTR_TRANSFER_ABORT);

	if (i >= IBI_RINGS)
		goto ring_ready;

	rh_reg_write(IBI_STATUS_RING_BASE_LO, lower_32_bits(rh->ibi_status_dma));
	rh_reg_write(IBI_STATUS_RING_BASE_HI, upper_32_bits(rh->ibi_status_dma));
	rh_reg_write(IBI_DATA_RING_BASE_LO, lower_32_bits(rh->ibi_data_dma));
	rh_reg_write(IBI_DATA_RING_BASE_HI, upper_32_bits(rh->ibi_data_dma));

	regval = FIELD_PREP(IBI_STATUS_RING_SIZE, rh->ibi_status_entries) |
		 FIELD_PREP(IBI_DATA_CHUNK_SIZE, ilog2(rh->ibi_chunk_sz) - 2) |
		 FIELD_PREP(IBI_DATA_CHUNK_COUNT, rh->ibi_chunks_total);
	rh_reg_write(IBI_SETUP, regval);

	regval = rh_reg_read(INTR_SIGNAL_ENABLE);
	regval |= INTR_IBI_READY;
	rh_reg_write(INTR_SIGNAL_ENABLE, regval);

ring_ready:
	/*
	 * The MIPI I3C HCI specification does not document reset values for
	 * RING_OPERATION1 fields and some controllers (e.g. Intel controllers)
	 * do not reset the values, so ensure the ring pointers are set to zero
	 * here.
	 */
	rh_reg_write(RING_OPERATION1, 0);

	rh_reg_write(RING_CONTROL, RING_CTRL_ENABLE);
	rh_reg_write(RING_CONTROL, RING_CTRL_ENABLE | RING_CTRL_RUN_STOP);

	rh->done_ptr = 0;
	rh->ibi_chunk_ptr = 0;
	rh->xfer_space = rh->xfer_entries;
}

static void hci_dma_init_rings(struct i3c_hci *hci)
{
	struct hci_rings_data *rings = hci->io_data;
	u32 regval;

	void __iomem *base_regs = hci->base_regs;
	void __iomem *base = (void *)((u64)base_regs & ~(u64)0xfff);
	regval = readl(base + 0x2F0); // DMA_Chkn_Mode
	if (regval & BIT(1)) {
		u32 new_regval = regval & ~BIT(1);
		dev_info(&hci->master.dev, "%s: Writing %#x to DMA_Chkn_Mode (was %#x)\n", __func__, new_regval, regval);
		writel(new_regval, base + 0x2F0); // DMA_Chkn_Mode
	}

	regval = FIELD_PREP(MAX_HEADER_COUNT, rings->total);
	rhs_reg_write(CONTROL, regval);

	for (int i = 0; i < rings->total; i++)
		hci_dma_init_rh(hci, &rings->headers[i], i);
}

static void hci_dma_suspend(struct i3c_hci *hci)
{
	struct hci_rings_data *rings = hci->io_data;
	int n = rings ? rings->total : 0;

	for (int i = 0; i < n; i++) {
		struct hci_rh_data *rh = &rings->headers[i];

		rh_reg_write(INTR_SIGNAL_ENABLE, 0);
		rh_reg_write(RING_CONTROL, 0);
	}

	i3c_hci_sync_irq_inactive(hci);
}

static void hci_dma_resume(struct i3c_hci *hci)
{
	struct hci_rings_data *rings = hci->io_data;

	if (rings)
		hci_dma_init_rings(hci);
}

static int hci_dma_init(struct i3c_hci *hci)
{
	struct hci_rings_data *rings;
	struct hci_rh_data *rh;
	struct device *sysdev;
	u32 regval;
	unsigned int i, nr_rings, xfers_sz, resps_sz;
	unsigned int ibi_status_ring_sz, ibi_data_ring_sz;
	int ret;

	/*
	 * Set pointer to a physical device that does DMA and has IOMMU setup
	 * done for it in case of enabled IOMMU and use it with the DMA API.
	 * Here such device is either
	 * "mipi-i3c-hci" platform device (OF/ACPI enumeration) parent or
	 * grandparent (PCI enumeration).
	 */
	sysdev = hci->master.dev.parent;
	if (sysdev->parent && dev_is_pci(sysdev->parent))
		sysdev = sysdev->parent;

	regval = rhs_reg_read(CONTROL);
	nr_rings = FIELD_GET(MAX_HEADER_COUNT_CAP, regval);
	dev_dbg(&hci->master.dev, "%d DMA rings available\n", nr_rings);
	if (unlikely(nr_rings > 8)) {
		dev_err(&hci->master.dev, "number of rings should be <= 8\n");
		nr_rings = 8;
	}
	if (nr_rings > XFER_RINGS)
		nr_rings = XFER_RINGS;
	rings = kzalloc(struct_size(rings, headers, nr_rings), GFP_KERNEL);
	if (!rings)
		return -ENOMEM;
	hci->io_data = rings;
	rings->total = nr_rings;
	rings->sysdev = sysdev;

	for (i = 0; i < rings->total; i++) {
		u32 offset = rhs_reg_read(RHn_OFFSET(i));

		dev_dbg(&hci->master.dev, "Ring %d at offset %#x\n", i, offset);
		ret = -EINVAL;
		if (!offset)
			goto err_out;
		rh = &rings->headers[i];
		rh->regs = hci->base_regs + offset;
		spin_lock_init(&rh->lock);
		init_completion(&rh->op_done);

		rh->xfer_entries = XFER_RING_ENTRIES;

		regval = rh_reg_read(CR_SETUP);
		rh->xfer_struct_sz = FIELD_GET(CR_XFER_STRUCT_SIZE, regval);
		rh->resp_struct_sz = FIELD_GET(CR_RESP_STRUCT_SIZE, regval);
		dev_dbg(&hci->master.dev,
			"xfer_struct_sz = %d, resp_struct_sz = %d",
			rh->xfer_struct_sz, rh->resp_struct_sz);
		xfers_sz = rh->xfer_struct_sz * rh->xfer_entries;
		resps_sz = rh->resp_struct_sz * rh->xfer_entries;

		rh->xfer = dma_alloc_coherent(rings->sysdev, xfers_sz,
					      &rh->xfer_dma, GFP_KERNEL);
		rh->resp = dma_alloc_coherent(rings->sysdev, resps_sz,
					      &rh->resp_dma, GFP_KERNEL);
		rh->src_xfers =
			kmalloc_array(rh->xfer_entries, sizeof(*rh->src_xfers),
				      GFP_KERNEL);
		ret = -ENOMEM;
		if (!rh->xfer || !rh->resp || !rh->src_xfers)
			goto err_out;

		/* IBIs */

		if (i >= IBI_RINGS)
			continue;

		regval = rh_reg_read(IBI_SETUP);
		rh->ibi_status_sz = FIELD_GET(IBI_STATUS_STRUCT_SIZE, regval);
		rh->ibi_status_entries = IBI_STATUS_RING_ENTRIES;
		rh->ibi_chunks_total = IBI_CHUNK_POOL_SIZE;

		rh->ibi_chunk_sz = dma_get_cache_alignment();
		rh->ibi_chunk_sz *= IBI_CHUNK_CACHELINES;
		/*
		 * Round IBI data chunk size to number of bytes supported by
		 * the HW. Chunk size can be 2^n number of DWORDs which is the
		 * same as 2^(n+2) bytes, where n is 0..6.
		 */
		rh->ibi_chunk_sz = umax(4, rh->ibi_chunk_sz);
		rh->ibi_chunk_sz = roundup_pow_of_two(rh->ibi_chunk_sz);
		if (rh->ibi_chunk_sz > 256) {
			ret = -EINVAL;
			goto err_out;
		}

		ibi_status_ring_sz = rh->ibi_status_sz * rh->ibi_status_entries;
		ibi_data_ring_sz = rh->ibi_chunk_sz * rh->ibi_chunks_total;

		rh->ibi_status =
			dma_alloc_coherent(rings->sysdev, ibi_status_ring_sz,
					   &rh->ibi_status_dma, GFP_KERNEL);
		rh->ibi_data = kmalloc(ibi_data_ring_sz, GFP_KERNEL);
		ret = -ENOMEM;
		if (!rh->ibi_status || !rh->ibi_data)
			goto err_out;
		rh->ibi_data_dma =
			dma_map_single(rings->sysdev, rh->ibi_data,
				       ibi_data_ring_sz, DMA_FROM_DEVICE);
		if (dma_mapping_error(rings->sysdev, rh->ibi_data_dma)) {
			rh->ibi_data_dma = 0;
			ret = -ENOMEM;
			goto err_out;
		}
	}

	ret = devm_add_action(hci->master.dev.parent, hci_dma_free, hci);
	if (ret)
		goto err_out;

	hci_dma_init_rings(hci);

	return 0;

err_out:
	hci_dma_free(hci);
	return ret;
}

static void hci_dma_unmap_xfer(struct i3c_hci *hci,
			       struct hci_xfer *xfer_list, unsigned int n)
{
	struct hci_xfer *xfer;
	unsigned int i;

	for (i = 0; i < n; i++) {
		xfer = xfer_list + i;
		if (!xfer->data || !xfer->dma)
			continue;
		i3c_master_dma_unmap_single(xfer->dma);
		xfer->dma = NULL;
	}
}

static struct i3c_dma *hci_dma_map_xfer(struct device *dev, struct hci_xfer *xfer)
{
	enum dma_data_direction dir = xfer->rnw ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	bool need_bounce = device_iommu_mapped(dev) && xfer->rnw && (xfer->data_len & 3);

	return i3c_master_dma_map_single(dev, xfer->data, xfer->data_len, need_bounce, dir);
}

static int hci_dma_map_xfer_list(struct i3c_hci *hci, struct device *dev,
				 struct hci_xfer *xfer_list, int n)
{
	for (int i = 0; i < n; i++) {
		struct hci_xfer *xfer = xfer_list + i;

		if (!xfer->data)
			continue;

		xfer->dma = hci_dma_map_xfer(dev, xfer);
		if (!xfer->dma) {
			hci_dma_unmap_xfer(hci, xfer_list, i);
			return -ENOMEM;
		}
	}

	return 0;
}

static int hci_dma_queue_xfer(struct i3c_hci *hci,
			      struct hci_xfer *xfer_list, int n)
{
	struct hci_rings_data *rings = hci->io_data;
	struct hci_rh_data *rh;
	unsigned int i, ring, enqueue_ptr;
	u32 op1_val, op2_val;
	int ret;

	ret = hci_dma_map_xfer_list(hci, rings->sysdev, xfer_list, n);
	if (ret)
		return ret;

	/* For now we only use ring 0 */
	ring = 0;
	rh = &rings->headers[ring];

	spin_lock_irq(&rh->lock);

	if (n > rh->xfer_space) {
		spin_unlock_irq(&rh->lock);
		hci_dma_unmap_xfer(hci, xfer_list, n);
		return -EBUSY;
	}

	op1_val = rh_reg_read(RING_OPERATION1);
	dev_info(&hci->master.dev, "%s: read op1 %#x\n", __func__, op1_val);
	op2_val = rh_reg_read(RING_OPERATION2);
	dev_info(&hci->master.dev, "%s: read op2 %#x\n", __func__, op2_val);
	enqueue_ptr = FIELD_GET(RING_OP1_CR_ENQ_PTR, op1_val);
	for (i = 0; i < n; i++) {
		struct hci_xfer *xfer = xfer_list + i;
		u32 *ring_data = rh->xfer + rh->xfer_struct_sz * enqueue_ptr;

		dev_info(&hci->master.dev, "%s: enqueue_ptr %u tid %u\n", __func__, enqueue_ptr, xfer->cmd_tid);

		/* store cmd descriptor */
		*ring_data++ = xfer->cmd_desc[0];
		*ring_data++ = xfer->cmd_desc[1];
		if (hci->cmd == &mipi_i3c_hci_cmd_v2) {
			*ring_data++ = xfer->cmd_desc[2];
			*ring_data++ = xfer->cmd_desc[3];
		}

		/* first word of Data Buffer Descriptor Structure */
		if (!xfer->data)
			xfer->data_len = 0;
		*ring_data++ =
			FIELD_PREP(DATA_BUF_BLOCK_SIZE, xfer->data_len) |
			((i == n - 1) ? DATA_BUF_IOC : 0);

		/* 2nd and 3rd words of Data Buffer Descriptor Structure */
		if (xfer->data) {
			*ring_data++ = lower_32_bits(xfer->dma->addr);
			*ring_data++ = upper_32_bits(xfer->dma->addr);
		} else {
			*ring_data++ = 0;
			*ring_data++ = 0;
		}

		/* remember corresponding xfer struct */
		rh->src_xfers[enqueue_ptr] = xfer;
		/* remember corresponding ring/entry for this xfer structure */
		xfer->ring_number = ring;
		xfer->ring_entry = enqueue_ptr;

		enqueue_ptr = (enqueue_ptr + 1) % rh->xfer_entries;
	}

	rh->xfer_space -= n;

	op1_val &= ~RING_OP1_CR_ENQ_PTR;
	op1_val |= FIELD_PREP(RING_OP1_CR_ENQ_PTR, enqueue_ptr);
	dev_info(&hci->master.dev, "%s: write op1 %#x\n", __func__, op1_val);
	rh_reg_write(RING_OPERATION1, op1_val);
	spin_unlock_irq(&rh->lock);

	return 0;
}

static bool hci_dma_dequeue_xfer(struct i3c_hci *hci,
				 struct hci_xfer *xfer_list, int n)
{
	struct hci_rings_data *rings = hci->io_data;
	struct hci_rh_data *rh = &rings->headers[xfer_list[0].ring_number];
	unsigned int i;
	bool did_unqueue = false;
	u32 ring_status;

	dma_dump(hci, DUMP_POINT_DEQUEUE);

	ring_status = rh_reg_read(RING_STATUS);
	if (ring_status & RING_STATUS_RUNNING) {
		/* stop the ring */
		reinit_completion(&rh->op_done);
		rh_reg_write(RING_CONTROL, RING_CTRL_ABORT);
		wait_for_completion_timeout(&rh->op_done, HZ);
		ring_status = rh_reg_read(RING_STATUS);
		if (ring_status & RING_STATUS_RUNNING) {
			/*
			 * We're deep in it if ever this condition is ever met.
			 * Hardware might still be writing to memory, etc.
			 */
			dev_err(&hci->master.dev, "unable to abort the ring\n");
			//WARN_ON(1);
		}
	}

	for (i = 0; i < n; i++) {
		struct hci_xfer *xfer = xfer_list + i;
		int idx = xfer->ring_entry;

		/*
		 * At the time the abort happened, the xfer might have
		 * completed already. If not then replace corresponding
		 * descriptor entries with a no-op.
		 */
		if (idx >= 0) {
			dev_info(&hci->master.dev, "%s: enqueue_ptr %d xfer %d of %d tid %u replace with no-op\n", __func__, idx, i + 1, n, xfer->cmd_tid);
			u32 *ring_data = rh->xfer + rh->xfer_struct_sz * idx;

			/* store no-op cmd descriptor */
			*ring_data++ = FIELD_PREP(CMD_0_ATTR, 0x7) | FIELD_PREP(CMD_0_TID, xfer->cmd_tid);
			*ring_data++ = 0;
			if (hci->cmd == &mipi_i3c_hci_cmd_v2) {
				*ring_data++ = 0;
				*ring_data++ = 0;
			}

			/* disassociate this xfer struct */
			rh->src_xfers[idx] = NULL;

			/* and unmap it */
			hci_dma_unmap_xfer(hci, xfer, 1);

			did_unqueue = true;
		}
	}

	/* restart the ring */
	mipi_i3c_hci_resume(hci);
	rh_reg_write(RING_CONTROL, RING_CTRL_ENABLE);
	rh_reg_write(RING_CONTROL, RING_CTRL_ENABLE | RING_CTRL_RUN_STOP);

	return did_unqueue;
}

static void hci_dma_xfer_done(struct i3c_hci *hci, struct hci_rh_data *rh)
{
	u32 op1_val, op2_val, resp, *ring_resp;
	unsigned int tid, done_ptr = rh->done_ptr;
	unsigned int done_cnt = 0;
	struct hci_xfer *xfer;

	for (;;) {
		op2_val = rh_reg_read(RING_OPERATION2);
		dev_info(&hci->master.dev, "%s: read op2 %#x\n", __func__, op2_val);
		if (done_ptr == FIELD_GET(RING_OP2_CR_DEQ_PTR, op2_val))
			break;

		ring_resp = rh->resp + rh->resp_struct_sz * done_ptr;
		resp = *ring_resp;
		tid = RESP_TID(resp);
		dev_info(&hci->master.dev, "resp = 0x%08x tid = %u done_ptr = %u\n", resp, tid, done_ptr);

		xfer = rh->src_xfers[done_ptr];
		if (!xfer) {
			dev_info(&hci->master.dev, "orphaned ring entry");
		} else {
			hci_dma_unmap_xfer(hci, xfer, 1);
			xfer->ring_entry = -1;
			xfer->response = resp;
			if (tid != xfer->cmd_tid) {
				dev_err(&hci->master.dev,
					"response tid=%d when expecting %d\n",
					tid, xfer->cmd_tid);
				/* TODO: do something about it? */
			}
			if (xfer->completion)
				complete(xfer->completion);
			else
				dev_info(&hci->master.dev, "no xfer->completion");
		}

		done_ptr = (done_ptr + 1) % rh->xfer_entries;
		rh->done_ptr = done_ptr;
		done_cnt += 1;
	}

	/* take care to update the software dequeue pointer atomically */
	spin_lock(&rh->lock);
	rh->xfer_space += done_cnt;
	op1_val = rh_reg_read(RING_OPERATION1);
	dev_info(&hci->master.dev, "%s: read op1 %#x\n", __func__, op1_val);
	op1_val &= ~RING_OP1_CR_SW_DEQ_PTR;
	op1_val |= FIELD_PREP(RING_OP1_CR_SW_DEQ_PTR, done_ptr);
	dev_info(&hci->master.dev, "%s: write op1 %#x\n", __func__, op1_val);
	rh_reg_write(RING_OPERATION1, op1_val);
	spin_unlock(&rh->lock);
}

static int hci_dma_request_ibi(struct i3c_hci *hci, struct i3c_dev_desc *dev,
			       const struct i3c_ibi_setup *req)
{
	struct i3c_hci_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct i3c_generic_ibi_pool *pool;
	struct hci_dma_dev_ibi_data *dev_ibi;

	dev_ibi = kmalloc(sizeof(*dev_ibi), GFP_KERNEL);
	if (!dev_ibi)
		return -ENOMEM;
	pool = i3c_generic_ibi_alloc_pool(dev, req);
	if (IS_ERR(pool)) {
		kfree(dev_ibi);
		return PTR_ERR(pool);
	}
	dev_ibi->pool = pool;
	dev_ibi->max_len = req->max_payload_len;
	dev_data->ibi_data = dev_ibi;
	return 0;
}

static void hci_dma_free_ibi(struct i3c_hci *hci, struct i3c_dev_desc *dev)
{
	struct i3c_hci_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct hci_dma_dev_ibi_data *dev_ibi = dev_data->ibi_data;

	dev_data->ibi_data = NULL;
	i3c_generic_ibi_free_pool(dev_ibi->pool);
	kfree(dev_ibi);
}

static void hci_dma_recycle_ibi_slot(struct i3c_hci *hci,
				     struct i3c_dev_desc *dev,
				     struct i3c_ibi_slot *slot)
{
	struct i3c_hci_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct hci_dma_dev_ibi_data *dev_ibi = dev_data->ibi_data;

	i3c_generic_ibi_recycle_slot(dev_ibi->pool, slot);
}

static void hci_dma_process_ibi(struct i3c_hci *hci, struct hci_rh_data *rh)
{
	struct hci_rings_data *rings = hci->io_data;
	struct i3c_dev_desc *dev;
	struct i3c_hci_dev_data *dev_data;
	struct hci_dma_dev_ibi_data *dev_ibi;
	struct i3c_ibi_slot *slot;
	u32 op1_val, op2_val, ibi_status_error;
	unsigned int ptr, enq_ptr, deq_ptr;
	unsigned int ibi_size, ibi_chunks, ibi_data_offset, first_part;
	int ibi_addr, last_ptr;
	void *ring_ibi_data;
	dma_addr_t ring_ibi_data_dma;

	op1_val = rh_reg_read(RING_OPERATION1);
	deq_ptr = FIELD_GET(RING_OP1_IBI_DEQ_PTR, op1_val);

	op2_val = rh_reg_read(RING_OPERATION2);
	enq_ptr = FIELD_GET(RING_OP2_IBI_ENQ_PTR, op2_val);

	ibi_status_error = 0;
	ibi_addr = -1;
	ibi_chunks = 0;
	ibi_size = 0;
	last_ptr = -1;

	/* let's find all we can about this IBI */
	for (ptr = deq_ptr; ptr != enq_ptr;
	     ptr = (ptr + 1) % rh->ibi_status_entries) {
		u32 ibi_status, *ring_ibi_status;
		unsigned int chunks;

		ring_ibi_status = rh->ibi_status + rh->ibi_status_sz * ptr;
		ibi_status = *ring_ibi_status;
		dev_dbg(&hci->master.dev, "status = %#x", ibi_status);

		if (ibi_status_error) {
			/* we no longer care */
		} else if (ibi_status & IBI_ERROR) {
			ibi_status_error = ibi_status;
		} else if (ibi_addr ==  -1) {
			ibi_addr = FIELD_GET(IBI_TARGET_ADDR, ibi_status);
		} else if (ibi_addr != FIELD_GET(IBI_TARGET_ADDR, ibi_status)) {
			/* the address changed unexpectedly */
			ibi_status_error = ibi_status;
		}

		chunks = FIELD_GET(IBI_CHUNKS, ibi_status);
		ibi_chunks += chunks;
		if (!(ibi_status & IBI_LAST_STATUS)) {
			ibi_size += chunks * rh->ibi_chunk_sz;
		} else {
			ibi_size += FIELD_GET(IBI_DATA_LENGTH, ibi_status);
			last_ptr = ptr;
			break;
		}
	}

	/* validate what we've got */

	if (last_ptr == -1) {
		/* this IBI sequence is not yet complete */
		dev_dbg(&hci->master.dev,
			"no LAST_STATUS available (e=%d d=%d)",
			enq_ptr, deq_ptr);
		return;
	}
	deq_ptr = last_ptr + 1;
	deq_ptr %= rh->ibi_status_entries;

	if (ibi_status_error) {
		dev_err(&hci->master.dev, "IBI error from %#x\n", ibi_addr);
		goto done;
	}

	/* determine who this is for */
	dev = i3c_hci_addr_to_dev(hci, ibi_addr);
	if (!dev) {
		dev_err(&hci->master.dev,
			"IBI for unknown device %#x\n", ibi_addr);
		goto done;
	}

	dev_data = i3c_dev_get_master_data(dev);
	dev_ibi = dev_data->ibi_data;
	if (ibi_size > dev_ibi->max_len) {
		dev_err(&hci->master.dev, "IBI payload too big (%d > %d)\n",
			ibi_size, dev_ibi->max_len);
		goto done;
	}

	/*
	 * This ring model is not suitable for zero-copy processing of IBIs.
	 * We have the data chunk ring wrap-around to deal with, meaning
	 * that the payload might span multiple chunks beginning at the
	 * end of the ring and wrap to the start of the ring. Furthermore
	 * there is no guarantee that those chunks will be released in order
	 * and in a timely manner by the upper driver. So let's just copy
	 * them to a discrete buffer. In practice they're supposed to be
	 * small anyway.
	 */
	slot = i3c_generic_ibi_get_free_slot(dev_ibi->pool);
	if (!slot) {
		dev_err(&hci->master.dev, "no free slot for IBI\n");
		goto done;
	}

	/* copy first part of the payload */
	ibi_data_offset = rh->ibi_chunk_sz * rh->ibi_chunk_ptr;
	ring_ibi_data = rh->ibi_data + ibi_data_offset;
	ring_ibi_data_dma = rh->ibi_data_dma + ibi_data_offset;
	first_part = (rh->ibi_chunks_total - rh->ibi_chunk_ptr)
			* rh->ibi_chunk_sz;
	if (first_part > ibi_size)
		first_part = ibi_size;
	dma_sync_single_for_cpu(rings->sysdev, ring_ibi_data_dma,
				first_part, DMA_FROM_DEVICE);
	memcpy(slot->data, ring_ibi_data, first_part);

	/* copy second part if any */
	if (ibi_size > first_part) {
		/* we wrap back to the start and copy remaining data */
		ring_ibi_data = rh->ibi_data;
		ring_ibi_data_dma = rh->ibi_data_dma;
		dma_sync_single_for_cpu(rings->sysdev, ring_ibi_data_dma,
					ibi_size - first_part, DMA_FROM_DEVICE);
		memcpy(slot->data + first_part, ring_ibi_data,
		       ibi_size - first_part);
	}

	/* submit it */
	slot->dev = dev;
	slot->len = ibi_size;
	i3c_master_queue_ibi(dev, slot);

done:
	/* take care to update the ibi dequeue pointer atomically */
	spin_lock(&rh->lock);
	op1_val = rh_reg_read(RING_OPERATION1);
	op1_val &= ~RING_OP1_IBI_DEQ_PTR;
	op1_val |= FIELD_PREP(RING_OP1_IBI_DEQ_PTR, deq_ptr);
	rh_reg_write(RING_OPERATION1, op1_val);
	spin_unlock(&rh->lock);

	/* update the chunk pointer */
	rh->ibi_chunk_ptr += ibi_chunks;
	rh->ibi_chunk_ptr %= rh->ibi_chunks_total;

	/* and tell the hardware about freed chunks */
	rh_reg_write(CHUNK_CONTROL, rh_reg_read(CHUNK_CONTROL) + ibi_chunks);
}

static bool hci_dma_irq_handler(struct i3c_hci *hci)
{
	struct hci_rings_data *rings = hci->io_data;
	unsigned int i;
	bool handled = false;

	for (i = 0; i < rings->total; i++) {
		struct hci_rh_data *rh;
		u32 status;

		rh = &rings->headers[i];
		status = rh_reg_read(INTR_STATUS);
		dev_info(&hci->master.dev, "Ring %d: RH_INTR_STATUS %#x",
			i, status);
		if (!status)
			continue;
		rh_reg_write(INTR_STATUS, status);

		if (status & INTR_IBI_READY)
			hci_dma_process_ibi(hci, rh);
		if (status & (INTR_TRANSFER_COMPLETION | INTR_TRANSFER_ERR))
			hci_dma_xfer_done(hci, rh);
		if (status & INTR_RING_OP)
			complete(&rh->op_done);

		if (status & INTR_TRANSFER_ABORT) {
			u32 ring_status;

			dev_dbg_ratelimited(&hci->master.dev, "Ring %d: Transfer Aborted\n", i);
			//mipi_i3c_hci_resume(hci);
			ring_status = rh_reg_read(RING_STATUS);
			dev_info(&hci->master.dev, "Ring %d: Transfer Aborted ring_status %#x\n", i, ring_status);
			if (!(ring_status & RING_STATUS_RUNNING) &&
			    status & INTR_TRANSFER_COMPLETION &&
			    status & INTR_TRANSFER_ERR) {
				/*
				 * Ring stop followed by run is an Intel
				 * specific required quirk after resuming the
				 * halted controller. Do it only when the ring
				 * is not in running state after a transfer
				 * error.
				 */
				mipi_i3c_hci_resume(hci);
				rh_reg_write(RING_CONTROL, RING_CTRL_ENABLE);
				rh_reg_write(RING_CONTROL, RING_CTRL_ENABLE |
							   RING_CTRL_RUN_STOP);
				ring_status = rh_reg_read(RING_STATUS);
				dev_info(&hci->master.dev, "Ring %d: Transfer Aborted ring_status %#x plus stop/run\n", i, ring_status);
			}
		}
		if (status & INTR_IBI_RING_FULL)
			dev_err_ratelimited(&hci->master.dev,
				"Ring %d: IBI Ring Full Condition\n", i);

		handled = true;
	}

	return handled;
}

const struct hci_io_ops mipi_i3c_hci_dma = {
	.init			= hci_dma_init,
	.cleanup		= hci_dma_cleanup,
	.queue_xfer		= hci_dma_queue_xfer,
	.dequeue_xfer		= hci_dma_dequeue_xfer,
	.irq_handler		= hci_dma_irq_handler,
	.request_ibi		= hci_dma_request_ibi,
	.free_ibi		= hci_dma_free_ibi,
	.recycle_ibi_slot	= hci_dma_recycle_ibi_slot,
	.suspend		= hci_dma_suspend,
	.resume			= hci_dma_resume,
};
