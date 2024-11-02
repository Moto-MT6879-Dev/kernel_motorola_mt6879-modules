/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __MAWD_REG_H__
#define __MAWD_REG_H__

#if (CFG_MTK_FPGA_PLATFORM == 1)
#define MAWD_REG_BASE			0x47000
#else
#define MAWD_REG_BASE			0x18047000
#endif

#define MAWD_IND_CMD_CTRL0		(MAWD_REG_BASE + 0x0)
#define MAWD_IND_CMD_CTRL1		(MAWD_REG_BASE + 0x4)
#define MAWD_IND_CMD_CTRL2		(MAWD_REG_BASE + 0x8)
#define MAWD_ADDR_ARRAY_BASE_L		(MAWD_REG_BASE + 0xC)
#define MAWD_ADDR_ARRAY_BASE_M		(MAWD_REG_BASE + 0x10)
#define MAWD_BA_PARAM			(MAWD_REG_BASE + 0x14)
#define MAWD_RRO_ACK_SN_BASE_L		(MAWD_REG_BASE + 0x18)
#define MAWD_RRO_ACK_SN_BASE_M		(MAWD_REG_BASE + 0x1C)
#define MAWD_AP_RX_BLK_CTRL0		(MAWD_REG_BASE + 0x20)
#define MAWD_AP_RX_BLK_CTRL1		(MAWD_REG_BASE + 0x24)
#define MAWD_AP_RX_BLK_CTRL2		(MAWD_REG_BASE + 0x28)
#define MAWD_MD_RX_BLK_CTRL0		(MAWD_REG_BASE + 0x2C)
#define MAWD_MD_RX_BLK_CTRL1		(MAWD_REG_BASE + 0x30)
#define MAWD_MD_RX_BLK_CTRL2		(MAWD_REG_BASE + 0x34)
#define MAWD_HIF_TXD_MD_CTRL0		(MAWD_REG_BASE + 0x38)
#define MAWD_HIF_TXD_MD_CTRL1		(MAWD_REG_BASE + 0x3C)
#define MAWD_HIF_TXD_MD_CTRL2		(MAWD_REG_BASE + 0x40)
#define MAWD_HIF_TXD_MD_CTRL3		(MAWD_REG_BASE + 0x44)
#define MAWD_HIF_TXD_MD_CTRL4		(MAWD_REG_BASE + 0x48)
#define MAWD_HIF_TXD_MD_CTRL5		(MAWD_REG_BASE + 0x4C)
#define MAWD_HIF_TXD_MD_CTRL6		(MAWD_REG_BASE + 0x50)
#define MAWD_HIF_TXD_MD_CTRL7		(MAWD_REG_BASE + 0x54)
#define MAWD_HIF_TXD_MD_CTRL8 		(MAWD_REG_BASE + 0x58)
#define MAWD_ERR_RPT_CTRL0		(MAWD_REG_BASE + 0x5C)
#define MAWD_ERR_RPT_CTRL1		(MAWD_REG_BASE + 0x60)
#define MAWD_ERR_RPT_CTRL2		(MAWD_REG_BASE + 0x64)
#define MAWD_WFDMA_RING_MD_CTRL0 	(MAWD_REG_BASE + 0x68)
#define MAWD_WFDMA_RING_MD_CTRL1 	(MAWD_REG_BASE + 0x6C)
#define MAWD_WFDMA_RING_MD_CTRL2 	(MAWD_REG_BASE + 0x70)
#define MAWD_WFDMA_RING_MD_CTRL3 	(MAWD_REG_BASE + 0x74)
#define MAWD_WFDMA_RING_MD_CTRL4 	(MAWD_REG_BASE + 0x78)
#define MAWD_WFDMA_RING_MD_CTRL5 	(MAWD_REG_BASE + 0x7C)
#define MAWD_WFDMA_RING_MD_CTRL6 	(MAWD_REG_BASE + 0x80)
#define MAWD_WFDMA_RING_MD_CTRL7 	(MAWD_REG_BASE + 0x84)
#define MAWD_WFDMA_RING_MD_CTRL8 	(MAWD_REG_BASE + 0x88)
#define MAWD_WFDMA_RING_MD_CTRL9 	(MAWD_REG_BASE + 0x8C)
#define MAWD_WFDMA_RING_MD_CTRL10 	(MAWD_REG_BASE + 0x90)
#define MAWD_WFDMA_RING_MD_CTRL11 	(MAWD_REG_BASE + 0x94)
#define MAWD_WFDMA_RING_MD_CTRL12 	(MAWD_REG_BASE + 0x98)
#define MAWD_WFDMA_RING_MD_CTRL13 	(MAWD_REG_BASE + 0x9C)
#define MAWD_WFDMA_RING_MD_CTRL14 	(MAWD_REG_BASE + 0xA0)
#define MAWD_SETTING0			(MAWD_REG_BASE + 0xA4)
#define MAWD_SETTING1			(MAWD_REG_BASE + 0xA8)
#define MAWD_SETTING2			(MAWD_REG_BASE + 0xAC)
#define MAWD_SETTING3			(MAWD_REG_BASE + 0xB0)
#define MAWD_IND_CMD_SIGNATURE0		(MAWD_REG_BASE + 0xB4)
#define MAWD_IND_CMD_SIGNATURE1		(MAWD_REG_BASE + 0xB8)
#define MAWD_MD_INTERRUPT_SETTING2	(MAWD_REG_BASE + 0xBC)
#define MAWD_AP_INTERRUPT_SETTING2	(MAWD_REG_BASE + 0xC0)
#define MAWD_MISC_SETTING1		(MAWD_REG_BASE + 0xC4)
#define MAWD_MISC_SETTING2		(MAWD_REG_BASE + 0xC8)
#define MAWD_POWER_UP			(MAWD_REG_BASE + 0xCC)
#define MAWD_R2AXI_CTRL1		(MAWD_REG_BASE + 0xD0)
#define MAWD_R2AXI_CTRL2		(MAWD_REG_BASE + 0xD4)
#define MAWD_R2AXI_CTRL3		(MAWD_REG_BASE + 0xD8)
#define MAWD_DEBUG_SETTING1		(MAWD_REG_BASE + 0x100)
#define MAWD_DEBUG_SETTING2		(MAWD_REG_BASE + 0x104)
#define MAWD_SOFTRESET			(MAWD_REG_BASE + 0x108)
#define MAWD_STATIC_BUFF_INFO		(MAWD_REG_BASE + 0x110)
#define MAWD_SETTING4			(MAWD_REG_BASE + 0x114)
#define MAWD_SETTING5			(MAWD_REG_BASE + 0x118)
#define MAWD_SETTING6			(MAWD_REG_BASE + 0x11C)
#define MAWD_MD_INTERRUPT_SETTING0	(MAWD_REG_BASE + 0x120)
#define MAWD_MD_INTERRUPT_SETTING1	(MAWD_REG_BASE + 0x124)
#define MAWD_AP_INTERRUPT_SETTING0	(MAWD_REG_BASE + 0x128)
#define MAWD_AP_INTERRUPT_SETTING1	(MAWD_REG_BASE + 0x12C)
#define MAWD_AXI_SLEEP_PROT_SETTING	(MAWD_REG_BASE + 0x130)
#define MAWD_AXI_SLEEP_PROT_VIO_ADR	(MAWD_REG_BASE + 0x134)
#define MAWD_IDX_REG_PATCH		(MAWD_REG_BASE + 0x138)
#define MAWD_AP_WAKE_UP			(MAWD_REG_BASE + 0x13C)
#define MAWD_INDEX_DBG_REG0		(MAWD_REG_BASE + 0x144)
#define MAWD_INDEX_DBG_REG1		(MAWD_REG_BASE + 0x148)
#define MAWD_INDEX_DBG_REG2		(MAWD_REG_BASE + 0x14C)
#define MAWD_INDEX_DBG_REG3		(MAWD_REG_BASE + 0x150)
#define MAWD_REG_PLL_CTRL_0		(MAWD_REG_BASE + 0x154)
#define MAWD_REG_PLL_CTRL_1		(MAWD_REG_BASE + 0x158)
#define MAWD_REG_BUSY_LATCH		(MAWD_REG_BASE + 0x15C)

#endif /* __MAWD_REG_H__ */
