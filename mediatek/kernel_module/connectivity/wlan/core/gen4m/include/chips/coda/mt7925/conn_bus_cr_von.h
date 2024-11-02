/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __CONN_BUS_CR_VON_REGS_H__
#define __CONN_BUS_CR_VON_REGS_H__
#include "hal_common.h"
#ifdef __cplusplus
extern "C" {
#endif
#define CONN_BUS_CR_VON_BASE \
	(0x18021000 + CONN_INFRA_REMAPPING_OFFSET)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_10_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x0000)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_32_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x0004)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_54_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x0008)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_76_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x000C)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_98_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x0010)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_BA_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x0014)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_DC_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x0018)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_FE_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x001C)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_10_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x0020)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_32_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x0024)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_54_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x0028)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_76_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x002C)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_98_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x0030)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_BA_ADDR \
	(CONN_BUS_CR_VON_BASE + 0x0034)
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_10_cr_pcie2ap_public_remapping_wf_01_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_10_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_10_cr_pcie2ap_public_remapping_wf_01_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_10_cr_pcie2ap_public_remapping_wf_01_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_10_cr_pcie2ap_public_remapping_wf_00_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_10_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_10_cr_pcie2ap_public_remapping_wf_00_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_10_cr_pcie2ap_public_remapping_wf_00_SHFT \
	0
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_32_cr_pcie2ap_public_remapping_wf_03_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_32_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_32_cr_pcie2ap_public_remapping_wf_03_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_32_cr_pcie2ap_public_remapping_wf_03_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_32_cr_pcie2ap_public_remapping_wf_02_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_32_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_32_cr_pcie2ap_public_remapping_wf_02_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_32_cr_pcie2ap_public_remapping_wf_02_SHFT \
	0
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_54_cr_pcie2ap_public_remapping_wf_05_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_54_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_54_cr_pcie2ap_public_remapping_wf_05_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_54_cr_pcie2ap_public_remapping_wf_05_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_54_cr_pcie2ap_public_remapping_wf_04_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_54_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_54_cr_pcie2ap_public_remapping_wf_04_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_54_cr_pcie2ap_public_remapping_wf_04_SHFT \
	0
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_76_cr_pcie2ap_public_remapping_wf_07_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_76_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_76_cr_pcie2ap_public_remapping_wf_07_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_76_cr_pcie2ap_public_remapping_wf_07_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_76_cr_pcie2ap_public_remapping_wf_06_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_76_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_76_cr_pcie2ap_public_remapping_wf_06_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_76_cr_pcie2ap_public_remapping_wf_06_SHFT \
	0
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_98_cr_pcie2ap_public_remapping_wf_09_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_98_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_98_cr_pcie2ap_public_remapping_wf_09_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_98_cr_pcie2ap_public_remapping_wf_09_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_98_cr_pcie2ap_public_remapping_wf_08_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_98_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_98_cr_pcie2ap_public_remapping_wf_08_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_98_cr_pcie2ap_public_remapping_wf_08_SHFT \
	0
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_BA_cr_pcie2ap_public_remapping_wf_0b_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_BA_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_BA_cr_pcie2ap_public_remapping_wf_0b_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_BA_cr_pcie2ap_public_remapping_wf_0b_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_BA_cr_pcie2ap_public_remapping_wf_0a_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_BA_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_BA_cr_pcie2ap_public_remapping_wf_0a_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_BA_cr_pcie2ap_public_remapping_wf_0a_SHFT \
	0
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_DC_cr_pcie2ap_public_remapping_wf_0d_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_DC_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_DC_cr_pcie2ap_public_remapping_wf_0d_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_DC_cr_pcie2ap_public_remapping_wf_0d_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_DC_cr_pcie2ap_public_remapping_wf_0c_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_DC_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_DC_cr_pcie2ap_public_remapping_wf_0c_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_DC_cr_pcie2ap_public_remapping_wf_0c_SHFT \
	0
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_FE_cr_pcie2ap_public_remapping_wf_0f_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_FE_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_FE_cr_pcie2ap_public_remapping_wf_0f_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_FE_cr_pcie2ap_public_remapping_wf_0f_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_FE_cr_pcie2ap_public_remapping_wf_0e_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_FE_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_FE_cr_pcie2ap_public_remapping_wf_0e_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_0_FE_cr_pcie2ap_public_remapping_wf_0e_SHFT \
	0
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_10_cr_pcie2ap_public_remapping_wf_11_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_10_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_10_cr_pcie2ap_public_remapping_wf_11_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_10_cr_pcie2ap_public_remapping_wf_11_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_10_cr_pcie2ap_public_remapping_wf_10_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_10_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_10_cr_pcie2ap_public_remapping_wf_10_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_10_cr_pcie2ap_public_remapping_wf_10_SHFT \
	0
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_32_cr_pcie2ap_public_remapping_wf_13_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_32_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_32_cr_pcie2ap_public_remapping_wf_13_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_32_cr_pcie2ap_public_remapping_wf_13_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_32_cr_pcie2ap_public_remapping_wf_12_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_32_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_32_cr_pcie2ap_public_remapping_wf_12_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_32_cr_pcie2ap_public_remapping_wf_12_SHFT \
	0
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_54_cr_pcie2ap_public_remapping_wf_15_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_54_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_54_cr_pcie2ap_public_remapping_wf_15_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_54_cr_pcie2ap_public_remapping_wf_15_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_54_cr_pcie2ap_public_remapping_wf_14_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_54_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_54_cr_pcie2ap_public_remapping_wf_14_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_54_cr_pcie2ap_public_remapping_wf_14_SHFT \
	0
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_76_cr_pcie2ap_public_remapping_wf_17_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_76_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_76_cr_pcie2ap_public_remapping_wf_17_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_76_cr_pcie2ap_public_remapping_wf_17_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_76_cr_pcie2ap_public_remapping_wf_16_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_76_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_76_cr_pcie2ap_public_remapping_wf_16_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_76_cr_pcie2ap_public_remapping_wf_16_SHFT \
	0
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_98_cr_pcie2ap_public_remapping_wf_19_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_98_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_98_cr_pcie2ap_public_remapping_wf_19_MASK \
	0xFFFF0000
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_98_cr_pcie2ap_public_remapping_wf_19_SHFT \
	16
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_98_cr_pcie2ap_public_remapping_wf_18_ADDR \
	CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_98_ADDR
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_98_cr_pcie2ap_public_remapping_wf_18_MASK \
	0x0000FFFF
#define CONN_BUS_CR_VON_CONN_INFRA_PCIE2AP_REMAP_WF_1_98_cr_pcie2ap_public_remapping_wf_18_SHFT \
	0
#ifdef __cplusplus
}
#endif
#endif
