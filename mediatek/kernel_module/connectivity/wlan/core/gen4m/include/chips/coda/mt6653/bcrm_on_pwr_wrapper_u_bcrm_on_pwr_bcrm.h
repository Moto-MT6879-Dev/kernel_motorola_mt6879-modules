/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

//[File]            : bcrm_on_pwr_wrapper_u_bcrm_on_pwr_bcrm.h
//[Revision time]   : Wed Jan  4 19:30:36 2023
//[Description]     : This file is auto generated by CODA
//[Copyright]       : Copyright (C) 2023 Mediatek Incorportion. All rights reserved.

#ifndef __BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_REGS_H__
#define __BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_REGS_H__

#include "hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif


//****************************************************************************
//
//                     BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM CR Definitions             
//
//****************************************************************************

#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_BASE            (0x1803B000 + CONN_INFRA_REMAPPING_OFFSET)

#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_lnk_von2on_intf_m_to_conn_infra_von2on_apb_bus_p_d_n14_APB_GALS_RX_CTRL_0_ADDR (BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_BASE + 0x000) // B000
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_p_d_n14_CTRL_0_ADDR (BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_BASE + 0x004) // B004
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_p_d_n15_CTRL_0_ADDR (BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_BASE + 0x008) // B008
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_off2on_apb_bus_u_lnk_off2on_intf_m_to_conn_infra_off2on_apb_bus_p_d_n16_APB_GALS_RX_CTRL_0_ADDR (BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_BASE + 0x00c) // B00C
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_off2on_apb_bus_u_p_d_n16_CTRL_0_ADDR (BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_BASE + 0x010) // B010




/* =====================================================================================

  ---conn_infra_von2on_apb_bus_u_lnk_von2on_intf_m_to_conn_infra_von2on_apb_bus_p_d_n14_APB_GALS_RX_CTRL_0 (0x1803B000 + 0x000)---

    conn_infra_von2on_apb_bus__u_lnk_von2on_intf_m_to_conn_infra_von2on_apb_bus_p_d_n14_APB_GALS_RX__rx_clock_cg_en[0] - (RW)  xxx 
    RESERVED1[31..1]             - (RO) Reserved bits

 =====================================================================================*/
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_lnk_von2on_intf_m_to_conn_infra_von2on_apb_bus_p_d_n14_APB_GALS_RX_CTRL_0_conn_infra_von2on_apb_bus__u_lnk_von2on_intf_m_to_conn_infra_von2on_apb_bus_p_d_n14_APB_GALS_RX__rx_clock_cg_en_ADDR BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_lnk_von2on_intf_m_to_conn_infra_von2on_apb_bus_p_d_n14_APB_GALS_RX_CTRL_0_ADDR
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_lnk_von2on_intf_m_to_conn_infra_von2on_apb_bus_p_d_n14_APB_GALS_RX_CTRL_0_conn_infra_von2on_apb_bus__u_lnk_von2on_intf_m_to_conn_infra_von2on_apb_bus_p_d_n14_APB_GALS_RX__rx_clock_cg_en_MASK 0x00000001                // conn_infra_von2on_apb_bus__u_lnk_von2on_intf_m_to_conn_infra_von2on_apb_bus_p_d_n14_APB_GALS_RX__rx_clock_cg_en[0]
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_lnk_von2on_intf_m_to_conn_infra_von2on_apb_bus_p_d_n14_APB_GALS_RX_CTRL_0_conn_infra_von2on_apb_bus__u_lnk_von2on_intf_m_to_conn_infra_von2on_apb_bus_p_d_n14_APB_GALS_RX__rx_clock_cg_en_SHFT 0

/* =====================================================================================

  ---conn_infra_von2on_apb_bus_u_p_d_n14_CTRL_0 (0x1803B000 + 0x004)---

    conn_infra_von2on_apb_bus__u_p_d_n14__way_en[1..0] - (RW)  xxx 
    RESERVED2[31..2]             - (RO) Reserved bits

 =====================================================================================*/
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_p_d_n14_CTRL_0_conn_infra_von2on_apb_bus__u_p_d_n14__way_en_ADDR BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_p_d_n14_CTRL_0_ADDR
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_p_d_n14_CTRL_0_conn_infra_von2on_apb_bus__u_p_d_n14__way_en_MASK 0x00000003                // conn_infra_von2on_apb_bus__u_p_d_n14__way_en[1..0]
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_p_d_n14_CTRL_0_conn_infra_von2on_apb_bus__u_p_d_n14__way_en_SHFT 0

/* =====================================================================================

  ---conn_infra_von2on_apb_bus_u_p_d_n15_CTRL_0 (0x1803B000 + 0x008)---

    conn_infra_von2on_apb_bus__u_p_d_n15__way_en[15..0] - (RW)  xxx 
    RESERVED16[31..16]           - (RO) Reserved bits

 =====================================================================================*/
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_p_d_n15_CTRL_0_conn_infra_von2on_apb_bus__u_p_d_n15__way_en_ADDR BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_p_d_n15_CTRL_0_ADDR
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_p_d_n15_CTRL_0_conn_infra_von2on_apb_bus__u_p_d_n15__way_en_MASK 0x0000FFFF                // conn_infra_von2on_apb_bus__u_p_d_n15__way_en[15..0]
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_von2on_apb_bus_u_p_d_n15_CTRL_0_conn_infra_von2on_apb_bus__u_p_d_n15__way_en_SHFT 0

/* =====================================================================================

  ---conn_infra_off2on_apb_bus_u_lnk_off2on_intf_m_to_conn_infra_off2on_apb_bus_p_d_n16_APB_GALS_RX_CTRL_0 (0x1803B000 + 0x00c)---

    conn_infra_off2on_apb_bus__u_lnk_off2on_intf_m_to_conn_infra_off2on_apb_bus_p_d_n16_APB_GALS_RX__rx_clock_cg_en[0] - (RW)  xxx 
    RESERVED1[31..1]             - (RO) Reserved bits

 =====================================================================================*/
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_off2on_apb_bus_u_lnk_off2on_intf_m_to_conn_infra_off2on_apb_bus_p_d_n16_APB_GALS_RX_CTRL_0_conn_infra_off2on_apb_bus__u_lnk_off2on_intf_m_to_conn_infra_off2on_apb_bus_p_d_n16_APB_GALS_RX__rx_clock_cg_en_ADDR BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_off2on_apb_bus_u_lnk_off2on_intf_m_to_conn_infra_off2on_apb_bus_p_d_n16_APB_GALS_RX_CTRL_0_ADDR
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_off2on_apb_bus_u_lnk_off2on_intf_m_to_conn_infra_off2on_apb_bus_p_d_n16_APB_GALS_RX_CTRL_0_conn_infra_off2on_apb_bus__u_lnk_off2on_intf_m_to_conn_infra_off2on_apb_bus_p_d_n16_APB_GALS_RX__rx_clock_cg_en_MASK 0x00000001                // conn_infra_off2on_apb_bus__u_lnk_off2on_intf_m_to_conn_infra_off2on_apb_bus_p_d_n16_APB_GALS_RX__rx_clock_cg_en[0]
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_off2on_apb_bus_u_lnk_off2on_intf_m_to_conn_infra_off2on_apb_bus_p_d_n16_APB_GALS_RX_CTRL_0_conn_infra_off2on_apb_bus__u_lnk_off2on_intf_m_to_conn_infra_off2on_apb_bus_p_d_n16_APB_GALS_RX__rx_clock_cg_en_SHFT 0

/* =====================================================================================

  ---conn_infra_off2on_apb_bus_u_p_d_n16_CTRL_0 (0x1803B000 + 0x010)---

    conn_infra_off2on_apb_bus__u_p_d_n16__way_en[15..0] - (RW)  xxx 
    RESERVED16[31..16]           - (RO) Reserved bits

 =====================================================================================*/
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_off2on_apb_bus_u_p_d_n16_CTRL_0_conn_infra_off2on_apb_bus__u_p_d_n16__way_en_ADDR BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_off2on_apb_bus_u_p_d_n16_CTRL_0_ADDR
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_off2on_apb_bus_u_p_d_n16_CTRL_0_conn_infra_off2on_apb_bus__u_p_d_n16__way_en_MASK 0x0000FFFF                // conn_infra_off2on_apb_bus__u_p_d_n16__way_en[15..0]
#define BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_conn_infra_off2on_apb_bus_u_p_d_n16_CTRL_0_conn_infra_off2on_apb_bus__u_p_d_n16__way_en_SHFT 0

#ifdef __cplusplus
}
#endif

#endif // __BCRM_ON_PWR_WRAPPER_U_BCRM_ON_PWR_BCRM_REGS_H__