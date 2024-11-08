/*
 * Copyright (c) 2018 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include "mmi_charger_class.h"
#include <linux/power_supply.h>
#include "mmi_charger_core.h"
#include <linux/math64.h>

#define	BAT_OVP_FAULT_SHIFT			8
#define	BAT_OCP_FAULT_SHIFT			9
#define	BUS_OVP_FAULT_SHIFT			10
#define	BUS_OCP_FAULT_SHIFT			11
#define	BAT_THERM_FAULT_SHIFT			12
#define	BUS_THERM_FAULT_SHIFT			13
#define	DIE_THERM_FAULT_SHIFT			14
#define	CONV_OCP_FAULT_SHIFT			15
#define	SS_TIMEOUT_FAULT_SHIFT			16
#define	TS_SHUT_FAULT_SHIFT			17
#define	CP_SWITCH_SHIFT                         18

#define	BAT_OVP_FAULT_MASK		(1 << BAT_OVP_FAULT_SHIFT)
#define	BAT_OCP_FAULT_MASK		(1 << BAT_OCP_FAULT_SHIFT)
#define	BUS_OVP_FAULT_MASK		(1 << BUS_OVP_FAULT_SHIFT)
#define	BUS_OCP_FAULT_MASK		(1 << BUS_OCP_FAULT_SHIFT)
#define	BAT_THERM_FAULT_MASK		(1 << BAT_THERM_FAULT_SHIFT)
#define	BUS_THERM_FAULT_MASK		(1 << BUS_THERM_FAULT_SHIFT)
#define	DIE_THERM_FAULT_MASK		(1 << DIE_THERM_FAULT_SHIFT)
#define	CONV_OCP_FAULT_MASK		(1 << CONV_OCP_FAULT_SHIFT)
#define	SS_TIMEOUT_FAULT_MASK		(1 << SS_TIMEOUT_FAULT_SHIFT)
#define	TS_SHUT_FAULT_MASK		(1 << TS_SHUT_FAULT_SHIFT)
#define	CP_SWITCH_MASK			(1 << CP_SWITCH_SHIFT)

#define	BAT_OVP_ALARM_SHIFT			0
#define	BAT_OCP_ALARM_SHIFT			1
#define	BUS_OVP_ALARM_SHIFT			2
#define	BUS_OCP_ALARM_SHIFT			3
#define	BUS_UCP_FAULT_SHIFT			4
#define	BUS_THERM_ALARM_SHIFT			5
#define	DIE_THERM_ALARM_SHIFT			6
#define	BAT_UCP_ALARM_SHIFT			7

#define	BAT_OVP_ALARM_MASK		(1 << BAT_OVP_ALARM_SHIFT)
#define	BAT_OCP_ALARM_MASK		(1 << BAT_OCP_ALARM_SHIFT)
#define	BUS_OVP_ALARM_MASK		(1 << BUS_OVP_ALARM_SHIFT)
#define	BUS_OCP_ALARM_MASK		(1 << BUS_OCP_ALARM_SHIFT)
#define	BUS_UCP_FAULT_MASK		(1 << BUS_UCP_FAULT_SHIFT)
#define	BUS_THERM_ALARM_MASK		(1 << BUS_THERM_ALARM_SHIFT)
#define	DIE_THERM_ALARM_MASK		(1 << DIE_THERM_ALARM_SHIFT)
#define	BAT_UCP_ALARM_MASK		(1 << BAT_UCP_ALARM_SHIFT)

#ifdef CONFIG_MOTO_CHARGER_PUMP_MEASURE_AVG_VOL
#define PRECISION_ENHANCE	5
#define CP_MEASURE_R_AVG_TIMES 10

static u32 cp_precise_div(u64 dividend, u64 divisor)
{
	u64 _val = div64_u64(dividend << PRECISION_ENHANCE, divisor);

	return (u32)((_val + (1 << (PRECISION_ENHANCE - 1))) >>
		PRECISION_ENHANCE);
}
#endif

static int cp_init_chip(struct mmi_charger_device *chrg)
{
	int rc;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	rc = charger_dev_init_chip(chrg->chg_dev);
	if (rc<0) {
		chrg_dev_info(chrg, "MMI chrg: init chip failed\n");
	} else {
		chrg_dev_info(chrg, "MMI chrg: init chip success\n");
	}

	return rc;
}

static int cp_enable_charging(struct mmi_charger_device *chrg, bool en)
{
	int rc;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	rc = charger_dev_enable(chrg->chg_dev, en);
	if (rc<0) {
		chrg_dev_info(chrg, "MMI chrg: charger_enabled failed, set to false\n");
		//chrg->charger_enabled  = false;
	} else {
		chrg->charger_enabled = !!en;
		chrg_dev_info(chrg, "MMI chrg: set charger_enabled = %d\n",chrg->charger_enabled);
	}

	return rc;
}

static int cp_is_charging_enabled(struct mmi_charger_device *chrg, bool *en)
{
	int rc;
	bool val;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	rc = charger_dev_is_enabled(chrg->chg_dev, &val);
	if (rc>=0)
		chrg->charger_enabled = !!val;

	*en = chrg->charger_enabled;

	return rc;
}

static int cp_get_charging_current(struct mmi_charger_device *chrg, u32 *uA)
{
	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	chrg_dev_info(chrg, "MMI CP not support get ibat from cp \n");

	*uA = 0;

	return 0;
}

#ifdef CONFIG_MOTO_CHARGER_PUMP_MEASURE_AVG_VOL
static int cp_get_input_voltage_settled(struct mmi_charger_device *chrg, u32 *vbus)
{
	int rc, vbus_voltage;
	int vbus1 = 0, vbus_max = 0, vbus_min = 0;
	int i = 0, valid_count = 0;;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	for (i = 0; i < CP_MEASURE_R_AVG_TIMES + 2 && valid_count < 7; i++) {
		rc = charger_dev_get_adc(chrg->chg_dev, ADC_CHANNEL_VBUS, &vbus_voltage, &vbus_voltage);
		if (rc < 0) {
			chrg_dev_info(chrg, "cp get vbus fail! retry\n");
			msleep(50);
			continue;
		}

		if (i == 0 || valid_count == 0) {
			vbus_max = vbus_min = vbus_voltage;
		} else {
			vbus_max = max(vbus_max, vbus_voltage);
			vbus_min = min(vbus_min, vbus_voltage);
		}
		vbus1 += vbus_voltage;
		msleep(30);
		chrg_dev_info(chrg, "vbus=%d vbus(max,min)=(%d,%d) vbus1=%d",
				vbus_voltage, vbus_max, vbus_min, vbus1);
		valid_count++;
	}

	vbus1 -= (vbus_min + vbus_max);

	if (valid_count > 3) {
		chrg_dev_info(chrg, "vbus total = %d, valid_count = %d", vbus1, valid_count);
		vbus1 = cp_precise_div(vbus1, valid_count - 2);
	} else {
		*vbus = chrg->charger_data.vbus_volt;
		chrg_dev_info(chrg, "cp get vbus fail all times! valid_count = %d, use pre vbus_voltage = %d\n",
				valid_count, chrg->charger_data.vbus_volt);
		return rc;
	}

	chrg->charger_data.vbus_volt = vbus1;
	*vbus = chrg->charger_data.vbus_volt;

	chrg_dev_info(chrg, "get final vbus: %d\n", chrg->charger_data.vbus_volt);
	return rc;
}

#else
static int cp_get_input_voltage_settled(struct mmi_charger_device *chrg, u32 *vbus)
{
	int rc, vbus_voltage;

	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	rc = charger_dev_get_adc(chrg->chg_dev, ADC_CHANNEL_VBUS, &vbus_voltage, &vbus_voltage);
	if (rc>=0)
		chrg->charger_data.vbus_volt = vbus_voltage;

	*vbus = chrg->charger_data.vbus_volt;

	return rc;
}
#endif

static int cp_get_input_current(struct mmi_charger_device *chrg, u32 *uA)
{
	int rc, ibus;
	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}
	rc = charger_dev_get_adc(chrg->chg_dev, ADC_CHANNEL_IBUS, &ibus, &ibus);
	if (rc>=0)
		chrg->charger_data.ibus_curr = ibus;

	*uA = chrg->charger_data.ibus_curr;

	return rc;
}

static int cp_update_charger_status(struct mmi_charger_device *chrg)
{
	int rc,ibus,vbus;
	bool enable;
	union power_supply_propval prop = {0,};
	struct power_supply	*battery_psy;

	if (!chrg->chrg_psy)
		return -ENODEV;
	if (!chrg->chg_dev) {
		chrg_dev_info(chrg, "MMI CP chrg: chg_dev is null! \n");
		return -ENODEV;
	}

	battery_psy = power_supply_get_by_name("battery");
	if (!battery_psy){
		chrg_dev_info(chrg, "battery psy not avalible\n");
		return -ENODEV;
	}

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	if (!rc)
		chrg->charger_data.vbatt_volt = prop.intval;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	if (!rc)
		chrg->charger_data.ibatt_curr = prop.intval;

	rc = power_supply_get_property(battery_psy,
				POWER_SUPPLY_PROP_TEMP, &prop);
	if (!rc)
		chrg->charger_data.batt_temp = prop.intval / 10;

	rc = charger_dev_get_adc(chrg->chg_dev, ADC_CHANNEL_VBUS, &vbus, &vbus);
	if (rc>=0)
		chrg->charger_data.vbus_volt = vbus;

	rc = charger_dev_get_adc(chrg->chg_dev, ADC_CHANNEL_IBUS, &ibus, &ibus);
	if (rc>=0)
		chrg->charger_data.ibus_curr = ibus;

	rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_ONLINE, &prop);
	if (!rc)
		chrg->charger_data.vbus_pres = !!prop.intval;

	rc = charger_dev_is_enabled(chrg->chg_dev, &enable);
	if (rc>=0)
		chrg->charger_enabled = !!enable;

	chrg_dev_info(chrg, "BQ2597x chrg: %s status update: --- info--- 1\n",chrg->name);
	chrg_dev_info(chrg, "vbatt %d\n", chrg->charger_data.vbatt_volt);
	chrg_dev_info(chrg, "ibatt %d\n", chrg->charger_data.ibatt_curr);
	chrg_dev_info(chrg, "batt temp %d\n", chrg->charger_data.batt_temp);
	chrg_dev_info(chrg, "vbus %d\n", chrg->charger_data.vbus_volt);
	chrg_dev_info(chrg, "ibus %d\n", chrg->charger_data.ibus_curr);
	chrg_dev_info(chrg, "vbus pres %d\n", chrg->charger_data.vbus_pres);
	chrg_dev_info(chrg, "charger_enabled %d\n", chrg->charger_enabled);

	return rc;
}

static int cp_update_charger_error_status(struct mmi_charger_device *chrg)
{
	int rc= 0;
	//union power_supply_propval prop = {0,};

	if (!chrg->chrg_psy)
		return -ENODEV;

	/*rc = power_supply_get_property(chrg->chrg_psy,
				POWER_SUPPLY_PROP_CP_STATUS1, &prop);
	if (!rc) {
		chrg->charger_error.chrg_err_type =
			((!!(prop.intval & BAT_OVP_ALARM_MASK)) ?
			1 << MMI_BAT_OVP_ALARM_BIT :
			0 << MMI_BAT_OVP_ALARM_BIT) |
			((!!(prop.intval & BAT_OCP_ALARM_MASK)) ?
			1 << MMI_BAT_OCP_ALARM_BIT :
			0 << MMI_BAT_OCP_ALARM_BIT) |
			((!!(prop.intval & BAT_UCP_ALARM_MASK)) ?
			1 << MMI_BAT_UCP_ALARM_BIT :
			0 << MMI_BAT_UCP_ALARM_BIT) |
			((!!(prop.intval & BUS_OVP_ALARM_MASK)) ?
			1 << MMI_BUS_OVP_ALARM_BIT :
			0 << MMI_BUS_OVP_ALARM_BIT) |
			((!!(prop.intval & BUS_OCP_ALARM_MASK)) ?
			1 << MMI_BUS_OCP_ALARM_BIT :
			0 << MMI_BUS_OCP_ALARM_BIT) |
			((!!(prop.intval & BAT_OVP_FAULT_MASK)) ?
			1 << MMI_BAT_OVP_FAULT_BIT :
			0 << MMI_BAT_OVP_FAULT_BIT) |
			((!!(prop.intval & BAT_OCP_FAULT_MASK)) ?
			1 << MMI_BAT_OCP_FAULT_BIT :
			0 << MMI_BAT_OCP_FAULT_BIT) |
			((!!(prop.intval & BUS_OVP_FAULT_MASK)) ?
			1 << MMI_BUS_OVP_FAULT_BIT :
			0 << MMI_BUS_OVP_FAULT_BIT) |
			((!!(prop.intval & BUS_OCP_FAULT_MASK)) ?
			1 << MMI_BUS_OCP_FAULT_BIT :
			0 << MMI_BUS_OCP_FAULT_BIT) |
			((!!(prop.intval & BUS_UCP_FAULT_MASK)) ?
			1 << MMI_BUS_UCP_FAULT_BIT :
			0 << MMI_BUS_UCP_FAULT_BIT) |
			((!!(prop.intval & CONV_OCP_FAULT_MASK)) ?
			1 << MMI_CONV_OCP_FAULT_BIT :
			0 << MMI_CONV_OCP_FAULT_BIT) |
			((!!(prop.intval & CP_SWITCH_MASK)) ?
			1 << MMI_CP_SWITCH_BIT :
			0 << MMI_CP_SWITCH_BIT);
	}*/
	return rc;
}

static int cp_clear_charger_error(struct mmi_charger_device *chrg)
{
	int rc;

	if (!chrg->chrg_psy)
		return -ENODEV;
	//rc = bq2597x_set_property(BQ2597X_PROP_UPDATE_NOW,true);
	rc = 0;

	return rc;
}

struct mmi_charger_ops mtk_pump_charger_ops = {
	.init_chip = cp_init_chip,
	.enable = cp_enable_charging,
	.is_enabled = cp_is_charging_enabled,
	.get_charging_current = cp_get_charging_current,
	.get_input_current = cp_get_input_current,
	.get_input_voltage_settled = cp_get_input_voltage_settled,
	.update_charger_status = cp_update_charger_status,
	.update_charger_error = cp_update_charger_error_status,
	.clear_charger_error = cp_clear_charger_error,
};
