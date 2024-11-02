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

#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include "mmi_charger_class.h"
#include "mmi_charger_core.h"
#include "mmi_charger_policy.h"

static int __debug_mask = 0x85;
module_param_named(
	debug_mask, __debug_mask, int, S_IRUSR | S_IWUSR
);

int pd_volt_max_init = 0;
int pd_curr_max_init = 0;
static int *dt_temp_zones;
static struct mmi_chrg_dts_info *chrg_name_list;

#define MIN_TEMP_C -20
#define MAX_TEMP_C 60
#define HYSTEREISIS_DEGC 2

#ifdef CONFIG_MOTO_CHG_WT6670F_SUPPORT
extern int wt6670f_get_charger_type(void);
extern void wt6670f_reset_chg_type(void);
extern int  wt6670f_start_detection(void);
extern int  wt6670f_get_protocol(void);
#endif

bool mmi_find_temp_zone(struct mmi_charger_manager *chip, int temp_c, bool ignore_hysteresis_degc)
{
	int prev_zone, num_zones;
	struct mmi_chrg_temp_zone *zones;
	int hotter_t = 0, hotter_fcc = 0;
	int colder_t = 0, colder_fcc = 0;
	int i;
	int max_temp;

	if (!chip) {
		mmi_chrg_err(chip, "called before chip valid!\n");
		return false;
	}

	zones = chip->temp_zones;
	num_zones = chip->num_temp_zones;
	prev_zone = chip->pres_temp_zone;

	mmi_chrg_info(chip, "prev zone %d, temp_c %d\n",prev_zone, temp_c);
	max_temp = zones[num_zones - 1].temp_c;

	if (prev_zone == ZONE_NONE) {
		for (i = num_zones - 1; i >= 0; i--) {
			if (temp_c >= zones[i].temp_c) {
				if (i == num_zones - 1)
					chip->pres_temp_zone = ZONE_HOT;
				else
					chip->pres_temp_zone = i + 1;
				return true;
			}
		}
		chip->pres_temp_zone = ZONE_COLD;
		return true;
	}

	if (prev_zone == ZONE_COLD) {
		if (temp_c >= MIN_TEMP_C + HYSTEREISIS_DEGC)
			chip->pres_temp_zone = ZONE_FIRST;
	} else if (prev_zone == ZONE_HOT) {
		if (temp_c <=  max_temp - HYSTEREISIS_DEGC)
			chip->pres_temp_zone = num_zones - 1;
	} else {
		if (prev_zone == ZONE_FIRST) {
			hotter_t = zones[prev_zone].temp_c;
			colder_t = MIN_TEMP_C;
			hotter_fcc = zones[prev_zone + 1].chrg_step_power->chrg_step_curr;
			colder_fcc = 0;
		} else if (prev_zone == num_zones - 1) {
			hotter_t = zones[prev_zone].temp_c;
			colder_t = zones[prev_zone - 1].temp_c;
			hotter_fcc = 0;
			colder_fcc = zones[prev_zone - 1].chrg_step_power->chrg_step_curr;
		} else {
			hotter_t = zones[prev_zone].temp_c;
			colder_t = zones[prev_zone - 1].temp_c;
			hotter_fcc = zones[prev_zone + 1].chrg_step_power->chrg_step_curr;
			colder_fcc = zones[prev_zone - 1].chrg_step_power->chrg_step_curr;
		}

		if (!ignore_hysteresis_degc) {
			if (zones[prev_zone].chrg_step_power->chrg_step_curr < hotter_fcc)
				hotter_t += HYSTEREISIS_DEGC;
			if (zones[prev_zone].chrg_step_power->chrg_step_curr < colder_fcc)
				colder_t -= HYSTEREISIS_DEGC;
		}

		if (temp_c <= MIN_TEMP_C)
			chip->pres_temp_zone = ZONE_COLD;
		else if (temp_c >= max_temp)
			chip->pres_temp_zone = ZONE_HOT;
		else if (temp_c >= hotter_t)
			chip->pres_temp_zone++;
		else if (temp_c < colder_t)
			chip->pres_temp_zone--;
	}

	mmi_chrg_info(chip, "batt temp_c %d, prev zone %d, pres zone %d, "
						"hotter_fcc %dmA, colder_fcc %dmA, "
						"hotter_t %dC, colder_t %dC\n",
						temp_c,prev_zone, chip->pres_temp_zone,
						hotter_fcc, colder_fcc, hotter_t, colder_t);

	if (prev_zone != chip->pres_temp_zone) {
		mmi_chrg_info(chip,
			   "Entered Temp Zone %d!\n",
			   chip->pres_temp_zone);
		return true;
	}
	return false;
}

bool mmi_find_chrg_step(struct mmi_charger_manager *chip, int temp_zone, int vbatt_volt)
{
	int batt_volt, i;
	bool find_step = false;
	struct mmi_chrg_temp_zone zone;
	struct mmi_chrg_step_power *chrg_steps;
	struct mmi_chrg_step_info chrg_step_inline;
	struct mmi_chrg_step_info prev_step;

	if (!chip) {
		mmi_chrg_err(chip, "called before chip valid!\n");
		return false;
	}

	if (chip->pres_temp_zone == ZONE_HOT ||
		chip->pres_temp_zone == ZONE_COLD ||
		chip->pres_temp_zone < ZONE_FIRST) {
		mmi_chrg_err(chip, "pres temp zone is HOT or COLD, "
							"can't find chrg step\n");
		return false;
	}

	zone = chip->temp_zones[chip->pres_temp_zone];
	chrg_steps = zone.chrg_step_power;
	prev_step = chip->chrg_step;

	batt_volt = vbatt_volt;
	chrg_step_inline.temp_c = zone.temp_c;

	mmi_chrg_info(chip, "batt_volt %d, chrg step %d, step nums %d\n",
						batt_volt, prev_step.pres_chrg_step,
						chip->chrg_step_nums);

	/*In the first search cycle, find out the vbatt is less than step volt*/

	for (i = 0; i < chip->chrg_step_nums; i++) {
		mmi_chrg_info(chip,
					"first cycle,i %d, step volt %d, batt_volt %d\n",
					i, chrg_steps[i].chrg_step_volt, batt_volt);
		if (chrg_steps[i].chrg_step_volt > 0
			&& batt_volt < chrg_steps[i].chrg_step_volt) {
			if ( (i + 1) < chip->chrg_step_nums
				&& chrg_steps[i + 1].chrg_step_volt > 0) {
				chrg_step_inline.chrg_step_cv_tapper_curr =
					chrg_steps[i + 1].chrg_step_curr;
			} else
				chrg_step_inline.chrg_step_cv_tapper_curr =
					chrg_steps[i].chrg_step_curr;
			chrg_step_inline.chrg_step_cc_curr =
				chrg_steps[i].chrg_step_curr;
			chrg_step_inline.chrg_step_cv_volt =
				chrg_steps[i].chrg_step_volt;
			chrg_step_inline.pres_chrg_step = i;
			find_step = true;
			mmi_chrg_info(chip, "find chrg step\n");
			break;
		}
	}

	if (find_step) {
		mmi_chrg_info(chip, "chrg step %d, "
					"step cc curr %d, step cv volt %d, "
					"step cv tapper curr %d\n",
					chrg_step_inline.pres_chrg_step,
					chrg_step_inline.chrg_step_cc_curr,
					chrg_step_inline.chrg_step_cv_volt,
					chrg_step_inline.chrg_step_cv_tapper_curr);
		chip->chrg_step = chrg_step_inline;
	} else {

	/*If can't find out any chrg step in the first search cycle,*/
	/*it means that vbatt is already greater than all chrg step volt, */
	/*therefore, start to enter second search cycle, */
	/*to find out the maximal chrg step volt*/
		for (i = 0; i < chip->chrg_step_nums; i++) {
			mmi_chrg_info(chip,
						"second cycle, i %d, step volt %d, batt_volt %d\n",
						i, chrg_steps[i].chrg_step_volt, batt_volt);
			if (chrg_steps[i].chrg_step_volt > 0
				&& batt_volt > chrg_steps[i].chrg_step_volt) {
				if ( (i + 1) < chip->chrg_step_nums
					&& chrg_steps[i + 1].chrg_step_volt > 0) {
					chrg_step_inline.chrg_step_cv_tapper_curr =
						chrg_steps[i + 1].chrg_step_curr;
				} else
					chrg_step_inline.chrg_step_cv_tapper_curr =
						chrg_steps[i].chrg_step_curr;
				chrg_step_inline.chrg_step_cc_curr =
					chrg_steps[i].chrg_step_curr;
				chrg_step_inline.chrg_step_cv_volt =
					chrg_steps[i].chrg_step_volt;
				chrg_step_inline.pres_chrg_step = i;
				find_step = true;
				mmi_chrg_info(chip, "find chrg step\n");
			}
		}

		if (find_step) {
			mmi_chrg_info(chip, "chrg step %d, "
					"step cc curr %d, step cv volt %d, "
					"step cv tapper curr %d\n",
					chrg_step_inline.pres_chrg_step,
					chrg_step_inline.chrg_step_cc_curr,
					chrg_step_inline.chrg_step_cv_volt,
					chrg_step_inline.chrg_step_cv_tapper_curr);
			chip->chrg_step = chrg_step_inline;
		}
	}


	if (find_step) {
		if (chip->chrg_step.chrg_step_cc_curr ==
			chip->chrg_step.chrg_step_cv_tapper_curr)
			chip->chrg_step.last_step = true;
		else
			chip->chrg_step.last_step = false;

		mmi_chrg_info(chip,"Temp zone %d, "
				"select chrg step %d, step cc curr %d,"
				"step cv volt %d, step cv tapper curr %d, "
				"is the last chrg step %d\n",
				chip->pres_temp_zone,
				chip->chrg_step.pres_chrg_step,
				chip->chrg_step.chrg_step_cc_curr,
				chip->chrg_step.chrg_step_cv_volt,
				chip->chrg_step.chrg_step_cv_tapper_curr,
				chip->chrg_step.last_step);

		if (prev_step.pres_chrg_step != chip->chrg_step.pres_chrg_step) {
			mmi_chrg_info(chip, "Find the next chrg step\n");
			return true;
		}
	}
	return false;
}

void mmi_set_pps_result_history(struct mmi_charger_manager *chip, int pps_result)
{
	if (chip->pps_result_history_idx >= PPS_RET_HISTORY_SIZE -1)
		chip->pps_result_history_idx = 0;

	if (pps_result < 0)
		chip->pps_result_history[chip->pps_result_history_idx] = 1;
	else
		chip->pps_result_history[chip->pps_result_history_idx] = 0;

	chip->pps_result_history_idx++;
}

bool mmi_get_pps_result_history(struct mmi_charger_manager *chip)
{
	int i = 0;
	int result = 0;
	for (i = 0; i < PPS_RET_HISTORY_SIZE; i++)
		result += chip->pps_result_history[i];

	if (result >= PPS_RET_HISTORY_SIZE)
		return RESET_POWER;
	else if (result >= PPS_RET_HISTORY_SIZE / 2)
		return BLANCE_POWER;
	else
		return NO_ERROR;
}

void mmi_clear_pps_result_history(struct mmi_charger_manager *chip)
{
	int i = 0;;
	for (i = 0; i < PPS_RET_HISTORY_SIZE; i++)
		chip->pps_result_history[i] = 0;
}

int mmi_calculate_delta_volt(int pps_voltage, int pps_current, int delta_curr)
{
	u64 power;
	int delta_volt;
	u64 pps_volt;
	u64 pps_curr;

	pps_volt = pps_voltage;
	pps_curr = pps_current;
	power = pps_volt * pps_curr;

	delta_volt = (int)(power / (pps_curr - delta_curr) - pps_volt);
	delta_volt -= delta_volt % 20000;

	if (delta_volt > 0)
		return delta_volt;
	else
		return 0;
}

void mmi_update_all_charger_status(struct mmi_charger_manager *chip)
{
	int chrg_num, i;
	int ret = 0;
	struct mmi_charger_device *chrg_dev;
	if (!chip) {
		mmi_chrg_err(chip, "called before chip valid!\n");
		return;
	}

	mmi_chrg_err(chip, "mmi_update_all_charger_status, chrg_num:%d\n",chip->mmi_chrg_dev_num);

	chrg_num = chip->mmi_chrg_dev_num;
	for (i = 0; i < chrg_num; i++) {
		chrg_dev = chip->chrg_list[i];
		ret = mmi_update_charger_status(chrg_dev);
		mmi_chrg_err(chip, "mmi_update_all_charger_status, chrg_num:%d, ret:%d\n",i,ret);
	}
	return;
}

void mmi_update_all_charger_error(struct mmi_charger_manager *chip)
{
	int chrg_num, i;
	struct mmi_charger_device *chrg_dev;
	if (!chip) {
		mmi_chrg_err(chip, "called before chip valid!\n");
		return;
	}

	chrg_num = chip->mmi_chrg_dev_num;
	for (i = 0; i < chrg_num; i++) {
		chrg_dev = chip->chrg_list[i];
		mmi_update_charger_error(chrg_dev);
	}
	return;
}

void mmi_dump_charger_error(struct mmi_charger_manager *chip,
									struct mmi_charger_device *chrg_dev)
{
	if (!chip || !chrg_dev) {
		mmi_chrg_err(chip, "called before chip valid!\n");
		return;
	}
	mmi_chrg_dbg(chip, PR_MOTO, "%s: charger error type %d, "
					"bus ucp err cnt %d, bus ocp err cnt %d\n",
					chrg_dev->name,
					chrg_dev->charger_error.chrg_err_type,
					chrg_dev->charger_error.bus_ucp_err_cnt,
					chrg_dev->charger_error.bus_ocp_err_cnt);
	return;
}

static enum power_supply_property batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	//POWER_SUPPLY_PROP_CHARGE_RATE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int batt_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct mmi_charger_manager *chip  = power_supply_get_drvdata(psy);
	union power_supply_propval prop = {0,};
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		rc = power_supply_get_property(chip->extrn_psy,
							psp, &prop);
		if (rc < 0) {
			mmi_chrg_err(chip, "Get Unknown prop %d from extrn_psy rc = %d\n",
								psp, rc);
			rc = power_supply_get_property(chip->mtk_psy,
							psp, &prop);
		}
		if (!rc)
			val->intval = prop.intval;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		if (chip->use_batt_age) {
			val->intval = chip->cycles;
		} else {
			rc = power_supply_get_property(chip->mtk_psy,
							psp, &prop);
			if (rc < 0) {
				mmi_chrg_err(chip, "Get Unknown prop %d rc = %d\n", psp, rc);
				rc = 0;
				val->intval = -EINVAL;
			}
			if (!rc)
				val->intval = prop.intval;
		}
		break;
	case POWER_SUPPLY_PROP_STATUS:
		rc = power_supply_get_property(chip->mtk_psy,
							psp, &prop);
		if (rc < 0) {
			mmi_chrg_err(chip, "Get Unknown prop %d rc = %d\n", psp, rc);
			rc = 0;
			val->intval = -EINVAL;
		} else {
			val->intval = prop.intval;
			if (prop.intval == POWER_SUPPLY_STATUS_NOT_CHARGING) {
				rc = power_supply_get_property(chip->extrn_psy,
									psp, &prop);
				if (!rc && (prop.intval == POWER_SUPPLY_STATUS_CHARGING
					|| prop.intval == POWER_SUPPLY_STATUS_FULL))
					val->intval = prop.intval;
			}
		}
		break;
	/*case POWER_SUPPLY_PROP_CHARGE_RATE:
		mmi_chrg_rate_check(chip);
		val->intval = chip->charger_rate;
		break;*/
	default:
		rc = power_supply_get_property(chip->mtk_psy,
							psp, &prop);
		if (rc < 0) {
			mmi_chrg_err(chip, "Get Unknown prop %d rc = %d\n", psp, rc);
			rc = 0;
			val->intval = -EINVAL;
		}
		if (!rc)
			val->intval = prop.intval;
		break;

	}
	return rc;
}

static int batt_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct mmi_charger_manager *chip  = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		chip->cycles = val->intval;
		break;
	default:
		rc = power_supply_set_property(chip->mtk_psy,
						prop, val);
		if (rc < 0) {
			mmi_chrg_err(chip, "Get Unknown prop %d rc = %d\n", prop, rc);
			rc = 0;
		}
		break;
	}
	return rc;
}

static int batt_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		return 1;
	default:
		break;
	}
	return 0;
}

static const struct power_supply_desc batt_psy_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = batt_props,
	.num_properties = ARRAY_SIZE(batt_props),
	.get_property = batt_get_property,
	.set_property = batt_set_property,
	.property_is_writeable = batt_is_writeable,
};

static int batt_psy_register(struct mmi_charger_manager *chip)
{
	struct power_supply_config batt_cfg = {};

	batt_cfg.drv_data = chip;
	batt_cfg.of_node = chip->dev->of_node;
	chip->batt_psy = power_supply_register(chip->dev,
						&batt_psy_desc,
						&batt_cfg);
	if (IS_ERR(chip->batt_psy)) {
		pr_err("Couldn't register batt_psy power supply\n");
		return PTR_ERR(chip->batt_psy);
	}

	pr_info("power supply register batt_psy successfully\n");
	return 0;
}

static enum power_supply_property mmi_chrg_mgr_props[] = {
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
};

static int mmi_chrg_mgr_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct mmi_charger_manager *chip  = power_supply_get_drvdata(psy);
	union power_supply_propval prop = {0,};
	int rc, i, chrg_cnt = 0;

	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy)
			return -ENODEV;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
		if (!rc)
			val->intval = prop.intval;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
		if (!rc)
			val->intval = prop.intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		if (chip->mmi_chrg_dev_num) {
			for (i = 0; i < chip->mmi_chrg_dev_num; i++) {
			 if (chip->chrg_list[i]->charger_enabled)
				chrg_cnt++;
			}
			val->intval = chrg_cnt;
		} else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->system_thermal_level;
		break;
	default:
		return -EINVAL;

	}
	return 0;
}

static int mmi_chrg_mgr_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct mmi_charger_manager *chip  = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		if (val->intval < 0) {
			chip->system_thermal_level = THERMAL_NOT_LIMIT;
			return 0;
		}

		if (chip->thermal_min_level <= 0)
			return -EINVAL;

		if (val->intval <= chip->thermal_min_level)
			chip->system_thermal_level =
				chip->thermal_min_level;
		else
			chip->system_thermal_level = val->intval;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mmi_chrg_mgr_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static const struct power_supply_desc mmi_chrg_mgr_psy_desc = {
	.name = "mmi_chrg_manager",
	.type = POWER_SUPPLY_TYPE_USB_PD,
	.properties = mmi_chrg_mgr_props,
	.num_properties = ARRAY_SIZE(mmi_chrg_mgr_props),
	.get_property = mmi_chrg_mgr_get_property,
	.set_property = mmi_chrg_mgr_set_property,
	.property_is_writeable = mmi_chrg_mgr_is_writeable,
};

static int mmi_chrg_mgr_psy_register(struct mmi_charger_manager *chip)
{
	struct power_supply_config mmi_chrg_mgr_cfg = {};

	mmi_chrg_mgr_cfg.drv_data = chip;
	mmi_chrg_mgr_cfg.of_node = chip->dev->of_node;
	chip->mmi_chrg_mgr_psy = power_supply_register(chip->dev,
						&mmi_chrg_mgr_psy_desc,
						&mmi_chrg_mgr_cfg);
	if (IS_ERR(chip->mmi_chrg_mgr_psy)) {
		pr_err("Couldn't register mmi_chrg_mgr_psy power supply\n");
		return PTR_ERR(chip->mmi_chrg_mgr_psy);
	}

	pr_info("power supply register mmi_chrg_mgr_psy successfully\n");
	return 0;
}
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
#if 0
int wt6670f_qc3p_detection(void)
{
	int result = 0;
	Charger_Detect_Init();
	pr_err("wt6670f_qc3p_detection start 1\n");
	wt6670f_reset_chg_type();
	wt6670f_start_detection();
	msleep(5000);
	wt6670f_get_protocol();
	result = wt6670f_get_charger_type();
	pr_err("wt6670f_qc3p_detection end with result:%d\n",result);
	Charger_Detect_Release();
	return result;
}
#endif
static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct mmi_charger_manager *chip =
					container_of(nb, struct mmi_charger_manager, psy_nb);

	pr_info("psy_changed enter 1 evt =%lu \n", evt);

	if (!chip->usb_psy) {
		mmi_chrg_err(chip, "usb psy is NULL, direct return\n");
		return NOTIFY_OK;
	}

//	pr_info("psy_changed enter ptr:%d,usb:%d\n",ptr,chip->usb_psy);
	if (ptr == chip->charger_psy && evt == PSY_EVENT_PROP_CHANGED) {
		pr_info("psy_changed enter in 2 \n");

		schedule_work(&chip->psy_changed_work);

		cancel_delayed_work(&chip->heartbeat_work);
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(1000));

	}

	return NOTIFY_OK;
}

static bool mmi_factory_check(void)
{
#if 1
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;

	if (np)
		factory = of_property_read_bool(np, "mmi,factory-cable");

	of_node_put(np);

	return factory;
#else
	char atm_str[64] = {0};
	char *ptr = NULL, *ptr_e = NULL;
	char keyword[] = "androidboot.atm=";
	int size = 0;
	bool is_fac_mode = false;

	ptr = strstr(saved_command_line, keyword);
	if (ptr != 0) {
		ptr_e = strstr(ptr, " ");
		if (ptr_e == NULL)
			goto end;

		size = ptr_e - (ptr + strlen(keyword));
		if (size <= 0)
			goto end;
		strncpy(atm_str, ptr + strlen(keyword), size);
		atm_str[size] = '\0';

		if (!strncmp(atm_str, "enable", strlen("enable")))
			is_fac_mode = true;
	}
end:
	pr_info("%s: fac mode = %d\n", __func__, is_fac_mode);
	return is_fac_mode;
#endif
}

static void kick_sm(struct mmi_charger_manager *chip, int ms)
{
	if (!chip->sm_work_running) {
		mmi_chrg_dbg(chip, PR_INTERRUPT,
					"launch mmi chrg sm work\n");
		mmi_chrg_policy_clear(chip);
		schedule_delayed_work(&chip->mmi_chrg_sm_work,
				msecs_to_jiffies(ms));
		chip->sm_work_running = true;
	} else
		mmi_chrg_dbg(chip, PR_INTERRUPT,
					"mmi chrg sm work already existed\n");
}

static void cancel_sm(struct mmi_charger_manager *chip)
{
	cancel_delayed_work_sync(&chip->mmi_chrg_sm_work);
	flush_delayed_work(&chip->mmi_chrg_sm_work);
	mmi_chrg_policy_clear(chip);
	chip->sm_work_running = false;
	chip->pd_volt_max = pd_volt_max_init;
	chip->pd_curr_max = pd_curr_max_init;
	mmi_chrg_dbg(chip, PR_INTERRUPT,
					"cancel sync and flush mmi chrg sm work\n");
}

void clear_chg_manager(struct mmi_charger_manager *chip)
{
	mmi_chrg_dbg(chip, PR_INTERRUPT, "clear mmi chrg manager!\n");
	chip->pd_request_volt = 0;
	chip->pd_request_curr = 0;
	chip->pd_request_volt_prev = 0;
	chip->pd_request_curr_prev = 0;
	chip->pd_target_curr = 0;
	chip->pd_target_volt = 0;
	chip->pd_batt_therm_volt = 0;
	chip->pd_batt_therm_curr = 0;
	chip->pd_sys_therm_volt = 0;
	chip->pd_sys_therm_curr = 0;
	chip->recovery_pmic_chrg = false;
	chip->sys_therm_cooling= false;
	chip->sys_therm_force_pmic_chrg = false;
	chip->batt_therm_cooling = false;
	chip->batt_therm_cooling_cnt = 0;
}

static void mmi_awake_vote(struct mmi_charger_manager *chip, bool awake)
{
	if (awake == chip->awake)
		return;

	chip->awake = awake;
	if (awake)
		pm_stay_awake(chip->dev);
	else
		pm_relax(chip->dev);
}

static void mmi_batt_age_init(struct mmi_charger_manager *chip)
{
	int rc = 0;
	union power_supply_propval pval;
	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy)
			return;
	}

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_STATUS, &pval);
	if (rc < 0) {
		mmi_chrg_err(chip, "Couldn't get batt status\n");
		return;
	}
	chip->pres_batt_status = pval.intval;
	chip->prev_batt_status = chip->pres_batt_status;

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (rc < 0) {
		mmi_chrg_err(chip, "Couldn't get batt capacity\n");
		return;
	}

	chip->soc_cycles_start= pval.intval;
	chip->batt_cap_delta = 0;
	return;
}

static void mmi_cycle_counts(struct mmi_charger_manager *chip)
{
	int rc = 0;
	int batt_design_capacity = 0, new_cycles = 0;
	int soc_report = 0, soc_delta = 0, batt_delta_capacity = 0;
	union power_supply_propval pval;
	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy)
			return;
	}

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &pval);
	if (rc < 0) {
		mmi_chrg_err(chip, "Couldn't get batt design capacity\n");
		return;
	}
		batt_design_capacity = pval.intval;

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_STATUS, &pval);
	if (rc < 0) {
		mmi_chrg_err(chip, "Couldn't get batt status\n");
		return;
	}
	chip ->pres_batt_status = pval.intval;

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (rc < 0) {
		mmi_chrg_err(chip, "Couldn't get batt capacity\n");
		return;
	}
	soc_report = pval.intval;

	mmi_chrg_dbg(chip, PR_MOTO, "cycle_count %d, pres %d, prev %d, soc_report %d, "
					"soc_cycles_start %d, batt_cap_delta %d, "
					"design_capcaity %d, batt_soc_delta %d\n",
					chip->cycles, chip->pres_batt_status,
					chip->prev_batt_status,
					soc_report, chip->soc_cycles_start,
					chip->batt_cap_delta, batt_design_capacity,
					chip->batt_soc_delta);

	if (chip->prev_batt_status == POWER_SUPPLY_STATUS_CHARGING
		&& chip->pres_batt_status != POWER_SUPPLY_STATUS_CHARGING) {
		chip->soc_cycles_start = soc_report;
	} else if (chip->prev_batt_status != POWER_SUPPLY_STATUS_CHARGING
		&& chip->pres_batt_status == POWER_SUPPLY_STATUS_CHARGING) {

		soc_delta = chip->soc_cycles_start - soc_report;
		if (soc_delta > 0) {
			chip->batt_soc_delta += soc_delta;

			batt_delta_capacity = (chip->batt_soc_delta * batt_design_capacity) / 100
								+ chip->batt_cap_delta;

			new_cycles = batt_delta_capacity / batt_design_capacity;
			if (new_cycles > 0) {
				chip->cycles += batt_delta_capacity / batt_design_capacity;
				chip->batt_cap_delta = batt_delta_capacity % batt_design_capacity;
				chip->batt_soc_delta = 0;
			}
		}
		chip->soc_cycles_start = soc_report;
	}

	chip->prev_batt_status = chip->pres_batt_status;
}

static enum alarmtimer_restart mmi_heartbeat_alarm_cb(struct alarm *alarm,
						      ktime_t now)
{
	struct mmi_charger_manager *chip = container_of(alarm,
						    struct mmi_charger_manager,
						    heartbeat_alarm);

	mmi_chrg_info(chip, "MMI: HB alarm fired\n");

//	__pm_stay_awake(&chip->mmi_hb_wake_source);
	cancel_delayed_work(&chip->heartbeat_work);
	/* Delay by 500 ms to allow devices to resume. */
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(500));

	return ALARMTIMER_NORESTART;
}

#define HEARTBEAT_DELAY_MS 60000
#define SMBCHG_HEARTBEAT_INTRVAL_NS	70000000000
#define CHG_SHOW_MAX_SIZE 50
static void mmi_heartbeat_work(struct work_struct *work)
{
	struct mmi_charger_manager *chip = container_of(work,
				struct mmi_charger_manager, heartbeat_work.work);
	int hb_resch_time = 0, ret = 0;
	union power_supply_propval val;

	mmi_chrg_info(chip, "MMI: Heartbeat!\n");
	/* Have not been resumed so wait another 100 ms */
	if (chip->suspended) {
		mmi_chrg_err(chip, "SMBMMI: HB running before Resume\n");
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(100));
		return;
	}

	mmi_awake_vote(chip, true);
	alarm_try_to_cancel(&chip->heartbeat_alarm);


	if (!chip->charger_psy) {
		chip->charger_psy= devm_power_supply_get_by_phandle(chip->dev, "charger");
		if (IS_ERR_OR_NULL(chip->charger_psy)) {
			mmi_chrg_err(chip, "%s Couldn't get bat_psy\n", __func__);
			return;
		}
	}

	ret = power_supply_get_property(chip->charger_psy,
			POWER_SUPPLY_PROP_ONLINE, &val);
	if (ret) {
		mmi_chrg_err(chip, "Unable to read USB PRESENT: %d\n", ret);
		return;
	}
	chip->vbus_present = val.intval;

	hb_resch_time = HEARTBEAT_DELAY_MS;

	if (chip->vbus_present)
		alarm_start_relative(&chip->heartbeat_alarm,
				     ns_to_ktime(SMBCHG_HEARTBEAT_INTRVAL_NS));

	if (!chip->vbus_present)
		mmi_awake_vote(chip, false);

	if (!chip->sm_work_running && chip->vbus_present) {
#ifndef CONFIG_MOTO_CHG_WT6670F_SUPPORT
		mmi_chrg_info(chip, "check all effective pdo info\n");
		chip->pd_pps_support = usbpd_get_pps_status(chip);
#endif
		if (chip->pd_pps_support
			&& !chip->factory_mode) {
			mmi_chrg_info(chip, "MMI: Heartbeat!, launch sm work\n");
			kick_sm(chip, 100);
		}
	}

	if (chip->use_batt_age)
		mmi_cycle_counts(chip);

//	__pm_relax(&chip->mmi_hb_wake_source);

	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(hb_resch_time));
}

extern bool wt6670f_is_detect;

static void psy_changed_work_func(struct work_struct *work)
{
	struct mmi_charger_manager *chip = container_of(work,
				struct mmi_charger_manager, psy_changed_work);
	union power_supply_propval val;
	int ret;
#ifdef CONFIG_MOTO_CHG_WT6670F_SUPPORT
//	int chrg_type = CHARGER_UNKNOWN;
	int chrg_type = 0;
#endif

	mmi_chrg_info(chip, "kick psy changed work.\n");

	if (!chip->charger_psy) {
		chip->charger_psy= devm_power_supply_get_by_phandle(chip->dev, "charger");
		if (IS_ERR_OR_NULL(chip->charger_psy)) {
			mmi_chrg_err(chip, "%s Couldn't get bat_psy\n", __func__);
			return;
		} 	}

	ret = power_supply_get_property(chip->charger_psy,
			POWER_SUPPLY_PROP_ONLINE, &val);
	if (ret) {
		mmi_chrg_err(chip, "Unable to read USB PRESENT: %d\n", ret);
		return;
	}
	chip->vbus_present = val.intval;

	if (chip->vbus_present) {
		mmi_chrg_info(chip, "check all effective pdo info at psy change\n");
#ifdef CONFIG_MOTO_CHG_WT6670F_SUPPORT
		chrg_type = wt6670f_get_charger_type();
		mmi_chrg_info(chip, "WT6670F return charger type: 0x%x", chrg_type);
                if (ret) {
			mmi_chrg_err(chip, "Unable to get charger type: %d\n", ret);
			return;
		}
/*
		if(chrg_type != 9)
			chrg_type = wt6670f_qc3p_detection();

		if(chrg_type != 9)
			chrg_type = wt6670f_qc3p_detection();

//		chrg_type = 9;//david test
*/
		switch(chrg_type){
//			case QC3P_18W_CHARGER:
			case 0x8:
				chip->pd_volt_max = 9500000;
				chip->pd_curr_max = 2000000;
				chip->pd_pps_support = true;
				break;
//			case QC3P_27W_CHARGER:
			case 0x9:
				chip->pd_volt_max = 10000000;
				chip->pd_curr_max = 3100000;
				chip->pd_pps_support = true;
				mmi_chrg_info(chip, "kick psy changed work. 1\n");
				break;
			default:
				chip->pd_pps_support = false;
				break;
		}
#else
		chip->pd_pps_support = usbpd_get_pps_status(chip);
#endif
	}
#ifdef CONFIG_MOTO_CHG_WT6670F_SUPPORT
	else {
		wt6670f_reset_chg_type();
		mmi_chrg_info(chip, "Reset WT6670F charger type\n");
	}
#endif

	mmi_chrg_info(chip, "vbus present %d, pd pps support %d, "
					"pps max voltage %d, pps max curr %d\n",
					chip->vbus_present,
					chip->pd_pps_support,
					chip->pd_volt_max,
					chip->pd_curr_max);

	if (chip->vbus_present
		&& chip->pd_pps_support
		&& !chip->factory_mode) {

		kick_sm(chip, 100);
	} else {
		cancel_sm(chip);
		clear_chg_manager(chip);
		chip->pd_pps_support =  false;
	}
	return;
}

#define DEFAULT_BATT_OVP_LMT		4475000
#define DEFAULT_PL_CHRG_VBATT_MIN	3600000
#define DEFAULT_PPS_VOLT_STEPS	20000
#define DEFAULT_PPS_CURR_STEPS	50000
#define DEFAULT_PPS_VOLT_MAX	11000000
#define DEFAULT_PPS_CURR_MAX	5000000
#define DEFAULT_THERMAL_MIN_LEVL	2000000
#define MMI_TEMP_ZONES	5
static int mmi_chrg_manager_parse_dt(struct mmi_charger_manager *chip)
{
	struct device_node *node = chip->dev->of_node, *child;
	int rc, byte_len, i, j, chrg_idx = 0, step_cnt = 0, idx = 0;
	const int *ptr;

	rc = of_property_read_u32(node,
				"mmi,batt-ovp-lmt",
				&chip->batt_ovp_lmt);
	if (rc < 0)
		chip->batt_ovp_lmt =
				DEFAULT_BATT_OVP_LMT;

	rc = of_property_read_u32(node,
				"mmi,pl-chrg-vbatt-min",
				&chip->pl_chrg_vbatt_min);
	if (rc < 0)
		chip->pl_chrg_vbatt_min =
				DEFAULT_PL_CHRG_VBATT_MIN;

	chip->use_batt_age = of_property_read_bool(node,
				"mmi,use-batt-age");

	chip->extrn_fg = of_property_read_bool(node,
				"mmi,extrn-fg");

	if (chip->extrn_fg) {
		byte_len = of_property_count_strings(node, "mmi,extrn-fg-name");
		if (byte_len <= 0) {
			mmi_chrg_err(chip, "Cannot parse mmi, extrn-fg: %d\n", byte_len);
			return byte_len;
		}

		for (i = 0; i < byte_len; i++) {
			rc = of_property_read_string_index(node, "mmi,extrn-fg-name",
					i, &chip->extrn_fg_name);
			if (rc < 0) {
				mmi_chrg_err(chip, "Cannot parse extrn-fg-name\n");
				return rc;
			}
		}
	}

	chip->extrn_sense = of_property_read_bool(node,
			"mmi,extrn-sense");

	chip->dont_rerun_aicl= of_property_read_bool(node,
			"mmi,dont-rerun-aicl");

	rc = of_property_read_u32(node,
				"mmi,typec-middle-current",
				&chip->typec_middle_current);
	if (rc < 0)
		chip->typec_middle_current =
				TYPEC_MIDDLE_CURRENT_UA;

	rc = of_property_read_u32(node,
				"mmi,step-first-current-comp",
				&chip->step_first_curr_comp);
	if (rc < 0)
		chip->step_first_curr_comp =
				STEP_FIREST_CURR_COMP;

	rc = of_property_read_u32(node,
				"mmi,pps-volt-steps",
				&chip->pps_volt_steps);
	if (rc < 0)
		chip->pps_volt_steps =
				DEFAULT_PPS_VOLT_STEPS;

	rc = of_property_read_u32(node,
				"mmi,pps-curr-steps",
				&chip->pps_curr_steps);
	if (rc < 0)
		chip->pps_curr_steps =
				DEFAULT_PPS_CURR_STEPS;
	rc = of_property_read_u32(node,
				"mmi,thermal-min-level",
				&chip->thermal_min_level);
	if (rc < 0)
		chip->thermal_min_level =
				DEFAULT_THERMAL_MIN_LEVL;
	rc = of_property_read_u32(node,
				"mmi,pd-volt-max",
				&chip->pd_volt_max);
	if (rc < 0)
		chip->pd_volt_max =
				DEFAULT_PPS_VOLT_MAX;
	pd_volt_max_init = chip->pd_volt_max;
	rc = of_property_read_u32(node,
				"mmi,pd-curr-max",
				&chip->pd_curr_max);
	if (rc < 0)
		chip->pd_curr_max =
				DEFAULT_PPS_CURR_MAX;
	pd_curr_max_init = chip->pd_curr_max;

	for_each_child_of_node(node, child)
		chip->mmi_chrg_dev_num++;

	if (!chip->mmi_chrg_dev_num) {
		mmi_chrg_err(chip,"No mmi_chrg_dev  list !\n");
		return -ENODEV;
	}

	chrg_name_list = (struct mmi_chrg_dts_info *)devm_kzalloc(chip->dev,
					sizeof(struct mmi_chrg_dts_info) *
					chip->mmi_chrg_dev_num, GFP_KERNEL);
	if (!chrg_name_list) {
		mmi_chrg_err(chip,"No memory for mmi charger device name list !\n");
		goto cleanup;
	}

	chip->chrg_list = (struct mmi_charger_device **)devm_kzalloc(chip->dev,
					sizeof(struct mmi_charger_device *) *
					chip->mmi_chrg_dev_num, GFP_KERNEL);
	if (!chip->chrg_list) {
		mmi_chrg_err(chip,"No memory for mmi charger device list !\n");
		goto cleanup;
	}

	for_each_child_of_node(node, child) {
		byte_len = of_property_count_strings(child, "chrg-name");
		if (byte_len <= 0) {
			mmi_chrg_err(chip, "Cannot parse chrg-name: %d\n", byte_len);
			goto cleanup;
		}

		for (i = 0; i < byte_len; i++) {
			rc = of_property_read_string_index(child, "chrg-name",
					i, &chrg_name_list[chrg_idx].chrg_name);
			if (rc < 0) {
				mmi_chrg_err(chip, "Cannot parse chrg-name\n");
				goto cleanup;
			}
		}

		byte_len = of_property_count_strings(child, "psy-name");
		if (byte_len <= 0) {
			mmi_chrg_err(chip, "Cannot parse psy-name: %d\n", byte_len);
			goto cleanup;
		}

		for (i = 0; i < byte_len; i++) {
			rc = of_property_read_string_index(child, "psy-name",
					i, &chrg_name_list[chrg_idx].psy_name);
			if (rc < 0) {
				mmi_chrg_err(chip, "Cannot parse psy-name\n");
				goto cleanup;
			}
		}

		mmi_chrg_info(chip, "mmi,chrg-name: %s, psy-name: %s\n",
					chrg_name_list[chrg_idx].chrg_name,
					chrg_name_list[chrg_idx].psy_name);

		of_property_read_u32(child, "charging-curr-limited",
			&chrg_name_list[chrg_idx].charging_curr_limited);

		of_property_read_u32(child, "charging-curr-min",
			&chrg_name_list[chrg_idx].charging_curr_min);

		chrg_idx++;
	}

	chip->sourcecap_dec_enable = of_property_read_bool(node, "mmi,enable-new-sourcecap-dec");
	chip->disable_ignore_hysteresis = of_property_read_bool(node, "mmi,disable-ignore-hysteresis");

	rc = of_property_read_u32(node,
				"mmi,chrg-temp-zones-num",
				&chip->num_temp_zones);
	if (rc < 0)
		chip->num_temp_zones =
				MMI_TEMP_ZONES;

	chip->temp_zones = (struct mmi_chrg_temp_zone *)
					devm_kzalloc(chip->dev,
					sizeof(struct mmi_chrg_temp_zone )
					* chip->num_temp_zones,
					GFP_KERNEL);
	if (chip->temp_zones == NULL) {
		rc = -ENOMEM;
		goto cleanup;
	}

	if (of_find_property(node, "mmi,mmi-chrg-temp-zones", &byte_len)) {
		if (byte_len <= 0) {
			mmi_chrg_err(chip,
					"Cannot parse mmi-chrg-temp-zones %d\n",byte_len);
			goto cleanup;
		}

		step_cnt = (byte_len / chip->num_temp_zones - sizeof(u32))
				/ (sizeof(u32) * 2);
		mmi_chrg_info(chip, "mmi chrg step number is %d\n", step_cnt);

		dt_temp_zones = (u32 *) devm_kzalloc(chip->dev, byte_len, GFP_KERNEL);
		if (dt_temp_zones == NULL) {
			rc = -ENOMEM;
			goto cleanup;
		}

		rc = of_property_read_u32_array(node,
				"mmi,mmi-chrg-temp-zones",
				(u32 *)dt_temp_zones,
				byte_len / sizeof(u32));
		if (rc < 0) {
			mmi_chrg_err(chip,
				   "Couldn't read mmi chrg temp zones rc = %d\n", rc);
			goto zones_failed;
		}

		mmi_chrg_err(chip, "temp zone nums: %d , byte_len %d, "
						"num %d, row size %d\n",
						chip->num_temp_zones, byte_len,
						(int)(byte_len / sizeof(u32)),
						(int)(sizeof(struct mmi_chrg_step_power) * step_cnt));
		ptr = dt_temp_zones;

		for (i = 0; i < byte_len / sizeof(u32); i++) {
			if (!(i % 9)) {
			  printk("\n");
			}
			printk("%u,", ptr[i]);
		}
		printk("\n");

		for (i = 0; i < chip->num_temp_zones; i++) {
			idx = (int)((sizeof(u32) +
				sizeof(struct mmi_chrg_step_power) * step_cnt )
				* i / sizeof(u32));
			chip->temp_zones[i].temp_c = dt_temp_zones[idx];
		}

		for (i = 0; i < chip->num_temp_zones; i++) {
			idx = (int)(((sizeof(u32) +
					sizeof(struct mmi_chrg_step_power) * step_cnt )
					* i + sizeof(u32)) / sizeof(u32));
			chip->temp_zones[i].chrg_step_power =
					(struct mmi_chrg_step_power *)&dt_temp_zones[idx];
		}

		for (i = 0; i < chip->num_temp_zones; i++) {

			printk( "mmi temp zones: Zone: %d, Temp: %d C " , i,
				   chip->temp_zones[i].temp_c);
			for (j = 0; j < step_cnt; j++) {
				chip->temp_zones[i].chrg_step_power[j].chrg_step_curr *= 1000;
				chip->temp_zones[i].chrg_step_power[j].chrg_step_volt *= 1000;

				printk("step_volt %dmV, step_curr  %dmA, ",
				   chip->temp_zones[i].chrg_step_power[j].chrg_step_volt,
				   chip->temp_zones[i].chrg_step_power[j].chrg_step_curr);
			}
			printk("\n");
		}

		chip->pres_temp_zone = ZONE_NONE;
		chip->chrg_step_nums = step_cnt;
	}

	return rc;

zones_failed:
	devm_kfree(chip->dev,chip->temp_zones);
cleanup:
	chip->mmi_chrg_dev_num = 0;
	devm_kfree(chip->dev,chip->chrg_list);
	return rc;
}

static int mmi_chrg_manager_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mmi_charger_manager *chip;

	dev_err(&pdev->dev,"mmi_chrg_manager_probe  1\n");

	if (!pdev)
		return -ENODEV;

	chip = devm_kzalloc(&pdev->dev, sizeof(struct mmi_charger_manager),
								GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev,
			"Unable to alloc memory for mmi_charger_manager\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	chip->name = "mmi_chrg_manager";
	chip->debug_mask = &__debug_mask;
	chip->suspended = false;
	chip->system_thermal_level = THERMAL_NOT_LIMIT;

	ret = mmi_charger_class_init();
	if (ret < 0) {
		dev_err(&pdev->dev,
			"mmi charger class init failed\n");
		goto cleanup;
	}

	ret = mmi_chrg_manager_parse_dt(chip);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"parse dt failed\n");
		goto cleanup;
	}

	chip->factory_mode = mmi_factory_check();

	chip->mtk_psy = power_supply_get_by_name("mtk_battery");
	if (chip->mtk_psy && chip->extrn_fg) {

		chip->extrn_psy = power_supply_get_by_name(chip->extrn_fg_name);
		if (!chip->extrn_psy) {
			mmi_chrg_err(chip, "Get extrn psy failed\n");
			goto cleanup;
		}
		ret = batt_psy_register(chip);
		if (ret) {
			mmi_chrg_err(chip, "Batt psy register failed\n");
			goto cleanup;
		}
	} else
		chip->batt_psy = power_supply_get_by_name("battery");

	if (!chip->batt_psy) {
		mmi_chrg_err(chip, "Could not get battery power_supply\n");
		ret = -EPROBE_DEFER;
		goto cleanup;
	}


	ret = mmi_chrg_policy_init(chip, chrg_name_list,
					chip->mmi_chrg_dev_num);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"mmi chrg policy init failed\n");
		ret = -EPROBE_DEFER;
		goto cleanup;
	}

	chip->charger_psy= devm_power_supply_get_by_phandle(chip->dev, "charger");
	if (IS_ERR_OR_NULL(chip->charger_psy)) {
		mmi_chrg_err(chip, "%s Couldn't get bat_psy\n", __func__);
		return -ENODEV;;
	}

	chip->usb_psy = power_supply_get_by_name(chrg_name_list[CP_MASTER].psy_name); //david
	if (!chip->usb_psy) {
		mmi_chrg_err(chip, "Could not get USB power_supply 1-1\n");
		return -ENODEV;
	}

	ret = init_tcpc(chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "tcpc init failed\n");
		goto cleanup;
	}
	mmi_chrg_err(chip, "Could get USB power_supply 2\n");

	INIT_WORK(&chip->psy_changed_work, psy_changed_work_func);
	INIT_DELAYED_WORK(&chip->heartbeat_work, mmi_heartbeat_work);
//	wakeup_source_init(&chip->mmi_hb_wake_source,
//			   "mmi_hb_wake");
	alarm_init(&chip->heartbeat_alarm, ALARM_BOOTTIME,
		   mmi_heartbeat_alarm_cb);

	mmi_chrg_err(chip, "Could get USB power_supply 3\n");
	ret = mmi_chrg_mgr_psy_register(chip);
	if (ret)
		goto cleantcpc;
	mmi_chrg_err(chip, "Could get USB power_supply 4\n");
	chip->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&chip->psy_nb);
	if (ret)
		goto cleantcpc;

	mmi_batt_age_init(chip);

	init_completion(&chip->sm_completion);
	platform_set_drvdata(pdev, chip);

	//create_sysfs_entries(chip);
	schedule_work(&chip->psy_changed_work);
//	mmi_chrg_info(chip, "mmi chrg manager initialized successfully, ret %d\n", ret);
	mmi_chrg_err(chip, "mmi_chrg_manager_probe ok\n");
	return 0;
cleantcpc:
	unregister_tcp_dev_notifier(chip->tcpc, &chip->tcp_nb,
                                 TCP_NOTIFY_TYPE_USB);

cleanup:
	mmi_charger_class_exit();
	devm_kfree(&pdev->dev, chip);
	return ret;
}

static int mmi_chrg_manager_remove(struct platform_device *pdev)
{
	struct mmi_charger_manager *chip =  platform_get_drvdata(pdev);

	//remove_sysfs_entries(chip);
	//cancel_delayed_work_sync(&chip->mmi_chrg_sm_work);
	power_supply_unreg_notifier(&chip->psy_nb);
	power_supply_unregister(chip->mmi_chrg_mgr_psy);

	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, chip);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mmi_chrg_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct mmi_charger_manager *chip = platform_get_drvdata(pdev);

	chip->suspended = true;

	return 0;
}

static int mmi_chrg_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct mmi_charger_manager *chip = platform_get_drvdata(pdev);

	chip->suspended = false;

	return 0;
}
#else
#define smb_mmi_suspend NULL
#define smb_mmi_resume NULL
#endif

static const struct dev_pm_ops mmi_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mmi_chrg_suspend, mmi_chrg_resume)
};

static const struct of_device_id mmi_chrg_manager_match_table[] = {
	{.compatible = "mmi,chrg-manager"},
	{},
};
MODULE_DEVICE_TABLE(of, mmi_chrg_manager_match_table);

static struct platform_driver mmi_chrg_manager_driver = {
	.probe = mmi_chrg_manager_probe,
	.remove = mmi_chrg_manager_remove,
	.driver = {
		.name = "mmi_chrg_manager",
		.owner = THIS_MODULE,
		.pm = &mmi_dev_pm_ops,
		.of_match_table = mmi_chrg_manager_match_table,
	},
};

static int __init mmi_chrg_manager_init(void)
{
	int ret;
	ret = platform_driver_register(&mmi_chrg_manager_driver);
	if (ret) {
		pr_err("mmi_chrg_manager failed to register driver\n");
		return ret;
	}
	return 0;
}

static void __exit mmi_chrg_manager_exit(void)
{
	platform_driver_unregister(&mmi_chrg_manager_driver);
}

module_init(mmi_chrg_manager_init);
module_exit(mmi_chrg_manager_exit);

MODULE_ALIAS("platform:mmi parallel charger");
MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_DESCRIPTION("Motorola Mobility parallel charger");
MODULE_LICENSE("GPL");