/*
 * Copyright (C) 2020 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
 #ifndef __MMI_SMART_BATTERY_CORE_H__
#define __MMI_SMART_BATTERY_CORE_H__

#include <linux/mmi_gauge_class.h>
#include <linux/power/moto_chg_tcmd.h>

#define mmi_err(chg, fmt, ...)			\
	do {						\
		pr_err("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
	} while (0)

#define mmi_warn(chg, fmt, ...)			\
	do {						\
		pr_warn("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
	} while (0)

#define mmi_info(chg, fmt, ...)			\
	do {						\
		pr_info("%s: %s: " fmt, chg->name,	\
		       __func__, ##__VA_ARGS__);	\
	} while (0)

#define mmi_dbg(chg, fmt, ...)			\
	do {							\
		if (*chg->debug_enabled)		\
			pr_info("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)

struct mmi_battery_pack {
	int	status;
	int	voltage_now;
	int	current_now;
	int	soc;
	int	curr_batt_temp;
	int	batt_tte;
	int	charge_full;
	int	charge_full_design;
	int	charge_counter;
	int	soh;
	int	cycle_count;
	struct gauge_device	*gauge_dev;
	struct list_head list;
};

struct mmi_smart_battery {
	struct device		*dev;
	char				*name;

	struct gauge_device	*gauge_dev;
	struct power_supply	*batt_psy;
	struct list_head		battery_list;
	struct moto_chg_tcmd_client batt_tcmd_client;

	struct workqueue_struct	*fg_workqueue;
	struct delayed_work		battery_delay_work;

	bool				*debug_enabled;
	unsigned long			manufacturing_date;
	unsigned long			first_usage_date;
	bool				resume_completed;
	bool				factory_mode;
	bool				sync_boardtemp_to_fg;
	int				ui_full_soc;
	int				uisoc;
	int				main_batt_soc;
	int				flip_batt_soc;
	int				batt_tte;
	int				combo_voltage_now;
	int				combo_current_now;
	int				combo_batt_temp;
	int				combo_charge_full;
	int				combo_charge_full_design;
	int				combo_charge_counter;
	int				combo_soh;
	int				combo_cycle_count;
	int				shutdown_threshold;
	int				soc100_curr_threshod;
	int				taper_count;
	int				is_ffc_charge;
	int				vbat0_flag;
	int				fake_soc;
	int				fake_temp;
	int				gauge_count;
	const char		**gauge_name_arry;
	struct mmi_battery_pack *battery;
};

#define QUEUS_DELAYED_WORK_TIME  8000
#define QUEUE_START_WORK_TIME    1
#define INVALID_TEMP (-2730)
#define SMART_BATT_SHOW_MAX_SIZE 64

extern int mmi_batt_health_check(void);

extern int mmi_charger_update_batt_status(void);

#endif
