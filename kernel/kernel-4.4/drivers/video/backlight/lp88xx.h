/*
 * TI LP88XX Backlight Core Driver
 *
 * Copyright 2016 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __BL_LP88XX_H
#define __BL_LP88XX_H

#include <linux/pwm.h>

#define EEPROM_NUM_ADDR	22

enum lp88xx_chip_id {
	LP8580,
	LP88XX,
};

struct lp88xx_io {
	int (*write)(void *p, u16 reg, u16 val);
	int (*read)(void *p, u16 reg, u16 *val);
};

struct eeprom_datum {
	u16 addr;
	u16 val;
};

struct lp88xx {
	struct device *dev;
	void *priv;
	struct lp88xx_io io;
	unsigned long region_used;	/* Bit mask for LED region map */
	struct pwm_device *pwm;
	unsigned int period;
	u16 max_dev_brt;
	struct eeprom_datum default_eeprom[EEPROM_NUM_ADDR];
	int eeprom_count;
	u32 max_input_brt;
	enum lp88xx_chip_id chip_id;
	bool eeprom_lock;
};

extern int lp88xx_common_probe(struct device *dev, struct lp88xx *lp);
#endif
