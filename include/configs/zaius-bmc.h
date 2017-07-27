/*
 * Copyright 2016 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#ifndef __AST_G5_NCSI_CONFIG_H
#define __AST_G5_NCSI_CONFIG_H

#define CONFIG_ARCH_AST2500
#define CONFIG_SYS_LOAD_ADDR		0x83000000

#include <configs/ast-common.h>

/* arm1176/start.S */
#define CONFIG_SYS_UBOOT_BASE		CONFIG_SYS_TEXT_BASE

/* Ethernet */
#define CONFIG_LIB_RAND
#define CONFIG_FTGMAC100
#define CONFIG_MAC_NUM		2
#define CONFIG_PHY_MAX_ADDR		32

#define CONFIG_MII
#define CONFIG_CMD_MII
#define CONFIG_RESET_PHY_R

#define CONFIG_PHY_MAX_ADDR	32	/* this comes from <linux/phy.h> */

/* FRU EEPROM */
#define CONFIG_EEPROM_OFFSET_LEN			(2)
#define CONFIG_EEPROM_I2C_BUS_NUM			(1)
#define CONFIG_EEPROM_I2C_ADDR			(0x50)
#define CONFIG_ID_EEPROM

/* platform.S settings */
#define CONFIG_DRAM_ECC
#define	CONFIG_DRAM_ECC_SIZE		0x20000000

#endif	/* __AST_G5_NCSI_CONFIG_H */
