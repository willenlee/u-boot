/*
 * Copyright 2016 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#ifndef __AST_G4_NCSI_CONFIG_H
#define __AST_G4_NCSI_CONFIG_H

#define CONFIG_ARCH_AST2400
#define CONFIG_SYS_LOAD_ADDR		0x43000000
#define CONFIG_FLASH_AST2400

#define CONFIG_MISC_INIT_R

#include <configs/ast-common.h>

/* platform.S settings */
#define CONFIG_CPU_420			1
#define CONFIG_DRAM_528			1
#define CONFIG_BAUDRATE                 115200
/*
 * Environment Config
 */
#define CONFIG_CMDLINE_TAG	 1		/* enable passing of ATAGs	*/
#define CONFIG_SETUP_MEMORY_TAGS 1
#define CONFIG_INITRD_TAG	 1
#define	CONFIG_BOOTARGS 	"console=ttyS4,115200n8 root=/dev/ram rw"
#define CONFIG_UPDATE           "tftp 40800000 ast2400.scr; so 40800000'"

#define CONFIG_BOOTDELAY	3		/* autoboot after 3 seconds	*/
#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_AUTOBOOT_PROMPT		\
	"autoboot in %d seconds (stop with 'Delete' key)...\n", bootdelay
#define CONFIG_AUTOBOOT_STOP_STR	"\x1b\x5b\x33\x7e" /* 'Delete', ESC[3~ */
#define CONFIG_ZERO_BOOTDELAY_CHECK

#ifdef CONFIG_FLASH_AST2400
#define CONFIG_BOOTCOMMAND	"bootm 20080000 20300000"
#else
#ifdef	CONFIG_SYS_FLASH_CFI
#define CONFIG_BOOTCOMMAND	"bootm 10080000 10300000"
#else
#define CONFIG_BOOTCOMMAND	"bootm 14080000 14300000"
#endif
#endif
#define CONFIG_BOOTFILE		"all.bin"
#define CONFIG_ENV_OVERWRITE


/*
 * Command line configuration.
 */

#define CONFIG_CMD_I2C
#define CONFIG_CMD_EEPROM
#define CONFIG_CMD_NET

/*
 * I2C configuration
 */
#define CONFIG_DRIVER_ASPEED_I2C
#define CONFIG_HARD_I2C
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_SYS_I2C_SLAVE		1
#define CONFIG_DRIVER_ASPEED_I2C

/*
* EEPROM configuration
*/
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN 	2
#define CONFIG_SYS_I2C_EEPROM_ADDR 	0xa0

/*
 * MAC01 & MAC02 Address read from eeprom
 */
#define CONFIG_SET_ETHADDR_EEPOM
#define CONFIG_EEPROM_BUS_NUM 4
#define CONFIG_SYS_I2C_MAC01_OFFSET 0x2000
#define CONFIG_SYS_I2C_MAC02_OFFSET 0x7ff0

#define __BYTE_ORDER __LITTLE_ENDIAN
#define __LITTLE_ENDIAN_BITFIELD

/*
 * NIC configuration
 */
#define CONFIG_ASPEEDNIC
#define CONFIG_NET_MULTI
#define CONFIG_MAC1_PHY_LINK_INTERRUPT
#define CONFIG_MAC2_ENABLE
#define CONFIG_MAC2_PHY_LINK_INTERRUPT
/*
*-------------------------------------------------------------------------------
* NOTICE: MAC1 and MAC2 now have their own seperate PHY configuration.
* We use 2 bits for each MAC in the scratch register(D[15:11] in 0x1E6E2040) to
* inform kernel driver.
* The meanings of the 2 bits are:
* 00(0): Dedicated PHY
* 01(1): ASPEED's EVA + INTEL's NC-SI PHY chip EVA
* 10(2): ASPEED's MAC is connected to NC-SI PHY chip directly
* 11: Reserved
*
* We use CONFIG_MAC1_PHY_SETTING and CONFIG_MAC2_PHY_SETTING in U-Boot
* 0: Dedicated PHY
* 1: ASPEED's EVA + INTEL's NC-SI PHY chip EVA
* 2: ASPEED's MAC is connected to NC-SI PHY chip directly
* 3: Reserved
*-------------------------------------------------------------------------------
*/
#define CONFIG_MAC1_PHY_SETTING		0
#define CONFIG_MAC2_PHY_SETTING		0
#define CONFIG_ASPEED_MAC_NUMBER  1
#define CONFIG_ASPEED_MAC_CONFIG  1 
#define _PHY_SETTING_CONCAT(mac) CONFIG_MAC##mac##_PHY_SETTING
#define _GET_MAC_PHY_SETTING(mac) _PHY_SETTING_CONCAT(mac)
#define CONFIG_ASPEED_MAC_PHY_SETTING \
  _GET_MAC_PHY_SETTING(CONFIG_ASPEED_MAC_CONFIG)
#define CONFIG_MAC_INTERFACE_CLOCK_DELAY	0x2255
#define CONFIG_RANDOM_MACADDR 0

#endif	/* __AST_G4_NCSI_CONFIG_H */
