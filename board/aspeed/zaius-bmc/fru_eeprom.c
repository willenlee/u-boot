/*
 * Copyright 2016 Google Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <i2c.h>
#include <net.h>

#define RETURN_IF_ERROR(expr) do {\
	int ret = (expr);\
	if (ret < 0) {\
		debug(#expr "in %s failed.\n", __func__);\
		return ret;\
	}\
} while (0)

#define ECHECKSUM	(2001)
#define MACADDR_SIZE	(6)

static struct __attribute__((packed)) {
	uint8_t version;
	uint8_t iua_offset;
	uint8_t cia_offset;
	uint8_t ba_offset;
	uint8_t pia_offset;
	uint8_t mra_offset;
	uint8_t padding;
	uint8_t checksum;
} fru_header;

static struct __attribute__((packed)) {
	uint8_t version;
	uint8_t type;
	uint8_t length;
	uint8_t mac_addr[MACADDR_SIZE];
	uint8_t num_mac_addrs;
	uint8_t padding[21];
	uint8_t checksum;
} fru_iua;

static bool is_eeprom_read = false;

static int fru_checksum(void *data, size_t len)
{
	uint8_t result = 0;
	uint8_t *p = (uint8_t *) data;

	size_t i = 0;
	for (; i < len; ++i, ++p) {
		result += *p;
	}

	return result == 0 ? 0 : -ECHECKSUM;
}

static int read_eeprom(void)
{
	if (is_eeprom_read)
		return 0;

	struct udevice *eeprom;

	RETURN_IF_ERROR(i2c_get_chip_for_busnum
			(CONFIG_EEPROM_I2C_BUS_NUM, CONFIG_EEPROM_I2C_ADDR, 1,
			 &eeprom));
	RETURN_IF_ERROR(i2c_set_chip_offset_len
			(eeprom, CONFIG_EEPROM_OFFSET_LEN));

	RETURN_IF_ERROR(dm_i2c_read
			(eeprom, 0, (uint8_t *) & fru_header,
			 sizeof(fru_header)));
	RETURN_IF_ERROR(fru_checksum(&fru_header, sizeof(fru_header)));

	RETURN_IF_ERROR(dm_i2c_read
			(eeprom, fru_header.iua_offset * 8,
			 (uint8_t *) & fru_iua, sizeof(fru_iua)));
	RETURN_IF_ERROR(fru_checksum(&fru_iua, sizeof(fru_iua)));

	is_eeprom_read = true;
	return 0;
}

/* CONFIG_ID_EEPROM setting, that enables mac_read_from_eeprom call
 * in board_r also enables this function/command.
 * We don't need any functionality of this command, definitely don't need
 * eeprom writing functionality, so I just made it print info about MACs.
 */
int do_mac(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = read_eeprom();
	if (ret < 0) {
		printf("Can't read EEPROM: %d\n", ret);
		return ret;
	}

	printf("Number of MAC Addrs in EEPROM: %d\n", fru_iua.num_mac_addrs);
	uint8_t fru_mac_addr[MACADDR_SIZE];
	memcpy(fru_mac_addr, fru_iua.mac_addr, MACADDR_SIZE);
	int i = 0;
	for (; i < fru_iua.num_mac_addrs; ++i) {
		printf("#%d: %02x:%02x:%02x:%02x:%02x:%02x\n", i + 1,
			   fru_mac_addr[0],
			   fru_mac_addr[1],
			   fru_mac_addr[2],
			   fru_mac_addr[3],
			   fru_mac_addr[4],
			   fru_mac_addr[5]);
		fru_mac_addr[5] += 1;
		if (fru_mac_addr[5] == 0) fru_mac_addr[4] += 1;
		if (fru_mac_addr[4] == 0) fru_mac_addr[3] += 1;
		if (fru_mac_addr[3] == 0) fru_mac_addr[2] += 1;
		if (fru_mac_addr[2] == 0) fru_mac_addr[1] += 1;
		if (fru_mac_addr[1] == 0) fru_mac_addr[0] += 1;
	}
	return 0;
}

int mac_read_from_eeprom(void)
{
	debug("Setting MACs from FRU\n");
	/* The failure to read eeprom is not fatal to boot,
	 * so just returning zero anyway.
	 */
	int err = read_eeprom();
	if (err < 0) {
		debug("Failed to read FRU EEPROM\n");
		return 0;
	}

	uint8_t temp_mac_addr[MACADDR_SIZE];
	uint8_t fru_mac_addr[MACADDR_SIZE];
	memcpy(fru_mac_addr, fru_iua.mac_addr, MACADDR_SIZE);

	int i = 1;
	debug("Number of Addrs: %d\n", fru_iua.num_mac_addrs);
	for (; i < fru_iua.num_mac_addrs; ++i) {
		fru_mac_addr[5] += 1;
		if (fru_mac_addr[5] == 0) fru_mac_addr[4] += 1;
		if (fru_mac_addr[4] == 0) fru_mac_addr[3] += 1;
		if (fru_mac_addr[3] == 0) fru_mac_addr[2] += 1;
		if (fru_mac_addr[2] == 0) fru_mac_addr[1] += 1;
		if (fru_mac_addr[1] == 0) fru_mac_addr[0] += 1;

		/* env always takes priority */
		if (!eth_getenv_enetaddr_by_index("eth", i-1, temp_mac_addr)) {
			eth_setenv_enetaddr_by_index("eth", i-1,
						     fru_mac_addr);
		} else {
			debug("eth%daddr already in env\n", i);
		}
	}

	return 0;
}
