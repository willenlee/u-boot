/*
 * Configuation settings for the ASPEED AST Chip.
 *
 * Copyright (C) 2012-2020 ASPEED Tech. Inc.
 * Ryan Chen (ryan_chen@aspeedtech.com)
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <spi.h>
#include <malloc.h>
#include <asm/io.h>

#include <spi_flash.h>
#include "../mtd/spi/sf_internal.h"

#include <asm/arch/regs-fmc.h>
#include <asm/arch/ast-scu.h>
#include <asm/arch/aspeed.h>

struct ast_spi_host {
	struct spi_slave slave;
	void		*base;
	void		*regs;
	void 		*buff;
	u32 	(*get_clk)(void);
};

/* SPI Define */
#define CE_CTRL				0x04
#define CS0_CTRL			0x10
#define CS1_CTRL			0x14
#define CS2_CTRL			0x18
#define CS3_CTRL			0x1c

#define NORMALREAD		0x00
#define FASTREAD		0x01
#define NORMALWRITE		0x02
#define USERMODE		0x03

#define IOMODEx1		0x00000000
#define IOMODEx2		0x20000000
#define IOMODEx2_dummy		0x30000000
#define IOMODEx4		0x40000000
#define IOMODEx4_dummy		0x50000000

/* for DMA */
#define REG_FLASH_INTERRUPT_STATUS	0x08
#define REG_FLASH_DMA_CONTROL		0x80
#define REG_FLASH_DMA_FLASH_BASE		0x84
#define REG_FLASH_DMA_DRAM_BASE		0x88
#define REG_FLASH_DMA_LENGTH			0x8c

#define FLASH_STATUS_DMA_BUSY			0x0000
#define FLASH_STATUS_DMA_READY		0x0800
#define FLASH_STATUS_DMA_CLEAR		0x0800

#define FLASH_DMA_ENABLE		0x01

//
#define CE_LOW			0x00
#define CE_HIGH			0x04
#define CMD_MASK		0xFFFFFFF8

unsigned int bus = CONFIG_SF_DEFAULT_BUS;
unsigned int cs = CONFIG_SF_DEFAULT_CS;
unsigned int speed = CONFIG_ENV_SPI_MAX_HZ;	//50MHz
unsigned int mode = CONFIG_ENV_SPI_MODE;
unsigned int boot_mode = 0;

unsigned int READ_CMD = NULL;
unsigned int WRITE_CMD = NULL;
unsigned int ERASE_CMD = NULL;

#define boot_in_4byte 1
#define boot_in_3byte 2

static struct spi_flash *flash;

//#define ASPEED_SPI_DEBUG

#ifdef ASPEED_SPI_DEBUG
#define SPIDBUG(fmt, args...) printf(fmt, ## args)
#else
#define SPIDBUG(fmt, args...)
#endif

static inline struct ast_spi_host *to_ast_spi(struct spi_slave *slave)
{
	return container_of(slave, struct ast_spi_host, slave);
}

static inline void
ast_spi_write(struct ast_spi_host *spi, u32 val, u32 reg)
{
	SPIDBUG(" val: %x , reg : %x \n",val,reg);
	if(reg == 0x0)
		__raw_writel(val, spi->regs + reg);
	else
		__raw_writel(val, spi->base + reg);
}

static inline u32
ast_spi_read(struct ast_spi_host *spi, u32 reg)
{
	if(reg == 0x0)
		return __raw_readl(spi->regs + reg);
	else
		return __raw_readl(spi->base + reg);
}

DECLARE_GLOBAL_DATA_PTR;

static u32 ast_spi_calculate_divisor(u32 max_speed_hz)
{
	// [0] ->15 : HCLK , HCLK/16
	u8 SPI_DIV[16] = {16, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 0};
	u32 i, hclk, spi_cdvr=0;

	hclk = ast_get_ahbclk();
	for(i=1;i<17;i++) {
		if(max_speed_hz >= (hclk/i)) {
			spi_cdvr = SPI_DIV[i-1];
//			printf("hclk = %d , spi_cdvr = %d \n",hclk, spi_cdvr);
			break;
		}
	}

//	printf("hclk is %d, divisor is %d, target :%d , cal speed %d\n", hclk, spi_cdvr, max_speed_hz, hclk/SPI_DIV[i]);
	return spi_cdvr;
}

void spi_init(void)
{
	SPIDBUG("spi_init \n");


	struct ast_spi_host	*spi;
	ulong ulCtrlData, CtrlOffset = CS0_CTRL;
	u32 div;
	u32 spi_ce_ctrl;
	ulong hw_strap_reg;

	//Always set to 4byte
	boot_mode = boot_in_4byte;
	spi_set_4byte(spi);
	flash_set_4byte(spi);
	printf("Strap set to 4byte\n");

	flash = spi_flash_probe(bus, cs, speed, mode);

	if (!flash) {
		printf("Failed to initialize SPI flash at %u:%u\n", bus, cs);
	}

	spi = malloc(sizeof(struct ast_spi_host));
	if (!spi){
		printf("Allocate memory fail\n");
		return NULL;
	}

	spi->slave.bus = bus;
	spi->slave.cs = cs;

	/* Flash Controller */
	*((volatile ulong*) AST_FMC_BASE) |= 0x800f0000; /* enable Flash Write */

	spi->base = AST_FMC_BASE;

	/* Setup FMC register and buffer address */
	switch (cs) {
#ifdef AST_FMC_CS0_BASE
	case 0:
		spi->regs = (void *)AST_FMC_BASE + 0x10;
		spi->buff = (void *)AST_FMC_CS0_BASE;
		CtrlOffset = CS0_CTRL;
		break;
#endif
#ifdef AST_FMC_CS1_BASE
	case 1:
		spi->regs = (void *)AST_FMC_BASE + 0x14;
		spi->buff = (void *)AST_FMC_CS1_BASE;
		ast_scu_multi_func_romcs(1);
		CtrlOffset = CS1_CTRL;
		break;
#endif
#ifdef AST_FMC_CS2_BASE
	case 2:
		spi->regs = (void *)AST_FMC_BASE + 0x18;
		spi->buff = (void *)AST_FMC_CS2_BASE;
		ast_scu_multi_func_romcs(2);
		CtrlOffset = CS2_CTRL;
		break;
#endif
#ifdef AST_FMC_CS3_BASE
	case 3:
		spi->regs = (void *)AST_FMC_BASE + 0x1c;
		spi->buff = (void *)AST_FMC_CS3_BASE;
		ast_scu_multi_func_romcs(3);
		CtrlOffset = CS3_CTRL;
		break;
#endif

	default:
		spi->regs = (void *)AST_FMC_BASE + 0x10;
		spi->buff = (void *)AST_FMC_CS0_BASE;
		CtrlOffset = CS0_CTRL;
		break;
	}

		printf("Init READ_CMD = %x \n", READ_CMD);

	READ_CMD = flash->read_cmd;
	WRITE_CMD = flash->write_cmd;
	ERASE_CMD = flash->erase_cmd;

	ulCtrlData |= IOMODEx1;
	//ulCtrlData |= SPI_DUAL_DATA;
	ulCtrlData |= SPI_CMD_FAST_R_MODE;
	ulCtrlData |= SPI_CMD_DATA(CMD_READ_ARRAY_FAST);
	ulCtrlData |= SPI_DUMMY_LOW(1);

	//For SPI colck freq selection
	div = ast_spi_calculate_divisor(CONFIG_ENV_SPI_MAX_HZ);
	ulCtrlData |= SPI_CLK_DIV(div);

	//ISSUE : ast spi ctrl couldn't use mode 3, so fix mode 0
	 ulCtrlData &= ~SPI_CPOL_1;

	//Set control data
	ast_spi_write(spi, ulCtrlData, CtrlOffset);

#if 0
	/* Environment protection ON by default */
#ifdef CONFIG_ENV_IS_IN_SPI_FLASH
	spi_flash_protect (flash,
		       CONFIG_ENV_ADDR,
		       CONFIG_ENV_ADDR + CONFIG_ENV_SECT_SIZE - 1,
		       true);
#endif

	/* Redundant environment protection ON by default */
#ifdef CONFIG_ENV_ADDR_REDUND
	spi_flash_protect (flash,
		       CONFIG_ENV_ADDR_REDUND,
		       CONFIG_ENV_ADDR_REDUND + CONFIG_ENV_SIZE_REDUND - 1,
		       true);
#endif
#endif
	//free flash
	//spi_flash_free(flash);

	printf("Init OK! \n");
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
				  unsigned int max_hz, unsigned int mode)
{
	struct ast_spi_host	*spi;
	u32			spi_ctrl;
	u32 		div;
	ulong reg;
	ulong ulCtrlData, CtrlOffset = CS0_CTRL;

	//printf("bus %d , cs %d , max_hz %d , mode %d \n", bus, cs, max_hz, mode);

	spi = malloc(sizeof(struct ast_spi_host));
	if (!spi)
		return NULL;

	spi->slave.bus = bus;
	spi->slave.cs = cs;
	spi->slave.memory_map = (void *)AST_FMC_CS0_BASE;

	/* Flash Controller */
	*((volatile ulong*) AST_FMC_BASE) |= 0x800f0000; /* enable Flash Write */

	spi->base = AST_FMC_BASE;

	switch (cs) {
#ifdef AST_FMC_CS0_BASE
	case 0:
		spi->regs = (void *)AST_FMC_BASE + 0x10;
		spi->buff = (void *)AST_FMC_CS0_BASE;
		break;
#endif
#ifdef AST_FMC_CS1_BASE
	case 1:
		spi->regs = (void *)AST_FMC_BASE + 0x14;
		spi->buff = (void *)AST_FMC_CS1_BASE;
		break;
#endif
#ifdef AST_FMC_CS2_BASE
	case 2:
		spi->regs = (void *)AST_FMC_BASE + 0x18;
		spi->buff = (void *)AST_FMC_CS2_BASE;
		break;
#endif
#ifdef AST_FMC_CS3_BASE
	case 3:
		spi->regs = (void *)AST_FMC_BASE + 0x1c;
		spi->buff = (void *)AST_FMC_CS3_BASE;
		break;
#endif

	default:
		return NULL;
	}

	//SPI slave support dual and quad, read_cmd will be determine in sf_flash.c (spi_flash_scan)
#if defined(CONFIG_ARCH_AST2500) || defined(CONFIG_ARCH_AST2400)
	spi->slave.mode |= SPI_RX_DUAL;
	spi->slave.mode |= SPI_TX_DUAL;
	spi->slave.mode &= ~SPI_RX_QUAD;//For 2500 not support quad
	spi->slave.mode &= ~SPI_TX_QUAD;//For 2500 not support quad
#endif

#if defined(CONFIG_ARCH_AST2400)
	spi->slave.mode |= SPI_RX_QUAD;
	//spi->slave.mode |= SPI_TX_QUAD;
#endif

	/* AST2300 limit Max SPI CLK to 50MHz (Datasheet v1.2) */
	spi_ctrl = ast_spi_read(spi, CS0_CTRL);

	//MASK first
	spi_ctrl &= ~SPI_CLK_DIV_MASK;

	div = ast_spi_calculate_divisor(max_hz);
	spi_ctrl |= SPI_CLK_DIV(div);

	//ISSUE : ast spi ctrl couldn't use mode 3, so fix mode 0
	spi_ctrl &= ~SPI_CPOL_1;

	ast_spi_write(spi, spi_ctrl, CS0_CTRL);

	//printf("READ_CMD = %x \n", READ_CMD);

   if(READ_CMD == CMD_READ_DUAL_OUTPUT_FAST){
	   //ulCtrlData |= SPI_CMD_DATA(CMD_READ_DUAL_OUTPUT_FAST);
	   ulCtrlData |= IOMODEx2_dummy;
	   ulCtrlData |= SPI_DUMMY_LOW(1);
	   printf("Enter Dual Read Mode\n");
   }
#if defined(CONFIG_ARCH_AST2400)
   else if(READ_CMD == CMD_READ_QUAD_OUTPUT_FAST){
	   //ulCtrlData |= SPI_CMD_DATA(CMD_READ_QUAD_OUTPUT_FAST);
	   ulCtrlData |= IOMODEx4_dummy;
	   ulCtrlData |= SPI_DUMMY_LOW(2);
	   printf("Enter Quad Read Mode\n");
   }
#endif
   else if(READ_CMD == CMD_READ_DUAL_IO_FAST){
	   //ulCtrlData |= SPI_CMD_DATA(CMD_READ_DUAL_IO_FAST);
	   ulCtrlData |= IOMODEx2_dummy;
	   ulCtrlData |= SPI_DUMMY_LOW(1);//1byte dummy
	   printf("Enter Dual IO Mode\n");
   }
#if defined(CONFIG_ARCH_AST2400)
   else if(READ_CMD == CMD_READ_QUAD_IO_FAST){
	   //ulCtrlData |= SPI_CMD_DATA(CMD_READ_QUAD_IO_FAST);
	   ulCtrlData |= IOMODEx4_dummy;
	   ulCtrlData |= SPI_DUMMY_LOW(2);//2byte dummy
	   printf("Enter Quad IO Mode\n");
   }
#endif
   else{
		ulCtrlData |= IOMODEx1;
		//ulCtrlData |= SPI_DUAL_DATA;
		ulCtrlData |= SPI_CMD_FAST_R_MODE;
		ulCtrlData |= SPI_CMD_DATA(CMD_READ_ARRAY_FAST);
		ulCtrlData |= SPI_DUMMY_LOW(1);
		printf("Enter Fast read Mode\n");
   }

   ast_spi_write(spi, ulCtrlData, CtrlOffset);

   return &spi->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
	free(slave);
}

int spi_claim_bus(struct spi_slave *slave)
{

	struct ast_spi_host *spi = to_ast_spi(slave);
	struct spi_flash *flash;

	SPIDBUG("spi_claim_bus bus %d, cs %d\n",slave->bus, slave->cs);

	ast_spi_write(spi, ast_spi_read(spi, 0x00) | SPI_CMD_USER_MODE, 0x00);
	ast_spi_write(spi, ast_spi_read(spi, 0x00) & ~SPI_CE_HIGH, 0x00);

#if defined(CONFIG_ARCH_AST2400) || defined(CONFIG_ARCH_AST2500)
	//Read hw straping (AST2400)
	if(boot_mode == boot_in_4byte){
		spi_set_4byte(spi);
		flash_set_4byte(spi);
		//printf("AST: release bus with 4byte\n");
	}else{
		spi_set_3byte(spi);
		flash_set_3byte(spi);
		//printf("AST: release bus with 3byte\n");
	}
#endif

return 0;

}

void spi_release_bus(struct spi_slave *slave)
{
	struct ast_spi_host *spi = to_ast_spi(slave);

	SPIDBUG("spi_release_bus >>>>> >>>> \n");

	ast_spi_write(spi, ast_spi_read(spi, 0x00) | SPI_CE_HIGH, 0x00);
	ast_spi_write(spi, ast_spi_read(spi, 0x00) & ~SPI_CMD_USER_MODE, 0x00);
	//Set to fast read mode
	ast_spi_write(spi, SPI_CMD_DATA(0x0b) | SPI_CLK_DIV(7) | SPI_DUMMY_LOW(1) | SPI_CMD_FAST_R_MODE, 0x00);

#if defined(CONFIG_ARCH_AST2400) || defined(CONFIG_ARCH_AST2500)
	//Read hw straping (AST2400)
	if(boot_mode == boot_in_4byte){
		spi_set_4byte(spi);
		flash_set_4byte(spi);
		//printf("AST: release bus with 4byte\n");
	}else{
		spi_set_3byte(spi);
		flash_set_3byte(spi);
		//printf("AST: release bus with 3byte\n");
	}
#endif

}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
	     void *din, unsigned long flags)
{
	struct ast_spi_host *spi = to_ast_spi(slave);
	/* assume spi core configured to do 8 bit transfers */
	uint bytes = bitlen / 8;
	const uchar *txp = dout;
	uchar *rxp = din;

	SPIDBUG("%s: bus:%i cs:%i bitlen:%i bytes:%i flags:%lx\n", __func__,
		slave->bus, slave->cs, bitlen, bytes, flags);

	if (bitlen == 0)
		goto done;

	if (bitlen % 8) {
		flags |= SPI_XFER_END;
		goto done;
	}

   if (flags & SPI_XFER_BEGIN) {
	   SPIDBUG("\n ----------Xfer BEGIN -------\n");
	   ast_spi_write(spi, ast_spi_read(spi, 0x00) | SPI_CMD_USER_MODE, 0x00);
	   ast_spi_write(spi, ast_spi_read(spi, 0x00) & ~SPI_CE_HIGH, 0x00);
   }

   while (bytes--) {
   	uchar d;
   	   if(txp) {
		   d = txp ? *txp++ : 0xff;
		   SPIDBUG("%s: tx:%x \n", __func__, d);
		   __raw_writeb(d, spi->buff);
		   //udelay(10);
   	   }
	   SPIDBUG("\n");
	   if (rxp) {
		   d = __raw_readb(spi->buff);
		   *rxp ++= d;
		   SPIDBUG("rx:%x \n", d);
	   }
   }
   	SPIDBUG("\n");
done:
   if (flags & SPI_XFER_END) {
   		SPIDBUG("SPI_XFER_END --- >\n");
	  ast_spi_write(spi, ast_spi_read(spi, 0x00) | SPI_CE_HIGH, 0x00);
	  ast_spi_write(spi, (ast_spi_read(spi, 0x00) & ~SPI_CMD_USER_MODE) , 0x00);//exit user mode, back to normal read
	  ast_spi_write(spi, SPI_CMD_DATA(0x0b) | SPI_CLK_DIV(7) | SPI_DUMMY_LOW(1) | SPI_CMD_FAST_R_MODE, 0x00);//fast read
   }

   return 0;

}

void spi_set_4byte(struct ast_spi_host *spi)
{
	ulong reg;

#if defined(CONFIG_ARCH_AST2400)
	/* set H/W Trappings, boot with 32bit mode */
	reg = *((volatile ulong*) (AST_SCU_BASE + 0x70));
	reg |= 0x10;
	*((volatile ulong*) (AST_SCU_BASE + 0x70)) = reg;
#endif
	/* enable spi 4byte mode */
	reg  = *((volatile ulong*) (AST_FMC_BASE + CE_CTRL));
	reg |= (0x01 << cs);
	*((volatile ulong*) (AST_FMC_BASE + CE_CTRL)) = reg;
}

void spi_set_3byte(struct ast_spi_host *spi)
{
	ulong reg;
#if defined(CONFIG_ARCH_AST2400)
	/* set H/W Trappings, boot with 24bit mode */
	reg = *((volatile ulong*) (AST_SCU_BASE + 0x70));
	reg &= ~0x10;
	*((volatile ulong*) (AST_SCU_BASE + 0x70)) = reg;
#endif
	/* disable spi 4byte mode */
	reg  = *((volatile ulong*) (AST_FMC_BASE + CE_CTRL));
	reg &= (~0x01 << cs);
	*((volatile ulong*) (AST_FMC_BASE + CE_CTRL)) = reg;
}

void flash_set_4byte(struct ast_spi_host *spi)
{
	spi_flash_cmd(spi, CMD_4BYTE_MODE, NULL, 0);
	//udelay(200);
}

void flash_set_3byte(struct ast_spi_host *spi)
{
	spi_flash_cmd(spi, CMD_3BYTE_MODE, NULL, 0);
	//udelay(200);
}

void memmove_dma(void * dest,const void *src,size_t count)
{
	ulong data;

	/* 4-bytes align */
	if(count % 4)
		count += 4 - (count%4);

      /* force end of burst read */
	*(volatile ulong *) (AST_FMC_BASE + CS0_CTRL) |= CE_HIGH;
	*(volatile ulong *) (AST_FMC_BASE + CS0_CTRL) &= ~CE_HIGH;

	*(ulong *) (AST_FMC_BASE + REG_FLASH_DMA_CONTROL) = (ulong) (~FLASH_DMA_ENABLE);
	*(ulong *) (AST_FMC_BASE + REG_FLASH_DMA_FLASH_BASE) = (ulong) (src);
	*(ulong *) (AST_FMC_BASE + REG_FLASH_DMA_DRAM_BASE) = (ulong) (dest);
	*(ulong *) (AST_FMC_BASE + REG_FLASH_DMA_LENGTH) = (ulong) (count);
	*(ulong *) (AST_FMC_BASE + REG_FLASH_DMA_CONTROL) = (ulong) (FLASH_DMA_ENABLE);

	/* wait poll */
	do {
		udelay(100);
		data = *(ulong *) (AST_FMC_BASE + REG_FLASH_INTERRUPT_STATUS);
	} while (!(data & FLASH_STATUS_DMA_READY));

	*(ulong *) (AST_FMC_BASE + REG_FLASH_DMA_CONTROL) &= ~(FLASH_DMA_ENABLE);
	/* clear status */
	*(ulong *) (AST_FMC_BASE + REG_FLASH_INTERRUPT_STATUS) |= FLASH_STATUS_DMA_CLEAR;
}
