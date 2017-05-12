/*
 * (C) Copyright 2002 Ryan Chen
 * Copyright 2016 IBM Corporation
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <netdev.h>

#include <asm/arch/platform.h>
#include <asm/arch/ast-sdmc.h>
#include <asm/arch/ast_scu.h>
#include <asm/arch/regs-ahbc.h>
#include <asm/arch/regs-scu.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

static void watchdog_init()
{
#ifdef CONFIG_ASPEED_ENABLE_WATCHDOG
#define AST_WDT_BASE 0x1e785000
#define AST_WDT_CLK (1*1000*1000) /* 1M clock source */
  u32 reload = AST_WDT_CLK * CONFIG_ASPEED_WATCHDOG_TIMEOUT;
  /* set the reload value */
  __raw_writel(reload, AST_WDT_BASE + 0x04);
  /* magic word to reload */
  __raw_writel(0x4755, AST_WDT_BASE + 0x08);
  /* start the watchdog with 1M clk src and reset whole chip */
  __raw_writel(0x33, AST_WDT_BASE + 0x0c);
  printf("Watchdog: %us\n", CONFIG_ASPEED_WATCHDOG_TIMEOUT);
#endif
}

int board_init(void)
{
	/* adress of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
	gd->flags = 0;
	return 0;
}

int misc_init_r(void)
{
	u32 reg;
	unsigned long duty = 0xffffffff;

	/* Unlock AHB controller */
	writel(AHBC_PROTECT_UNLOCK, AST_AHBC_BASE);

	/* Map DRAM to 0x00000000 */
	reg = readl(AST_AHBC_BASE + AST_AHBC_ADDR_REMAP);
	writel(reg | BIT(0), AST_AHBC_BASE + AST_AHBC_ADDR_REMAP);

	/* Unlock SCU */
	writel(SCU_PROTECT_UNLOCK, AST_SCU_BASE);

        /* Set PWM dytu to 100% */
         reg = *((volatile ulong*) 0x1e6e2088);
         reg |= 0x3e;  // enable PWM1~6 function pin
         *((volatile ulong*) 0x1e6e2088) = reg;
 
        // reset PWM
        reg = *((volatile ulong*) 0x1e6e2004);
        reg &= ~(0x200); /* stop the reset */
        *((volatile ulong*) 0x1e6e2004) = reg;
       // enable clock and and set all tacho/pwm to type M
        *((volatile ulong*) 0x1e786000) = 1;
        *((volatile ulong*) 0x1e786040) = 1;
       /* set clock division and period of type M/N */
       /* 0xFF11 --> 24000000 / (2 * 2 * 256) = 23437.5 Hz */
       *((volatile ulong*) 0x1e786004) = 0xFF11FF11;
       *((volatile ulong*) 0x1e786044) = 0xFF11FF11;
       //PWM0-1
       *((volatile ulong*) 0x1e786008) = duty;
       //PWM2-3
       *((volatile ulong*) 0x1e78600c) = duty;
       //PWM4-5
       *((volatile ulong*) 0x1e786048) = duty;
       //PWM6-7
       *((volatile ulong*) 0x1e78604C) = duty;
 
       *((volatile ulong*) 0x1e786010) = 0x10000001;
       *((volatile ulong*) 0x1e786018) = 0x10000001;
       *((volatile ulong*) 0x1e786014) = 0x10000000;
       *((volatile ulong*) 0x1e78601c) = 0x10000000;
       *((volatile ulong*) 0x1e786020) = 0;
       *((volatile ulong*) 0x1e786000) = 0xf01;
       *((volatile ulong*) 0x1e786040) = 0xf01;

	/*
	 * The original file contained these comments.
	 * TODO: verify the register write does what it claims
	 *
	 * LHCLK = HPLL/8
	 * PCLK  = HPLL/8
	 * BHCLK = HPLL/8
	 */
	reg = readl(AST_SCU_BASE + AST_SCU_CLK_SEL);
	reg &= 0x1c0fffff;
	reg |= 0x61800000;
	writel(reg, AST_SCU_BASE + AST_SCU_CLK_SEL);

        watchdog_init();
	return 0;
}

int dram_init(void)
{
	u32 vga = ast_scu_get_vga_memsize();
	u32 dram = ast_sdmc_get_mem_size();
	gd->ram_size = dram - vga;

	return 0;
}

#ifdef CONFIG_FTGMAC100
int board_eth_init(bd_t *bd)
{
	return ftgmac100_initialize(bd);
}
#endif

#ifdef CONFIG_ASPEEDNIC
int board_eth_init(bd_t *bd)
{
	return aspeednic_initialize(bd);
}
#endif
