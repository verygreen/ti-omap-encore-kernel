
/*
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 * Vikram Pandita <vikram.pandita@ti.com>
 *
 * Modified from mach-omap2/board-ldp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/switch.h>
#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <mach/gpio.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/gpmc.h>
#include <plat/usb.h>
#include <plat/mux.h>

#include <asm/system.h> // For system_serial_high & system_serial_low
#include <asm/io.h>
#include <asm/delay.h>
#include <plat/control.h>
#include <plat/sram.h>

#include <plat/display.h>
#include <plat/omap-serial.h>

#include <plat/opp_twl_tps.h>

#include <linux/usb/android_composite.h>

#include <plat/system.h>

#include "mux.h"
#include "omap3-opp.h"
#include "prcm-common.h"

#include "sdram-hynix-h8mbx00u0mer-0em.h"

#include <media/v4l2-int-device.h>

#ifndef CONFIG_TWL4030_CORE
#error "no power companion board defined!"
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#include <linux/bootmem.h>
#endif

static int encore_twl4030_keymap[] = {
	KEY(0, 0, KEY_HOME),
	KEY(0, 1, KEY_VOLUMEUP),
	KEY(0, 2, KEY_VOLUMEDOWN),
	0
};

static struct matrix_keymap_data encore_twl4030_keymap_data = {
	.keymap			= encore_twl4030_keymap,
	.keymap_size	= ARRAY_SIZE(encore_twl4030_keymap),
};

static struct twl4030_keypad_data encore_kp_twl4030_data = {
	.rows			= 8,
	.cols			= 8,
	.keymap_data		= &encore_twl4030_keymap_data,
	.rep			= 1,
};

// HOME key code for HW > EVT2A
static struct gpio_keys_button encore_gpio_buttons[] = {
	{
		.code			= KEY_POWER,
		.gpio			= 14,
		.desc			= "POWER",
		.active_low		= 0,
		.wakeup			= 1,
	},
	{
		.code			= KEY_HOME,
		.gpio			= 48,
		.desc			= "HOME",
		.active_low		= 1,
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data encore_gpio_key_info = {
	.buttons	= encore_gpio_buttons,
	.nbuttons	= ARRAY_SIZE(encore_gpio_buttons),
//	.rep		= 1,		/* auto-repeat */
};

static struct platform_device encore_keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &encore_gpio_key_info,
	},
};

#define t2_out(c, r, v) twl_i2c_write_u8(c, r, v)

/* Use address that is most likely unused and untouched by u-boot */
#define ENCORE_RAM_CONSOLE_START 0x8e000000
#define ENCORE_RAM_CONSOLE_SIZE (0x20000)

static struct resource encore_ram_console_resource[] = { 
    {
        .start  = ENCORE_RAM_CONSOLE_START,
        .end    = ENCORE_RAM_CONSOLE_START + ENCORE_RAM_CONSOLE_SIZE - 1,
        .flags  = IORESOURCE_MEM,
    }
};

static struct platform_device encore_ram_console_device = {
    .name           = "ram_console",
    .id             = 0,
    .num_resources  = ARRAY_SIZE(encore_ram_console_resource),
    .resource       = encore_ram_console_resource,
};

static struct platform_device *encore_devices[] __initdata = {
	&encore_keys_gpio,
};

static void __init omap_encore_init_irq(void)
{
	omap2_init_common_hw(h8mbx00u0mer0em_sdrc_params,
			     h8mbx00u0mer0em_sdrc_params);
	omap_init_irq();
}

static struct twl4030_usb_data encore_usb_data = {
      .usb_mode	= T2_USB_MODE_ULPI,
#ifdef CONFIG_REGULATOR_MAXIM_CHARGER
      .bci_supply     = &bq24073_vcharge_supply,
#endif
};

#if 0
static struct twl4030_gpio_platform_data encore_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= encore_twl_gpio_setup,
};
#endif

static struct twl4030_madc_platform_data encore_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_ins sleep_on_seq[] = {

	/* Turn off HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_OFF), 2},
	/* Turn OFF VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_OFF), 2},
	/* Turn OFF VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_OFF), 2},
	/* Turn OFF VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_OFF), 2},
	/* Turn OFF REGEN */
    /* test stability without REGEN {MSG_SINGULAR(DEV_GRP_P1, 0x15, RES_STATE_OFF), 2}, */
};

static struct twl4030_script sleep_on_script = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

static struct twl4030_ins wakeup_p12_seq[] = {
	/* Turn on HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	/* Turn ON VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_ACTIVE), 2},
	/* Turn ON VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_ACTIVE), 2},
	/* Turn ON VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_ACTIVE), 2},
    /* Turn ON REGEN */
    /* test stability, regen never turned off    {MSG_SINGULAR(DEV_GRP_P1, 0x15, RES_STATE_ACTIVE), 2}, */
};

static struct twl4030_script wakeup_p12_script = {
	.script = wakeup_p12_seq,
	.size   = ARRAY_SIZE(wakeup_p12_seq),
	.flags  = TWL4030_WAKEUP12_SCRIPT,
};

static struct twl4030_ins wakeup_p3_seq[] = {
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p3_script = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

static struct twl4030_ins wrst_seq[] = {
/*
 * Reset twl4030.
 * Reset VMMC1 regulator.
 * Reset VDD1 regulator.
 * Reset VDD2 regulator.
 * Reset VPLL1 regulator.
 * Enable sysclk output.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, 0x5, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_WRST), 0x60},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wrst_script = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] = {
	&sleep_on_script,
	&wakeup_p12_script,
	&wakeup_p3_script,
	&wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] = {
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3, .type = -1,
		.type2 = -1 },
	{ .resource = RES_VDD1, .devgroup = DEV_GRP_P1, .type = -1,
		.type2 = -1 },
	{ .resource = RES_VDD2, .devgroup = DEV_GRP_P1, .type = -1,
		.type2 = -1 },
	{ 0, 0},
};

static struct twl4030_power_data encore_t2scripts_data = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

static struct twl4030_platform_data __refdata encore_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &encore_madc_data,
	.usb		= &encore_usb_data,
//	.gpio		= &encore_gpio_data,
	.keypad		= &encore_kp_twl4030_data,
//	.power		= &encore_t2scripts_data,
};


static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.flags		= 0
	}
};

#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.vendor = "B&N     ",
	.product = "Ebook Disk      ",
	.release = 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &mass_storage_pdata,
		},
};

// Reserved for serial number passed in from the bootloader.
static char adb_serial_number[32] = "";

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
	"usb_mass_storage",
	"adb",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= ENCORE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= ENCORE_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= ENCORE_RNDIS_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= ENCORE_RNDIS_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= ENCORE_VENDOR_ID,
	.product_id	= ENCORE_PRODUCT_ID,
	.manufacturer_name = "B&N",
	.product_name	= "NookColor",
	.serial_number	= "11223344556677",
	.num_products   = ARRAY_SIZE(usb_products),
	.products	= usb_products,
	.num_functions	= ARRAY_SIZE(usb_functions_all),
	.functions	= usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name		= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
        { .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux       NULL
#endif

#ifdef CONFIG_PM
#ifdef CONFIG_TWL4030_CORE
static struct omap_volt_pmic_info omap_pmic_mpu = { /* and iva */
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x0, /* (vdd0) VDD1 -> VDD1_CORE -> VDD_MPU */
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x14,
	.vp_vlimitto_vddmax = 0x44,
};

static struct omap_volt_pmic_info omap_pmic_core = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x1, /* (vdd1) VDD2 -> VDD2_CORE -> VDD_CORE */
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x18,
	.vp_vlimitto_vddmax = 0x42,
};
#endif /* CONFIG_TWL4030_CORE */
#endif /* CONFIG_PM */

static struct omap_board_config_kernel encore_config[] __initdata = {
};

static void __init omap_encore_init(void)
{
	platform_device_register(&encore_ram_console_device);

	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap_serial_init(omap_serial_platform_data);
	usb_musb_init(&musb_board_data);

	omap_board_config = encore_config;
	omap_board_config_size = ARRAY_SIZE(encore_config);

#ifdef CONFIG_PM
#ifdef CONFIG_TWL4030_CORE
        omap_voltage_register_pmic(&omap_pmic_core, "core");
        omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
#endif
#endif


#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
	platform_device_register(&usb_mass_storage_device);
	// Set the device serial number passed in from the bootloader.
	if (system_serial_high != 0 || system_serial_low != 0) {
		snprintf(adb_serial_number, sizeof(adb_serial_number), "%08x%08x", system_serial_high, system_serial_low);
		adb_serial_number[16] = '\0';
		android_usb_pdata.serial_number = adb_serial_number;
	}
	platform_device_register(&android_usb_device);
#endif
        BUG_ON(!cpu_is_omap3630());
}

static void __init omap_encore_map_io(void)
{
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	reserve_bootmem(ENCORE_RAM_CONSOLE_START, ENCORE_RAM_CONSOLE_SIZE, 0);
#endif /* CONFIG_ANDROID_RAM_CONSOLE */
	omap2_set_globals_343x();
	omap34xx_map_common_io();
}
 
MACHINE_START(ENCORE, "encore")
	/* phys_io is only used for DEBUG_LL early printing.  The Boxer's
	 * console is on an external quad UART sitting at address 0x10000000
	 */
	.phys_io	= ENCORE_UART_BASE,
	.io_pg_offst	= ((ENCORE_UART_VIRT) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_encore_map_io,
	.init_irq	= omap_encore_init_irq,
	.init_machine	= omap_encore_init,
	.timer		= &omap_timer,
MACHINE_END
