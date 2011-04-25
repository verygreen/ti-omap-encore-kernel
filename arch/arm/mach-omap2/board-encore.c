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
#include <linux/cyttsp.h>

#ifdef CONFIG_INPUT_KXTF9
#include <linux/kxtf9.h>
#define KXTF9_DEVICE_ID			"kxtf9"
#define KXTF9_I2C_SLAVE_ADDRESS		0x0F
#define KXTF9_GPIO_FOR_PWR		34
#define	KXTF9_GPIO_FOR_IRQ		113
#endif /* CONFIG_INPUT_KXTF9 */

#include <linux/max17042.h>

#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/switch.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board-boxer.h>
#include <plat/mcspi.h>
#include <mach/gpio.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/gpmc.h>
#include <plat/usb.h>
#include <plat/mux.h>
#include "mux.h"

#include <asm/system.h> // For system_serial_high & system_serial_low
#include <asm/io.h>
#include <asm/delay.h>
#include <plat/control.h>
#include <plat/sram.h>

#include <plat/display.h>

#include <linux/usb/android_composite.h>

#include "twl4030.h"
#include "mmc-twl4030.h"
#include "omap3-opp.h"
#include "prcm-common.h"

#include "sdram-hynix-h8mbx00u0mer-0em.h"

#include <media/v4l2-int-device.h>

#ifdef CONFIG_PM
#include <media/videobuf-core.h>
#include <media/v4l2-device.h>
#include <plat/vrfb.h>
#include <../drivers/media/video/omap/omap_voutdef.h>
#endif

#ifndef CONFIG_TWL4030_CORE
#error "no power companion board defined!"
#endif

#ifdef CONFIG_WL127X_RFKILL
#include <linux/wl127x-rfkill.h>
#endif

#ifdef CONFIG_BT_WILINK
#include <linux/ti_wilink_st.h>
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#include <linux/bootmem.h>
#endif

#define DEFAULT_BACKLIGHT_BRIGHTNESS 105

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_I2C
/* tma340 i2c address */
#define CYTTSP_I2C_SLAVEADDRESS	34
#define OMAP_CYTTSP_GPIO	99
#define OMAP_CYTTSP_RESET_GPIO	46
#endif
#define LCD_EN_GPIO		36

#define TWL4030_MSECURE_GPIO	22

#define WL127X_BTEN_GPIO	60

#define BOXER_EXT_QUART_PHYS	0x48000000
#define BOXER_EXT_QUART_VIRT	0xfa000000
#define BOXER_EXT_QUART_SIZE	SZ_256

#define ENCORE_WL1271_NSHUTDOWN_GPIO	60

#ifdef CONFIG_WL127X_RFKILL
static struct wl127x_rfkill_platform_data encore_wl127x_pdata = {
	.bt_nshutdown_gpio = ENCORE_WL1271_NSHUTDOWN_GPIO,	/* UART_GPIO (spare) Enable GPIO */
	.fm_enable_gpio = -1,		/* FM Enable GPIO */
};

static struct platform_device encore_wl127x_device = {
	.name           = "wl127x-rfkill",
	.id             = -1,
	.dev.platform_data = &encore_wl127x_pdata,
};

#endif

#ifdef CONFIG_TI_ST
/* wl128x BT, FM, GPS connectivity chip */
struct ti_st_plat_data wilink_pdata = {
        .nshutdown_gpio = 60,
        .dev_name = "/dev/ttyO1",
        .flow_cntrl = 1,
        .baud_rate = 115200 // was 3000000,
};

static struct platform_device kim_wl127x_device = {
        .name           = "kim",
        .id             = -1,
        .dev.platform_data = &wilink_pdata,
};

#endif
#ifdef CONFIG_BT_WILINK

static struct platform_device btwilink_device = {
       .name = "btwilink",
       .id = -1,
};

#endif


static int boxer_twl4030_keymap[] = {
	KEY(0, 0, KEY_HOME),
	KEY(0, 1, KEY_VOLUMEUP),
	KEY(0, 2, KEY_VOLUMEDOWN),
	0
};

static struct matrix_keymap_data boxer_twl4030_keymap_data = {
	.keymap			= boxer_twl4030_keymap,
	.keymap_size	= ARRAY_SIZE(boxer_twl4030_keymap),
};

static struct twl4030_keypad_data boxer_kp_twl4030_data = {
	.rows			= 8,
	.cols			= 8,
	.keymap_data	= &boxer_twl4030_keymap_data,
	.rep			= 1,
};

// HOME key code for HW > EVT2A
static struct gpio_keys_button boxer_gpio_buttons[] = {
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

static struct gpio_keys_platform_data boxer_gpio_key_info = {
	.buttons	= boxer_gpio_buttons,
	.nbuttons	= ARRAY_SIZE(boxer_gpio_buttons),
//	.rep		= 1,		/* auto-repeat */
};

static struct platform_device boxer_keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &boxer_gpio_key_info,
	},
};

#ifdef CONFIG_CHARGER_MAX8903
static struct platform_device max8903_charger_device = {
	.name		= "max8903_charger",
	.id		= -1,
};
#endif

static struct regulator_consumer_supply boxer_vlcdtp_supply[] = {
    { .supply = "vlcd" },
    { .supply = "vtp" }, 
};

static struct regulator_init_data boxer_vlcdtp = {
    .supply_regulator_dev = NULL,
    .constraints = {
        .min_uV = 3300000,
        .max_uV = 3300000,
        .valid_modes_mask = REGULATOR_MODE_NORMAL,
        .valid_ops_mask = REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies = 2,
    .consumer_supplies = boxer_vlcdtp_supply,
};

static struct fixed_voltage_config boxer_lcd_touch_regulator_data = {
    .supply_name = "vdd_lcdtp",
    .microvolts = 3300000,
    .gpio = LCD_EN_GPIO,
    .enable_high = 1,
    .enabled_at_boot = 0,
    .init_data = &boxer_vlcdtp,
};

static struct platform_device boxer_lcd_touch_regulator_device = {
    .name   = "reg-fixed-voltage",
    .id     = -1,
    .dev    = {
        .platform_data = &boxer_lcd_touch_regulator_data,
    },
};

/* Use address that is most likely unused and untouched by u-boot */
#define BOXER_RAM_CONSOLE_START 0x8e000000
#define BOXER_RAM_CONSOLE_SIZE (0x20000)

static struct resource boxer_ram_console_resource[] = { 
    {
        .start  = BOXER_RAM_CONSOLE_START,
        .end    = BOXER_RAM_CONSOLE_START + BOXER_RAM_CONSOLE_SIZE - 1,
        .flags  = IORESOURCE_MEM,
    }
};

static struct platform_device boxer_ram_console_device = {
    .name           = "ram_console",
    .id             = 0,
    .num_resources  = ARRAY_SIZE(boxer_ram_console_resource),
    .resource       = boxer_ram_console_resource,
};

static struct platform_device *boxer_devices[] __initdata = {
#ifdef CONFIG_ANDROID_RAM_CONSOLE
    &boxer_ram_console_device,
#endif
    &boxer_lcd_touch_regulator_device,    
	&boxer_keys_gpio,
#ifdef CONFIG_WL127X_RFKILL
//	&encore_wl127x_device,
#endif
#ifdef CONFIG_CHARGER_MAX8903
	&max8903_charger_device,
#endif
};

static void __init omap_encore_init_irq(void)
{
	omap_init_irq();
	omap2_init_common_hw(	h8mbx00u0mer0em_sdrc_params , NULL,
				omap3630_mpu_rate_table,
				omap3630_dsp_rate_table,
				omap3630_l3_rate_table);
}

static struct regulator_consumer_supply boxer_vmmc1_supply = {
	.supply		= "vmmc",
};

/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data boxer_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &boxer_vmmc1_supply,
};

static struct twl4030_hsmmc_info mmc[] __initdata = {
	{
		.name		= "external",
		.mmc		= 1,
		.wires		= 4,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.power_saving	= 1,
	},
	{
		.name		= "internal",
		.mmc		= 2,
		.wires		= 8,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.nonremovable	= 1,
		.power_saving	= 1,
		.ocr_mask	= MMC_VDD_165_195,
	},
	{
		.mmc		= 3,
		.wires		= 4,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{}      /* Terminator */
};

static int __ref boxer_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ),
	 * gpio + 1 is "mmc1_cd" (input/IRQ)
	 */
	mmc[0].gpio_cd = gpio + 0;
	mmc[1].gpio_cd = gpio + 1;
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	boxer_vmmc1_supply.dev = mmc[0].dev;

	return 0;
}

static struct omap_lcd_config boxer_lcd_config __initdata = {
        .ctrl_name      = "internal",
};

static struct omap_uart_config boxer_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3)),
};

static struct omap_board_config_kernel boxer_config[] __initdata = {
	{ OMAP_TAG_UART,	&boxer_uart_config },
        { OMAP_TAG_LCD,         &boxer_lcd_config },
};

static struct twl4030_usb_data boxer_usb_data = {
      .usb_mode	= T2_USB_MODE_ULPI,
};
static struct twl4030_gpio_platform_data boxer_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= boxer_twl_gpio_setup,
};

static struct twl4030_madc_platform_data boxer_madc_data = {
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

static struct twl4030_power_data boxer_t2scripts_data = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

static struct twl4030_platform_data __refdata boxer_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &boxer_madc_data,
	.usb		= &boxer_usb_data,
	.gpio		= &boxer_gpio_data,
	.keypad		= &boxer_kp_twl4030_data,
	.power		= &boxer_t2scripts_data,
	.vmmc1          = &boxer_vmmc1,
};


int  cyttsp_dev_init(int resource) 
{
	if (resource)
	{
		if (gpio_request(OMAP_CYTTSP_RESET_GPIO, "tma340_reset") < 0) {
			printk(KERN_ERR "can't get tma340 xreset GPIO\n");
			return -1;
		}

		if (gpio_request(OMAP_CYTTSP_GPIO, "cyttsp_touch") < 0) {
			printk(KERN_ERR "can't get cyttsp interrupt GPIO\n");
			return -1;
		}

		gpio_direction_input(OMAP_CYTTSP_GPIO);
		omap_set_gpio_debounce(OMAP_CYTTSP_GPIO, 0);
	}
	else
	{
		gpio_free(OMAP_CYTTSP_GPIO);
		gpio_free(OMAP_CYTTSP_RESET_GPIO);
	}
    return 0;
}

static struct cyttsp_platform_data cyttsp_platform_data = {
	.maxx = 600,
	.maxy = 1024,
	.flags = 0,
	.gen = CY_GEN3,
	.use_st = CY_USE_ST,
	.use_mt = CY_USE_MT,
	.use_hndshk = CY_SEND_HNDSHK,
	.use_trk_id = CY_USE_TRACKING_ID,
	.use_sleep = CY_USE_SLEEP,
	.use_gestures = CY_USE_GESTURES,
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CY_GEST_GRP1 | CY_GEST_GRP2 |
		CY_GEST_GRP3 | CY_GEST_GRP4 |
		CY_ACT_DIST,
	/* change act_intrvl to customize the Active power state 
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state 
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
};


#if 0
// This code is yanked from arch/arm/mach-omap2/prcm.c
void machine_emergency_restart(void)
{
	s16 prcm_offs;
	u32 l;

	// delay here to allow eMMC to finish any internal housekeeping before reset
	// even if mdelay fails to work correctly, 8 second button press should work 
	// this used to be an msleep but scheduler is gone here and calling msleep
	// will cause a panic
	mdelay(1600);

	prcm_offs = OMAP3430_GR_MOD;
	l = ('B' << 24) | ('M' << 16) | 'h';
	/* Reserve the first word in scratchpad for communicating
	* with the boot ROM. A pointer to a data structure
	* describing the boot process can be stored there,
	* cf. OMAP34xx TRM, Initialization / Software Booting
	* Configuration. */
	omap_writel(l, OMAP343X_SCRATCHPAD + 4);
//	omap3_configure_core_dpll_warmreset();
}
#endif

#ifdef CONFIG_INPUT_KXTF9
/* KIONIX KXTF9 Digital Tri-axis Accelerometer */

static void kxtf9_dev_init(void)
{
	printk("board-encore.c: kxtf9_dev_init ...\n");

//	if (gpio_request(KXTF9_GPIO_FOR_PWR, "kxtf9_pwr") < 0) {
//		printk(KERN_ERR "+++++++++++++ Can't get GPIO for kxtf9 power\n");
//		return;
//	}
        // Roberto's comment: G-sensor is powered by VIO and does not need to be powered enabled
	//gpio_direction_output(KXTF9_GPIO_FOR_PWR, 1);
	
	if (gpio_request(KXTF9_GPIO_FOR_IRQ, "kxtf9_irq") < 0) {
		printk(KERN_ERR "Can't get GPIO for kxtf9 IRQ\n");
		return;
	}

	printk("board-encore.c: kxtf9_dev_init > Init kxtf9 irq pin %d !\n", KXTF9_GPIO_FOR_IRQ);
	gpio_direction_input(KXTF9_GPIO_FOR_IRQ);
	omap_set_gpio_debounce(KXTF9_GPIO_FOR_IRQ, 0);
}

struct kxtf9_platform_data kxtf9_platform_data_here = {
        .min_interval   = 1,
        .poll_interval  = 1000,

        .g_range        = KXTF9_G_8G,
        .shift_adj      = SHIFT_ADJ_2G,

		// Map the axes from the sensor to the device.
		
		//. SETTINGS FOR THE EVT1A TEST RIG:
        .axis_map_x     = 1,
        .axis_map_y     = 0,
        .axis_map_z     = 2,
        .negate_x       = 1,
        .negate_y       = 0,
        .negate_z       = 0,
		
		//. SETTINGS FOR THE ENCORE PRODUCT:
        //. .axis_map_x     = 1,
        //. .axis_map_y     = 0,
        //. .axis_map_z     = 2,
        //. .negate_x       = 1,
        //. .negate_y       = 0,
        //. .negate_z       = 0,

        .data_odr_init          = ODR12_5F,
        .ctrl_reg1_init         = KXTF9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
        .int_ctrl_init          = KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
        .int_ctrl_init          = KXTF9_IEN,
        .tilt_timer_init        = 0x03,
        .engine_odr_init        = OTP12_5 | OWUF50 | OTDT400,
        .wuf_timer_init         = 0x16,
        .wuf_thresh_init        = 0x28,
        .tdt_timer_init         = 0x78,
        .tdt_h_thresh_init      = 0xFF,
        .tdt_l_thresh_init      = 0x14,
        .tdt_tap_timer_init     = 0x53,
        .tdt_total_timer_init   = 0x24,
        .tdt_latency_timer_init = 0x10,
        .tdt_window_timer_init  = 0xA0,

        .gpio = KXTF9_GPIO_FOR_IRQ,
};
#endif	/* CONFIG_INPUT_KXTF9 */

#ifdef CONFIG_BATTERY_MAX17042
static void max17042_dev_init(void)
{
        printk("board-encore.c: max17042_dev_init ...\n");

        if (gpio_request(MAX17042_GPIO_FOR_IRQ, "max17042_irq") < 0) {
                printk(KERN_ERR "Can't get GPIO for max17042 IRQ\n");
                return;
        }

        printk("board-encore.c: max17042_dev_init > Init max17042 irq pin %d !\n", MAX17042_GPIO_FOR_IRQ);
        gpio_direction_input(MAX17042_GPIO_FOR_IRQ);
        omap_set_gpio_debounce(MAX17042_GPIO_FOR_IRQ, 0);        
        printk("max17042 GPIO pin read %d\n", gpio_get_value(MAX17042_GPIO_FOR_IRQ));
}
#endif

#ifdef CONFIG_BATTERY_MAX17042
struct max17042_platform_data max17042_platform_data_here = {

	//fill in device specific data here
	//load stored parameters from Rom Tokens? 
	//.val_FullCAP =
	//.val_Cycles =
	//.val_FullCAPNom =
	//.val_SOCempty =
	//.val_Iavg_empty =
	//.val_RCOMP0 =
	//.val_TempCo=
	//.val_k_empty0 =
	//.val_dQacc =
	//.val_dPacc =
	
        .gpio = MAX17042_GPIO_FOR_IRQ,
};
#endif

static struct i2c_board_info __initdata boxer_i2c_bus1_info[] = {
#ifdef CONFIG_BATTERY_MAX17042
	{
		I2C_BOARD_INFO(MAX17042_DEVICE_ID, MAX17042_I2C_SLAVE_ADDRESS),
		.platform_data = &max17042_platform_data_here,
		.irq = OMAP_GPIO_IRQ(MAX17042_GPIO_FOR_IRQ),
	},
#endif	/*CONFIG_BATTERY_MAX17042*/	
	{
		I2C_BOARD_INFO("tps65921", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &boxer_twldata,
	},
#ifdef CONFIG_INPUT_KXTF9
	{
		I2C_BOARD_INFO(KXTF9_DEVICE_ID, KXTF9_I2C_SLAVE_ADDRESS),
		.platform_data = &kxtf9_platform_data_here,
		.irq = OMAP_GPIO_IRQ(KXTF9_GPIO_FOR_IRQ),
	},
#endif /* CONFIG_INPUT_KXTF9 */
};

#if defined(CONFIG_SND_SOC_TLV320DAC3100) || defined(CONFIG_SND_SOC_TLV320DAC3100_MODULE)
#define AUDIO_CODEC_POWER_ENABLE_GPIO    103
#define AUDIO_CODEC_RESET_GPIO           37
#define AUDIO_CODEC_IRQ_GPIO             59
#define AIC3100_NAME			"tlv320dac3100"
#define AIC3100_I2CSLAVEADDRESS		0x18

static void audio_dac_3100_dev_init(void)
{
        printk("board-encore.c: audio_dac_3100_dev_init ...\n");
        if (gpio_request(AUDIO_CODEC_RESET_GPIO, "AUDIO_CODEC_RESET_GPIO") < 0) {
                printk(KERN_ERR "can't get AUDIO_CODEC_RESET_GPIO \n");
                return;
        }

        printk("board-encore.c: audio_dac_3100_dev_init > set AUDIO_CODEC_RESET_GPIO to output Low!\n");
        gpio_direction_output(AUDIO_CODEC_RESET_GPIO, 0);
	gpio_set_value(AUDIO_CODEC_RESET_GPIO, 0);

        printk("board-encore.c: audio_dac_3100_dev_init ...\n");
        if (gpio_request(AUDIO_CODEC_POWER_ENABLE_GPIO, "AUDIO DAC3100 POWER ENABLE") < 0) {
                printk(KERN_ERR "can't get AUDIO_CODEC_POWER_ENABLE_GPIO \n");
                return;
        }

        printk("board-encore.c: audio_dac_3100_dev_init > set AUDIO_CODEC_POWER_ENABLE_GPIO to output and value high!\n");
        gpio_direction_output(AUDIO_CODEC_POWER_ENABLE_GPIO, 0);
	gpio_set_value(AUDIO_CODEC_POWER_ENABLE_GPIO, 1);

	/* 1 msec delay needed after PLL power-up */
        mdelay (1);

        printk("board-encore.c: audio_dac_3100_dev_init > set AUDIO_CODEC_RESET_GPIO to output and value high!\n");
	gpio_set_value(AUDIO_CODEC_RESET_GPIO, 1);

}
#endif
static struct i2c_board_info __initdata boxer_i2c_bus2_info[] = {
	{
        I2C_BOARD_INFO(CY_I2C_NAME, CYTTSP_I2C_SLAVEADDRESS),
        .platform_data = &cyttsp_platform_data,
        .irq = OMAP_GPIO_IRQ(OMAP_CYTTSP_GPIO),
	},

#if defined(CONFIG_SND_SOC_TLV320DAC3100) || defined(CONFIG_SND_SOC_TLV320DAC310_MODULE)
	{
		I2C_BOARD_INFO(AIC3100_NAME,  AIC3100_I2CSLAVEADDRESS),
                .irq = OMAP_GPIO_IRQ(AUDIO_CODEC_IRQ_GPIO),
	},
#endif
};


#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE)
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.vendor = "B&N     ",
	.product = "Ebook Disk      ",
	.release = 0x0101,
	.nluns = 2
};

static struct platform_device usb_mass_storage_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &mass_storage_pdata,
		},
};
#endif
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode			= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 100,
};

static int __init omap_i2c_init(void)
{
	/* Disable OMAP 3630 internal pull-ups for I2Ci */
	if (cpu_is_omap3630()) {

		u32 prog_io;

		prog_io = omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO1);
		/* Program (bit 19)=1 to disable internal pull-up on I2C1 */
		prog_io |= OMAP3630_PRG_I2C1_PULLUPRESX;
		/* Program (bit 0)=1 to disable internal pull-up on I2C2 */
		prog_io |= OMAP3630_PRG_I2C2_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP343X_CONTROL_PROG_IO1);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO_WKUP1);
		/* Program (bit 5)=1 to disable internal pull-up on I2C4(SR) */
		prog_io |= OMAP3630_PRG_SR_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO_WKUP1);
	}

	omap_register_i2c_bus(1, 100, NULL, boxer_i2c_bus1_info,
			ARRAY_SIZE(boxer_i2c_bus1_info));
	omap_register_i2c_bus(2, 400, NULL, boxer_i2c_bus2_info,
			ARRAY_SIZE(boxer_i2c_bus2_info));
	return 0;
}

#if 0
static int __init wl127x_vio_leakage_fix(void)
{
	int ret = 0;

	ret = gpio_request(WL127X_BTEN_GPIO, "wl127x_bten");
	if (ret < 0) {
		printk(KERN_ERR "wl127x_bten gpio_%d request fail",
						WL127X_BTEN_GPIO);
		goto fail;
	}

	gpio_direction_output(WL127X_BTEN_GPIO, 1);
	mdelay(10);
	gpio_direction_output(WL127X_BTEN_GPIO, 0);
	udelay(64);

	gpio_free(WL127X_BTEN_GPIO);
fail:
	return ret;
}
#endif
#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux       NULL
#endif

extern void evt_lcd_panel_init(void);

static void __init omap_encore_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	twl4030_get_scripts(&boxer_t2scripts_data);
	omap_i2c_init();
	/* Fix to prevent VIO leakage on wl127x */
//	wl127x_vio_leakage_fix();

	platform_add_devices(boxer_devices, ARRAY_SIZE(boxer_devices));

	omap_board_config = boxer_config;
	omap_board_config_size = ARRAY_SIZE(boxer_config);

	omap_serial_init();
	evt_lcd_panel_init();
	usb_musb_init(&musb_board_data);

#ifdef CONFIG_INPUT_KXTF9
	kxtf9_dev_init();
#endif /* CONFIG_INPUT_KXTF9 */

#ifdef CONFIG_BATTERY_MAX17042
	max17042_dev_init();
#endif

#if defined(CONFIG_SND_SOC_TLV320DAC3100) || defined(CONFIG_SND_SOC_TLV320DAC310_MODULE)
        audio_dac_3100_dev_init();
#endif

#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE)
	platform_device_register(&usb_mass_storage_device);
#endif
#endif

#ifdef CONFIG_TI_ST
	printk("encore: registering wl127x device.\n");
        platform_device_register(&kim_wl127x_device);
#endif
#ifdef CONFIG_BT_WILINK
	printk("encore: registering btwilink device.\n");
        platform_device_register(&btwilink_device);
#endif
        BUG_ON(!cpu_is_omap3630());
}

static void __init omap_encore_map_io(void)
{
#ifdef CONFIG_ANDROID_RAM_CONSOLE
    reserve_bootmem(BOXER_RAM_CONSOLE_START, BOXER_RAM_CONSOLE_SIZE, 0);
#endif /* CONFIG_ANDROID_RAM_CONSOLE */
	omap2_set_globals_343x();
	omap2_map_common_io();
}
 
MACHINE_START(ENCORE, "encore")
	/* phys_io is only used for DEBUG_LL early printing.  The Boxer's
	 * console is on an external quad UART sitting at address 0x10000000
	 */
	.phys_io	= BOXER_EXT_QUART_PHYS,
	.io_pg_offst	= ((BOXER_EXT_QUART_VIRT) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_encore_map_io,
	.init_irq	= omap_encore_init_irq,
	.init_machine	= omap_encore_init,
	.timer		= &omap_timer,
MACHINE_END
