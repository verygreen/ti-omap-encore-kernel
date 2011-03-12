
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

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_I2C
#include <linux/cyttsp.h>
#endif

#ifdef CONFIG_INPUT_KXTF9
#include <linux/kxtf9.h>
#endif /* CONFIG_INPUT_KXTF9 */

#ifdef CONFIG_BATTERY_MAX17042
#include <linux/max17042.h>
#endif

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
#include <linux/mmc/host.h>

#include <plat/board-encore.h>
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
#include <plat/mmc.h>
#include <plat/omap-serial.h>

#include <plat/opp_twl_tps.h>

#include <plat/system.h>

#include "mux.h"
#include "hsmmc.h"
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

static void __init omap_encore_init_irq(void)
{
	omap2_init_common_hw(h8mbx00u0mer0em_sdrc_params,
			     h8mbx00u0mer0em_sdrc_params);
	omap_init_irq();
}

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
	gpio_set_debounce(KXTF9_GPIO_FOR_IRQ, 0);
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

        .data_odr_init          = ODR12_5,
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

#ifdef CONFIG_CHARGER_MAX8903
static struct platform_device max8903_charger_device = {
	.name		= "max8903_charger",
	.id		= -1,
};
#endif

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
        gpio_set_debounce(MAX17042_GPIO_FOR_IRQ, 0);        
        printk("max17042 GPIO pin read %d\n", gpio_get_value(MAX17042_GPIO_FOR_IRQ));
}

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

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_I2C

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
		gpio_set_debounce(OMAP_CYTTSP_GPIO, 0);
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

#endif

static struct twl4030_usb_data encore_usb_data = {
      .usb_mode	= T2_USB_MODE_ULPI,
};

static struct regulator_consumer_supply encore_vmmc1_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply encore_vsim_supply = {
	.supply		= "vmmc_aux",
};

static struct regulator_consumer_supply encore_vmmc2_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply encore_vdda_dac_supply = {
	.supply		= "vdda_dac",
};

static struct regulator_consumer_supply encore_vdds_dsi_supply = {
	.supply		= "vdds_dsi",
};


/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data encore_vmmc1 = {
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
	.consumer_supplies      = &encore_vmmc1_supply,
};

/* VMMC2 for MMC2 card */
static struct regulator_init_data encore_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 1850000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &encore_vmmc2_supply,
};

/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data encore_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &encore_vsim_supply,
};


static struct regulator_init_data encore_vdac = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &encore_vdda_dac_supply,
};

static struct regulator_init_data encore_vdsi = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &encore_vdds_dsi_supply,
};


static struct omap2_hsmmc_info mmc[] __initdata = {
	{
		.name		= "external",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.power_saving	= true,
	},

	{
		.name		= "internal",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
		.power_saving	= true,
	},
	{
		.name		= "internal",
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
		.power_saving	= true,
	},
	{}      /* Terminator */
};

static int encore_hsmmc_card_detect(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	/* Encore board EVT2 and later has pin high when card is present) */
	return gpio_get_value_cansleep(mmc->slots[0].switch_pin);
}

static int encore_twl4030_hsmmc_late_init(struct device *dev)
{
        int ret = 0;
        struct platform_device *pdev = container_of(dev,
                                struct platform_device, dev);
        struct omap_mmc_platform_data *pdata = dev->platform_data;

	if(is_encore_board_evt2()) {
		/* Setting MMC1 (external) Card detect */
		if (pdev->id == 0) {
			pdata->slots[0].card_detect = encore_hsmmc_card_detect;
		}
	}
        return ret;
}

static __init void encore_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev)
		return;

	pdata = dev->platform_data;
	pdata->init = encore_twl4030_hsmmc_late_init;
}

static int __ref encore_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	struct omap2_hsmmc_info *c;
	/* gpio + 0 is "mmc0_cd" (input/IRQ),
	 * gpio + 1 is "mmc1_cd" (input/IRQ)
	 */
printk("******IN boxer_twl_gpio_setup********\n");
	mmc[0].gpio_cd = gpio + 0;
	mmc[1].gpio_cd = gpio + 1;
	omap2_hsmmc_init(mmc);
	for (c = mmc; c->mmc; c++)
                encore_hsmmc_set_late_init(c->dev);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	encore_vmmc1_supply.dev = mmc[0].dev;
	encore_vsim_supply.dev = mmc[0].dev;
	encore_vmmc2_supply.dev = mmc[1].dev;

	return 0;
}

static struct twl4030_gpio_platform_data encore_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= encore_twl_gpio_setup,
};

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
	.gpio		= &encore_gpio_data,
	.keypad		= &encore_kp_twl4030_data,
	.vmmc1		= &encore_vmmc1,
	.vmmc2		= &encore_vmmc2,
	.vsim		= &encore_vsim,
	.vdac		= &encore_vdac,
	.vpll2		= &encore_vdsi,
//	.power		= &encore_t2scripts_data,
};

static struct i2c_board_info __initdata encore_i2c_bus1_info[] = {
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
		.platform_data = &encore_twldata,
	},
#ifdef CONFIG_INPUT_KXTF9
	{
		I2C_BOARD_INFO(KXTF9_DEVICE_ID, KXTF9_I2C_SLAVE_ADDRESS),
		.platform_data = &kxtf9_platform_data_here,
		.irq = OMAP_GPIO_IRQ(KXTF9_GPIO_FOR_IRQ),
	},
#endif /* CONFIG_INPUT_KXTF9 */
#ifdef CONFIG_MAX9635
    {
        I2C_BOARD_INFO(MAX9635_NAME, MAX9635_I2C_SLAVE_ADDRESS),
        .platform_data = &max9635_platform_data,
        .irq = OMAP_GPIO_IRQ(MAX9635_GPIO_FOR_IRQ),
    },
#endif /* CONFIG_MAX9635 */
};

static struct i2c_board_info __initdata encore_i2c_bus2_info[] = {
#ifdef CONFIG_TOUCHSCREEN_PIXCIR_I2C 
	{
		I2C_BOARD_INFO(PIXCIR_I2C_S32_NAME, PIXCIR_I2C_S32_SLAVEADDRESS),
		.platform_data = &pixcir_platform_data,
		.irq = OMAP_GPIO_IRQ(PIXCIR_I2C_S32_GPIO),
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_CYTTSP_I2C
	{
        I2C_BOARD_INFO(CY_I2C_NAME, CYTTSP_I2C_SLAVEADDRESS),
        .platform_data = &cyttsp_platform_data,
        .irq = OMAP_GPIO_IRQ(OMAP_CYTTSP_GPIO),
	},
#endif

#if defined(CONFIG_SND_SOC_DAC3100) || defined(CONFIG_SND_SOC_DAC3100_MODULE)  || defined (CONFIG_SND_OMAP_SOC_OMAP3_EDP)
	{
		I2C_BOARD_INFO(AIC3100_NAME,  AIC3100_I2CSLAVEADDRESS),
                .irq = OMAP_GPIO_IRQ(AUDIO_CODEC_IRQ_GPIO),
	},
#endif
};


static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_PERIPHERAL,
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

#define LCD_EN_GPIO                     36
#define LCD_BACKLIGHT_GPIO              58
#define LCD_BACKLIGHT_EN_EVT2           47

#define LCD_CABC0_GPIO					44
#define LCD_CABC1_GPIO					45

// Panel on completion to signal when panel is on.
static DECLARE_COMPLETION(panel_on);

static int encore_panel_enable_lcd(struct omap_dss_device *dssdev)
{
printk("encore panel enable\n");
	complete_all(&panel_on);	
	//omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT,166 * 1000 * 4);
	return 0;
}

static void encore_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	INIT_COMPLETION(panel_on);
	/* disabling LCD on Boxer EVT1 shuts down SPI bus, so don't touch! */
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);
}

/* EVT1 is connected to full 24 bit display panel so gamma table correction */
/* is only to compensate for LED backlight color */
/*
 * omap2dss - fixed size: 256 elements each four bytes / XRGB
 */
#define RED_MASK 	0x00FF0000
#define GREEN_MASK 	0x0000FF00
#define BLUE_MASK	0x000000FF
#define RED_SHIFT	16
#define GREEN_SHIFT	8
#define BLUE_SHIFT	0
#define MAX_COLOR_DEPTH	255

static const u8 led_color_correction_samsung[256]=
{
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,
30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,
25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
15,15,15,15,15,15,15,10,10,10,10,10,10,9,8,7,
6,5,4,4,2,2,2,2,2,2,1,1,1,1,0,0,
};

static const u8 led_color_correction_nichia[256]=
{
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,
30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,
25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
15,15,15,15,15,15,15,10,10,10,10,10,10,9,8,7,
6,5,4,4,2,2,2,2,2,2,1,1,1,1,0,0,
};

/* Use the Samsung table as the default */
static const u8 *color_component_correction = led_color_correction_samsung;

/* Select the appropriate gamma table based on the LED backlight attached */
static int __init get_backlighttype(char *str)
{
	if (!strcmp(str, "1476AY")) {
		printk("LED backlight type set to Nichia\n");
		color_component_correction = led_color_correction_nichia;
	}
	/* Samsung will have the value 1577AS, but since Samsung is the */
	/* default, nothing needs to be done here */
	return 1;
}

__setup("backlighttype=", get_backlighttype);

static int encore_clut_fill(void * ptr, u32 size)
{
	u16 count;
	u32 temp;
	u32 *byte = (u32 *)ptr;
	u16 color_corrected_value;
	u8 red, green, blue;
	for (count = 0; count < size / sizeof(u32); count++) {
	  red   = count;
	  green = count;
	  blue  = count;
	  color_corrected_value = color_component_correction[count]+blue;
	  color_corrected_value = (color_corrected_value >= MAX_COLOR_DEPTH) ? MAX_COLOR_DEPTH:color_corrected_value; 
	  temp = (((red << RED_SHIFT) & RED_MASK) | ((green << GREEN_SHIFT) & GREEN_MASK) | ((color_corrected_value << BLUE_SHIFT) & BLUE_MASK));
	  *byte++ = temp;
	}
	return 0;
}

static struct omap_dss_device encore_lcd_device = {
	.name = "lcd",
	.driver_name = "boxer_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
	.platform_enable	= encore_panel_enable_lcd,
	.platform_disable	= encore_panel_disable_lcd,
//	.clut_size		= sizeof(u32) * 256,
//	.clut_fill		= encore_clut_fill,
 };

#if 0
static struct omap_dss_device encore_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_COMPOSITE,
	.platform_enable = encore_panel_enable_tv,
	.platform_disable = encore_panel_disable_tv,
};
#endif

static struct omap_dss_device *encore_dss_devices[] = {
	&encore_lcd_device,
//	&encore_tv_device,
};

static struct omap_dss_board_info encore_dss_data = {
	.num_devices = ARRAY_SIZE(encore_dss_devices),
	.devices = encore_dss_devices,
	.default_device = &encore_lcd_device,
};

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

static struct omap2_mcspi_device_config encore_lcd_mcspi_config = {
	.turbo_mode		= 0,
	.single_channel		= 1,
};

static struct spi_board_info encore_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "boxer_disp_spi",
		.bus_num		= 4,	/* McSPI4 */
		.chip_select		= 0,
		.max_speed_hz		= 375000,
		.controller_data	= &encore_lcd_mcspi_config,
	},
};

static void encore_backlight_set_power(struct omap_pwm_led_platform_data *self, int on_off)
{
	if (on_off) {
		// Wait for panel to turn on.
//		wait_for_completion_interruptible(&panel_on);

		gpio_direction_output(LCD_BACKLIGHT_EN_EVT2, 0);
		gpio_set_value(LCD_BACKLIGHT_EN_EVT2, 0);
	} else {
		gpio_direction_output(LCD_BACKLIGHT_EN_EVT2, 1);
		gpio_set_value(LCD_BACKLIGHT_EN_EVT2, 1);
	}
}

#define DEFAULT_BACKLIGHT_BRIGHTNESS 105

static struct omap_pwm_led_platform_data encore_backlight_data = {
	.name = "lcd-backlight",
	.intensity_timer = 8,
	.def_on = 0,
	.def_brightness = DEFAULT_BACKLIGHT_BRIGHTNESS,
	.blink_timer = 0,
	.set_power = encore_backlight_set_power, 
};

static struct platform_device encore_backlight_led_device = {
	.name		= "omap_pwm_led",
	.id		= -1,
	.dev		= {
		.platform_data = &encore_backlight_data,
	},
};


static struct omap_board_config_kernel encore_config[] __initdata = {
};

static void encore_backlight_init(void)
{
	printk("Enabling backlight PWM for LCD\n");

	encore_backlight_data.def_on = 1; // change the PWM polarity
        gpio_request(LCD_BACKLIGHT_EN_EVT2, "lcd backlight evt2");

//	omap_cfg_reg(N8_34XX_GPIO58_PWM);
	omap_mux_init_gpio(58, OMAP_PIN_OUTPUT);

	gpio_request(LCD_CABC0_GPIO, "lcd CABC0");
	gpio_direction_output(LCD_CABC0_GPIO,0);
	gpio_set_value(LCD_CABC0_GPIO,0);

	gpio_request(LCD_CABC1_GPIO, "lcd CABC1");
	gpio_direction_output(LCD_CABC1_GPIO,0);
	gpio_set_value(LCD_CABC1_GPIO,0);
}

void __init encore_display_init(void)
{

	spi_register_board_info(encore_spi_board_info,
			ARRAY_SIZE(encore_spi_board_info));

	if (platform_device_register(&encore_backlight_led_device) < 0)
		printk(KERN_ERR "Unable to register OMAP-VOUT device\n");

	omap_display_init(&encore_dss_data);
	encore_backlight_init();
	encore_backlight_set_power(&encore_backlight_data, 1);
}

static int __init omap_i2c_init(void)
{
    int i2c1_devices;
    printk("***********IN omap_i2c_init***********\n");

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

	i2c1_devices = ARRAY_SIZE(encore_i2c_bus1_info);

#ifdef CONFIG_MAX9635
	// right now evt2 is not stuffed with the max9635 light sensor due to i2c conflict 
	// tbd if it will be reworked on specific units
	--i2c1_devices;
#endif
    printk("****omap_i2c_init(): Number of devices on bus1: %i\n", i2c1_devices);
	omap_register_i2c_bus(1, 100, NULL, encore_i2c_bus1_info,
			i2c1_devices);
	omap_register_i2c_bus(2, 400, NULL, encore_i2c_bus2_info,
			ARRAY_SIZE(encore_i2c_bus2_info));
	return 0;
}

static struct platform_device *encore_devices[] __initdata = {
	&encore_ram_console_device,
	&boxer_lcd_touch_regulator_device,
	&encore_keys_gpio,
#ifdef CONFIG_CHARGER_MAX8903
	&max8903_charger_device,
#endif
};

static void __init omap_encore_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap_i2c_init();

	platform_add_devices(encore_devices, ARRAY_SIZE(encore_devices));

	omap_board_config = encore_config;
	omap_board_config_size = ARRAY_SIZE(encore_config);

	omap_serial_init(omap_serial_platform_data);
	usb_musb_init(&musb_board_data);

	encore_display_init();
#ifdef CONFIG_PM
#ifdef CONFIG_TWL4030_CORE
        omap_voltage_register_pmic(&omap_pmic_core, "core");
        omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
#endif
#endif

#ifdef CONFIG_INPUT_KXTF9
	kxtf9_dev_init();
#endif /* CONFIG_INPUT_KXTF9 */

#ifdef CONFIG_BATTERY_MAX17042
	max17042_dev_init();
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
