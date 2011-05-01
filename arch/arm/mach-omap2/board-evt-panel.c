
/*
 * Copyright (C) 2011 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-zoom2.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/leds-omap-display.h>
#include <linux/leds.h>
#include <linux/spi/spi.h>

#include <plat/display.h>
#include <plat/mcspi.h>
#include <plat/omap-pm.h>
#include <plat/board.h>

#include "mux.h"


#define LCD_PANEL_ENABLE_GPIO		36
#define LCD_CABC0_GPIO                  44
#define LCD_CABC1_GPIO                  45
#define LCD_BACKLIGHT_EN_EVT2           47

#define RED_MASK        0x00FF0000
#define GREEN_MASK      0x0000FF00
#define BLUE_MASK       0x000000FF
#define RED_SHIFT       16
#define GREEN_SHIFT     8
#define BLUE_SHIFT      0
#define MAX_COLOR_DEPTH 255

#define DEFAULT_BACKLIGHT_BRIGHTNESS    75


extern unsigned get_last_off_on_transaction_id(struct device *dev);

/*---backlight--------------------------------------------------------------------*/
static void boxer_backlight_set_power(struct omap_pwm_led_platform_data *self, int on_off)
{
	gpio_direction_output(LCD_BACKLIGHT_EN_EVT2, !on_off);
	gpio_set_value(LCD_BACKLIGHT_EN_EVT2, !on_off);
}

static struct omap_pwm_led_platform_data boxer_backlight_data = {
	.name = "lcd-backlight",
	.intensity_timer = 8,
	.def_on = 0,   // PWM high == backlight OFF, PWM low == backlight ON
	.def_brightness = DEFAULT_BACKLIGHT_BRIGHTNESS,
	.set_power = boxer_backlight_set_power,
};

static struct platform_device boxer_backlight_led_device = {
	.name		= "omap_pwm_led",
	.id		= -1,
	.dev		= {
		.platform_data = &boxer_backlight_data,
	},
};

static void __init boxer_backlight_init(void)
{
        printk("Enabling backlight PWM for LCD\n");
        boxer_backlight_data.def_on = 1; // change the PWM polarity

        gpio_request(LCD_BACKLIGHT_EN_EVT2, "lcd backlight evt2");

        gpio_request(LCD_CABC0_GPIO, "lcd CABC0");
        gpio_direction_output(LCD_CABC0_GPIO,0);
        gpio_set_value(LCD_CABC0_GPIO,0);

        gpio_request(LCD_CABC1_GPIO, "lcd CABC1");
        gpio_direction_output(LCD_CABC1_GPIO,0);
        gpio_set_value(LCD_CABC1_GPIO,0);
}


/*--------------------------------------------------------------------------*/
#define RED_MASK 	0x00FF0000
#define GREEN_MASK 	0x0000FF00
#define BLUE_MASK	0x000000FF
#define RED_SHIFT	16
#define GREEN_SHIFT	8
#define BLUE_SHIFT	0
#define MAX_COLOR_DEPTH	255

u8 color_component_correction[256]=
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

static int evt_clut_fill(void * ptr, u32 size)
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
		temp = (((red << RED_SHIFT) & RED_MASK) | ((green << GREEN_SHIFT) & GREEN_MASK) | ((blue << BLUE_SHIFT) & BLUE_MASK)); 
		*byte++ = temp;
	}
	/* Roll back the pointer*/
	byte = (u32 *)ptr;

	for (count = 0; count < size / sizeof(u32); count++) {
		blue = *byte & BLUE_MASK;
		color_corrected_value = color_component_correction[count]+blue;
		color_corrected_value = (color_corrected_value >= MAX_COLOR_DEPTH) ? MAX_COLOR_DEPTH:color_corrected_value; 
		temp = ((*byte & (~BLUE_MASK)) | (color_corrected_value & BLUE_MASK));
		*byte++ = temp;
	}
	return 0;
}

/*--------------------------------------------------------------------------*/
static struct omap_dss_device evt_lcd_device = {
	.name = "lcd",
	.driver_name = "boxer_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
//	.clut_size = sizeof(u32) * 256,
//	.clut_fill = evt_clut_fill,
};

static struct omap_dss_device *evt_dss_devices[] = {
	&evt_lcd_device,
};

static struct omap_dss_board_info evt_dss_data = {
	.get_last_off_on_transaction_id = get_last_off_on_transaction_id,
	.num_devices = ARRAY_SIZE(evt_dss_devices),
	.devices = evt_dss_devices,
	.default_device = &evt_lcd_device,
};

static struct platform_device evt_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &evt_dss_data,
	},
};

/*--------------------------------------------------------------------------*/
static struct omap2_mcspi_device_config evt_lcd_mcspi_config = {
	.turbo_mode             = 0,
	.single_channel         = 1,  /* 0: slave, 1: master */

};

struct spi_board_info evt_spi_board_info[] __initdata = {
	[0] = {
		.modalias               = "boxer_disp_spi",
		.bus_num                = 4,
		.chip_select            = 0,
		.max_speed_hz           = 375000,
		.controller_data        = &evt_lcd_mcspi_config,
	},
};

/*--------------------------------------------------------------------------*/
static struct platform_device *evt_panel_devices[] __initdata = {
	&evt_dss_device,
	&boxer_backlight_led_device,
};

void __init evt_lcd_panel_init(void)
{
        boxer_backlight_init();

	spi_register_board_info(evt_spi_board_info,
				ARRAY_SIZE(evt_spi_board_info));

	platform_add_devices(evt_panel_devices, ARRAY_SIZE(evt_panel_devices));
}


