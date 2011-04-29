/*
 * linux/sound/soc/codecs/tlv320dac3100.c
 *
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 *
 * Based on sound/soc/codecs/wm8753.c by Liam Girdwood
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * Rev 0.1   ASoC driver support   dac3100         21-Apr-2010
 *          static int dac3100_power_down(struct snd_soc_codec *codec)10
 *
 *			 The AIC3252 ASoC driver is ported for the codec dac3100.
 *
 * Rev 0.2   Mistral Codec driver cleanup 	   27-Jun-2010
 *
 * Rev 0.3   Updated the SUSPEND-RESUME fix from the tlv320aic3111.c into this
 *           code-base. 			   12-Jul-2010
 *
 * Rev 0.4   Updated the STANDBY and ON Code to switch OFF/ON the
 *           DAC/Headphone/Speaker Drivers.        21-Jul-2010
 *
 * Rev 0.5   Updated with the calls to the dac3100_parse_biquad_array()
 *           and dac3100_update_biquad_array()	   30-Jul-2010
 *
 * Rev 0.6   Updated the dac3100_set_bias_level() to get rid of POP sounds
 *                                                 13-Sep-2010
 *
 * Rev 0.7   First round of migration to 2.6.32 Linux Kernel
 *                                                  9-Sep-2010
 * Rev 0.8   Updated with a new version of the dac3100_mute() which
 *           switches off the Digital Domain during Audio Pause condition.
 *           This is a better way to control Audio rather than playing with
 *           the Analog Blocks.			    21-Oct-2010
 * Rev 0.9   Updated the DACL and DACR outputs to
 *           -1.5db from -6db based on TI recommendations     22-Oct-2010
 *
 * Rev 1.0   Ported the code to 2.6.32 Kernel       01-Dec-2010
 *
 * Rev 1.1   Function dac3100_power_up and dac3100_power_down() functions added
 *           added by TI                            24-Dec-2010
 *
 * Rev 1.2   dac3100_power_down() function updated for the Headphone Power-down
 *                                                  27-Dec-2010
 *
 * Rev 1.3  dac3100_mute_codec() has been updated to poll for the HPL/HPR Registers
 *          with time-outs                          14-Jan-2011
 */

/***************************** INCLUDES ************************************/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h> /* Mistral: Added for mdelay */
#include <linux/interrupt.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>

#include "tlv320dac3100.h"
#include <mach/gpio.h>

#include <linux/i2c/twl.h>
#include <linux/clk.h>
#include <plat/clock.h>

/* Flag to control Switch OFF and ON of Headphone Drivers */


//#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

#define NUM_INIT_REGS (sizeof(dac3100_reg_init) /       \
                       sizeof(struct dac3100_configs))

extern struct clk *gpt11_fclk;

/*
*****************************************************************************
* Macros
*****************************************************************************
*/
#define AIC_FORCE_SWITCHES_ON


#ifdef CONFIG_ADAPTIVE_FILTER
extern void dac3100_add_biquads_controls (struct snd_soc_codec *codec);

extern int dac3100_add_EQ_mixer_controls (struct snd_soc_codec *codec);

extern int dac3100_parse_biquad_array (struct snd_soc_codec *codec);

extern int dac3100_update_biquad_array (struct snd_soc_codec *codec, int speaker_active, int playback_active);
#endif

#define SOC_SINGLE_dac3100(xname)                                       \
        {                                                               \
                .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,     \
                        .info = __new_control_info, .get = __new_control_get, \
                        .put = __new_control_put,                       \
                        .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,      \
                        }

#define SOC_DOUBLE_R_dac3100(xname, reg_left, reg_right, shift, mask, invert) \
        {	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),   \
                        .info = snd_soc_info_volsw_2r_dac3100,          \
                        .get = snd_soc_get_volsw_2r_dac3100, .put = snd_soc_put_volsw_2r_dac3100, \
                        .private_value = (reg_left) | ((shift) << 8)  | \
                        ((mask) << 12) | ((invert) << 20) | ((reg_right) << 24) }

#define    AUDIO_CODEC_HPH_DETECT_GPIO		(157)
#define    AUDIO_CODEC_PWR_ON_GPIO		(103)
#define    AUDIO_CODEC_RESET_GPIO		(37)
#define    AUDIO_CODEC_PWR_ON_GPIO_NAME		"audio_codec_pwron"
#define    AUDIO_CODEC_RESET_GPIO_NAME		"audio_codec_reset"

#define snd_soc_codec_get_drvdata(codec) (codec->private_data)

/*
*****************************************************************************
* Function Prototype
*****************************************************************************
*/
static int dac3100_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params, struct snd_soc_dai *tmp);

static int dac3100_mute(struct snd_soc_dai *dai, int mute);

static int dac3100_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir);

static int dac3100_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt);

static int dac3100_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level level);

unsigned int dac3100_read(struct snd_soc_codec *codec, unsigned int reg);

static int dac3100_hw_free (struct snd_pcm_substream *substream, struct snd_soc_dai *device);

static int __new_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo);

static int __new_control_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);

static int __new_control_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);

static int snd_soc_info_volsw_2r_dac3100(struct snd_kcontrol *kcontrol,
                                         struct snd_ctl_elem_info *uinfo);

static int snd_soc_get_volsw_2r_dac3100(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

static int snd_soc_put_volsw_2r_dac3100(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

static int dac3100_headset_speaker_path(struct snd_soc_codec *codec);
static irqreturn_t dac3100_irq_handler(int irq, void *data);

static int dac3100_power_down(struct snd_soc_codec *codec);
static int dac3100_power_up(struct snd_soc_codec *codec);
static int dac3100_mute_codec (struct snd_soc_codec *codec, int mute);
/*
*****************************************************************************
* Global Variable
*****************************************************************************
*/
static u8 dac3100_reg_ctl;

/* whenever aplay/arecord is run, dac3100_hw_params() function gets called.
 * This function reprograms the clock dividers etc. this flag can be used to
 * disable this when the clock dividers are programmed by pps config file
 */
static int soc_static_freq_config = 1;

/* Codec Private Struct variable */
struct dac3100_priv dac3100_codec_data;

/* snd_soc_codec pointer variable shared between I2C and Codec Probe routines */
static struct snd_soc_codec *dac3100_codec;

/*
*****************************************************************************
* Structure Declaration
*****************************************************************************
*/
static struct snd_soc_device *dac3100_socdev;

/* Global Variables introduced to reduce Headphone Analog Volume Control Registers at run-time */
struct i2c_msg i2c_right_transaction[120];
struct i2c_msg i2c_left_transaction[120];

/*
*****************************************************************************
* soc_enum array Structure Initialization
*****************************************************************************
*/
static const char *dac_mute_control[] = {"UNMUTE" , "MUTE"};
static const char *hpdriver_voltage_control[] = {"1.35V", "1.5V", "1.65V", "1.8V"};
static const char *drc_status_control[] = {"DISABLED", "ENABLED"};

static const struct soc_enum dac3100_dapm_enum[] = {
	SOC_ENUM_SINGLE (DAC_MUTE_CTRL_REG, 3, 2, dac_mute_control),
	SOC_ENUM_SINGLE (DAC_MUTE_CTRL_REG, 2, 2, dac_mute_control),
	SOC_ENUM_SINGLE (HPHONE_DRIVERS, 3, 4, hpdriver_voltage_control),
        SOC_ENUM_DOUBLE (DRC_CTRL_1, 6, 5, 2,  drc_status_control),
};
/*
*****************************************************************************
* snd_kcontrol_new Structure Initialization
*****************************************************************************
*/
static const struct snd_kcontrol_new dac3100_snd_controls[] = {
	/* Output */
	/* sound new kcontrol for PCM Playback volume control */
	SOC_DOUBLE_R_dac3100("DAC Playback Volume", LDAC_VOL, RDAC_VOL, 0, 0xAf,
			     0),
	/* sound new kcontrol for HP driver gain */
	SOC_DOUBLE_R_dac3100("HP Driver Gain", HPL_DRIVER, HPR_DRIVER, 0, 0x23, 0),
	/* sound new kcontrol for LO driver gain */
	SOC_DOUBLE_R_dac3100("LO Driver Gain", SPL_DRIVER, SPR_DRIVER, 0, 0x23, 0),
	/* sound new kcontrol for HP mute */
	SOC_DOUBLE_R("HP DAC Playback Switch", HPL_DRIVER, HPR_DRIVER, 2,
		     0x01, 1),
	/* sound new kcontrol for LO mute */
	SOC_DOUBLE_R("LO DAC Playback Switch", SPL_DRIVER, SPR_DRIVER, 2,
		     0x01, 1),

	/* sound new kcontrol for Analog Volume Control for headphone and Speaker Outputs
         * Please refer to Table 5-24 of the Codec DataSheet
         */
        SOC_DOUBLE_R_dac3100("HP Analog Volume",   LEFT_ANALOG_HPL, RIGHT_ANALOG_HPR, 0, 0x7F, 1),
        SOC_DOUBLE_R_dac3100("SPKR Analog Volume", LEFT_ANALOG_SPL, RIGHT_ANALOG_SPR, 0, 0x7F, 1),

	/* sound new kcontrol for Programming the registers from user space */
	SOC_SINGLE_dac3100("Program Registers"),

        /* Enumerations SOCs Controls */
        SOC_ENUM ("LEFT  DAC MUTE", dac3100_dapm_enum[LEFT_DAC_MUTE_ENUM]),
        SOC_ENUM ("RIGHT DAC MUTE", dac3100_dapm_enum[RIGHT_DAC_MUTE_ENUM]),
        SOC_ENUM ("HP Driver Voltage level", dac3100_dapm_enum[HP_DRIVER_VOLTAGE_ENUM]),
        SOC_ENUM ("DRC Status", dac3100_dapm_enum[DRC_STATUS_ENUM]),

        /* Dynamic Range Compression Control */
        SOC_SINGLE ("DRC Hysteresis Value (0=0db 3=db)", DRC_CTRL_1, 0, 0x03, 0),
        SOC_SINGLE ("DRC Threshold Value (0=-3db,7=-24db)",  DRC_CTRL_1, 2, 0x07, 0),
        SOC_SINGLE ("DRC Hold Time",   DRC_CTRL_2, 3, 0x0F, 0),
        SOC_SINGLE ("DRC Attack Time", DRC_CTRL_3, 4, 0x0F, 0),
        SOC_SINGLE ("DRC Delay Rate",  DRC_CTRL_3, 0, 0x0F, 0),



};

/* the sturcture contains the different values for mclk */
static const struct dac3100_rate_divs dac3100_divs[] = {
/*
 * mclk, rate, p_val, pll_j, pll_d, dosr, ndac, mdac, aosr, nadc, madc, blck_N,
 * codec_speficic_initializations
 */
	/* 8k rate */
	// DDenchev (MMS)
	{12000000, 8000, 1, 7, 1680, 128, 2, 42, 4,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 1}}},
	{13000000, 8000, 1, 6, 3803, 128, 3, 27, 4,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 4}}},
	{24000000, 8000, 2, 7, 6800, 768, 15, 1, 24,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/* 11.025k rate */
	// DDenchev (MMS)
	{12000000, 11025, 1, 7, 560, 128, 5, 12, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 11025, 1, 6, 1876, 128, 3, 19, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},
	{24000000, 11025, 2, 7, 5264, 512, 16, 1, 16,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/* 12k rate */
	// DDenchev (MMS)
	{12000000, 12000, 1, 7, 1680, 128, 2, 28, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 12000, 1, 6, 3803, 128, 3, 18, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},

	/* 16k rate */
	// DDenchev (MMS)
	{12000000, 16000, 1, 7, 1680, 128, 2, 21, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 16000, 1, 6, 6166, 128, 3, 14, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},
	{24000000, 16000, 2, 7, 6800, 384, 15, 1, 12,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/* 22.05k rate */
	// DDenchev (MMS)
	{12000000, 22050, 1, 7, 560, 128, 5, 6, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 22050, 1, 6, 5132, 128, 3, 10, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},
	{24000000, 22050, 2, 7, 5264, 256, 16, 1, 8,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/* 24k rate */
	// DDenchev (MMS)
	{12000000, 24000, 1, 7, 1680, 128, 2, 14, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 24000, 1, 6, 3803, 128, 3, 9, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},

	/* 32k rate */
	// DDenchev (MMS)
	{12000000, 32000, 1, 6, 1440, 128, 2, 9, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 32000, 1, 6, 6166, 128, 3, 7, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},
	{24000000, 32000, 2, 7, 1680, 192, 7, 2, 6,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/* 44.1k rate */
	// DDenchev (MMS)
	{12000000, 44100, 1, 7, 560, 128, 5, 3, 4,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 1}}},
	{13000000, 44100, 1, 6, 5132, 128, 3, 5, 4,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 4}}},
	{24000000, 44100, 2, 7, 5264, 128, 8, 2, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/* 48k rate */
	// DDenchev (MMS)
	{12000000, 48000, 1, 7, 1680, 128, 2, 7, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 48000, 1, 6, 6166, 128, 7, 2, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},
	{24000000, 48000, 2, 8, 1920, 128, 8, 2, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/*96k rate : GT 21/12/2009: NOT MODIFIED */
	{12000000, 96000, 1, 8, 1920, 64, 2, 8, 2,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 7}}},
	{13000000, 96000, 1, 6, 6166, 64, 7, 2, 2,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 10}}},
	{24000000, 96000, 2, 8, 1920, 64, 4, 4, 2,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 7}}},

	/*192k : GT 21/12/2009: NOT MODIFIED */
	{12000000, 192000, 1, 8, 1920, 32, 2, 8, 1,
	 {{DAC_INSTRUCTION_SET, 17}, {61, 13}}},
	{13000000, 192000, 1, 6, 6166, 32, 7, 2, 1,
	 {{DAC_INSTRUCTION_SET, 17}, {61, 13}}},
	{24000000, 192000, 2, 8, 1920, 32, 4, 4, 1,
	 {{DAC_INSTRUCTION_SET, 17}, {61, 13}}},
};

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_dai_ops |
 *          It is SoC Codec DAI OPS Structure introduced in 2.6.32 Kernel.
 *----------------------------------------------------------------------------
 */
static struct snd_soc_dai_ops dac3100_dai_ops = {
        .hw_params      = dac3100_hw_params,
        .digital_mute   = dac3100_mute,
        .set_sysclk     = dac3100_set_dai_sysclk,
        .set_fmt        = dac3100_set_dai_fmt,
/*         .hw_free        = dac3100_hw_free,          */
};


/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_dai |
 *          It is SoC Codec DAI structure which has DAI capabilities viz.,
 *          playback and capture, DAI runtime information viz. state of DAI
 *			and pop wait state, and DAI private data.
 *          The dac3100 rates ranges from 8k to 192k
 *          The PCM bit format supported are 16, 20, 24 and 32 bits
 *----------------------------------------------------------------------------
 */
struct snd_soc_dai tlv320dac3100_dai = {
	.name = "TLV320dac3100",
	.playback = {
                .stream_name = "Playback",
                .channels_min = 2,
                .channels_max = 2,
                .rates = dac3100_RATES,
                .formats = dac3100_FORMATS,},
	.ops = &dac3100_dai_ops,
};

EXPORT_SYMBOL_GPL(tlv320dac3100_dai);

#ifdef DEBUG
/*
 *----------------------------------------------------------------------------
 * Function : debug_print_registers
 * Purpose  : Debug routine to dump all the Registers of Page 0
 *
 *----------------------------------------------------------------------------
 */
void debug_print_registers (struct snd_soc_codec *codec)
{
	int i;
	u32 data;

	for (i = 0 ; i < 80 ; i++) {
		data = dac3100_read(codec, i);
		printk(KERN_ALERT "reg = %d val = %x\n", i, data);
	}
}
#endif //DEBUG

/*
*****************************************************************************
* Initializations
*****************************************************************************
*/
/*
 * dac3100 register cache
 * We are caching the registers here.
 * There is no point in caching the reset register.
 * NOTE: In AIC32, there are 127 registers supported in both page0 and page1
 *       The following table contains the page0 and page 1registers values.
 */
static const u8 dac3100_reg[dac3100_CACHEREGNUM] = {
	0x00, 0x00, 0x00, 0x02,	/* 0 */
	0x00, 0x11, 0x04, 0x00,	/* 4 */
	0x00, 0x00, 0x00, 0x01,	/* 8 */
	0x01, 0x00, 0x80, 0x80,	/* 12 */
	0x08, 0x00, 0x01, 0x01,	/* 16 */
	0x80, 0x80, 0x04, 0x00,	/* 20 */
	0x00, 0x00, 0x01, 0x00,	/* 24 */
	0x00, 0x00, 0x01, 0x00,	/* 28 */
	0x00, 0x00, 0x00, 0x00,	/* 32 */
	0x00, 0x00, 0x00, 0x00,	/* 36 */
	0x00, 0x00, 0x00, 0x00,	/* 40 */
	0x00, 0x00, 0x00, 0x00,	/* 44 */
	0x00, 0x00, 0x00, 0x00,	/* 48 */
//	0x00, 0x02, 0x02, 0x00,	/* 52 */
	0x00, 0x00, 0x02, 0x00,	/* 52 */
	0x00, 0x00, 0x00, 0x00,	/* 56 */
	0x01, 0x04, 0x00, 0x14,	/* 60 */
	0x0C, 0x00, 0x00, 0x00,	/* 64 */
	0x0F, 0x38, 0x00, 0x00,	/* 68 */
	0x00, 0x00, 0x00, 0xEE,	/* 72 */
	0x10, 0xD8, 0x7E, 0xE3,	/* 76 */
	0x00, 0x00, 0x80, 0x00,	/* 80 */
	0x00, 0x00, 0x00, 0x00,	/* 84 */
//	0x7F, 0x00, 0x00, 0x00,	/* 88 */
	0x00, 0x00, 0x00, 0x00,	/* 88 */
	0x00, 0x00, 0x00, 0x00,	/* 92 */
	0x00, 0x00, 0x00, 0x00,	/* 96 */
	0x00, 0x00, 0x00, 0x00,	/* 100 */
	0x00, 0x00, 0x00, 0x00,	/* 104 */
	0x00, 0x00, 0x00, 0x00,	/* 108 */
	0x00, 0x00, 0x00, 0x00,	/* 112 */
	0x00, 0x00, 0x00, 0x00,	/* 116 */
	0x00, 0x00, 0x00, 0x00,	/* 120 */
	0x00, 0x00, 0x00, 0x00,	/* 124 - PAGE0 Registers(127) ends here */
	0x00, 0x00, 0x00, 0x00,	/* 128, PAGE1-0 */
	0x00, 0x00, 0x00, 0x00,	/* 132, PAGE1-4 */
	0x00, 0x00, 0x00, 0x00,	/* 136, PAGE1-8 */
	0x00, 0x00, 0x00, 0x00,	/* 140, PAGE1-12 */
	0x00, 0x00, 0x00, 0x00,	/* 144, PAGE1-16 */
	0x00, 0x00, 0x00, 0x00,	/* 148, PAGE1-20 */
	0x00, 0x00, 0x00, 0x00,	/* 152, PAGE1-24 */
	0x00, 0x00, 0x00, 0x04,	/* 156, PAGE1-28 */
	0x06, 0x3E, 0x00, 0x00,	/* 160, PAGE1-32 */
	0x7F, 0x7F, 0x7F, 0x7F,	/* 164, PAGE1-36 */
	0x02, 0x02, 0x00, 0x00,	/* 168, PAGE1-40 */
	0x00, 0x00, 0x00, 0x80,	/* 172, PAGE1-44 */
	0x00, 0x00, 0x00,	/* 176, PAGE1-48 */

};

/*
 * dac3100 initialization data
 * This structure initialization contains the initialization required for
 * dac3100.
 * These registers values (reg_val) are written into the respective dac3100
 * register offset (reg_offset) to  initialize dac3100.
 * These values are used in dac3100_init() function only.
 */
static const struct dac3100_configs dac3100_reg_init[] = {
	/* Carry out the software reset */
//	{RESET, 0x01},
//	{RESET, 0x00},
	/* Connect MIC1_L and MIC1_R to CM */
        //	{MICPGA_CM, 0xC0},
	/* PLL is CODEC_CLKIN */
	{CLK_REG_1, PLLCLK_2_CODEC_CLKIN},
        /* Switch off PLL */
        {CLK_REG_2,  0x00},
        {CLK_REG_3,  0x00},
        {CLK_REG_4,  0x00},
        {CLK_REG_5,  0x00},
        /* Switch off NDAC and MDAC and BCLK_N Dividers */
        {NDAC_CLK_REG_6, 0x00},
        {MDAC_CLK_REG_7,  0x00},
        {DAC_OSR_MSB,     0x00},
        {DAC_OSR_LSB,     0x00},
        /* DAC_MOD_CLK is BCLK source */
	{AIS_REG_3, DAC_MOD_CLK_2_BDIV_CLKIN},

	/* Switch off BCLK_N Divider */
	{CLK_REG_11, 0x00},

	/* Setting up DAC Channel */
	{DAC_CHN_REG, LDAC_2_LCHN | RDAC_2_RCHN /*| SOFT_STEP_2WCLK8*/},
	{DAC_MUTE_CTRL_REG, 0x0C},   	/* Mistral: Updated this value from 0x00 to 0x0C to MUTE DAC Left and Right channels */

        /* Mistral Added following configuration for EQ Setup */
        /* Mistral: Updated the DACR volume also to -6dB */
        {LDAC_VOL, 0xFC}, /* Update DACL volume control from -6db to -1.5db to get speaker and Headphone volume loud enough on Encore device */
        {RDAC_VOL, 0xFC}, /* Update DACR volume control from -6db to -1.5db to get speaker and Headphone volume loud enough on Encore device */
        {VOL_MICDECT_ADC, 0x00}, /* Configure the DAC Volume to default settings */

        /* reg[1][42] should be configured at 0x04. This is already done above */
        /* reg[1][35] should be configured correctly. This is already done above */
        /* reg[1][32] should be powered up, Will be done at run-time */
        /* reg[1][38] should be routed to Class-D Output Driver. This is already done above */


        /* Headphone drivers */
	{HPHONE_DRIVERS, 0x04}, /* 0xCC Mistral: Updated this value from 0xC4. We do not need to Power up HPL and HPR at startup */
	{CLASS_D_SPK, 0x06},            /* Mistral: Updated this value from 0xc6 to 0x06 */
	/* Headphone powerup */
	{HP_OUT_DRIVERS, (BIT7 | HP_POWER_UP_15_3MS 
                      | HP_RAMP_UP_STIME_3_9MS 
                      | HP_BANDGAP_COMMON_MODE)},  /* Mistral modified value to power down DAC after HP and SPK Amplifiers */
        {PGA_RAMP, 0x70},        /* Speaker Ramp up time scaled to 30.5ms */
	/* DAC_L and DAC_R Output Mixer Routing */
	{DAC_MIXER_ROUTING, 0x44},  //DAC_X is routed to the channel mixer amplifier
	/* HPL gain 0db initial */
	{LEFT_ANALOG_HPL, 0x9E},
	/* HPR gain 0db initial */
	{RIGHT_ANALOG_HPR, 0x9E},

	{LEFT_ANALOG_SPL, 0x80},        /* Mistral: Updated Values en plus 0x80 - 0x92 */

        /* HPL mute and gain 0db */
	{HPL_DRIVER, 0x00}, /* Mistral: Updated this value from 0x2. Put HPL to common-mode and MUTED state at startup */
	/* HPR mute and gain 0db */
	{HPR_DRIVER, 0x00}, /* Mistral: Updated this value from 0x2. Put HPR to common-mode and MUTED state at startup */
        {SPL_DRIVER, 0x00},		/* Keep the Speaker driver in MUTE State by Default */

	{HP_DRIVER_CONTROL, 0x00}, /* Configured the default value to 0x00 */
};

/*
 * DAPM Mixer Controls
 */
/* Left DAC_L Mixer */
static const struct snd_kcontrol_new hpl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", DAC_MIXER_ROUTING, 6, 2, 1),
	SOC_DAPM_SINGLE("MIC1_L switch", DAC_MIXER_ROUTING, 5, 1, 0),
//	SOC_DAPM_SINGLE("Left_Bypass switch", HPL_ROUTE_CTL, 1, 1, 0),
};

/* Right DAC_R Mixer */
static const struct snd_kcontrol_new hpr_output_mixer_controls[] = {

	SOC_DAPM_SINGLE("R_DAC switch", DAC_MIXER_ROUTING, 2, 2, 1),
	SOC_DAPM_SINGLE("MIC1_R switch", DAC_MIXER_ROUTING, 1, 1, 0),
//	SOC_DAPM_SINGLE("Right_Bypass switch", HPR_ROUTE_CTL, 1, 1, 0),
};

static const struct snd_kcontrol_new lol_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", DAC_MIXER_ROUTING, 6, 2, 1),    //SOC_DAPM_SINGLE("L_DAC switch", LOL_ROUTE_CTL, 3, 1, 0),
	SOC_DAPM_SINGLE("MIC1_L switch", DAC_MIXER_ROUTING, 5, 1, 0),
//	SOC_DAPM_SINGLE("Left_Bypass switch", HPL_ROUTE_CTL, 1, 1, 0),
};

static const struct snd_kcontrol_new lor_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", DAC_MIXER_ROUTING, 2, 2, 1),    //SOC_DAPM_SINGLE("R_DAC switch", LOR_ROUTE_CTL, 3, 1, 0),
	SOC_DAPM_SINGLE("MIC1_R switch", DAC_MIXER_ROUTING, 1, 1, 0),
//	SOC_DAPM_SINGLE("Right_Bypass switch", LOR_ROUTE_CTL, 1, 1, 0),
};

/* Right DAC_R Mixer */
static const struct snd_kcontrol_new left_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("MIC1_L switch", MICPGA_PIN_CFG, 6, 1, 0),
	SOC_DAPM_SINGLE("MIC1_M switch", MICPGA_PIN_CFG, 2, 1, 0),
};

static const struct snd_kcontrol_new right_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("MIC1_R switch", MICPGA_PIN_CFG, 4, 1, 0),
	SOC_DAPM_SINGLE("MIC1_M switch", MICPGA_PIN_CFG, 2, 1, 0),
};

/*
 * DAPM Widget Controls
 */
static const struct snd_soc_dapm_widget dac3100_dapm_widgets[] = {
	/* Left DAC to Left Outputs */
	/* dapm widget (stream domain) for left DAC */
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", DAC_CHN_REG, 7, 1),

	/* dapm widget (path domain) for left DAC_L Mixer */

	SND_SOC_DAPM_MIXER("HPL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpl_output_mixer_controls[0],
			   ARRAY_SIZE(hpl_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPL Power", HPHONE_DRIVERS, 7, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("LOL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &lol_output_mixer_controls[0],
			   ARRAY_SIZE(lol_output_mixer_controls)),
	SND_SOC_DAPM_PGA("LOL Power", CLASS_D_SPK, 7, 1, NULL, 0),

	/* Right DAC to Right Outputs */

	/* dapm widget (stream domain) for right DAC */
	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", DAC_CHN_REG, 6, 1),
	/* dapm widget (path domain) for right DAC_R mixer */
	SND_SOC_DAPM_MIXER("HPR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpr_output_mixer_controls[0],
			   ARRAY_SIZE(hpr_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPR Power", HPHONE_DRIVERS, 6, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("LOR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &lor_output_mixer_controls[0],
			   ARRAY_SIZE(lor_output_mixer_controls)),
	SND_SOC_DAPM_PGA("LOR Power", CLASS_D_SPK, 6, 1, NULL, 0),

	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
			   &left_input_mixer_controls[0],
			   ARRAY_SIZE(left_input_mixer_controls)),

	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
			   &right_input_mixer_controls[0],
			   ARRAY_SIZE(right_input_mixer_controls)),

	/* No widgets are required for ADC since the DAC3100 Audio
         * Codec Chipset does not contain a ADC
         */


	/* dapm widget (platform domain) name for HPLOUT */
	SND_SOC_DAPM_OUTPUT("HPL"),
	/* dapm widget (platform domain) name for HPROUT */
	SND_SOC_DAPM_OUTPUT("HPR"),
	/* dapm widget (platform domain) name for LOLOUT */
	SND_SOC_DAPM_OUTPUT("LOL"),
	/* dapm widget (platform domain) name for LOROUT */
	SND_SOC_DAPM_OUTPUT("LOR"),

	/* dapm widget (platform domain) name for MIC1LP */
	SND_SOC_DAPM_INPUT("MIC1LP"),
	/* dapm widget (platform domain) name for MIC1RP*/
	SND_SOC_DAPM_INPUT("MIC1RP"),
	/* dapm widget (platform domain) name for MIC1LM */
	SND_SOC_DAPM_INPUT("MIC1LM"),
};


/*
 * DAPM audio route definition. *
 * Defines an audio route originating at source via control and finishing
 * at sink.
 */
static const struct snd_soc_dapm_route dac3100_dapm_routes[] = {
	/* ******** Right Output ******** */
	{"HPR Output Mixer", "R_DAC switch", "Right DAC"},
	{"HPR Output Mixer",  "MIC1_R switch", "MIC1RP"},
	//{"HPR Output Mixer", "Right_Bypass switch", "Right_Bypass"},

	{"HPR Power", NULL, "HPR Output Mixer"},
	{"HPR", NULL, "HPR Power"},


	{"LOR Output Mixer", "R_DAC switch", "Right DAC"},
	{"LOR Output Mixer",  "MIC1_R switch", "MIC1RP"},
//	{"LOR Output Mixer", "Right_Bypass switch", "Right_Bypass"},

	{"LOR Power", NULL, "LOR Output Mixer"},
	{"LOR", NULL, "LOR Power"},

	/* ******** Left Output ******** */
	{"HPL Output Mixer", "L_DAC switch", "Left DAC"},
	{"HPL Output Mixer", "MIC1_L switch", "MIC1LP"},
	//{"HPL Output Mixer", "Left_Bypass switch", "Left_Bypass"},

	{"HPL Power", NULL, "HPL Output Mixer"},
	{"HPL", NULL, "HPL Power"},


	{"LOL Output Mixer", "L_DAC switch", "Left DAC"},
	{"LOL Output Mixer", "MIC1_L switch", "MIC1LP"},
//	{"LOL Output Mixer", "Left_Bypass switch", "Left_Bypass"},

	{"LOL Power", NULL, "LOL Output Mixer"},
	{"LOL", NULL, "LOL Power"},

	/* ******** terminator ******** */
	//{NULL, NULL, NULL},
};

#define dac3100_DAPM_ROUTE_NUM (sizeof(dac3100_dapm_routes)/sizeof(struct snd_soc_dapm_route))

static void i2c_dac3100_headset_access_work(struct work_struct *work);
static struct work_struct works;
static struct snd_soc_codec *codec_work_var_glob;

/*
*****************************************************************************
* Function Definitions
*****************************************************************************
*/

/*
 *----------------------------------------------------------------------------
 * Function : snd_soc_info_volsw_2r_dac3100
 * Purpose  : Info routine for the ASoC Widget related to the Volume Control
 *
 *----------------------------------------------------------------------------
 */
static int snd_soc_info_volsw_2r_dac3100 (struct snd_kcontrol *kcontrol,
                                          struct snd_ctl_elem_info *uinfo)
{
        int mask = (kcontrol->private_value >> 12) & 0xff;

	DBG("snd_soc_info_volsw_2r_dac3100 (%s)\n", kcontrol->id.name);

        uinfo->type =
                mask == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN : SNDRV_CTL_ELEM_TYPE_INTEGER;
        uinfo->count = 2;
        uinfo->value.integer.min = 0;
        uinfo->value.integer.max = mask;
        return 0;
}
/*
 *----------------------------------------------------------------------------
 * Function : snd_soc_get_volsw_2r_dac3100
 * Purpose  : Callback to get the value of a double mixer control that spans
 *            two registers.
 *
 *----------------------------------------------------------------------------
 */
int snd_soc_get_volsw_2r_dac3100 (struct snd_kcontrol *kcontrol,
                                  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & dac3100_8BITS_MASK;
	int reg2 = (kcontrol->private_value >> 24) & dac3100_8BITS_MASK;
	int mask;
	int shift;
	unsigned short val, val2;

	DBG("snd_soc_get_volsw_2r_dac3100 %s\n", kcontrol->id.name);

        /* Check the id name of the kcontrol and configure the mask and shift */
	if (!strcmp(kcontrol->id.name, "DAC Playback Volume")) {
		mask = dac3100_8BITS_MASK;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		mask = 0xF;
		shift = 3;
	} else if (!strcmp(kcontrol->id.name, "LO Driver Gain")) {
		mask = 0x3;
		shift = 3;
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		mask = 0x7F;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "HP Analog Volume")) {
                mask = 0x7F;
                shift = 0;
        } else if (!strcmp(kcontrol->id.name, "SPKR Analog Volume")) {
                mask = 0x7F;
                shift = 0;
        } else {
		printk(KERN_ALERT "Invalid kcontrol name\n");
		return -1;
	}

	val = (snd_soc_read(codec, reg) >> shift) & mask;
	val2 = (snd_soc_read(codec, reg2) >> shift) & mask;

	if (!strcmp(kcontrol->id.name, "DAC Playback Volume")) {
		ucontrol->value.integer.value[0] =
                        (val <= 48) ? (val + 127) : (val - 129);
		ucontrol->value.integer.value[1] =
                        (val2 <= 48) ? (val2 + 127) : (val2 - 129);
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		ucontrol->value.integer.value[0] =
                        (val <= 9) ? (val + 0) : (val - 15);
		ucontrol->value.integer.value[1] =
                        (val2 <= 9) ? (val2 + 0) : (val2 - 15);
	} else if (!strcmp(kcontrol->id.name, "LO Driver Gain")) {
		ucontrol->value.integer.value[0] =
                        ((val/6) <= 4) ? ((val/6 -1)*6) : ((val/6 - 0)*6);
		ucontrol->value.integer.value[1] =
                        ((val2/6) <= 4) ? ((val2/6-1)*6) : ((val2/6 - 0)*6);
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		ucontrol->value.integer.value[0] =
                        ((val*2) <= 40) ? ((val*2 + 24)/2) : ((val*2 - 254)/2);
		ucontrol->value.integer.value[1] =
                        ((val2*2) <= 40) ? ((val2*2 + 24)/2) : ((val2*2 - 254)/2);
        } else if (!strcmp(kcontrol->id.name, "HP Analog Volume")) {
                ucontrol->value.integer.value[0] =
                        ((val*2) <= 40) ? ((val*2 +24)/2) :((val2*2 - 254)/2);
                ucontrol->value.integer.value[1] =
                        ((val*2) <=40) ? ((val*2 + 24)/2) : ((val2*2 - 254)/2);
	} else if (!strcmp(kcontrol->id.name, "SPKR Analog Volume")) {
                ucontrol->value.integer.value[0] =
                        ((val*2) <= 40) ? ((val*2 +24)/2) :((val2*2 - 254)/2);
                ucontrol->value.integer.value[1] =
                        ((val*2) <=40) ? ((val*2 + 24)/2) : ((val2*2 - 254)/2);
        }
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : snd_soc_put_volsw_2r_dac3100
 * Purpose  : Callback to set the value of a double mixer control that spans
 *            two registers.
 *
 *----------------------------------------------------------------------------
 */
int snd_soc_put_volsw_2r_dac3100 (struct snd_kcontrol *kcontrol,
                                  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & dac3100_8BITS_MASK;
	int reg2 = (kcontrol->private_value >> 24) & dac3100_8BITS_MASK;
	int err;
	unsigned short val, val2, val_mask;

	DBG("snd_soc_put_volsw_2r_dac3100 (%s)\n", kcontrol->id.name);

	val = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

        /* Mistral: This block needs to be revisted */
	if (!strcmp(kcontrol->id.name, "DAC Playback Volume")) {
		val = (val >= 127) ? (val - 127) : (val + 129);
//		 (val <= 48) ? (val + 127) : (val - 129);
		val2 = (val2 >= 127) ? (val2 - 127) : (val2 + 129);
		val_mask = dac3100_8BITS_MASK;	/* 8 bits */
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		val = (val >= 0) ? (val - 0) : (val + 15);
		val2 = (val2 >= 0) ? (val2 - 0) : (val2 + 15);
		val_mask = 0xF;	/* 4 bits */
	} else if (!strcmp(kcontrol->id.name, "LO Driver Gain")) {
		val = (val/6 >= 1) ? ((val/6 +1)*6) : ((val/6 + 0)*6);
		val2 = (val2/6 >= 1) ? ((val2/6 +1)*6) : ((val2/6 + 0)*6);
		val_mask = 0x3;	/* 2 bits */
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		val = (val*2 >= 24) ? ((val*2 - 24)/2) : ((val*2 + 254)/2);
		val2 = (val2*2 >= 24) ? ((val2*2 - 24)/2) : ((val2*2 + 254)/2);
		val_mask = 0x7F;	/* 7 bits */
	} else if (!strcmp(kcontrol->id.name, "HP Analog Volume")) {
                val_mask = 0x7F; /* 7 Bits */
        } else if (!strcmp(kcontrol->id.name, "SPKR Analog Volume")) {
                val_mask = 0x7F; /* 7 Bits */
	} else {
		printk(KERN_ALERT "Invalid control name\n");
		return -1;
	}

	if ((err = snd_soc_update_bits(codec, reg, val_mask, val)) < 0) {
		printk(KERN_ALERT "Error while updating bits\n");
		return err;
	}

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
	return err;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info
 * Purpose  : This function is to initialize data for new control required to
 *            program the dac3100 registers.
 *
 *----------------------------------------------------------------------------
 */
static int __new_control_info (struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_info *uinfo)
{

	DBG("+ new control info\n");

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 65535;

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get
 * Purpose  : This function is to read data of new control for
 *            program the dac3100 registers.
 *
 *----------------------------------------------------------------------------
 */
static int __new_control_get (struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val;

	DBG("+ new control get (%d)\n", dac3100_reg_ctl);

	val = dac3100_read(codec, dac3100_reg_ctl);
	ucontrol->value.integer.value[0] = val;

	DBG("+ new control get val(%d)\n", val);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put
 * Purpose  : new_control_put is called to pass data from user/application to
 *            the driver.
 *
 *----------------------------------------------------------------------------
 */
static int __new_control_put (struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dac3100_priv *dac3100 = codec->private_data;

	u32 data_from_user = ucontrol->value.integer.value[0];
	u8 data[2];

	DBG("+ new control put (%s)\n", kcontrol->id.name);

	DBG("reg = %d val = %x\n", data[0], data[1]);

	dac3100_reg_ctl = data[0] = (u8) ((data_from_user & 0xFF00) >> 8);
	data[1] = (u8) ((data_from_user & 0x00FF));

	if (!data[0]) {
		dac3100->page_no = data[1];
	}

	DBG("reg = %d val = %x\n", data[0], data[1]);

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ALERT "Error in i2c write\n");
		return -EIO;
	}
	DBG("- new control put\n");

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_change_page
 * Purpose  : This function is to switch between page 0 and page 1.
 *
 *----------------------------------------------------------------------------
 */
int dac3100_change_page(struct snd_soc_codec *codec, u8 new_page)
{
	struct dac3100_priv *dac3100 = codec->private_data;
	u8 data[2];

	data[0] = 0;
	data[1] = new_page;
	dac3100->page_no = new_page;
	DBG("##dac3100_change_page => %d\nw 30 %02x %02x\n", new_page, data[0], data[1]);

//	DBG("w 30 %02x %02x\n", data[0], data[1]);

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ALERT "Error in changing page to %d\n", new_page);
		return -1;
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_write_reg_cache
 * Purpose  : This function is to write dac3100 register cache
 *
 *----------------------------------------------------------------------------
 */
static inline void dac3100_write_reg_cache (struct snd_soc_codec *codec,
                                            u16 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= dac3100_CACHEREGNUM) {
		return;
	}
	cache[reg] = value;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_read_reg_cache
 * Purpose  : This function is to read the dac3100 registers through the
 *            Register Cache Array instead of I2C Transfers
 *
 *----------------------------------------------------------------------------
 */
static unsigned char dac3100_read_reg_cache(struct snd_soc_codec *codec, unsigned int reg)
{
        u8 *cache = codec->reg_cache;

        /* Confirm the Register Offset is within the Array bounds */
        if (reg >= dac3100_CACHEREGNUM) {
                return (0);
        }

        return (cache[reg]);
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_write
 * Purpose  : This function is to write to the dac3100 register space.
 *
 *----------------------------------------------------------------------------
 */
int dac3100_write (struct snd_soc_codec *codec, unsigned int reg,
                   unsigned int value)
{
	struct dac3100_priv *dac3100 = codec->private_data;
	u8 data[2];
	u8 page;

	page = reg / 128;
	data[dac3100_REG_OFFSET_INDEX] = reg % 128;
	/* DBG("# dac3100 write reg(%d) new_page(%d) old_page(%d) "
           "value(0x%02x)\n", reg, page, dac3100->page_no, value); */


	if (dac3100->page_no != page) {
		dac3100_change_page(codec, page);
	}

	DBG("w 30 %02x %02x\n", data[dac3100_REG_OFFSET_INDEX], value);


	/* data is
	 *   D15..D8 dac3100 register offset
	 *   D7...D0 register data
	 */
	data[dac3100_REG_DATA_INDEX] = value & dac3100_8BITS_MASK;
#if defined(EN_REG_CACHE)
	if ((page == 0) || (page == 1)) {
		dac3100_write_reg_cache(codec, reg, value);
	}
#endif
	if (!data[dac3100_REG_OFFSET_INDEX]) {
		/* if the write is to reg0 update dac3100->page_no */
		dac3100->page_no = value;
	}

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ALERT "Error in i2c write\n");
		return -EIO;
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_read
 * Purpose  : This function is to read the dac3100 register space.
 *
 *----------------------------------------------------------------------------
 */
unsigned int dac3100_read (struct snd_soc_codec *codec, unsigned int reg)
{
	struct dac3100_priv *dac3100 = codec->private_data;
	u8 value;
	u8 page = reg / 128;
        u8 cache_value;

        /* Can be used to optimize the Reads from page 0 and 1 */
#if defined (EN_REG_CACHE)
        if ((page == 0) || (page == 1)) {
                cache_value = dac3100_read_reg_cache (codec, reg);
                //DBG("Reg%x-Cache %02x\n", reg, value);
        }
#endif
	reg = reg % 128;

	//DBG("r 30 %02x\n", reg);

	if (dac3100->page_no != page) {
		dac3100_change_page(codec, page);
	}

	codec->hw_write (codec->control_data, (char *)&reg, 1);
	/*codec->hw_read  (codec->control_data, &value, 1);*/ /* hw_read has changed in the 2.6.32 kernel */
        i2c_master_recv(codec->control_data, &value, 1);

        //DBG ("Reg %X Val %x Cache %x\r\n", reg, value, cache_value);
	return value;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_get_divs
 * Purpose  : This function is to get required divisor from the "dac3100_divs"
 *            table.
 *
 *----------------------------------------------------------------------------
 */
static inline int dac3100_get_divs (int mclk, int rate)
{
	int i;

	DBG("###+ dac3100_get_divs mclk(%d) rate(%d)\n", mclk, rate);

	for (i = 0; i < ARRAY_SIZE(dac3100_divs); i++) {
		if ((dac3100_divs[i].rate == rate)
		    && (dac3100_divs[i].mclk == mclk)) {
                        DBG("##%d %d %d %d %d %d %d\n",
                            dac3100_divs[i].p_val,
                            dac3100_divs[i].pll_j,
                            dac3100_divs[i].pll_d,
                            dac3100_divs[i].dosr,
                            dac3100_divs[i].ndac,
                            dac3100_divs[i].mdac,
                            dac3100_divs[i].blck_N);

			return i;
		}
	}
	printk(KERN_ALERT "Master clock and sample rate is not supported\n");
	return -EINVAL;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_add_controls
 * Purpose  : This function is to add non dapm kcontrols.  The different
 *            controls are in "dac3100_snd_controls" table.
 *            The following different controls are supported
 *                # DAC Playback volume control
 *		  # PCM Playback Volume
 *		  # HP Driver Gain
 *		  # HP DAC Playback Switch
 *		  # PGA Capture Volume
 *		  # Program Registers
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_add_controls (struct snd_soc_codec *codec)
{
	int err, i;

	DBG("+ dac3100_add_controls num_controls(%d)\n",
            ARRAY_SIZE(dac3100_snd_controls));
	for (i = 0; i < ARRAY_SIZE(dac3100_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
                                  snd_soc_cnew(&dac3100_snd_controls[i], codec,
                                               NULL));
		if (err < 0) {
			printk(KERN_ALERT "Invalid control\n");
			return err;
		}
	}
	DBG("- dac3100_add_controls \n");

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_add_widgets
 * Purpose  : This function is to add the dapm widgets
 *            The following are the main widgets supported
 *                # Left DAC to Left Outputs
 *                # Right DAC to Right Outputs
 *		  # Left Inputs to Left ADC
 *		  # Right Inputs to Right ADC
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_add_widgets(struct snd_soc_codec *codec)
{
	int i;

	DBG("+ dac3100_add_widgets num_widgets(%d) num_routes(%d)\n",
            ARRAY_SIZE(dac3100_dapm_widgets), dac3100_DAPM_ROUTE_NUM);
	for (i = 0; i < ARRAY_SIZE(dac3100_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &dac3100_dapm_widgets[i]);
	}

	DBG("snd_soc_dapm_add_routes\n");

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, &dac3100_dapm_routes[0],
				dac3100_DAPM_ROUTE_NUM);


	DBG("snd_soc_dapm_new_widgets\n");
	snd_soc_dapm_new_widgets(codec);
	DBG("- dac3100_add_widgets\n");
	return 0;
}


/*
 *----------------------------------------------------------------------------
 * Function : dac3100_hw_params
 * Purpose  : This function is to set the hardware parameters for dac3100.
 *            The functions set the sample rate and audio serial data word
 *            length.
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_hw_params (struct snd_pcm_substream *substream,
                              struct snd_pcm_hw_params *params,
                              struct snd_soc_dai *tmp)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct dac3100_priv *dac3100 = codec->private_data;
	int i;
	u8 data;

	mutex_lock(&dac3100->mutex);

	DBG("##+ SET dac3100_hw_params\n");


	dac3100_power_down(codec);
	codec->bias_level = 2;

        //dac3100_set_bias_level (codec, SND_SOC_BIAS_STANDBY);

	i = dac3100_get_divs(dac3100->sysclk, params_rate(params));
	DBG("##- Sampling rate: %d, %d\n", params_rate(params), i);


	if (i < 0) {
		printk(KERN_ALERT "sampling rate not supported\n");
		return i;
	}

	if (soc_static_freq_config) {

		/* We will fix R value to 1 and will make P & J=K.D as varialble */

		/* Setting P & R values */
		dac3100_write(codec, CLK_REG_2,
			      ((dac3100_divs[i].p_val << 4) | 0x01));

		/* J value */
		dac3100_write(codec, CLK_REG_3, dac3100_divs[i].pll_j);

		/* MSB & LSB for D value */
		dac3100_write(codec, CLK_REG_4, (dac3100_divs[i].pll_d >> 8));
		dac3100_write(codec, CLK_REG_5,
			      (dac3100_divs[i].pll_d & dac3100_8BITS_MASK));

		/* NDAC divider value */
		dac3100_write(codec, NDAC_CLK_REG_6, dac3100_divs[i].ndac);

		/* MDAC divider value */
		dac3100_write(codec, MDAC_CLK_REG_7, dac3100_divs[i].mdac);

		/* DOSR MSB & LSB values */
		dac3100_write(codec, DAC_OSR_MSB, dac3100_divs[i].dosr >> 8);
		dac3100_write(codec, DAC_OSR_LSB,
			      dac3100_divs[i].dosr & dac3100_8BITS_MASK);
	}
	/* BCLK N divider */
	dac3100_write(codec, CLK_REG_11, dac3100_divs[i].blck_N);

	data = dac3100_read(codec, INTERFACE_SET_REG_1);

	data = data & ~(3 << 4);

	printk(KERN_ALERT "##- Data length: %d\n", params_format(params));

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (dac3100_WORD_LEN_20BITS << DAC_OSR_MSB_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (dac3100_WORD_LEN_24BITS << DAC_OSR_MSB_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (dac3100_WORD_LEN_32BITS << DAC_OSR_MSB_SHIFT);
		break;
	}

 	/* Write to Page 0 Reg 27 for the Codec Interface control 1 Register */
	dac3100_write(codec, INTERFACE_SET_REG_1, data);

  	/* Switch on the Codec into ON State after all the above configuration */
//	dac3100_power_up(codec);

        /* Add the Processing blocks section as per the discussion with Design team */
        dac3100_write (codec, DAC_INSTRUCTION_SET, 0x03);

	DBG("##- SET dac3100_hw_params\n");

	mutex_unlock(&dac3100->mutex);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_config_hp_volume
 * Purpose  : This function is used to configure the I2C Transaction Global
 *            Variables. One of them is for ramping down the HP Analog Volume
 *            and the other one is for ramping up the HP Analog Volume
 *
 *----------------------------------------------------------------------------
 */
void dac3100_config_hp_volume (struct snd_soc_codec *codec, int mute)
{
	struct i2c_client *client = codec->control_data;
	struct dac3100_priv *dac3100 = codec->private_data;
	unsigned int count;
	struct dac3100_configs  *pReg;
	signed char regval;
        unsigned char low_value;
        unsigned int  reg_update_count;

	/* User has requested to mute or bring down the Headphone Analog Volume
         * Move from 0 db to -35.2 db
         */
	if (mute > 0) {
		pReg = &dac3100->hp_analog_right_vol[0];

		for (count = 0, regval = 0; regval <= 30; count++, regval +=1) {
			(pReg + count)->reg_offset = RIGHT_ANALOG_HPR;
			(pReg + count)->reg_val = (0x80 | regval);
		}
                (pReg +  (count -1))->reg_val = 0x1E;

		pReg = &dac3100->hp_analog_left_vol[0];

		for (count = 0, regval = 0; regval <= 30; count++, regval +=1) {
			(pReg + count)->reg_offset = LEFT_ANALOG_HPL;
			(pReg + count)->reg_val = (0x80 | regval);
		}
                (pReg +  (count -1))->reg_val = 0x1E;
                reg_update_count = count - 1;
                DBG("##CFG_HP_VOL count %d reg_update %d regval %d\n", count,
                    reg_update_count, regval);
	} else {
		/* User has requested to unmute or bring up the Headphone Analog
                 * Volume Move from -35.2 db to 0 db
	         */
		pReg = &dac3100->hp_analog_right_vol[0];

                low_value = (dac3100_read(codec, RIGHT_ANALOG_HPR) & 0x7F);
		for (count = 0, regval = low_value; regval >= 0;
                     count++, regval-=1) {
			(pReg + count)->reg_offset = RIGHT_ANALOG_HPR;
			(pReg + count)->reg_val = (regval);
		}
                (pReg + (count-1))->reg_val = (0x80);

		pReg = &dac3100->hp_analog_left_vol[0];

		for (count = 0, regval = low_value; regval >=0;
                     count++, regval-=1) {
			(pReg + count)->reg_offset = LEFT_ANALOG_HPL;
			(pReg + count)->reg_val = (regval);
		}
                (pReg + (count -1))->reg_val = (0x80);
                reg_update_count = count;
                DBG("##CFG_HP_VOL count %d reg_update %d regval %d\n",
                    count, reg_update_count, regval);
	}

	/* Change to Page 1 */
	dac3100_change_page (codec, 1);

	if (dac3100->i2c_regs_status == 0) {
                for (count = 0; count < reg_update_count; count++) {
			i2c_right_transaction[count].addr = client->addr;
			i2c_right_transaction[count].flags =
                                client->flags & I2C_M_TEN;
			i2c_right_transaction[count].len = 2;
			i2c_right_transaction[count].buf =
                                &dac3100->hp_analog_right_vol[count];
                }

                for (count = 0; count < reg_update_count; count++) {
			i2c_left_transaction[count].addr = client->addr;
			i2c_left_transaction[count].flags =
                                client->flags & I2C_M_TEN;
			i2c_left_transaction[count].len = 2;
			i2c_left_transaction[count].buf =
                                &dac3100->hp_analog_left_vol[count];
                }
                dac3100->i2c_regs_status = 1;
	}
	/* Perform bulk I2C transactions */
	if(i2c_transfer(client->adapter, i2c_right_transaction,
                        reg_update_count) != reg_update_count) {
		printk ("Error while Write brust i2c data error on "
                        "RIGHT_ANALOG_HPR!\n");
	}


	if(i2c_transfer(client->adapter, i2c_left_transaction,
                        reg_update_count) != reg_update_count) {
		printk ("Error while Write brust i2c data error on "
                        "LEFT_ANALOG_HPL!\n");
	}

        return;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_mute_codec
 * Purpose  : This function is to mute or unmute the left and right DAC
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_mute_codec (struct snd_soc_codec *codec, int mute)
{
	u8       dac_reg;
	volatile u8 value;
	struct dac3100_priv *dac3100 = codec->private_data;
	volatile u16 time_out_counter;

	DBG("##+ new dac3100_mute %d (current state is %d, headset_connected=%d) \n",
               mute, dac3100->mute, dac3100->headset_connected);

	dac_reg = dac3100_read(codec, DAC_MUTE_CTRL_REG);

	/* Also update the global Playback Status Flag. This is required for
           biquad update. */
	if ((mute) && (dac3100->mute != 1)) {
		dac3100->playback_status = 0;
                if (dac3100->headset_connected) {

                        /* Switch ON the Class_D Speaker Amplifier */
			value = dac3100_read (codec, CLASS_D_SPK);
			dac3100_write (codec, CLASS_D_SPK, (value | 0x80));

                        /* Page 47 of the datasheets requires unmuting HP and
                           Speaker drivers first */
                        /* MUTE the Headphone Left and Right */
                        value = dac3100_read (codec, HPL_DRIVER);
                        dac3100_write (codec, HPL_DRIVER, (value & ~0x04));

                        value = dac3100_read (codec, HPR_DRIVER);
                        dac3100_write (codec, HPR_DRIVER, (value & ~0x04));
                        DBG("##MUTED the HPL and HPR DRIVER REGS\n");
#if 1
                    dac3100_config_hp_volume (codec, mute);
//                        mdelay(10);
#else
                    /* Bring the HP Analog Volume Control Registers back to default value */
                    value = dac3100_read (codec, RIGHT_ANALOG_HPR);
                    while (value < 0x9F) {
                            value++;
                            dac3100_write (codec, RIGHT_ANALOG_HPR, value);
                            mdelay(2);
                            dac3100_write (codec, LEFT_ANALOG_HPL, value);
                            mdelay(2);
                    }

                    DBG("Moved RIGHT_ANALOG_HPR to %d\r\n", (value & 0x7F));
#endif

                } else {
                        /* MUTE THE Class-D Speaker Driver */
                        value = dac3100_read (codec, SPL_DRIVER);
                        dac3100_write (codec, SPL_DRIVER, (value & ~0x04));

                        DBG("##SPL MUTE REGS\n");
                }

		DBG("##muting DAC .................\n");

	        dac3100_write (codec, DAC_MUTE_CTRL_REG, dac_reg | MUTE_ON);

                DBG("##DAC MUTE Completed..\r\n");

                /* Change the DACL and DACR volumes values to lowest value */
                dac3100_write (codec, LDAC_VOL, 0x81);
                dac3100_write (codec, RDAC_VOL, 0x81);

                do {
                        mdelay(5);
                        /* Poll the DAC_FLAG register Page 0 38 for the DAC MUTE
                           Operation Completion Status */
                        value = dac3100_read (codec, DAC_FLAG_2);
                        time_out_counter++;
                } while ((time_out_counter < 20) && ((value & 0x11) == 0));
                DBG("##DAC Vol Poll Completed counter  %d regval %x\r\n",
                    time_out_counter, value);

		dac3100->mute = 1;
	} else if ((!mute) && (dac3100->mute != 0)) {
               	dac3100->playback_status = 1;
                /* We will enable the DAC UNMUTE first and finally the
                   Headphone UNMUTE to avoid pops */


                if (dac3100->headset_connected) {
                        /*Read the contents of the Page 0 Reg 63 DAC Data-Path
                         Setup Register. Just retain the upper two bits and
                         lower two bits
                        */
                        value = (dac3100_read(codec, DAC_CHN_REG) & 0xC3);
                        dac3100_write(codec, DAC_CHN_REG, (value | LDAC_2_LCHN | RDAC_2_RCHN));
                	/* Restore the values of the DACL and DACR */
                	dac3100_write (codec, LDAC_VOL, 0xFC);
                	dac3100_write (codec, RDAC_VOL, 0xFC);

			time_out_counter = 0;
                	do {
                	        mdelay (5);
                	        value = dac3100_read (codec, DAC_FLAG_2);
                	        time_out_counter ++;
                	} while ((time_out_counter < 100) && ((value & 0x11) == 0));

			DBG("##Changed DAC Volume back counter %d.\n", time_out_counter);

	                dac3100_write (codec, DAC_MUTE_CTRL_REG, (dac_reg & ~MUTE_ON));

	                DBG ("##DAC UNMUTED ...\n");
#if 1
                    dac3100_config_hp_volume (codec, mute);
    //                    mdelay(10);
#else
                    /* Bring the HP Analog Volume Control Registers back to default value */
                    value = dac3100_read (codec, RIGHT_ANALOG_HPR);
                    while (value > 0x80) {
                            value--;
                            dac3100_write (codec, RIGHT_ANALOG_HPR, value);
                            mdelay(2);
                            dac3100_write (codec, LEFT_ANALOG_HPL, value);
                            mdelay(2);
                    }

                    DBG("Moved RIGHT_ANALOG_HPR to %d\r\n", (value & 0x7F));

#endif
                    /* Page 47 of the datasheets requires unmuting HP and
                       Speaker drivers first */
                    /* UNMUTE the Headphone Left and Right */
                    value = dac3100_read (codec, HPL_DRIVER);
                    dac3100_write (codec, HPL_DRIVER, (value | 0x04));

                    value = dac3100_read (codec, HPR_DRIVER);
                    dac3100_write (codec, HPR_DRIVER, (value | 0x04));
                    DBG("##UNMUTED the HPL and HPR DRIVER REGS\n");


                    /* Switch OFF the Class_D Speaker Amplifier */
                    value = dac3100_read (codec, CLASS_D_SPK);
                    dac3100_write (codec, CLASS_D_SPK, (value & ~0x80));


                } else {
                        /*Read the contents of the Page 0 Reg 63 DAC Data-Path
                         Setup Register. Just retain the upper two bits and
                         lower two bits
                        */
                        value = (dac3100_read(codec, DAC_CHN_REG) & 0xC3);
                        dac3100_write(codec, DAC_CHN_REG,
                                      (value | LDAC_LCHN_RCHN_2));

                	/* Restore the values of the DACL and DACR */
                	dac3100_write (codec, LDAC_VOL, 0xFC);
                	dac3100_write (codec, RDAC_VOL, 0xFC);

			time_out_counter = 0;
                	do {
                	        mdelay (5);
                	        value = dac3100_read (codec, DAC_FLAG_2);
                	        time_out_counter ++;
                	} while ((time_out_counter < 100) && ((value & 0x11) == 0));

			DBG("##Changed DAC Volume back counter %d.\n", time_out_counter);

	                dac3100_write (codec, DAC_MUTE_CTRL_REG, (dac_reg & ~MUTE_ON));

	                DBG ("##DAC UNMUTED ...\n");
                        /* UNMUTE THE Class-D Speaker Driver */
                        value = dac3100_read (codec, SPL_DRIVER);
                        dac3100_write (codec, SPL_DRIVER, (value | 0x04));

                        DBG("##SPL UNMUTE REGS\n");
                }

        	dac3100->power_status = 1;
        	dac3100->mute = 0;
        }

	DBG("##-dac3100_mute %d\n", mute);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_mute
 * Purpose  : This function is to mute or unmute the left and right DAC
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_mute (struct snd_soc_dai *dai, int mute)
{
	int result;
	struct dac3100_priv *dac3100 = dai->codec->private_data;

        DBG ("##dac3100_mute handler mute=%d.\r\n", mute);

	mutex_lock(&dac3100->mutex);

	result = dac3100_mute_codec(dai->codec, mute);

	mutex_unlock(&dac3100->mutex);

	return result;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_set_dai_sysclk
 * Purpose  : This function is to set the DAI system clock
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_set_dai_sysclk (struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct dac3100_priv *dac3100 = codec->private_data;

	DBG("##dac3100_set_dai_sysclk clk_id(%d) (%d)\n", clk_id, freq);

	switch (freq) {
	case dac3100_FREQ_12000000:
	case dac3100_FREQ_24000000:
	case dac3100_FREQ_13000000:
		dac3100->sysclk = freq;
		return 0;
	}
	printk(KERN_ALERT "Invalid frequency to set DAI system clock\n");
	return -EINVAL;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_set_dai_fmt
 * Purpose  : This function is to set the DAI format
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_set_dai_fmt (struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct dac3100_priv *dac3100 = codec->private_data;
	u8 iface_reg;

	iface_reg = dac3100_read(codec, INTERFACE_SET_REG_1);
	iface_reg = iface_reg & ~(3 << 6 | 3 << 2);

	DBG("##+ dac3100_set_dai_fmt (%x) \n", fmt);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		dac3100->master = 1;
		iface_reg |= BIT_CLK_MASTER | WORD_CLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		dac3100->master = 0;
		break;
	default:
		printk(KERN_ALERT "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg |= (dac3100_DSP_MODE << CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg |= (dac3100_RIGHT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg |= (dac3100_LEFT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
		break;
	default:
		printk(KERN_ALERT "Invalid DAI interface format\n");
		return -EINVAL;
	}

	DBG("##- dac3100_set_dai_fmt (%x) \n", iface_reg);
	dac3100_write(codec, INTERFACE_SET_REG_1, iface_reg);
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_power_up
 * Purpose  : This function powers up the codec.
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_power_up (struct snd_soc_codec *codec)
{
	struct dac3100_priv *dac3100 = codec->private_data;
	u8 value;
	u8 counter;

	DBG("##++ dac3100_power_up dac3100->master=%d, dac3100->power_status=%d, "
            "dac3100->headset_connected=%d\n",
            dac3100->master, dac3100->power_status, dac3100->headset_connected);

	DBG("##dac3100 power up .................\n");
	//mdelay(2000);
	/* all power is driven by DAPM system */
	if (dac3100->master && (dac3100->power_status != 1) ) {

		/* WA, enable the gpt11_fclk to avoid audio quality problems
                   when screen OFF */
		clk_enable(gpt11_fclk);

        mdelay(5);
		/* Switch on PLL */
		value = dac3100_read(codec, CLK_REG_2);
		dac3100_write(codec, CLK_REG_2, (value | ENABLE_PLL));

		/* Switch on NDAC Divider */
		value = dac3100_read(codec, NDAC_CLK_REG_6);
		dac3100_write(codec, NDAC_CLK_REG_6, value | ENABLE_NDAC);

		/* Switch on MDAC Divider */
		value = dac3100_read(codec, MDAC_CLK_REG_7);
		dac3100_write(codec, MDAC_CLK_REG_7,
			      value | ENABLE_MDAC);

		/* Switch on BCLK_N Divider */
		value = dac3100_read(codec, CLK_REG_11);
		dac3100_write(codec, CLK_REG_11, value | ENABLE_BCLK);

                /* Switch ON Left and Right DACs */
		value = dac3100_read (codec, DAC_CHN_REG);
		dac3100_write (codec, DAC_CHN_REG, (value | ENABLE_DAC));

        /* Check for the DAC FLAG register to know if the DAC is
           really powered up */
        counter = 0;
        do {
                mdelay(10);
                value = dac3100_read (codec, DAC_FLAG_1);
                counter++;
                DBG("##DACEn Poll\r\n");
        } while ((counter < 20) && ((value & 0x88) == 0));

        DBG("##-- dac3100_power_up\n");

		/* Check whether the Headset or Speaker Driver needs Power Up */
		if (dac3100->headset_connected) {
                        /* It is observed that turning ON Speaker, helps reduce
                           the pop up noise */
                        /* Switch ON the Class_D Speaker Amplifier */
                        value = dac3100_read (codec, CLASS_D_SPK);
                        dac3100_write (codec, CLASS_D_SPK, (value | 0x80));
#if 0
                        /* Enable routing between left analog volume to HPL
                         * and right analog volume to HPR
                         */
                        value = dac3100_read(codec, LEFT_ANALOG_HPL);
                        dac3100_write(codec, LEFT_ANALOG_HPL, (0x14 | 0x80));
                        value = dac3100_read(codec, RIGHT_ANALOG_HPR);
                        dac3100_write(codec, RIGHT_ANALOG_HPR, (0x14 | 0x80));
                        mdelay(20);

                        dac3100_config_hp_volume(codec, 0);
#endif

                        /* Switch ON Left and Right Headphone Drivers */
                        value = dac3100_read (codec, HPHONE_DRIVERS);
                        dac3100_write (codec, HPHONE_DRIVERS, (value | 0xC0)); /* 0xCC */

                        /* Check for the DAC FLAG Register to know if the Left
                           Output Driver is powered up */
                        counter = 0;
                        do {
                                mdelay (10);
                                value = dac3100_read (codec, DAC_FLAG_1);
                                counter++;
                                DBG("##HPL Poll..\n");
                        } while ((value & 0x22) == 0);
                        DBG("##HPL Power up Iterations %d\r\n", counter);

#if 0
                        value = dac3100_read(codec, HPHONE_DRIVERS);
                        dac3100_write(codec, HPHONE_DRIVERS, (value | 0x40));

                        /* Check for the DAC FLAG Register to know if the HPHONE
                           Driver is powered up */
                        counter = 0;
                        do {
                                mdelay (5);
                                value = dac3100_read(codec, DAC_FLAG_1);
                                counter++;
                                DBG("##HPR Poll..\n");
                        } while ((value & 0x02) == 0);
#endif
                        DBG("##HPR Power Up Iterations %d\n", counter);
                        //mdelay (60);


		} else {
                        /* Left Analog Speaker Volume update */
                        dac3100_write (codec, LEFT_ANALOG_SPL, 0x80);

                        /* Switch ON the Class_D Speaker Amplifier */
                        value = dac3100_read (codec, CLASS_D_SPK);
                        dac3100_write (codec, CLASS_D_SPK, (value | 0x80));
                        mdelay(100);
                }
                dac3100->power_status = 1;
        }

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_power_down
 * Purpose  : This function powers down the codec.
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_power_down (struct snd_soc_codec *codec)
{
	struct dac3100_priv *dac3100 = codec->private_data;
	volatile u8 value;
        volatile u32 counter;

	DBG("##++ dac3100_power_down dac3100->master=%d\n", dac3100->master);

	if (dac3100->master && (dac3100->power_status != 0)) {

		/* Check whether the Headset or Speaker Driver needs Power Down */
		if(dac3100->headset_connected) {

                        /* Switch off the Head phone Drivers */
                        value = dac3100_read (codec, HPHONE_DRIVERS);
                        dac3100_write (codec, HPHONE_DRIVERS, (value & ~0xC0)); /* 0xCC */

                        /* Now first check if the HPR is fully powered down */
                        counter = 0;
                        do {
                                mdelay(5);
                                value = dac3100_read (codec, DAC_FLAG_1);
                                counter++;
                        }while ((counter < 100) && ((value & 0x22) != 0));
                        DBG ("##HPHONE RIGHT DRIVER Power Down. counter %d\r\n",
                             counter);
                        //mdelay(50);
#if 0
                        value = dac3100_read (codec, HPHONE_DRIVERS);
                        dac3100_write (codec, HPHONE_DRIVERS, (value & ~0x80));

                        /* Now poll the Page 0 Reg 37 DAC FLAG Register for HPL
                         * fully powered down */
                        counter = 0;
                        do {
                                mdelay(5);
                                value = dac3100_read (codec, DAC_FLAG_1);
                                counter++;
                        } while ((counter < 20) && ((value & 0x20) != 0));
#endif
                        DBG("##HPHONE LEFT DRIVER Power Down counter %d \r\n",
                            counter);

                        value = dac3100_read (codec, CLASS_D_SPK);
                        dac3100_write (codec, CLASS_D_SPK, (value & ~0x80));
#if 0
                        /* Disable routing between left analog volume to HPL
                         * and right analog volume to HPR
                         */
                        value = dac3100_read(codec, LEFT_ANALOG_HPL);
                        dac3100_write(codec, LEFT_ANALOG_HPL, (value & ~0x80));
                        value = dac3100_read(codec, RIGHT_ANALOG_HPR);
                        dac3100_write(codec, RIGHT_ANALOG_HPR, (value & ~0x80));

                        dac3100_config_hp_volume(codec, 1);
#endif
		} else {
			/* Switch OFF the Class_D Speaker Amplifier */
			value = dac3100_read (codec, CLASS_D_SPK);
			dac3100_write (codec, CLASS_D_SPK, (value & ~0x80));

                        /* MUTE THE Class-D Speaker Driver */
                        value = dac3100_read (codec, SPL_DRIVER);
                        value &= ~0x04;
                        dac3100_write (codec, SPL_DRIVER, value);
                        DBG ("##SPL_DRIVER MUTE Completed..\r\n");
	        }
                /* Switch OFF Left and Right DACs */

		value = dac3100_read (codec, DAC_CHN_REG);
		dac3100_write (codec, DAC_CHN_REG, (value & ~ENABLE_DAC));

                /* Check for the DAC FLAG register to know if the DAC is really
                   powered down */
                counter = 0;
                do {
                        mdelay(10);
                        value = dac3100_read (codec, DAC_FLAG_1);
                        counter++;
                } while ((counter < 100) && ((value & 0x88) != 0));
                DBG ("##Left and Right DAC off Counter %d\r\n", counter);

                /* Switch off BCLK_N Divider */
		dac3100_write(codec, CLK_REG_11, value & ~ENABLE_BCLK);

                /* Switch off MDAC Divider */
		value = dac3100_read(codec, MDAC_CLK_REG_7);
		dac3100_write(codec, MDAC_CLK_REG_7,
                              value & ~ENABLE_MDAC);

		/* Switch off NDAC Divider */
		value = dac3100_read(codec, NDAC_CLK_REG_6);
		dac3100_write(codec, NDAC_CLK_REG_6,
                              value & ~ENABLE_NDAC);

		/* Switch off PLL */
		value = dac3100_read(codec, CLK_REG_2);
		dac3100_write(codec, CLK_REG_2, (value & ~ENABLE_PLL));

		if (gpt11_fclk->usecount > 0)
			clk_disable(gpt11_fclk);

		dac3100->power_status = 0;
	}

	DBG("##-- dac3100_power_down\n");

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_set_bias_level
 * Purpose  : This function is to get triggered when dapm events occurs.
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_set_bias_level (struct snd_soc_codec *codec,
                                   enum snd_soc_bias_level level)
{
	struct dac3100_priv *dac3100 = codec->private_data;
	u8 value;

	mutex_lock(&dac3100->mutex);

	DBG("##++ dac3100_set_bias_level %d\n", level);

	if (level == codec->bias_level) {
		DBG("##set_bias_level: level returning...\r\n");
		return 0;
	}

	switch (level) {
		/* full On */
	case SND_SOC_BIAS_ON:
		DBG("##dac3100_set_bias_level ON\n");
		dac3100_power_up(codec);
//                dac3100_mute_codec(codec, 0);
		break;


	case SND_SOC_BIAS_PREPARE:                  /* partial On */
		DBG("##dac3100_set_bias_level PREPARE\n");
		break;


	case SND_SOC_BIAS_STANDBY:                 /* Off, with power ??? */
		DBG("##dac3100_set_bias_level STANDBY\n");
                dac3100_power_down(codec);
		break;


	case SND_SOC_BIAS_OFF:                     /* Off, without power */
		DBG("##dac3100_set_bias_level OFF\n");
                dac3100_mute_codec(codec, 1);
		dac3100_power_down(codec);
   		break;
	}
	codec->bias_level = level;
	DBG("##-- dac3100_set_bias_level\n");

	mutex_unlock(&dac3100->mutex);

	return 0;
}


/*
 *----------------------------------------------------------------------------
 * Function : dac3100_hw_free
 * Purpose  : This function is to get triggered when dapm events occurs.
 *            Note that Android layer will invoke this function during closure
 *            of the Audio Resources.
 *----------------------------------------------------------------------------
 */
static int dac3100_hw_free (struct snd_pcm_substream *substream,
                            struct snd_soc_dai *device)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec   = socdev->card->codec;

        DBG("+ dac3100_hw_free \n");

        /* Android seems to call the hw_free callback function first even before
         * muting the codec. So first mute the Audio Codec and then put it in
         * STANDBY State
         */
        dac3100_mute (device, 1);


        dac3100_set_bias_level (codec, SND_SOC_BIAS_STANDBY);

        DBG("- dac3100_hw_free \n");
        return (0);
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_suspend
 * Purpose  : This function is to suspend the dac3100 driver.
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_suspend (struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	struct dac3100_priv *dac3100 = codec->private_data;

	u8 regvalue;

	DBG("+ dac3100_suspend\n");

        DBG("dac3100_suspend OFF State\r\n");
	dac3100_set_bias_level(codec, SND_SOC_BIAS_OFF);

        /* Perform the Device Soft Power Down */
        regvalue = dac3100_read (codec, MICBIAS_CTRL);
        dac3100_write (codec, MICBIAS_CTRL, (regvalue | 0x80));

	DBG("-dac3100_suspend\n");

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_resume
 * Purpose  : This function is to resume the dac3100 driver
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_resume (struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	u8 regvalue;

	DBG("+ dac3100_resume\n");
	DBG("dac3100_resume: Level %d\r\n", codec->suspend_bias_level);

        /* Perform the Device Soft Power UP */
        regvalue = dac3100_read (codec, MICBIAS_CTRL);
        dac3100_write (codec, MICBIAS_CTRL, (regvalue & ~0x80));
        /* Added delay as per the suggestion from TI Audio team */
        mdelay (50);

	/* Sync reg_cache with the hardware
           for (i = 0; i < ARRAY_SIZE(dac3100_reg); i++) {
           dac3100_write(codec, i, cache[i]);
           }*/

        DBG("dac3100_resume: Suspend_bias_level %d\r\n", codec->suspend_bias_level);

        /* We will not use the codec->suspend_bias_level, since it tries to
           power-up the codec after resuming from suspend. */
	dac3100_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	DBG("- dac3100_resume\n");

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : tlv320dac3100_init
 * Purpose  : This function is to initialise the dac3100 driver
 *            register the mixer and codec interfaces with the kernel.
 *
 *----------------------------------------------------------------------------
 */
#define TRITON_AUDIO_IF_PAGE 	0x1
static int tlv320dac3100_init (struct snd_soc_codec * codec)
{
	struct dac3100_priv *dac3100 = codec->private_data;
	int ret = 0;
	int i = 0;
	int hph_detect_gpio = AUDIO_CODEC_HPH_DETECT_GPIO;
	int hph_detect_irq = 0;
	unsigned char regval;

	printk(KERN_ALERT "##+tlv320dac3100_init\n");

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* Initialize private data for the codec */
	dac3100->mute              =  -1;
	dac3100->headset_connected =  -1;
	dac3100->power_status      =  -1;
	dac3100->playback_status   =   0;
        dac3100->i2c_regs_status   =   0;

	mutex_init(&dac3100->mutex);

	ret = gpio_request(hph_detect_gpio, "dac3100-headset");

	if (ret < 0) {
		goto err1;
	}
	gpio_direction_input(hph_detect_gpio);
	omap_set_gpio_debounce(hph_detect_gpio,1);
	omap_set_gpio_debounce_time(hph_detect_gpio,0xFF);
	hph_detect_irq = OMAP_GPIO_IRQ(hph_detect_gpio);

	codec->name = "dac3100";
	codec->owner = THIS_MODULE;
	codec->read = dac3100_read;
	codec->write = dac3100_write;

	/* Mistral: Enabled the bias_level routine */
	codec->set_bias_level = dac3100_set_bias_level;
	codec->dai = &tlv320dac3100_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(dac3100_reg);

	codec->reg_cache =
                kmemdup(dac3100_reg, sizeof(dac3100_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL) {
		DBG(KERN_ERR "dac3100: kmemdup failed\n");
		return -ENOMEM;
	}

	dac3100->page_no = 0;

	DBG(KERN_ALERT "##*** Configuring dac3100 registers ***\n");
        dac3100_change_page(codec, 0x00);
	dac3100_write(codec, RESET, 0x01);
        mdelay(10);

	for (i = 0; i < sizeof(dac3100_reg_init) / sizeof(struct dac3100_configs); i++) {
		dac3100_write(codec, dac3100_reg_init[i].reg_offset, dac3100_reg_init[i].reg_val);
                mdelay (10); /* Added delay across register writes */
	}

	DBG(KERN_ALERT "##*** Done Configuring dac3100 registers ***\n");

	ret = request_irq(hph_detect_irq, dac3100_irq_handler,
                          IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
                          IRQF_DISABLED | IRQF_SHARED , "dac3100", codec);

	printk(KERN_ALERT "##-tlv320dac3100_init\n");

	return ret;

err1:
	free_irq(hph_detect_irq, codec);

card_err:
	snd_soc_free_pcms(dac3100_socdev);
	snd_soc_dapm_free(dac3100_socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_headset_speaker_path
 * Purpose  : This function is to check for the presence of Headset and
 *            configure the Headphone of the Class D Speaker driver
 *            Registers appropriately.
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_headset_speaker_path (struct snd_soc_codec *codec)
{
	struct dac3100_priv *dac3100 = codec->private_data;
	int headset_detect_gpio = AUDIO_CODEC_HPH_DETECT_GPIO;
	int headset_present = 0;
	u8 value;

	mutex_lock(&dac3100->mutex);

	headset_present = !(gpio_get_value(headset_detect_gpio));

	dac3100->headset_connected = headset_present;

        DBG("%s: %d\n", __func__, headset_present);

        if (dac3100->playback_status == 1) {

		/* If codec was not powered up, power up the same. */
		if(headset_present) {
			DBG("headset present and headset path Activated\n");
                        dac3100_write(codec, HPHONE_DRIVERS, 0xC4); // 0xCC ON
                        dac3100_write(codec, CLASS_D_SPK, 0x06); // OFF
		} else {
			DBG( "headset removed and headset path "
                               "Deactivated\n");

                        dac3100_write(codec, HPHONE_DRIVERS ,0x04); // OFF
                        dac3100_write(codec, CLASS_D_SPK ,0xC6 ); //ON
                }

                /* We will force the dac3100->mute to 1 to ensure that the
                 * following function executes completely.
                 */
                dac3100->mute = 1;
                /* Now unmute the appropriate Codec sections with Volume Ramping */
                dac3100_mute_codec (codec, 0);

#ifdef CONFIG_ADAPTIVE_FILTER
		/* Update the Biquad Array */
		dac3100_update_biquad_array(codec, headset_present,
					    dac3100->playback_status);
#endif
	}
	mutex_unlock(&dac3100->mutex);
	return 0;
}

/*
 * This interrupt is called when HEADSET is insert or remove from conector.
 On this interupt sound will be rouote to HEadset or speaker path.
*/
static irqreturn_t dac3100_irq_handler (int irq, void* data)
{
	struct snd_soc_codec *codec = data;

	DBG (KERN_ALERT "interrupt of headset found\n");

	codec_work_var_glob = codec;

	schedule_work(&works);

	return IRQ_HANDLED;
}

/*
 *----------------------------------------------------------------------------
 * Function : i2c_dac3100_headset_access_work
 * Purpose  : Worker Thread Function.
 *
 *----------------------------------------------------------------------------
 */
static void i2c_dac3100_headset_access_work (struct work_struct *work)
{
	dac3100_headset_speaker_path(codec_work_var_glob);
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_probe
 * Purpose  : This is first driver function called by the SoC core driver.
 *
 *----------------------------------------------------------------------------
 */

static int dac3100_probe (struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = dac3100_codec;
	int ret = 0;

        DBG("+dac3100_probe: function entered\r\n");

        if (!codec) {
                printk(KERN_ERR "dac3100_probe: Codec not yet Registered..\n");
                return -ENODEV;
        }

        socdev->card->codec = codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "dac3100_probe: failed to create pcms\n");
		goto pcm_err;
	}

	dac3100_add_controls(codec);
//	dac3100_add_widgets(codec);

#ifdef CONFIG_ADAPTIVE_FILTER
	dac3100_add_biquads_controls (codec);

	dac3100_add_EQ_mixer_controls (codec);

        dac3100_parse_biquad_array (codec);
#endif
        /* Moved the dac3100_headset_speaker_path here after the biquad configuration */
        dac3100_headset_speaker_path(codec);

	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "dac3100_probe: failed to register card\n");
		goto card_err;
	}
        else {
		DBG("snd_soc_init_card: success...\r\n");
        }

	dac3100_socdev = socdev;

        DBG("-dac3100_probe function exited..\r\n");

	return ret;
card_err:
        snd_soc_free_pcms(socdev);
        snd_soc_dapm_free(socdev);

pcm_err:
        kfree(codec->reg_cache);
        return ret;

}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_remove
 * Purpose  : to remove dac3100 soc device
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_remove (struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	/* power down chip */
        /* Mistral: perform set_bias_level for SND_SOC_BIAS_OFF.*/
	if (codec->control_data)
		dac3100_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_register
 * Purpose  : Helper function used to register the Codec with SOC Layer
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_register (struct snd_soc_codec *codec)
{
        int ret;
        int gpio = AUDIO_CODEC_PWR_ON_GPIO;

        DBG("+dac3100_register: function entered...\r\n");

        /* Switch on the Power GPIO Line so that the Codec Gets the Power */
        ret = gpio_request(gpio, AUDIO_CODEC_PWR_ON_GPIO_NAME);
        gpio_direction_output(gpio, 0);
        gpio_set_value(gpio, 1);

        /* Codec Initialization Routine*/
        ret = tlv320dac3100_init(codec);
        if (ret < 0) {
                printk("dac3100_register: Failed to initialise device\n");
                return ret;
        }

        dac3100_codec = codec;
        /* Register the Codec and the DAI Structures */
        ret = snd_soc_register_codec(codec);
        if (ret) {
                printk("dac3100_register: Failed to register codec\n");
                return ret;
        }

        ret = snd_soc_register_dai(&tlv320dac3100_dai);
        if (ret) {
                printk("dac3100_register: Failed to register dai\n");
                snd_soc_unregister_codec(codec);
                return ret;
        }
        DBG("-dac3100_register function exited...\r\n");
        return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : dac3100_unregister
 * Purpose  : Helper function used to un-register the Codec with SOC Layer
 *
 *----------------------------------------------------------------------------
 */
static int dac3100_unregister (struct dac3100_priv *dac3100)
{
        dac3100_set_bias_level(&dac3100->codec, SND_SOC_BIAS_OFF);

        snd_soc_unregister_dai(&tlv320dac3100_dai);
        snd_soc_unregister_codec(&dac3100->codec);

        kfree(dac3100);

        dac3100_codec = NULL;

        return 0;
}

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
/*
 *----------------------------------------------------------------------------
 * Function : tlv320dac3100_i2c_probe
 * Purpose  : This function attaches the i2c client and initializes
 *				dac3100 CODEC.
 *            NOTE:
 *            This function is called from i2c core when the I2C address is
 *            valid.
 *            If the i2c layer weren't so broken, we could pass this kind of
 *            data around
 *
 *----------------------------------------------------------------------------
 */
static int tlv320dac3100_i2c_probe (struct i2c_client *i2c,
                                    const struct i2c_device_id *id)
{
	struct snd_soc_codec *codec;
	int ret;
        struct dac3100_priv *dac310x;

        dac310x = kzalloc (sizeof (struct dac3100_priv), GFP_KERNEL);
        if (dac310x == NULL) {
                printk("tlv320dac3100_i2c_probe: Failed to create Codec Private Data...\n");
                return -ENOMEM;
        }
        DBG("tlv320dac3100_i2c_probe function..\r\n");
        /* Initialize the snd_soc_codec device members */
        codec = &dac310x->codec;
        codec->dev = &i2c->dev;
        codec->private_data = (void *)dac310x;
	codec->control_data = i2c;
        codec->hw_write = (hw_write_t) i2c_master_send;
        /*codec->hw_read = (hw_read_t) i2c_master_recv;*/

	i2c_set_clientdata(i2c, codec);

	INIT_WORK(&works, i2c_dac3100_headset_access_work);

	ret = dac3100_register (codec);

	if (ret < 0) {
		printk(KERN_ERR "tlv320dac3100_i2c_probe: failed to attach codec at addr\n");
	} else {

                DBG ("dac3100_register sucess...\r\n");
        }
        DBG ("tlv320dac3100_i2c_probe exited...\r\n");
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : tlv320dac3100_i2c_remove
 * Purpose  : This function removes the i2c client and uninitializes
 *                              DAC3100 CODEC.
 *            NOTE:
 *            This function is called from i2c core
 *            If the i2c layer weren't so broken, we could pass this kind of
 *            data around
 *
 *----------------------------------------------------------------------------
 */
static int __exit tlv320dac3100_i2c_remove (struct i2c_client *i2c)
{
	struct dac3100_priv *dac310x = i2c_get_clientdata (i2c);

        return dac3100_unregister (dac310x);
}

/* i2c Device ID Struct used during Driver Initialization and Shutdown */
static const struct i2c_device_id tlv320dac3100_id[] = {
        {"tlv320dac3100", 0},
        {}
};

MODULE_DEVICE_TABLE(i2c, tlv320dac3100_id);

/* Definition of the struct i2c_driver structure */
static struct i2c_driver tlv320dac3100_i2c_driver = {
	.driver = {
		.name = "tlv320dac3100",
                .owner = THIS_MODULE,
	},
	.probe = tlv320dac3100_i2c_probe,
	.remove = __exit_p(tlv320dac3100_i2c_remove),
	.id_table = tlv320dac3100_id,
};

/* I2C Init Routine */
static inline void dac3100_i2c_init (void)
{
        int ret;

        ret = i2c_add_driver(&tlv320dac3100_i2c_driver);
        if (ret)
                printk(KERN_ERR "%s: error regsitering i2c driver, %d\n",
                       __func__, ret);
}

/* I2C Exit Routine */
static inline void dac3100_i2c_exit (void)
{
        i2c_del_driver(&tlv320dac3100_i2c_driver);
}

#else

static inline void dac3100_i2c_init(void) { }
static inline void dac3100_i2c_exit(void) { }

#endif //#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)


/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_device |
 *          This structure is soc audio codec device sturecute which pointer
 *          to basic functions dac3100_probe(), dac3100_remove(),
 *          dac3100_suspend() and dac3100_resume()
 *
 */
struct snd_soc_codec_device soc_codec_dev_dac3100 = {
	.probe = dac3100_probe,
	.remove = dac3100_remove,
	.suspend = dac3100_suspend,
	.resume = dac3100_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_dac3100);

/*
 *----------------------------------------------------------------------------
 * Function : tlv320dac3100_modinit
 * Purpose  : Module INIT Routine
 *
 *----------------------------------------------------------------------------
 */
static int __init tlv320dac3100_modinit(void)
{
	dac3100_i2c_init ();
        return 0;
}

module_init(tlv320dac3100_modinit);

/*
 *----------------------------------------------------------------------------
 * Function : tlv320dac3100_exit
 * Purpose  : Module EXIT Routine
 *
 *----------------------------------------------------------------------------
 */
static void __exit tlv320dac3100_exit(void)
{
	dac3100_i2c_exit();
}

module_exit(tlv320dac3100_exit);

MODULE_DESCRIPTION("ASoC TLV320dac3100 codec driver");
MODULE_AUTHOR("s-griffoul@ti.com");
MODULE_AUTHOR("ravindra@mistralsolutions.com");
MODULE_LICENSE("GPL");
