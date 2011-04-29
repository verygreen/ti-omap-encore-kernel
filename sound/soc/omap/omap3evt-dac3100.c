/*
 * omap3evm-dac3100.c - SoC audio for OMAP3530 EVM.
 * 
 * Copyright (C) 2010 Mistral Solutions
 *
 * Author: Sandeep S Prabhu	,sandeepsp@mistralsolutions.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Modified from n810.c  --  SoC audio for Nokia N810
 *
 * Revision History
 *
 * Inital code : May 7, 2009 :	Sandeep S Prabhu 
 * 					<sandeepsp@mistralsolutions.com>
 * Revision 0.1         01 Dec 2010         Updated the code-base for 2.6.32 Kernel
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/soundcard.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#include <plat/omap-pm.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/io.h>
#include <asm/io.h>


#include "omap-pcm.h"
#include "omap-mcbsp.h"
#include "../codecs/tlv320dac3100.h"

static struct wake_lock omap3edp_wakelock;
static struct clk *sys_clkout2;
static struct clk *clkout2_src_ck;
static struct clk *sys_ck;

struct clk *gpt11_fclk;
EXPORT_SYMBOL_GPL(gpt11_fclk);

#define CODEC_SYSCLK_FREQ	13000000lu


#define MCBSP1_ID	0
#define MCBSP2_ID	1
#define MCBSP3_ID	2
#define MCBSP4_ID	3

/*
 *----------------------------------------------------------------------------
 * Function : omap3evt_hw_params
 * Purpose  : Machine Driver's hw_params call-back handler routine.
 *----------------------------------------------------------------------------
 */
static int omap3evt_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int err;
	
	/* Set codec DAI configuration */
	err = codec_dai->ops->set_fmt(codec_dai,
					 SND_SOC_DAIFMT_I2S |
					 SND_SOC_DAIFMT_NB_NF |
					 SND_SOC_DAIFMT_CBM_CFM);

	if (err < 0)
		return err;
	
	/* Set cpu DAI configuration */
	err = cpu_dai->ops->set_fmt(cpu_dai,
				       SND_SOC_DAIFMT_I2S  |
				       SND_SOC_DAIFMT_NB_NF |
				       SND_SOC_DAIFMT_CBM_CFM);

	if (err < 0)
		return err;

	/* Set the codec system clock for DAC and ADC */
	err = codec_dai->ops->set_sysclk(codec_dai, 0, CODEC_SYSCLK_FREQ,
					    SND_SOC_CLOCK_IN);

	/* Use CLKX input for mcBSP2 */
	err = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKX_EXT,
			0, SND_SOC_CLOCK_IN);

	return err;
}

static int snd_hw_latency;
extern void omap_dpll3_errat_wa(int disable);

/*
 *----------------------------------------------------------------------------
 * Function : omap3evt_startup
 * Purpose  : Machine Driver's startup routine.
 *----------------------------------------------------------------------------
 */
static int omap3evt_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	/*
	 * Hold C2 as min latency constraint. Deeper states
	 * MPU RET/OFF is overhead and consume more power than
	 * savings.
	 * snd_hw_latency check takes care of playback and capture
	 * usecase.
	 */
	if (!snd_hw_latency++) {
		omap_pm_set_max_mpu_wakeup_lat(rtd->socdev->dev, 18);
		/*
		 * As of now for MP3 playback case need to enable dpll3
		 * autoidle part of dpll3 lock errata.
		 * REVISIT: Remove this, Once the dpll3 lock errata is
		 * updated with with a new workaround without impacting mp3 usecase.
		 */
		omap_dpll3_errat_wa(0);
	}

	wake_lock(&omap3edp_wakelock);	
	return clk_enable(sys_clkout2);
}

/*
 *----------------------------------------------------------------------------
 * Function : omap3evt_shutdown
 * Purpose  : Machine Driver's shutdown routine.
 *----------------------------------------------------------------------------
 */
static void omap3evt_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	clk_disable(sys_clkout2);
	wake_unlock(&omap3edp_wakelock);

	/* remove latency constraint */
	snd_hw_latency--;
	if (!snd_hw_latency) {
		omap_pm_set_max_mpu_wakeup_lat(rtd->socdev->dev, -1);
		omap_dpll3_errat_wa(1);
	}

}

/* Machine Driver's SOC OPS structure. */
static struct snd_soc_ops omap3evt_ops = {
	.startup = omap3evt_startup,
	.hw_params = omap3evt_hw_params,
	.shutdown = omap3evt_shutdown,
};

static const struct snd_soc_dapm_widget aic3111_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Ext Spk", NULL),
};

/*
 *----------------------------------------------------------------------------
 * Function : omap3evt_dac3100_init
 * Purpose  : Initialization routine.
 *----------------------------------------------------------------------------
 */
static int omap3evt_dac3100_init(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_dai_link omap3evt_dai = {
	.name = "TLV320DAC3100",
	.stream_name = "DAC3100",
	.codec_dai = &tlv320dac3100_dai,
	.init = omap3evt_dac3100_init,
	.cpu_dai = &omap_mcbsp_dai[0],
	.ops = &omap3evt_ops,
};


static struct snd_soc_card snd_soc_card_omap3evt = {
    .name = "OMAP3 EDP",
    .platform = &omap_soc_platform,
    .dai_link = &omap3evt_dai,
    .num_links = 1,
};

static struct snd_soc_device omap3evt_snd_devdata = {
    .card = &snd_soc_card_omap3evt,
    .codec_dev = &soc_codec_dev_dac3100,
};

static struct platform_device *omap3evt_snd_device;

/*
 *----------------------------------------------------------------------------
 * Function : omap3evt_init
 * Purpose  : Machine Driver's Initialization routine.
 *----------------------------------------------------------------------------
 */
static int __init omap3evt_init(void)
{
	int ret = 0;
	struct device *dev;

	pr_debug("omap3epd-sound: Audio SoC init\n");
	omap3evt_snd_device = platform_device_alloc("soc-audio", -1);

	if (!omap3evt_snd_device)
		return -ENOMEM;

	platform_set_drvdata(omap3evt_snd_device, &omap3evt_snd_devdata);
	omap3evt_snd_devdata.dev = &omap3evt_snd_device->dev;

	dev = &omap3evt_snd_device->dev;

	/* Set McBSP2 as audio McBSP */
	*(unsigned int *)omap3evt_dai.cpu_dai->private_data = MCBSP2_ID; /* McBSP2 */

	ret = platform_device_add(omap3evt_snd_device);
	if (ret)
		goto err1;


	clkout2_src_ck = clk_get(dev, "clkout2_src_ck");
	if (IS_ERR(clkout2_src_ck)) {
		dev_err(dev, "Could not get clkout2_src_ck\n");
		ret = PTR_ERR(clkout2_src_ck);
		goto err2;
	}

	sys_clkout2 = clk_get(dev, "sys_clkout2");
	if (IS_ERR(sys_clkout2)) {
		dev_err(dev, "Could not get sys_clkout2\n");
		ret = PTR_ERR(sys_clkout2);
		goto err3;
	}

	sys_ck = clk_get(dev, "sys_ck");
	if (IS_ERR(sys_ck)) {
		dev_err(dev, "Could not get sys_ck\n");
		ret = PTR_ERR(sys_clkout2);
		goto err4;
	}

	ret = clk_set_parent(clkout2_src_ck, sys_ck);
	if (ret) {
		dev_err(dev, "Could not set clkout2_src_ck's parent to sys_ck\n");
		goto err5;
	}

	ret = clk_set_parent(sys_clkout2, clkout2_src_ck);
	if (ret) {
		dev_err(dev, "Could not set sys_clkout2's parent to clkout2_src_ck\n");
		goto err5;
	}

	ret = clk_set_rate(sys_clkout2, CODEC_SYSCLK_FREQ);
	if (ret) {
		dev_err(dev, "Could not set sys_clkout2 rate to %lu\n",
							CODEC_SYSCLK_FREQ);
		goto err5;
	}

	gpt11_fclk = clk_get(dev, "gpt11_fck");
	if (IS_ERR(gpt11_fclk)) {
		dev_err(dev, "Could not get gpt11_fclk\n");
		ret = PTR_ERR(gpt11_fclk);
		goto err6;
	}
		
	ret=clk_set_parent(gpt11_fclk, sys_ck);
	if (ret) {
		dev_err(dev, "Could not set sys_clkout2's parent to gpt11_fclk\n");
		goto err6;
	}

	wake_lock_init(&omap3edp_wakelock, WAKE_LOCK_SUSPEND, "omap3-dac3100");

	dev_dbg(dev, "sys_ck = %lu\n", clk_get_rate(sys_ck));
	dev_dbg(dev, "clkout2_src_ck = %lu\n", clk_get_rate(clkout2_src_ck));
	dev_dbg(dev, "sys_clkout2 = %lu\n", clk_get_rate(sys_clkout2));

	return 0;

err6:
	clk_put(gpt11_fclk);
err5:
	clk_put(sys_ck);
err4:
	clk_put(sys_clkout2);
err3:
	clk_put(clkout2_src_ck);
err2:
	platform_device_del(omap3evt_snd_device);
err1:
	platform_device_put(omap3evt_snd_device);

	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : omap3evt_exit
 * Purpose  : Machine Driver's Shutdown routine.
 *----------------------------------------------------------------------------
 */
static void __exit omap3evt_exit(void)
{
	clk_put(clkout2_src_ck);
	clk_put(sys_clkout2);
	clk_put(sys_ck);

  	clk_put(gpt11_fclk);

	wake_lock_destroy(&omap3edp_wakelock);

	platform_device_unregister(omap3evt_snd_device);
}

module_init(omap3evt_init);
module_exit(omap3evt_exit);

MODULE_AUTHOR("Sandeep S Prabhu<sandeepsp@mistralsolutions.com>");
MODULE_DESCRIPTION("ALSA SoC OMAP3530 EVM");
MODULE_LICENSE("GPL");
