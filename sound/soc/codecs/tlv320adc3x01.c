/*
 * linux/sound/soc/codecs/tlv320aic325x.c
 *
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
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
 * Rev 0.1   ASoC driver support			31-04-2009
 * The AIC32 ASoC driver is ported for the codec AIC325x.
 *
 *
 * Rev 1.0   Mini DSP support				11-05-2009
 * Added mini DSP programming support
 *
 * Rev 1.1   Mixer controls				18-01-2011
 * Added all the possible mixer controls.
 *
 * Rev 1.2   Additional Codec driver support		2-02-2011
 * Support for AIC3253, AIC3206, AIC3256
 *
 * Rev 2.0   Ported the Codec driver to 2.6.39 kernel	30-03-2012
 *
 * Rev 2.1   PLL DAPM support added to the codec driver	03-04-2012
 *
 * Rev 2.2   Added event handlers for DAPM widgets	16-05-2012
 *	     Updated ENUM declerations
 *
 */

/*
 * Includes
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/mutex.h>

#include <linux/slab.h>
#include <linux/firmware.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/jack.h>

#include "tlv320aic325x.h"
#include <linux/mfd/tlv320aic3xxx-registers.h>
#include <linux/mfd/tlv320aic3256-registers.h>
#include <linux/mfd/tlv320aic3xxx-core.h>


/*
 * enable debug prints in the driver
 */
#define DBG
#ifdef DBG
	#define dprintk(x...)   printk(x)
#else
	#define dprintk(x...)
#endif

/* User defined Macros kcontrol builders */
#define SOC_SINGLE_AIC325x(xname)                                       \
	{                                                               \
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,     \
		.info = __new_control_info, .get = __new_control_get, \
		.put = __new_control_put,                       \
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,      \
	}


/*
* Function Prototype
*/
static int aic325x_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *);
static int aic325x_mute(struct snd_soc_dai *dai, int mute);
static int aic325x_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt);
static int aic325x_set_bias_level(struct snd_soc_codec *codec,
					enum snd_soc_bias_level level);
static int aic325x_dai_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
				unsigned int Fin, unsigned int Fout);
static int adc3x01_set_tristate(struct snd_soc_dai *dai, int tristate);
static int adc3x01_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
	unsigned int rx_mask, int slots, int slot_width);

static int adc3x01_prepare(struct snd_pcm_substream *stream, struct snd_soc_dai *dai);
static unsigned int aic325x_codec_read(struct snd_soc_codec *codec,
			unsigned int reg);

static int aic325x_codec_write(struct snd_soc_codec *codec, unsigned int reg,
			unsigned int value);


#define CONFIG_MFD_ADC3X01	1

#ifndef CONFIG_MFD_ADC3X01
static int aic325x_hp_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event);
static int aic3256_cp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event);
#endif
static int aic3256_get_runstate(struct snd_soc_codec *codec);
static int aic3256_dsp_pwrdwn_status(struct snd_soc_codec *codec);
static int aic3256_dsp_pwrup(struct snd_soc_codec *codec, int state);
static int aic3256_restart_dsps_sync(struct snd_soc_codec *codec, int rs);

static inline unsigned int dsp_non_sync_mode(unsigned int state)
			{ return (!((state & 0x03) && (state & 0x30))); }

static void aic3256_firmware_load(const struct firmware *fw, void *context);
/*
* Global Variable
*/

/* whenever aplay/arecord is run, aic325x_hw_params() function gets called.
 * This function reprograms the clock dividers etc. this flag can be used to
 * disable this when the clock dividers are programmed by pps config file
 */


static const char * const mute[] = { "Unmute", "Mute" };

/* DAC Volume Soft Step Control */
static const char * const dacsoftstep_control[] = { "1 step/sample",
						"1 step/2 sample",
						"disabled" };
SOC_ENUM_SINGLE_DECL(dac_vol_soft_setp_enum, AIC3256_DAC_CHN_REG, 0,
						dacsoftstep_control);

/* Volume Mode Selection Control */
static const char * const volume_extra[] = { "L&R Ind Vol", "LVol=RVol",
								"RVol=LVol" };
/* DAC Volume Mode Selection */
SOC_ENUM_SINGLE_DECL(dac_vol_extra_enum, AIC3256_DAC_MUTE_CTRL_REG, 0,
						volume_extra);

/* Beep Master Volume Control */
SOC_ENUM_SINGLE_DECL(beep_master_vol_enum, AIC3256_BEEP_CTRL_REG2, 6,
						volume_extra);

/* Headset Detection Enable/Disable Control */
static const char * const headset_detection[] = { "Enabled", "Disabled" };
SOC_ENUM_SINGLE_DECL(hs_det_ctrl_enum, AIC3256_HEADSET_DETECT, 7,
						headset_detection);

#ifndef CONFIG_MFD_ADC3X01
/* MIC BIAS Voltage Control */
static const char * const micbias_voltage[] = { "1.04/1.25V", "1.425/1.7V",
						"2.075/2.5V", "POWER SUPPY" };
SOC_ENUM_SINGLE_DECL(micbias_voltage_enum, AIC3256_MICBIAS_CTRL, 4,
						micbias_voltage);
#endif
#ifdef CONFIG_MFD_ADC3X01
/* MIC BIAS Voltage Control */
static const char * const micbias_voltage[] = { "Power Down", "2V",
						"2.5V", "AVDD" };
SOC_ENUM_SINGLE_DECL(micbias1_voltage_enum, AIC3256_MICBIAS_CTRL, 5,
						micbias_voltage);
SOC_ENUM_SINGLE_DECL(micbias2_voltage_enum, AIC3256_MICBIAS_CTRL, 3,
						micbias_voltage);
#endif

/* IN1L to Left MICPGA Positive Terminal Selection */
static const char * const micpga_selection[] = { "off", "10k", "20k", "40k" };
SOC_ENUM_SINGLE_DECL(IN1L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 6,
						micpga_selection);

/* Left HP Driver Mute Control */
SOC_ENUM_SINGLE_DECL(left_hp_mute_enum, AIC3256_HPL_GAIN, 6, mute);

/* Right HP Driver Mute Control */
SOC_ENUM_SINGLE_DECL(right_hp_mute_enum, AIC3256_HPR_GAIN, 6, mute);

/* IN2L to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN2L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 4,
						micpga_selection);

/* IN3L to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN3L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 4,
						micpga_selection);

/* IN1R to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN1R_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 2,
						micpga_selection);

/* CM1L to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(CM1L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 0,
						micpga_selection);

/* IN2R to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN2R_LMICPGA_P_sel_enum, AIC3256_LMICPGA_NIN_CFG, 6,
						micpga_selection);

/* IN3R to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN3R_LMICPGA_P_sel_enum, AIC3256_LMICPGA_NIN_CFG, 4,
						micpga_selection);

/*CM2L to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(CM2L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_NIN_CFG, 2,
						micpga_selection);

/* IN1R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in1r_rmicpga_enum, AIC3256_RMICPGA_PIN_CFG, 6,
						micpga_selection);

/* IN2R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in2r_rmicpga_enum, AIC3256_RMICPGA_PIN_CFG, 4,
						micpga_selection);

/* IN3R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in3r_rmicpga_enum, AIC3256_RMICPGA_PIN_CFG, 2,
						micpga_selection);

/* IN2L to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in2l_rmicpga_enum, AIC3256_RMICPGA_PIN_CFG, 0,
						micpga_selection);

/* CM1R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(cm1r_rmicpga_enum, AIC3256_RMICPGA_NIN_CFG, 6,
						micpga_selection);

/* IN1L to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in1l_rmicpga_enum, AIC3256_RMICPGA_NIN_CFG, 4,
						micpga_selection);

/* IN3L to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in3l_rmicpga_enum, AIC3256_RMICPGA_NIN_CFG, 2,
						micpga_selection);

/* CM2R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(cm2r_rmicpga_enum, AIC3256_RMICPGA_NIN_CFG, 0,
						micpga_selection);

/* Power up/down */
static const char * const powerup[] = { "Power Down", "Power Up" };

/* Mic Bias Power up/down */
SOC_ENUM_SINGLE_DECL(micbias_pwr_ctrl_enum, AIC3256_MICBIAS_CTRL, 6, powerup);
/* Power up/down */
static const char * const wclk_source[] = { "DAC_FS", "ADC_FS", "Secondary","Reserved" };

/* Mic Bias Power up/down */
SOC_ENUM_SINGLE_DECL(wclk_source_ctrl_enum, AIC3256_AIS_REG_6, 4, wclk_source);

/* Left DAC Power Control */
SOC_ENUM_SINGLE_DECL(ldac_power_enum, AIC3256_DAC_CHN_REG, 7, powerup);

/* Right DAC Power Control */
SOC_ENUM_SINGLE_DECL(rdac_power_enum, AIC3256_DAC_CHN_REG, 6, powerup);

/* Left ADC Power Control */
SOC_ENUM_SINGLE_DECL(ladc_pwr_ctrl_enum, AIC3256_ADC_CHN_REG, 7, powerup);
/* Right ADC Power Control */
SOC_ENUM_SINGLE_DECL(radc_pwr_ctrl_enum, AIC3256_ADC_CHN_REG, 6, powerup);

#ifndef CONFIG_MFD_ADC3X01
/* HeadPhone Driver Power Control */
SOC_ENUM_DOUBLE_DECL(hp_pwr_ctrl_enum, AIC3256_OUT_PWR_CTRL, 5, 4, powerup);

/*Line-Out Driver Power Control */
SOC_ENUM_DOUBLE_DECL(lineout_pwr_ctrl_enum, AIC3256_OUT_PWR_CTRL, 3, 2,
						powerup);

/* Mixer Amplifiers Power Control */
SOC_ENUM_DOUBLE_DECL(mixer_amp_pwr_ctrl_enum, AIC3256_OUT_PWR_CTRL, 1, 0,
						powerup);

/* Mic Bias Generation */
static const char * const vol_generation[] = { "AVDD", "LDOIN" };
SOC_ENUM_SINGLE_DECL(micbias_voltage_ctrl_enum, AIC3256_MICBIAS_CTRL, 3,
						vol_generation);

/* DAC Data Path Control */
static const char * const path_control[] = { "Disabled", "LDAC Data",
						"RDAC Data", "L&RDAC Data" };
/* Left DAC Data Path Control */
SOC_ENUM_SINGLE_DECL(ldac_data_path_ctrl_enum, AIC3256_DAC_CHN_REG, 4,
						path_control);

/* Right DAC Data Path Control */
SOC_ENUM_SINGLE_DECL(rdac_data_path_ctrl_enum, AIC3256_DAC_CHN_REG, 2,
						path_control);

/* Audio gain control (AGC) Enable/Disable Control */
static const char * const disable_enable[] = { "Disabled", "Enabled" };

/* Left Audio gain control (AGC) Enable/Disable Control */
SOC_ENUM_SINGLE_DECL(left_agc_enable_disable_enum, AIC3256_LEFT_AGC_REG1, 7,
						disable_enable);

/* Left/Right Audio gain control (AGC) Enable/Disable Control */
SOC_ENUM_SINGLE_DECL(right_agc_enable_disable_enum, AIC3256_RIGHT_AGC_REG1, 7,
						disable_enable);

/* Left MICPGA Gain Enabled/Disable */
SOC_ENUM_SINGLE_DECL(left_micpga_ctrl_enum, AIC3256_LMICPGA_VOL_CTRL, 7,
						disable_enable);

/* Right MICPGA Gain Enabled/Disable */
SOC_ENUM_SINGLE_DECL(right_micpga_ctrl_enum, AIC3256_RMICPGA_VOL_CTRL, 7,
						disable_enable);

/* DRC Enable/Disable Control */
SOC_ENUM_DOUBLE_DECL(drc_ctrl_enum, AIC3256_DRC_CTRL_REG1, 6, 5,
						disable_enable);

/* Beep generator Enable/Disable control */
SOC_ENUM_SINGLE_DECL(beep_gen_ctrl_enum, AIC3256_BEEP_CTRL_REG1, 7,
						disable_enable);

/* Headphone ground centered mode enable/disable control */
SOC_ENUM_SINGLE_DECL(hp_gnd_centred_mode_ctrl, AIC3256_HP_DRIVER_CONF_REG, 4,
						disable_enable);
SOC_ENUM_SINGLE_DECL(audio_loopback_enum, AIC3256_INTERFACE_SET_REG_3, 5, disable_enable);

#endif
/* Audio loopback enable/disable control */

#ifndef CONFIG_MFD_ADC3X01
/* DMIC intput Selection control */
static const char * const dmic_input_sel[] = { "GPIO", "SCLK", "DIN" };
SOC_ENUM_SINGLE_DECL(dmic_input_enum, AIC3256_ADC_CHN_REG, 4, dmic_input_sel);
#else
/* DMIC intput Selection control */
static const char * const dmic_input_sel[] = { "DMDIN", "DMDCLK" };
SOC_ENUM_SINGLE_DECL(ldmic_input_enum, AIC3256_ADC_CHN_REG, 5, dmic_input_sel);
SOC_ENUM_SINGLE_DECL(rdmic_input_enum, AIC3256_ADC_CHN_REG, 4, dmic_input_sel);
#endif
/*charge pump Enable*/
static const char * const charge_pump_ctrl_enum[] = { "Power_Down",
							"Reserved",
							"Power_Up" };
SOC_ENUM_SINGLE_DECL(charge_pump_ctrl, AIC3256_POW_CFG, 0,
						charge_pump_ctrl_enum);

static const char *asibclk_text[] = {
        "DAC_CLK",
        "DAC_MOD_CLK",
        "ADC_CLK",
        "ADC_MOD_CLK",
};
SOC_ENUM_SINGLE_DECL(asibclk_enum, AIC3256_INTERFACE_SET_REG_3, 0, asibclk_text);
static const struct snd_kcontrol_new asibclk_control =
SOC_DAPM_ENUM("ASI_BCLK Route", asibclk_enum);

/* DAC volume DB scale */
static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -6350, 50, 0);
/* ADC volume DB scale */
static const DECLARE_TLV_DB_SCALE(adc_vol_tlv, -1200, 50, 0);
/* Output Gain in DB scale */
static const DECLARE_TLV_DB_SCALE(output_gain_tlv, -600, 100, 0);
/* MicPGA Gain in DB */
static const DECLARE_TLV_DB_SCALE(micpga_gain_tlv, 0, 50, 0);

/* Various Controls For AIC325x */
static const struct snd_kcontrol_new aic325x_snd_controls[] = {
#ifndef CONFIG_MFD_ADC3X01
	/* IN1L to HPL Volume Control */
	SOC_SINGLE("IN1L to HPL volume control", AIC3256_IN1L_HPL_CTRL,
						0, 0x72, 1),

	SOC_ENUM("Left HP driver mute", left_hp_mute_enum),
	SOC_ENUM("Right HP driver mute", right_hp_mute_enum),	

	/* IN1R to HPR Volume Control */
	SOC_SINGLE("IN1R to HPR volume control", AIC3256_IN1R_HPR_CTRL,
						0, 0x72, 1),

	/* IN1L to HPL routing */
	SOC_SINGLE("IN1L to HPL Route", AIC3256_HPL_ROUTE_CTRL, 2, 1, 0),

	/* MAL output to HPL */
	SOC_SINGLE("MAL Output to HPL Route", AIC3256_HPL_ROUTE_CTRL, 1, 1, 0),

	/*MAR output to HPL */
	SOC_SINGLE("MAR Output to HPL Route", AIC3256_HPL_ROUTE_CTRL, 0, 1, 0),

	/* IN1R to HPR routing */
	SOC_SINGLE("IN1R to HPR Route", AIC3256_HPR_ROUTE_CTRL, 2, 1, 0),

	/* MAR to HPR routing */
	SOC_SINGLE("MAR Output to HPR Route", AIC3256_HPR_ROUTE_CTRL, 1, 1, 0),

	/* HPL Output to HRP routing */
	SOC_SINGLE("HPL Output to HPR Route", AIC3256_HPR_ROUTE_CTRL, 0, 1, 0),

	/* MAL Output to LOL routing*/
	SOC_SINGLE("MAL Output to LOL Route", AIC3256_LOL_ROUTE_CTRL, 1, 1, 0),

	/* LOR Output to LOL routing*/
	SOC_SINGLE("LOR Output to LOL Route", AIC3256_LOL_ROUTE_CTRL, 0, 1, 0),

	/* MAR Output to LOR routing*/
	SOC_SINGLE("MAR Outout to LOR Route", AIC3256_LOR_ROUTE_CTRL, 1, 1, 0),

	/* DRC Threshold value Control */
	SOC_SINGLE("DRC Threshold value",
					AIC3256_DRC_CTRL_REG1, 2, 0x07, 0),

	/* DRC Hysteresis value control */
	SOC_SINGLE("DRC Hysteresis value",
					AIC3256_DRC_CTRL_REG1, 0, 0x03, 0),

	/* DRC Hold time control */
	SOC_SINGLE("DRC hold time", AIC3256_DRC_CTRL_REG2, 3, 0x0F, 0),

	/* DRC Attack rate control */
	SOC_SINGLE("DRC attack rate", AIC3256_DRC_CTRL_REG3, 4, 0x0F, 0),

	/* DRC Decay rate control */
	SOC_SINGLE("DRC decay rate", AIC3256_DRC_CTRL_REG3, 0, 0x0F, 0),

	/* Beep Length MSB control */
	SOC_SINGLE("Beep Length MSB", AIC3256_BEEP_CTRL_REG3, 0, 255, 0),

	/* Beep Length MID control */
	SOC_SINGLE("Beep Length MID", AIC3256_BEEP_CTRL_REG4, 0, 255, 0),

	/* Beep Length LSB control */
	SOC_SINGLE("Beep Length LSB", AIC3256_BEEP_CTRL_REG5, 0, 255, 0),

	/* Beep Sin(x) MSB control */
	SOC_SINGLE("Beep Sin(x) MSB", AIC3256_BEEP_CTRL_REG6, 0, 255, 0),
	/* Beep Sin(x) LSB control */
	SOC_SINGLE("Beep Sin(x) LSB", AIC3256_BEEP_CTRL_REG7, 0, 255, 0),

	/* Beep Cos(x) MSB control */
	SOC_SINGLE("Beep Cos(x) MSB", AIC3256_BEEP_CTRL_REG8, 0, 255, 0),

	/* Beep Cos(x) LSB control */
	SOC_SINGLE("Beep Cos(x) LSB", AIC3256_BEEP_CTRL_REG9, 0, 255, 0),


	/* Left/Right DAC Digital Volume Control */
	SOC_DOUBLE_R_SX_TLV("DAC Digital Volume Control",
			AIC3256_LDAC_VOL, AIC3256_RDAC_VOL, 8, 0xffffff81, 0x30,
			dac_vol_tlv),
#endif

	/* Left/Right ADC Fine Gain Adjust */
	SOC_DOUBLE("L&R ADC Fine Gain Adjust", AIC3256_ADC_FGA, 4, 0, 0x04, 0),

	/* Left/Right ADC Volume Control */
	SOC_DOUBLE_R_SX_TLV("ADC Digital Volume Control",
		AIC3256_LADC_VOL, AIC3256_RADC_VOL, 7, 0xffffff68, 0x28 ,
						adc_vol_tlv),

#ifndef CONFIG_MFD_ADC3X01
	/*HP Driver Gain Control*/
	SOC_DOUBLE_R_SX_TLV("HP Driver Gain", AIC3256_HPL_GAIN,
					AIC3256_HPR_GAIN, 6, 0xfffffffa,
					0xe, output_gain_tlv),

	/*LO Driver Gain Control*/
	SOC_DOUBLE_R_SX_TLV("Line Driver Gain", AIC3256_LOL_GAIN,
					AIC3256_LOR_GAIN, 6,
					0xfffffffa, 0x1d , output_gain_tlv),


	/* Mixer Amplifier Volume Control */
	SOC_DOUBLE_R("Mixer_Amp_Vol_Ctrl",
			AIC3256_MAL_CTRL_REG, AIC3256_MAR_CTRL_REG,
			0, 0x28, 1),

	/* Beep generator Volume Control */
	SOC_DOUBLE_R("Beep_gen_Vol_Ctrl",
			AIC3256_BEEP_CTRL_REG1, AIC3256_BEEP_CTRL_REG2,
			0, 0x3F, 1),
	/* DAC Signal Processing Block Control*/
	SOC_SINGLE("DAC PRB Selection(1 to 25)", AIC3256_DAC_PRB, 0, 0x19, 0),
#endif
	/*Left/Right MICPGA Volume Control */
	SOC_DOUBLE_R_TLV("MICPGA Volume Control",
	AIC3256_LMICPGA_VOL_CTRL, AIC3256_RMICPGA_VOL_CTRL, 0, 0x5F,
			0, micpga_gain_tlv),


	/* Left/Right AGC Target level control */
	SOC_DOUBLE_R("AGC Target Level Control",
			AIC3256_LEFT_AGC_REG1, AIC3256_RIGHT_AGC_REG1,
			4, 0x07, 1),

	/* Left/Right AGC Hysteresis Control */
	SOC_DOUBLE_R("AGC Hysteresis Control",
			AIC3256_LEFT_AGC_REG1, AIC3256_RIGHT_AGC_REG1,
			0, 0x03, 0),

	/*Left/Right AGC Maximum PGA applicable */
	SOC_DOUBLE_R("AGC Maximum PGA Control",
			AIC3256_LEFT_AGC_REG3, AIC3256_RIGHT_AGC_REG3,
			0, 0x7F, 0),

	/* Left/Right AGC Noise Threshold */
	SOC_DOUBLE_R("AGC Noise Threshold",
			AIC3256_LEFT_AGC_REG2, AIC3256_RIGHT_AGC_REG2,
			1, 0x1F, 1),

	/* Left/Right AGC Attack Time control */
	SOC_DOUBLE_R("AGC Attack Time control",
			AIC3256_LEFT_AGC_REG4, AIC3256_RIGHT_AGC_REG4,
			3, 0x1F, 0),

	/* Left/Right AGC Decay Time control */
	SOC_DOUBLE_R("AGC Decay Time control",
			AIC3256_LEFT_AGC_REG5, AIC3256_RIGHT_AGC_REG5,
			3, 0x1F, 0),

	/* Left/Right AGC Noise Debounce control */
	SOC_DOUBLE_R("AGC Noice bounce control",
			AIC3256_LEFT_AGC_REG6, AIC3256_RIGHT_AGC_REG6,
			0, 0x1F, 0),

	/* Left/Right AGC Signal Debounce control */
	SOC_DOUBLE_R("AGC_Signal bounce ctrl",
		AIC3256_LEFT_AGC_REG7, AIC3256_RIGHT_AGC_REG7, 0, 0x0F, 0),

	/* ADC Signal Processing Block Control */
	SOC_SINGLE("ADC PRB Selection(1 to 18)", AIC3256_ADC_PRB, 0, 0x12, 0),

#ifndef CONFIG_MFD_ADC3X01
	/*charge pump configuration for n/8 peak load current*/
	SOC_SINGLE("Charge_pump_peak_load_conf",
				AIC3256_CHRG_CTRL_REG, 4, 8, 0),

	/*charge pump clock divide control*/
	SOC_SINGLE("charge_pump_clk_divider_ctrl", AIC3256_CHRG_CTRL_REG,
			0, 16, 0),

	/*HPL, HPR master gain control in ground centerd mode */
	SOC_SINGLE("HP_gain_ctrl_gnd_centered_mode",
				AIC3256_HP_DRIVER_CONF_REG, 5, 3, 0),

	/*headphone amplifier compensation adjustment */
	SOC_SINGLE(" hp_amp_compensation_adjustment",
				AIC3256_HP_DRIVER_CONF_REG, 7, 1, 0),

	/*headphone driver power configuration*/
	SOC_SINGLE(" HP_drv_pwr_conf",
				AIC3256_HP_DRIVER_CONF_REG, 2, 4, 0),

	/*DC offset correction*/
	SOC_SINGLE("DC offset correction", AIC3256_HP_DRIVER_CONF_REG, 0, 4, 0),


	SOC_ENUM("WCLK Source", wclk_source_ctrl_enum),
	SOC_ENUM("Mic_Bias_Power_ctrl", micbias_pwr_ctrl_enum),
#endif

#ifdef CONFIG_MFD_ADC3X01
	SOC_ENUM("Mic Bias1 Voltage", micbias1_voltage_enum),
	SOC_ENUM("Mic Bias2 Voltage", micbias2_voltage_enum),
	SOC_ENUM("Left DMIC Input Pin",ldmic_input_enum),
	SOC_ENUM("Right DMIC Input Pin",rdmic_input_enum),
#endif

};

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_dai_ops |
 *          It is SoC Codec DAI Operations structure
 *----------------------------------------------------------------------------
 */
struct snd_soc_dai_ops aic325x_dai_ops = {
	.hw_params = aic325x_hw_params,
	.digital_mute = aic325x_mute,
/*	.set_sysclk = aic325x_set_dai_sysclk, */
	.set_fmt = aic325x_set_dai_fmt,
	.set_pll = aic325x_dai_set_pll,
	.set_tdm_slot = adc3x01_set_tdm_slot,	
	.set_tristate = adc3x01_set_tristate,
	.prepare = adc3x01_prepare,
};

/*
 * It is SoC Codec DAI structure which has DAI capabilities viz., playback
 * and capture, DAI runtime information viz. state of DAI and pop wait state,
 * and DAI private data. The aic31xx rates ranges from 8k to 192k The PCM
 * bit format supported are 16, 20, 24 and 32 bits
 */


static struct snd_soc_dai_driver tlv320aic325x_dai_driver[] = {
	{
	.name = "tlv320aic325x-MM_EXT",
	.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIC325x_RATES,
			.formats = AIC325x_FORMATS,
		},
	.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIC325x_RATES,
			.formats = AIC325x_FORMATS,
		},
	.ops = &aic325x_dai_ops,
},

};
static struct snd_soc_dai_driver tlv320adc3x01_dai_driver[] = {
	{
	.name = "tlv320adc3x01-asi",
	.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 4,
			.rates = AIC325x_RATES,
			.formats = AIC325x_FORMATS,
		},
	.ops = &aic325x_dai_ops,
},

};

/* Left HPL Mixer */
static const struct snd_kcontrol_new hpl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", AIC3256_HPL_ROUTE_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("IN1_L switch", AIC3256_HPL_ROUTE_CTRL, 2, 1, 0),
	SOC_DAPM_SINGLE("MAL switch", AIC3256_HPL_ROUTE_CTRL, 1, 1, 0),
	SOC_DAPM_SINGLE("MAR switch", AIC3256_HPL_ROUTE_CTRL, 0, 1, 0),
};

static const char * const adc_mux_text[] = {
	"Analog", "Digital",
};

SOC_ENUM_SINGLE_DECL(adcl_enum, AIC3256_ADC_CHN_REG, 3, adc_mux_text);
SOC_ENUM_SINGLE_DECL(adcr_enum, AIC3256_ADC_CHN_REG, 2, adc_mux_text);

static const struct snd_kcontrol_new adcl_mux =
	SOC_DAPM_ENUM("Left ADC Route", adcl_enum);

static const struct snd_kcontrol_new adcr_mux =
	SOC_DAPM_ENUM("Right ADC Route", adcr_enum);

/* Right HPR Mixer */
static const struct snd_kcontrol_new hpr_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", AIC3256_HPR_ROUTE_CTRL, 4, 1, 0),
	SOC_DAPM_SINGLE("R_DAC switch", AIC3256_HPR_ROUTE_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("IN1_R switch", AIC3256_HPR_ROUTE_CTRL, 2, 1, 0),
	SOC_DAPM_SINGLE("MAR switch", AIC3256_HPR_ROUTE_CTRL, 1, 1, 0),
};

/* Left Line out mixer */
static const struct snd_kcontrol_new lol_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", AIC3256_LOL_ROUTE_CTRL, 4, 1, 0),
	SOC_DAPM_SINGLE("L_DAC switch", AIC3256_LOL_ROUTE_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("MAL switch", AIC3256_LOL_ROUTE_CTRL, 1, 1, 0),
	SOC_DAPM_SINGLE("LOR switch", AIC3256_LOL_ROUTE_CTRL, 0, 1, 0),
};
/* Right Line out Mixer */
static const struct snd_kcontrol_new lor_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", AIC3256_LOR_ROUTE_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("MAR switch", AIC3256_LOR_ROUTE_CTRL, 1, 1, 0),
};

/* AIC325X Left Input Mixer */
static const struct snd_kcontrol_new aic325x_left_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1_L switch", AIC3256_LMICPGA_PIN_CFG, 6, 3, 0),
	SOC_DAPM_SINGLE("IN2_L switch", AIC3256_LMICPGA_PIN_CFG, 4, 3, 0),
	SOC_DAPM_SINGLE("IN3_L switch", AIC3256_LMICPGA_PIN_CFG, 2, 3, 0),
	SOC_DAPM_SINGLE("IN1_R switch", AIC3256_LMICPGA_PIN_CFG, 0, 3, 0),

	SOC_DAPM_SINGLE("CM1L switch", AIC3256_LMICPGA_NIN_CFG, 6, 3, 0),
	SOC_DAPM_SINGLE("IN2_R switch", AIC3256_LMICPGA_NIN_CFG, 4, 3, 0),
	SOC_DAPM_SINGLE("IN3_R switch", AIC3256_LMICPGA_NIN_CFG, 2, 3, 0),
	SOC_DAPM_SINGLE("CM2L switch", AIC3256_LMICPGA_NIN_CFG, 0, 3, 0),
};

/* AIC325X Right Input Mixer */
static const struct snd_kcontrol_new aic325x_right_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1_R switch", AIC3256_RMICPGA_PIN_CFG, 6, 3, 0),
	SOC_DAPM_SINGLE("IN2_R switch", AIC3256_RMICPGA_PIN_CFG, 4, 3, 0),
	SOC_DAPM_SINGLE("IN3_R switch", AIC3256_RMICPGA_PIN_CFG, 2, 3, 0),
	SOC_DAPM_SINGLE("IN2_L switch", AIC3256_RMICPGA_PIN_CFG, 0, 3, 0),
	SOC_DAPM_SINGLE("CM1R switch", AIC3256_RMICPGA_NIN_CFG, 6, 3, 0),
	SOC_DAPM_SINGLE("IN1_L switch", AIC3256_RMICPGA_NIN_CFG, 4, 3, 0),
	SOC_DAPM_SINGLE("IN3_L switch", AIC3256_RMICPGA_NIN_CFG, 2, 3, 0),
	SOC_DAPM_SINGLE("CM2R switch", AIC3256_RMICPGA_NIN_CFG, 0, 3, 0),
};

/* ADC3X01 Left Input Mixer */
static const struct snd_kcontrol_new adc3x01_left_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1L P SE switch", AIC3256_LMICPGA_PIN_CFG, 0, 3, 1),
	SOC_DAPM_SINGLE("IN2L P SE switch", AIC3256_LMICPGA_PIN_CFG, 2, 3, 1),
	SOC_DAPM_SINGLE("IN3L M SE switch", AIC3256_LMICPGA_PIN_CFG, 4, 3, 1),
	SOC_DAPM_SINGLE("IN2L P IN3L M DE switch", AIC3256_LMICPGA_PIN_CFG, 6, 3, 1),

	SOC_DAPM_SINGLE("CM switch", AIC3256_LMICPGA_NIN_CFG, 6, 1, 0),
	SOC_DAPM_SINGLE("IN1L P IN1R M DE switch", AIC3256_LMICPGA_NIN_CFG, 4, 3, 1),
	SOC_DAPM_SINGLE("IN2R P IN3R M DE switch", AIC3256_LMICPGA_NIN_CFG, 2, 3, 1),
	SOC_DAPM_SINGLE("IN1R M SE switch", AIC3256_LMICPGA_NIN_CFG, 0, 3, 1),
};

/* ADC3X01 Right Input Mixer */
static const struct snd_kcontrol_new adc3x01_right_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN2R P IN3R M DE switch", AIC3256_RMICPGA_PIN_CFG, 6, 3, 1),
	SOC_DAPM_SINGLE("IN3R M SE switch", AIC3256_RMICPGA_PIN_CFG, 4, 3, 1),
	SOC_DAPM_SINGLE("IN2R P SE switch", AIC3256_RMICPGA_PIN_CFG, 2, 3, 1),
	SOC_DAPM_SINGLE("IN1R M SE switch", AIC3256_RMICPGA_PIN_CFG, 0, 3, 1),

	SOC_DAPM_SINGLE("CM switch", AIC3256_RMICPGA_NIN_CFG, 6, 1, 0),
	SOC_DAPM_SINGLE("IN1L P IN1R M switch", AIC3256_RMICPGA_NIN_CFG, 4, 3, 1),
	SOC_DAPM_SINGLE("IN2L P IN3L M switch", AIC3256_RMICPGA_NIN_CFG, 2, 3, 1),
	SOC_DAPM_SINGLE("IN1L P SE switch", AIC3256_RMICPGA_NIN_CFG, 0, 3, 1),
};

static const char *asilin_text[] = {
	"Off", "ASI Left In","ASI Right In","ASI MonoMix In"
};
SOC_ENUM_SINGLE_DECL(asilin_enum, AIC3256_DAC_CHN_REG, 4, asilin_text);
static const struct snd_kcontrol_new asilin_control =
SOC_DAPM_ENUM("ASIIn Left Route", asilin_enum);      

static const char *asirin_text[] = {
	"Off", "ASI Right In","ASI Left In","ASI MonoMix In"
};
SOC_ENUM_SINGLE_DECL(asirin_enum, AIC3256_DAC_CHN_REG, 2, asirin_text);
static const struct snd_kcontrol_new asirin_control =
SOC_DAPM_ENUM("ASIIn Right Route", asirin_enum);      

/**$
 * pll_power_on_event: provide delay after widget power up
 * @w: pointer variable to dapm_widget,
 * @kcontrolr: pointer variable to sound control,
 * @event:	integer to event,
 *
 * Return value: 0 for success
 */
static int pll_power_on_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	if (event == (SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD))
		mdelay(10);
	return 0;
}

#ifndef CONFIG_MFD_ADC3X01
/**
 *aic325x_dac_event: Headset popup reduction and powering up dsps together
 *			when they are in sync mode
 * @w: pointer variable to dapm_widget
 * @kcontrol: pointer to sound control
 * @event: event element information
 *
 * Returns 0 for success.
 */
static int aic325x_dac_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	int reg_mask = 0;
	int ret_wbits = 0;
	int run_state_mask;
	int sync_needed = 0, non_sync_state = 0;
	int other_dsp = 0, run_state = 0;
	struct aic325x_priv *aic325x = snd_soc_codec_get_drvdata(w->codec);

	if (w->shift == 7) {
		reg_mask = AIC3256_LDAC_POWER_STATUS_MASK ;
		run_state_mask = AIC3XXX_COPS_MDSP_D_L;
	}
	if (w->shift == 6) {
		reg_mask = AIC3256_RDAC_POWER_STATUS_MASK ;
		run_state_mask = AIC3XXX_COPS_MDSP_D_R ;
	}

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret_wbits = aic3xxx_wait_bits(w->codec->control_data,
						AIC3256_DAC_FLAG,
						reg_mask, reg_mask,
						AIC3256_TIME_DELAY,
						AIC3256_DELAY_COUNTER);
		sync_needed = aic3xxx_reg_read(w->codec->control_data,
						AIC3256_DAC_PRB);
			non_sync_state =
				dsp_non_sync_mode(aic325x->dsp_runstate);
			other_dsp =
				aic325x->dsp_runstate & AIC3XXX_COPS_MDSP_A;

		if (sync_needed && non_sync_state && other_dsp) {
			run_state =
				aic3256_get_runstate(
					aic325x->codec);
			aic3256_dsp_pwrdwn_status(aic325x->codec);
			aic3256_dsp_pwrup(aic325x->codec, run_state);
		}
		aic325x->dsp_runstate |= run_state_mask;
		if (!ret_wbits) {
			dev_err(w->codec->dev, "DAC_post_pmu timed out\n");
			return -1;
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret_wbits = aic3xxx_wait_bits(w->codec->control_data,
			AIC3256_DAC_FLAG, reg_mask, 0,
			AIC3256_TIME_DELAY, AIC3256_DELAY_COUNTER);
		aic325x->dsp_runstate =
			(aic325x->dsp_runstate & ~run_state_mask);
	if (!ret_wbits) {
		dev_err(w->codec->dev, "DAC_post_pmd timed out\n");
		return -1;
	}
		break;
	default:
		BUG();
		return -EINVAL;
	}
	return 0;
}

#endif
/**
 * aic325x_adc_event: To get DSP run state to perform synchronization
 * @w: pointer variable to dapm_widget
 * @kcontrol: pointer to sound control
 * @event: event element information
 *
 * Returns 0 for success.
 */
static int aic325x_adc_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec= /*w->codec;*/snd_soc_dapm_to_codec(w->dapm);
	int run_state = 0;
	int non_sync_state = 0, sync_needed = 0;
	int other_dsp = 0;
	int run_state_mask = 0;
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(/*w->*/codec);
	struct aic3xxx *aic3xxx = dev_get_drvdata(codec->dev->parent);
	int reg_mask = 0;
	int dig_mic_mask = 0;
	int ret_wbits = 0;

	if (w->shift == 7) {
		reg_mask = AIC3256_LADC_POWER_MASK;
		dig_mic_mask = AIC3256_LADC_DIGITAL_MASK; 
		run_state_mask = AIC3XXX_COPS_MDSP_A_L;
	}
	if (w->shift == 6) {
		reg_mask = AIC3256_RADC_POWER_MASK;
		dig_mic_mask = AIC3256_RADC_DIGITAL_MASK; 
		run_state_mask = AIC3XXX_COPS_MDSP_A_R;
	}

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret_wbits = aic3xxx_wait_bits(/*w->*/codec->control_data,
						AIC3256_ADC_FLAG , reg_mask,
						reg_mask, AIC3256_TIME_DELAY,
						AIC3256_DELAY_COUNTER);

		sync_needed = aic3xxx_reg_read(/*w->*/codec->control_data,
						AIC3256_DAC_PRB);
		non_sync_state = dsp_non_sync_mode(aic3256->dsp_runstate);
		other_dsp = aic3256->dsp_runstate & AIC3XXX_COPS_MDSP_D;
		if (sync_needed && non_sync_state && other_dsp) {
			run_state = aic3256_get_runstate(
						aic3256->codec);
			aic3256_dsp_pwrdwn_status(aic3256->codec);
			aic3256_dsp_pwrup(aic3256->codec, run_state);
		}
		aic3256->dsp_runstate |= run_state_mask;
		if (!ret_wbits) {
			dev_err(/*w->*/codec->dev, "ADC POST_PMU timedout\n");
			return -1;
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (aic3xxx->type == TLV320ADC3001) {
			snd_soc_update_bits(/*w->*/codec,AIC3256_ADC_CHN_REG,
				dig_mic_mask, dig_mic_mask); 
		}
		break;
		

	case SND_SOC_DAPM_POST_PMD:
		ret_wbits = aic3xxx_wait_bits(/*w->*/codec->control_data,
						AIC3256_ADC_FLAG, reg_mask, 0,
						AIC3256_TIME_DELAY,
						AIC3256_DELAY_COUNTER);
		aic3256->dsp_runstate = (aic3256->dsp_runstate &
					 ~run_state_mask);
		if (!ret_wbits) {
			dev_err(/*w->*/codec->dev, "ADC POST_PMD timedout\n");
			return -1;
		}
		if (aic3xxx->type == TLV320ADC3001) {
			snd_soc_update_bits(/*w->*/codec,AIC3256_ADC_CHN_REG,
				dig_mic_mask, 0); 
		}
		break;

	default:
		BUG();
		return -EINVAL;
	}
	return 0;
}

/* AIC325x Widget Structure */
static const struct snd_soc_dapm_widget aic325x_dapm_widgets[] = {
#ifndef CONFIG_MFD_ADC3X01
	SND_SOC_DAPM_AIF_IN("ASIIN", "ASI Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_PGA("ASILIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASIRIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASIMonoMixIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI IN Port", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("ASIIn Left Route",
			SND_SOC_NOPM, 0, 0, &asilin_control),       
	SND_SOC_DAPM_MUX("ASIIn Right Route",
			SND_SOC_NOPM, 0, 0, &asirin_control),       
	/* dapm widget (stream domain) for left DAC */
	SND_SOC_DAPM_DAC_E("Left DAC", NULL, AIC3256_DAC_CHN_REG,
			7, 0, aic325x_dac_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	/* dapm widget (stream domain) for right DAC */
	SND_SOC_DAPM_DAC_E("Right DAC", NULL, AIC3256_DAC_CHN_REG,
			6, 0, aic325x_dac_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	/* dapm widget (path domain) for left DAC_L Mixer */
	SND_SOC_DAPM_MIXER("HPL Output Mixer", SND_SOC_NOPM, 0, 0,
				&hpl_output_mixer_controls[0],
				ARRAY_SIZE(hpl_output_mixer_controls)),

	/* dapm widget (path domain) for right DAC_R mixer */
	SND_SOC_DAPM_MIXER("HPR Output Mixer", SND_SOC_NOPM, 0, 0,
	&hpr_output_mixer_controls[0], ARRAY_SIZE(hpr_output_mixer_controls)),

	/* dapm widget for Left Head phone Power */
	SND_SOC_DAPM_PGA_S("HPL PGA", 5,AIC3256_OUT_PWR_CTRL, 5, 0,
				aic325x_hp_event,
				NULL),

	/* dapm widget (path domain) for Left Line-out Output Mixer */
	SND_SOC_DAPM_MIXER("LOL Output Mixer", SND_SOC_NOPM, 0, 0,
	&lol_output_mixer_controls[0], ARRAY_SIZE(lol_output_mixer_controls)),

	/* dapm widget for Left Line-out Power */
	SND_SOC_DAPM_PGA_S("LOL PGA", 2,AIC3256_OUT_PWR_CTRL, 3, 0,
				NULL, 0),



	/* dapm widget for Right Head phone Power */
	SND_SOC_DAPM_PGA_S("HPR PGA", 5,AIC3256_OUT_PWR_CTRL, 4, 0,
				aic325x_hp_event, SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),

	/* dapm widget for (path domain) Right Line-out Output Mixer */
	SND_SOC_DAPM_MIXER("LOR Output Mixer", SND_SOC_NOPM, 0, 0,
			&lor_output_mixer_controls[0],
			ARRAY_SIZE(lor_output_mixer_controls)),

	/* dapm widget for Right Line-out Power */
	SND_SOC_DAPM_PGA_S("LOR PGA", 2,AIC3256_OUT_PWR_CTRL, 2, 0,
				NULL, 0),

	/* dapm supply widget for Charge pump */
	SND_SOC_DAPM_SUPPLY_S("Charge Pump", 4,AIC3256_POW_CFG, 1, 0, aic3256_cp_event,
						SND_SOC_DAPM_POST_PMU ),
        SND_SOC_DAPM_PGA("CM1L", SND_SOC_NOPM, 0, 0, NULL, 0),                                                                                               
        SND_SOC_DAPM_PGA("CM2L", SND_SOC_NOPM, 0, 0, NULL, 0),                                                                                               
        SND_SOC_DAPM_PGA("CM1R", SND_SOC_NOPM, 0, 0, NULL, 0),                                                                                               
        SND_SOC_DAPM_PGA("CM2R", SND_SOC_NOPM, 0, 0, NULL, 0), 
#endif
	/* Input DAPM widget for CM */
	 SND_SOC_DAPM_PGA("CM", SND_SOC_NOPM, 0, 0, NULL, 0),                                                                                                 

	/* Stream widget for Left ADC */
	SND_SOC_DAPM_ADC_E("Left ADC", "Left Capture", AIC3256_ADC_CHN_REG,
			7, 0, aic325x_adc_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),


	/* Stream widget for Right ADC */
	SND_SOC_DAPM_ADC_E("Right ADC", "Right Capture", AIC3256_ADC_CHN_REG,
			6, 0, aic325x_adc_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	/* Left Inputs to Left MicPGA */
	SND_SOC_DAPM_PGA_S("Left MicPGA", 0,AIC3256_LMICPGA_VOL_CTRL ,
			7, 1, NULL, 0),

	/* Right Inputs to Right MicPGA */
	SND_SOC_DAPM_PGA_S("Right MicPGA", 0,AIC3256_RMICPGA_VOL_CTRL,
			7, 1, NULL, 0),


#ifndef CONFIG_MFD_ADC3X01
	/* Left MicPGA to Mixer PGA Left */
	SND_SOC_DAPM_PGA_S("MAL PGA", 1,AIC3256_OUT_PWR_CTRL , 1, 0, NULL, 0),

	/* Right Inputs to Mixer PGA Right */
	SND_SOC_DAPM_PGA_S("MAR PGA", 1,AIC3256_OUT_PWR_CTRL, 0, 0, NULL, 0),
	/* dapm widget (platform domain) name for HPLOUT */
	SND_SOC_DAPM_OUTPUT("HPL"),

	/* dapm widget (platform domain) name for HPROUT */
	SND_SOC_DAPM_OUTPUT("HPR"),

	/* dapm widget (platform domain) name for LOLOUT */
	SND_SOC_DAPM_OUTPUT("LOL"),

	/* dapm widget (platform domain) name for LOROUT */
	SND_SOC_DAPM_OUTPUT("LOR"),
#endif

	/* dapm widget (platform domain) name for LINE1L */
	SND_SOC_DAPM_INPUT("IN1_L"),

	/* dapm widget (platform domain) name for LINE1R */
	SND_SOC_DAPM_INPUT("IN1_R"),

	/* dapm widget (platform domain) name for LINE2L */
	SND_SOC_DAPM_INPUT("IN2_L"),

	/* dapm widget (platform domain) name for LINE2R */
	SND_SOC_DAPM_INPUT("IN2_R"),

	/* dapm widget (platform domain) name for LINE3L */
	SND_SOC_DAPM_INPUT("IN3_L"),

	/* dapm widget (platform domain) name for LINE3R */
	SND_SOC_DAPM_INPUT("IN3_R"),


	/* Left DMIC Input Widget */
	SND_SOC_DAPM_INPUT("Left DMIC"),
	/* Right DMIC Input Widget */
	SND_SOC_DAPM_INPUT("Right DMIC"),

	/* Left Channel ADC Route */
	SND_SOC_DAPM_MUX("Left ADC Route", SND_SOC_NOPM, 0, 0, &adcl_mux),
	/* Right Channel ADC Route */
	SND_SOC_DAPM_MUX("Right ADC Route", SND_SOC_NOPM, 0, 0, &adcr_mux),

	/* Supply widget for PLL */
	SND_SOC_DAPM_SUPPLY_S("PLLCLK", 0,  AIC3256_CLK_REG_2, 7, 0,
			pll_power_on_event,
			SND_SOC_DAPM_POST_PMU),

	/* Supply widget for CODEC_CLK_IN */
	SND_SOC_DAPM_SUPPLY_S("CODEC_CLK_IN", 1,SND_SOC_NOPM, 0, 0, NULL, 0),
	/* Supply widget for NDAC divider */
	SND_SOC_DAPM_SUPPLY_S("NDAC_DIV", 2,AIC3256_NDAC_CLK_REG_6, 7, 0, NULL, 0),
	/* Supply widget for MDAC divider */
	SND_SOC_DAPM_SUPPLY_S("MDAC_DIV", 3,AIC3256_MDAC_CLK_REG_7, 7, 0, NULL, 0),
	/* Supply widget for NADC divider */
	SND_SOC_DAPM_SUPPLY_S("NADC_DIV", 2,AIC3256_NADC_CLK_REG_8, 7, 0, NULL, 0),
	/* Supply widget for MADC divider */
	SND_SOC_DAPM_SUPPLY_S("MADC_DIV", 3,AIC3256_MADC_CLK_REG_9, 7, 0, NULL, 0),
	/* Supply widget for Bit Clock divider */
	//SND_SOC_DAPM_SUPPLY("BCLK_N_DIV", AIC3256_CLK_REG_11, 7, 0, NULL, 0),
	SND_SOC_DAPM_MUX("ASI_BCLK Route",
                        SND_SOC_NOPM, 0, 0, &asibclk_control),
};

static const struct snd_soc_dapm_widget aic325x_input_mixer_widgets[] = {
	/* dapm widget for Left Input Mixer*/
	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
			&aic325x_left_input_mixer_controls[0],
			ARRAY_SIZE(aic325x_left_input_mixer_controls)),

	/* dapm widget for Right Input Mixer*/
	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
			&aic325x_right_input_mixer_controls[0],
			ARRAY_SIZE(aic325x_right_input_mixer_controls)),
	/* DAPM widget for MICBIAS power control */
	SND_SOC_DAPM_MICBIAS("Mic Bias", AIC3256_MICBIAS_CTRL, 6, 0),
};

static const struct snd_soc_dapm_widget adc3x01_input_mixer_widgets[] = {
	/* dapm widget for Left Input Mixer*/
	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
			&adc3x01_left_input_mixer_controls[0],
			ARRAY_SIZE(adc3x01_left_input_mixer_controls)),

	/* dapm widget for Right Input Mixer*/
	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
			&adc3x01_right_input_mixer_controls[0],
			ARRAY_SIZE(adc3x01_right_input_mixer_controls)),
	/* DAPM widget for MICBIAS power control */
	SND_SOC_DAPM_MICBIAS("MicBias1", AIC3256_MICBIAS_CTRL, 5, 0),
	SND_SOC_DAPM_MICBIAS("MicBias2", AIC3256_MICBIAS_CTRL, 3, 0),
};
static const  struct snd_soc_dapm_route aic325x_dapm_routes[] = {

	/* PLL routing */
	{"CODEC_CLK_IN", NULL, "PLLCLK"},
	{"NDAC_DIV", NULL, "CODEC_CLK_IN"},
	{"NADC_DIV", NULL, "CODEC_CLK_IN"},
	{"MDAC_DIV", NULL, "NDAC_DIV"},
	{"MADC_DIV", NULL, "NADC_DIV"},
//	{"BCLK_N_DIV", NULL, "MADC_DIV"},
//	{"BCLK_N_DIV", NULL, "MDAC_DIV"},

	{"ASI_BCLK Route","DAC_CLK","NDAC_DIV"},
        {"ASI_BCLK Route","DAC_MOD_CLK","MDAC_DIV"},
        {"ASI_BCLK Route","ADC_CLK","NADC_DIV"},
        {"ASI_BCLK Route","ADC_MOD_CLK","MADC_DIV"},

	{"Left ADC", NULL, "MDAC_DIV"},
	{"Right ADC", NULL, "MDAC_DIV"},
	/* Clock routing for ADC */
	{"Left ADC", NULL, "MADC_DIV"},
	{"Right ADC", NULL, "MADC_DIV"},

#ifndef CONFIG_MFD_ADC3X01
	/* Clock routing for DAC */
	{"Left DAC", NULL, "MDAC_DIV" },
	{"Right DAC", NULL, "MDAC_DIV"},

/* ASI routing */
	{"ASILIN", NULL, "ASIIN"},
	{"ASIRIN", NULL, "ASIIN"},
	{"ASIMonoMixIN", NULL, "ASIIN"},

	{"ASIIn Left Route","ASI Left In","ASILIN"},
	{"ASIIn Left Route","ASI Right In","ASIRIN"},
	{"ASIIn Left Route","ASI MonoMix In","ASIMonoMixIN"},

	{"ASIIn Right Route","ASI Left In","ASILIN"},
	{"ASIIn Right Route","ASI Right In","ASIRIN"},
	{"ASIIn Right Route","ASI MonoMix In","ASIMonoMixIN"},

	{"ASI IN Port", NULL, "ASIIn Left Route"},
	{"ASI IN Port", NULL, "ASIIn Right Route"},

	{"Left DAC", NULL, "ASI IN Port"},
	{"Right DAC", NULL, "ASI IN Port"},

	/* Left Headphone Output */
	{"HPL Output Mixer", "L_DAC switch", "Left DAC"},
	{"HPL Output Mixer", "IN1_L switch", "IN1_L"},
	{"HPL Output Mixer", "MAL switch", "MAL PGA"},
	{"HPL Output Mixer", "MAR switch", "MAR PGA"},

	/* Right Headphone Output */
	{"HPR Output Mixer", "R_DAC switch", "Right DAC"},
	{"HPR Output Mixer", "IN1_R switch", "IN1_R"},
	{"HPR Output Mixer", "MAR switch", "MAR PGA"},
	{"HPR Output Mixer", "L_DAC switch", "Left DAC"},

	/* HP output mixer to HP PGA */
	{"HPL PGA", NULL, "HPL Output Mixer"},
	{"HPR PGA", NULL, "HPR Output Mixer"},

	/* HP PGA to HP output pins */
	{"HPL", NULL, "HPL PGA"},
	{"HPR", NULL, "HPR PGA"},

	/* Charge pump to HP PGA */
	{"HPL PGA", NULL, "Charge Pump"},
	{"HPR PGA", NULL, "Charge Pump"},

	/* Left Line-out Output */
	{"LOL Output Mixer", "L_DAC switch", "Left DAC"},
	{"LOL Output Mixer", "MAL switch", "MAL PGA"},
	{"LOL Output Mixer", "R_DAC switch", "Right DAC"},
	{"LOL Output Mixer", "LOR switch", "LOR PGA"},

	/* Right Line-out Output */
	{"LOR Output Mixer", "R_DAC switch", "Right DAC"},
	{"LOR Output Mixer", "MAR switch", "MAR PGA"},

	{"LOL PGA", NULL, "LOL Output Mixer"},
	{"LOR PGA", NULL, "LOR Output Mixer"},

	{"LOL", NULL, "LOL PGA"},
	{"LOR", NULL, "LOR PGA"},
#endif


	/* Left MicPGA */
	{"Left MicPGA", NULL, "Left Input Mixer"},

	/* Right MicPGA */
	{"Right MicPGA", NULL, "Right Input Mixer"},

//	{"Left ADC", NULL, "Left MicPGA"},
//	{"Right ADC", NULL, "Right MicPGA"},

	{"Left ADC Route", "Analog", "Left MicPGA"},
	{"Left ADC Route", "Digital", "Left DMIC"},

	/* Selection of Digital/Analog Mic */
	{"Right ADC Route", "Analog", "Right MicPGA"},
	{"Right ADC Route", "Digital", "Right DMIC"},

	{"Left ADC", NULL, "Left ADC Route"},
	{"Right ADC", NULL, "Right ADC Route"},

};

static const struct snd_soc_dapm_route aic325x_input_dapm_routes[] = {
	/* ADC portions */
	/* Left Positive PGA input */
	{"Left Input Mixer", "IN1_L switch", "IN1_L"},
	{"Left Input Mixer", "IN2_L switch", "IN2_L"},
	{"Left Input Mixer", "IN3_L switch", "IN3_L"},
	{"Left Input Mixer", "IN1_R switch", "IN1_R"},
	/* Left Negative PGA input */
	{"Left Input Mixer", "IN2_R switch", "IN2_R"},
	{"Left Input Mixer", "IN3_R switch", "IN3_R"},
	{"Left Input Mixer", "CM1L switch", "CM1L"},
	{"Left Input Mixer", "CM2L switch", "CM2L"},
	/* Right Positive PGA Input */
	{"Right Input Mixer", "IN1_R switch", "IN1_R"},
	{"Right Input Mixer", "IN2_R switch", "IN2_R"},
	{"Right Input Mixer", "IN3_R switch", "IN3_R"},
	{"Right Input Mixer", "IN2_L switch", "IN2_L"},
	/* Right Negative PGA Input */
	{"Right Input Mixer", "IN1_L switch", "IN1_L"},
	{"Right Input Mixer", "IN3_L switch", "IN3_L"},
	{"Right Input Mixer", "CM1R switch", "CM1R"},
	{"Right Input Mixer", "CM2R switch", "CM2R"},

	{"CM1L", NULL, "CM"},
	{"CM2L", NULL, "CM"},
	{"CM1R", NULL, "CM"},
	{"CM1R", NULL, "CM"},

	{"MAL PGA", NULL, "Left MicPGA"},
	{"MAR PGA", NULL, "Right MicPGA"},
};

static const struct snd_soc_dapm_route adc3x01_input_dapm_routes[] = {
	/* ADC portions */
	/* Left Positive PGA input */
	{"Left Input Mixer", "IN1L P SE switch", "IN1_L"},
	{"Left Input Mixer", "IN2L P SE switch", "IN2_L"},
	{"Left Input Mixer", "IN3L M SE switch", "IN3_L"},
	{"Left Input Mixer", "IN2L P IN3L M DE switch", "IN2_L"},
	/* Left Negative PGA input */
	{"Left Input Mixer", "CM switch", "CM"},
	{"Left Input Mixer", "IN1L P IN1R M DE switch", "IN1_L"},
	{"Left Input Mixer", "IN2R P IN3R M DE switch", "IN2_R"},
	{"Left Input Mixer", "IN1R M SE switch", "IN1_R"},
	/* Right Positive PGA Input */
	{"Right Input Mixer", "IN2R P IN3R M DE switch", "IN2_R"},
	{"Right Input Mixer", "IN3R M SE switch", "IN3_R"},
	{"Right Input Mixer", "IN2R P SE switch", "IN2_R"},
	{"Right Input Mixer", "IN1R M SE switch", "IN1_R"},
	/* Right Negative PGA Input */
	{"Right Input Mixer", "CM switch", "CM"},
	{"Right Input Mixer", "IN1L P IN1R M switch", "IN1_L"},
	{"Right Input Mixer", "IN2L P IN3L M switch", "IN2_L"},
	{"Right Input Mixer", "IN1L P SE switch", "IN1_L"},

};

/* aic3256_firmware_load: This function is called by the
 *		request_firmware_nowait function as soon
 *		as the firmware has been loaded from the file.
 *		The firmware structure contains the data and$
 *		the size of the firmware loaded.
 * @fw: pointer to firmware file to be dowloaded
 * @context: pointer variable to codec
 *
 * Returns 0 for success.
 */
void aic3256_firmware_load(const struct firmware *fw, void *context)
{
	struct snd_soc_codec *codec = context;
	struct aic325x_priv *private_ds = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	aic3xxx_cfw_lock(private_ds->cfw_p, 1); /* take the lock */
	if (private_ds->cur_fw != NULL)
		release_firmware(private_ds->cur_fw);
	private_ds->cur_fw = NULL;

	if (fw != NULL)	{
		dev_dbg(codec->dev, "Firmware binary load\n");
		private_ds->cur_fw = (void *)fw;
		ret = aic3xxx_cfw_reload(private_ds->cfw_p, (void *)fw->data,
			fw->size);
		if (ret < 0) { /* reload failed */
			dev_err(codec->dev, "Firmware binary load failed\n");
			release_firmware(private_ds->cur_fw);
			private_ds->cur_fw = NULL;
			fw = NULL;
		}
	} else {
		dev_err(codec->dev, "Codec Firmware failed\n");
		ret = -1;
	}
	aic3xxx_cfw_lock(private_ds->cfw_p, 0); /* release the lock */
	if (ret >= 0) {
		/* init function for transition */
		aic3xxx_cfw_transition(private_ds->cfw_p, "INIT");
		aic3xxx_cfw_add_modes(codec, private_ds->cfw_p);
		aic3xxx_cfw_add_controls(codec, private_ds->cfw_p);
		aic3xxx_cfw_setmode_cfg(private_ds->cfw_p, 0, 0);
	}
}

#ifndef CONFIG_MFD_ADC3X01
/*
 * Event for headphone amplifier power changes.Special
 * power up/down sequences are required in order to maximise pop/click
 * performance.
 */
static int aic325x_hp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{

	struct snd_soc_codec *codec = w->codec;
	//struct aic325x_priv *aic325x = snd_soc_codec_get_drvdata(codec);
	int ret_wbits=0;
	unsigned int reg_mask = 0;
#if 1
	if(w->shift == 5)// HPL
	{
		reg_mask = (1 << 5);

  	}
	if(w->shift == 4)// HPR
	{
		reg_mask = (1 << 1);
  	}
#endif
	// Wait for HP power on 
	if (event & SND_SOC_DAPM_POST_PMU) {

//		ret_wbits = aic3xxx_wait_bits(codec->control_data, AIC3256_PWR_CTRL_REG,
//				HP_DRIVER_BUSY_MASK, HP_DRIVER_BUSY_MASK,AIC3256_TIME_DELAY,AIC3256_DELAY_COUNTER);
		ret_wbits = aic3xxx_wait_bits(codec->control_data, AIC3256_DAC_FLAG,
				reg_mask, reg_mask,AIC3256_TIME_DELAY,AIC3256_DELAY_COUNTER);
		if(!ret_wbits)
			dev_err(codec->dev,"HP %s power-up timedout\n", (w->shift == 5) ? "Left":"Right");
	}
	else if (event & SND_SOC_DAPM_POST_PMD) {
//		ret_wbits = aic3xxx_wait_bits(codec->control_data, AIC3256_PWR_CTRL_REG,
//				HP_DRIVER_BUSY_MASK, 0x0,AIC3256_TIME_DELAY,AIC3256_DELAY_COUNTER);
		ret_wbits = aic3xxx_wait_bits(codec->control_data, AIC3256_DAC_FLAG,
				reg_mask, 0x0,AIC3256_TIME_DELAY,AIC3256_DELAY_COUNTER);
		if(!ret_wbits)
			dev_err(codec->dev,"HP %s power- down timedout \n", (w->shift == 5) ? "Left": "Right");
		
	}
	return 0;
}

static int aic3256_cp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	if (event & SND_SOC_DAPM_POST_PMU) {
		msleep(20);
		snd_soc_write(codec, AIC3256_HP_DRIVER_CONF_REG, 0x13);	
	}
	return 0;
}
#endif
/*
 *----------------------------------------------------------------------------
 * Function : aic325x_hw_write
 * Purpose  : i2c write function
 *----------------------------------------------------------------------------
 */
int aic325x_hw_write(struct snd_soc_codec *codec, const char *buf,
						unsigned int count)
{
	u8 data[3];
	int ret;

	data[0] = *buf;
	data[1] = *(buf+1);
	data[2] = *(buf+2);

	ret = i2c_master_send(codec->control_data, data, count);

	if (ret < count) {
		printk(KERN_ERR "#%s: I2C write Error: bytes written = %d\n\n",
				__func__, ret);
		return -EIO;
	}
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic325x_hw_read
 * Purpose  : i2c read function
 *----------------------------------------------------------------------------
 */
int aic325x_hw_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct i2c_client *client = codec->control_data;
	int data = 0;

	if (i2c_master_send(client, (char *)&reg, 1) < 0) {
		printk(KERN_ERR "%s: I2C write Error\n", __func__);
		return -EIO;
	}

	if (i2c_master_recv(client, (char *)&data, 1) < 0) {
		printk(KERN_ERR "%s: I2C read Error\n", __func__);
		return -EIO;
	}

	return data & 0x00FF;
}


/**
 * Methods for CFW Operations
 *
 * Due to incompatibilites between structures used by MFD and CFW
 * we need to transform the register format before linking to
 * CFW operations.
 */
static inline unsigned int aic3256_ops_cfw2reg(unsigned int reg)
{
	union cfw_register *c = (union cfw_register *) &reg;
	union aic3xxx_reg_union mreg;

	mreg.aic3xxx_register.offset = c->offset;
	mreg.aic3xxx_register.page = c->page;
	mreg.aic3xxx_register.reserved = 0;

	return mreg.aic3xxx_register_int;
}

static int aic3256_ops_reg_read(struct snd_soc_codec *codec, unsigned int reg)
{
	return aic3xxx_reg_read(codec->control_data, aic3256_ops_cfw2reg(reg));
}

static int aic3256_ops_reg_write(struct snd_soc_codec *codec, unsigned int reg,
					unsigned char val)
{
	return aic3xxx_reg_write(codec->control_data,
					aic3256_ops_cfw2reg(reg), val);
}

static int aic3256_ops_set_bits(struct snd_soc_codec *codec, unsigned int reg,
					unsigned char mask, unsigned char val)
{
	return aic3xxx_set_bits(codec->control_data,
				aic3256_ops_cfw2reg(reg), mask, val);

}

static int aic3256_ops_bulk_read(struct snd_soc_codec *codec, unsigned int reg,
					int count, u8 *buf)
{
	return aic3xxx_bulk_read(codec->control_data,
					aic3256_ops_cfw2reg(reg), count, buf);
}

static int aic3256_ops_bulk_write(struct snd_soc_codec *codec, unsigned int reg,
					int count, const u8 *buf)
{
	return aic3xxx_bulk_write(codec->control_data,
					aic3256_ops_cfw2reg(reg), count, buf);
}


/*
********************************************************************************
Function Name : aic3256_ops_dlock_lock
Argument      : pointer argument to the codec
Return value  : Integer
Purpose	      : To Read the run state of the DAC and ADC
by reading the codec and returning the run state

Run state Bit format

------------------------------------------------------
D31|..........| D7 | D6|  D5  |  D4  | D3 | D2 | D1  |   D0  |
R               R    R   LADC   RADC    R    R   LDAC   RDAC
------------------------------------------------------

********************************************************************************
*/
int aic3256_ops_lock(struct snd_soc_codec *codec)
{
/*	mutex_lock(&codec->mutex);*/
	/* Reading the run state of adc and dac */
	return aic3256_get_runstate(codec);
}

/*
*******************************************************************************
Function name	: aic3256_ops_dlock_unlock
Argument	: pointer argument to the codec
Return Value	: integer returning 0
Purpose		: To unlock the mutex acqiured for reading
run state of the codec
********************************************************************************
*/
int aic3256_ops_unlock(struct snd_soc_codec *codec)
{
	/*Releasing the lock of mutex */
/*	mutex_unlock(&codec->mutex);*/
	return 0;
}
/*
*******************************************************************************
Function Name	: aic3256_ops_dlock_stop
Argument	: pointer Argument to the codec
mask tells us the bit format of the
codec running state

Bit Format:
------------------------------------------------------
D31|..........| D7 | D6| D5 | D4 | D3 | D2 | D1 | D0 |
R               R    R   AL   AR    R    R   DL   DR
------------------------------------------------------
R  - Reserved
A  - minidsp_A
D  - minidsp_D
********************************************************************************
*/
int aic3256_ops_stop(struct snd_soc_codec *codec, int mask)
{
	int run_state = 0;

	run_state = aic3256_get_runstate(codec);

	if (mask & AIC3XXX_COPS_MDSP_A) /* power-down ADCs */
		aic3xxx_set_bits(codec->control_data,
					AIC3256_ADC_DATAPATH_SETUP, 0xC0, 0);

	if (mask & AIC3XXX_COPS_MDSP_D) /* power-down DACs */
		aic3xxx_set_bits(codec->control_data,
					AIC3256_DAC_DATAPATH_SETUP, 0xC0, 0);

	if ((mask & AIC3XXX_COPS_MDSP_A) &&
		!aic3xxx_wait_bits(codec->control_data, AIC3256_ADC_FLAG,
					AIC3256_ADC_POWER_MASK,
					0, AIC3256_TIME_DELAY,
					AIC3256_DELAY_COUNTER))
		goto err;

	if ((mask & AIC3XXX_COPS_MDSP_D) &&
		 !aic3xxx_wait_bits(codec->control_data, AIC3256_DAC_FLAG,
					AIC3256_DAC_POWER_MASK,	0,
					AIC3256_TIME_DELAY,
					AIC3256_DELAY_COUNTER))
			goto err;
	return run_state;

err:
	dev_err(codec->dev, "Unable to turn off ADCs or DACs at [%s:%d]",
				__FILE__, __LINE__);
	return -EINVAL;

}

/*
****************************************************************************
Function name	: aic3256_ops_dlock_restore
Argument	: pointer argument to the codec, run_state
Return Value	: integer returning 0
Purpose		: To unlock the mutex acqiured for reading
run state of the codec and to restore the states of the dsp
******************************************************************************
*/
static int aic3256_ops_restore(struct snd_soc_codec *codec, int run_state)
{
	int sync_state;

	/*	This is for read the sync mode register state */
	sync_state = aic3xxx_reg_read(codec->control_data, AIC3256_DAC_PRB);
	/* checking whether the sync mode has been set -
		- or not and checking the current state */
	if (((run_state & 0x30) && (run_state & 0x03)) && (sync_state & 0x80))
		aic3256_restart_dsps_sync(codec, run_state);
	else
		aic3256_dsp_pwrup(codec, run_state);

	return 0;
}
/**
 * aic3256_ops_adaptivebuffer_swap: To swap the coefficient buffers
 *                               of minidsp according to mask
 * @pv: pointer argument to the codec,
 * @mask: tells us which dsp has to be chosen for swapping
 *
 * Return Value    : returning 0 on success
 */
int aic3256_ops_adaptivebuffer_swap(struct snd_soc_codec *codec, int mask)
{
	const int sbuf[][2] = {
		{ AIC3XXX_ABUF_MDSP_A, AIC3256_ADC_ADAPTIVE_CRAM_REG },
		{ AIC3XXX_ABUF_MDSP_D1, AIC3256_DAC_ADAPTIVE_CRAM_REG},
		/* { AIC3XXX_ABUF_MDSP_D2, AIC3256_DAC_ADAPTIVE_BANK2_REG }, */
	};
	int i;

	for (i = 0; i < sizeof(sbuf)/sizeof(sbuf[0]); ++i) {
		if (!(mask & sbuf[i][0]))
			continue;
		aic3xxx_set_bits(codec->control_data, sbuf[i][1], 0x1, 0x1);
		if (!aic3xxx_wait_bits(codec->control_data,
			sbuf[i][1], 0x1, 0, 15, 1))
			goto err;
	}
	return 0;
err:
	dev_err(codec->dev, "miniDSP buffer swap failure at [%s:%d]",
				__FILE__, __LINE__);
	return -EINVAL;
}

/*****************************************************************************
Function name	: aic3256_get_runstate
Argument	: pointer argument to the codec
Return Value	: integer returning the runstate
Purpose		: To read the current state of the dac's and adc's
 ******************************************************************************/

static int aic3256_get_runstate(struct snd_soc_codec *codec)
{
	unsigned int dac, adc;
	/* Read the run state */
	dac = aic3xxx_reg_read(codec->control_data, AIC3256_DAC_FLAG);
	adc = aic3xxx_reg_read(codec->control_data, AIC3256_ADC_FLAG);

	return (((adc>>6)&1)<<5)  |
		(((adc>>2)&1)<<4) |
		(((dac>>7)&1)<<1) |
		(((dac>>3)&1)<<0);
}

/*****************************************************************************
Function name	: aic3256_dsp_pwrdwn_status
Argument	: pointer argument to the codec , cur_state of dac's and adc's
Return Value	: integer returning 0
Purpose		: To read the status of dsp's
 ******************************************************************************/

int aic3256_dsp_pwrdwn_status(
		struct snd_soc_codec *codec /* ptr to the priv data structure */
		)
{

	aic3xxx_set_bits(codec->control_data, AIC3256_ADC_DATAPATH_SETUP,
				0XC0, 0);
	aic3xxx_set_bits(codec->control_data, AIC3256_DAC_DATAPATH_SETUP,
				0XC0, 0);

	if (!aic3xxx_wait_bits(codec->control_data, AIC3256_ADC_FLAG,
			AIC3256_ADC_POWER_MASK, 0, AIC3256_TIME_DELAY,
			AIC3256_DELAY_COUNTER))
		goto err;

	if (!aic3xxx_wait_bits(codec->control_data, AIC3256_DAC_FLAG,
			AIC3256_DAC_POWER_MASK, 0, AIC3256_TIME_DELAY,
			AIC3256_DELAY_COUNTER))
		goto err;

	return 0;

err:
	dev_err(codec->dev, "DAC/ADC Power down timedout at [%s:%d]",
				__FILE__, __LINE__);
	return -EINVAL;

}

static int aic3256_dsp_pwrup(struct snd_soc_codec *codec, int state)
{
	int adc_reg_mask = 0;
	int adc_power_mask = 0;
	int dac_reg_mask = 0;
	int dac_power_mask = 0;
	int ret_wbits;

	if (state & AIC3XXX_COPS_MDSP_A_L) {
		adc_reg_mask	|= 0x80;
		adc_power_mask	|= AIC3256_LADC_POWER_MASK;
	}
	if (state & AIC3XXX_COPS_MDSP_A_R) {
		adc_reg_mask	|= 0x40;
		adc_power_mask	|= AIC3256_RADC_POWER_MASK;
	}

	if (state & AIC3XXX_COPS_MDSP_A)
		aic3xxx_set_bits(codec->control_data,
					AIC3256_ADC_DATAPATH_SETUP,
					0XC0, adc_reg_mask);

	if (state & AIC3XXX_COPS_MDSP_D_L) {
		dac_reg_mask	|= 0x80;
		dac_power_mask	|= AIC3256_LDAC_POWER_MASK;
	}

	if (state & AIC3XXX_COPS_MDSP_D_R) {
		dac_reg_mask	|= 0x40;
		dac_power_mask	|= AIC3256_RDAC_POWER_MASK;
	}

	if (state & AIC3XXX_COPS_MDSP_D)
		aic3xxx_set_bits(codec->control_data,
					AIC3256_DAC_DATAPATH_SETUP,
					0XC0, dac_reg_mask);

	if (state & AIC3XXX_COPS_MDSP_A) {
		ret_wbits = aic3xxx_wait_bits(codec->control_data,
				AIC3256_ADC_FLAG, AIC3256_ADC_POWER_MASK,
				adc_power_mask, AIC3256_TIME_DELAY,
			AIC3256_DELAY_COUNTER);
		if (!ret_wbits)
			dev_err(codec->dev, "ADC Power down timedout\n");
	}

	if (state & AIC3XXX_COPS_MDSP_D) {
		ret_wbits = aic3xxx_wait_bits(codec->control_data,
				AIC3256_DAC_FLAG, AIC3256_DAC_POWER_MASK,
				dac_power_mask, AIC3256_TIME_DELAY,
				AIC3256_DELAY_COUNTER);
		if (!ret_wbits)
			dev_err(codec->dev, "ADC Power down timedout\n");
	}

	return 0;
}

static int aic3256_restart_dsps_sync(struct snd_soc_codec *codec, int run_state)
{

	aic3256_dsp_pwrdwn_status(codec);
	aic3256_dsp_pwrup(codec, run_state);
	return 0;
}

static const struct aic3xxx_codec_ops aic3256_cfw_codec_ops = {
	.reg_read	=	aic3256_ops_reg_read,
	.reg_write	=	aic3256_ops_reg_write,
	.set_bits	=	aic3256_ops_set_bits,
	.bulk_read	=	aic3256_ops_bulk_read,
	.bulk_write	=	aic3256_ops_bulk_write,
	.lock		=	aic3256_ops_lock,
	.unlock		=	aic3256_ops_unlock,
	.stop		=	aic3256_ops_stop,
	.restore	=	aic3256_ops_restore,
	.bswap		=	aic3256_ops_adaptivebuffer_swap,
};

/**
 * aic325x_codec_read: provide read api to read aic3256 registe space
 * @codec: pointer variable to codec having codec information,
 * @reg: register address,
 *
 * Return: Return value will be value read.
 */
unsigned int aic325x_codec_read(struct snd_soc_codec *codec, unsigned int reg)
{
	u8 value;

	union aic3xxx_reg_union *aic_reg = (union aic3xxx_reg_union *)&reg;
	value = aic3xxx_reg_read(codec->control_data, reg);
	dev_dbg(codec->dev, "p %d, r 30 %x %x\n",
		aic_reg->aic3xxx_register.page,
		aic_reg->aic3xxx_register.offset, value);
	return value;
}

/**
 * aic325x_codec_write: provide write api to write at aic3256 registe space
 * @codec: Pointer variable to codec having codec information,
 * @reg: Register address,
 * @value: Value to be written to address space
 *
 * Return: Total no of byte written to address space.
 */
int aic325x_codec_write(struct snd_soc_codec *codec, unsigned int reg,
				unsigned int value)
{
	union aic3xxx_reg_union *aic_reg = (union aic3xxx_reg_union *)&reg;
	dev_dbg(codec->dev, "p %d, w 30 %x %x\n",
		aic_reg->aic3xxx_register.page,
		aic_reg->aic3xxx_register.offset, value);
	return aic3xxx_reg_write(codec->control_data, reg, value);
}

/**
 * aic325x_hw_params: This function is to set the hardware parameters
 *		for AIC3256.
 *		The functions set the sample rate and audio serial data word
 *		length.
 * @substream: pointer variable to sn_pcm_substream,
 * @params: pointer to snd_pcm_hw_params structure,
 * @dai: ponter to dai Holds runtime data for a DAI,
 *
 * Return: Return 0 on success.
 */
static int aic325x_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
					struct snd_soc_dai *dai)
{

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	u8 data = 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		data |= (0x00);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (0x10);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (0x20);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (0x30);
		break;
	}

	snd_soc_update_bits(codec, AIC3256_INTERFACE_SET_REG_1,
				INTERFACE_REG1_DATA_LEN_MASK, data);

	return 0;
}

/**
 * aic325x_mute: This function is to mute or unmute the left and right DAC
 * @dai: ponter to dai Holds runtime data for a DAI,
 * @mute: integer value one if we using mute else unmute,
 *
 * Return: return 0 on success.
 */
static int aic325x_mute(struct snd_soc_dai *dai, int mute)
{
	dprintk(KERN_INFO "Function: %s\n", __func__);
	dprintk(KERN_INFO "Function: %s Exiting\n", __func__);

	return 0;
}

/**
 * aic325x_set_dai_fmt: This function is to set the DAI format
 * @codec_dai: ponter to dai Holds runtime data for a DAI,
 * @fmt: asi format info,
 *
 * return: return 0 on success.
 */
static int aic325x_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec;
	struct aic325x_priv *aic325x;

	u8 iface_reg1 = 0;
	u8 iface_reg3 = 0;
	u8 dsp_a_val = 0;

	codec	= codec_dai->codec;
	aic325x	= snd_soc_codec_get_drvdata(codec);

	
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic325x->master = 1;
		iface_reg1 |= AIC3256_BIT_CLK_MASTER | AIC3256_WORD_CLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic325x->master = 0;
		iface_reg1 &= ~(AIC3256_BIT_CLK_MASTER |
					AIC3256_WORD_CLK_MASTER);
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		aic325x->master = 0;
		iface_reg1 |= AIC3256_BIT_CLK_MASTER;
		iface_reg1 &= ~(AIC3256_WORD_CLK_MASTER);
		break;
	default:
		printk(KERN_INFO "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
			dsp_a_val = 0x1;
		case SND_SOC_DAIFMT_DSP_B:
			switch(fmt & SND_SOC_DAIFMT_INV_MASK) {
				case SND_SOC_DAIFMT_NB_NF:
					break;
				case SND_SOC_DAIFMT_IB_NF:
					iface_reg3 |= BCLK_INV_MASK;
					break;
				default: 
					return -EINVAL;
			}
			iface_reg1 |= (AIC325x_DSP_MODE << CLK_REG_3_SHIFT);
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			iface_reg1 |= (AIC325x_RIGHT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			iface_reg1 |= (AIC325x_LEFT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
			break;
		default:
			printk(KERN_INFO "Invalid DAI interface format\n");
			return -EINVAL;
	}

	snd_soc_update_bits(codec, AIC3256_INTERFACE_SET_REG_1, 
			INTERFACE_REG1_DATA_TYPE_MASK |
			INTERFACE_REG1_MASTER_MASK,
			iface_reg1);
	snd_soc_update_bits(codec, AIC3256_INTERFACE_SET_REG_2, INTERFACE_REG2_MASK,
			dsp_a_val);
	snd_soc_update_bits(codec, AIC3256_INTERFACE_SET_REG_3, INTERFACE_REG3_MASK,
			iface_reg3);
	return 0;
}

/**
 * aic325x_dai_set_pll: This function is to Set pll for aic3256 codec dai
 * @dai: ponter to dai Holds runtime data for a DAI, $
 * @pll_id: integer pll_id
 * @fin: frequency in,
 * @fout: Frequency out,
 *
 * Return: return 0 on success
*/
static int aic325x_dai_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
				unsigned int Fin, unsigned int Fout)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);

	return aic3xxx_cfw_set_pll(aic3256->cfw_p, dai->id);

}

static int adc3x01_prepare(struct snd_pcm_substream *stream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_codec *codec = dai->codec;
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_card *card = rtd->card;
	int i;
	
	if(aic3256->master == 0) {
		for(i = 0; i < card->num_rtd; i++) {
			struct snd_soc_dai *dai = card->rtd[i].codec_dai;
			struct snd_soc_dai_driver *drv = dai->driver;
			
			if(drv && drv->ops->set_tristate)
				drv->ops->set_tristate(dai, 0);
		}
	}

	return 0;
}

static int adc3x01_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	struct snd_soc_codec *codec = dai->codec;

	/* TODO: for 1 master 1 slave, if 1master multi slaves, need to think
	 * of something better
    	*/
	if(!tristate) { 
		snd_soc_update_bits(codec, AIC3256_DOUT_CTRL,  
			AIC3256_DOUT_ENABLE_M, AIC3256_DOUT_ENABLE_V);
	}
	else { 
		snd_soc_update_bits(codec, AIC3256_DOUT_CTRL,  
			AIC3256_DOUT_ENABLE_M, 0x0);
		
	}
	return 0;
	
}
static int adc3x01_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
	unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int asi = snd_soc_read(codec,AIC3256_INTERFACE_SET_REG_2); 
	unsigned int slot_count = fls(rx_mask) - 2;

	if (slots < 0 || slots > 16)
		return -EINVAL;	
	
	
	asi |= slot_count * slot_width; 

	snd_soc_write(codec, AIC3256_INTERFACE_SET_REG_2, asi);
	return 0;
		
	
}
/**
 *
 * aic325x_set_bias_level: This function is to get triggered
 *			 when dapm events occurs.
 * @codec: pointer variable to codec having informaton related to codec,
 * @level: Bias level-> ON, PREPARE, STANDBY, OFF.
 *
 * Return: Return 0 on success.
 */
static int aic325x_set_bias_level(struct snd_soc_codec *codec,
					enum snd_soc_bias_level level)
{
	struct aic325x_priv *aic325x = snd_soc_codec_get_drvdata(codec);

	switch (level) {

	/* full On */
	case SND_SOC_BIAS_ON:
		/* all power is driven by DAPM system */
	dprintk(KERN_INFO "Function : %s: BIAS ON\n", __func__);
		if (aic325x->master == 1) {
			snd_soc_update_bits(codec, AIC3256_CLK_REG_11, 
					BCLK_DIV_POWER_MASK, 0x80);	
		}
		break;

	/* partial On */
	case SND_SOC_BIAS_PREPARE:
	dprintk(KERN_INFO "Function : %s: BIAS prepare\n", __func__);
		if (snd_soc_codec_get_bias_level(codec)/*codec->dapm.bias_level*/ == SND_SOC_BIAS_ON) {
				snd_soc_update_bits(codec, AIC3256_CLK_REG_11,
						BCLK_DIV_POWER_MASK, 0);
			}	
		break;

	/* Off, with power */
	case SND_SOC_BIAS_STANDBY:

//		snd_soc_update_bits(codec, AIC3256_INTERFACE_SET_REG_3,
//				AIC3256_BCLK_WCLK_BUFFER_POWER_CONTROL_MASK, 0);

#ifndef CONFIG_MFD_ADC3X01
		snd_soc_update_bits(codec, AIC3256_REF_PWR_UP_CONF_REG,
				AIC3256_REF_PWR_UP_MASK,
				AIC3256_FORCED_REF_PWR_UP);

		/*
		 * all power is driven by DAPM system,
	0	 * so output power is safe if bypass was set
		 */

		if (snd_soc_codec_get_bias_level(codec)/*codec->dapm.bias_level*/ == SND_SOC_BIAS_OFF) {
			snd_soc_update_bits(codec, AIC3256_POW_CFG,
				AIC3256_AVDD_CONNECTED_TO_DVDD_MASK,
				AIC3256_DISABLE_AVDD_TO_DVDD);
			snd_soc_update_bits(codec, AIC3256_PWR_CTRL_REG,
					AIC3256_ANALOG_BLOCK_POWER_CONTROL_MASK,
					AIC3256_ENABLE_ANALOG_BLOCK);

		}
#endif
		break;

		/* Off, without power */
	case SND_SOC_BIAS_OFF:
#ifndef CONFIG_MFD_ADC3X01
		if (snd_soc_codec_get_bias_level(codec)/*codec->dapm.bias_level*/ == SND_SOC_BIAS_STANDBY) {
			snd_soc_update_bits(codec, AIC3256_REF_PWR_UP_CONF_REG,
					AIC3256_REF_PWR_UP_MASK,
					AIC3256_AUTO_REF_PWR_UP);
			snd_soc_update_bits(codec, AIC3256_PWR_CTRL_REG,
					AIC3256_ANALOG_BLOCK_POWER_CONTROL_MASK,
					AIC3256_DISABLE_ANALOG_BLOCK);
			snd_soc_update_bits(codec, AIC3256_POW_CFG,
				AIC3256_AVDD_CONNECTED_TO_DVDD_MASK,
				AIC3256_ENABLE_AVDD_TO_DVDD);
		}
#endif
	/* force all power off */
		break;
	}
	//codec->dapm.bias_level = level;
	snd_soc_codec_init_bias_level(codec, level);

	return 0;
}

/**
 *
 * aic325x_suspend; This function is to suspend the AIC3256 driver.
 * @codec: pointer variable to codec having informaton related to codec,
 *
 * Return: Return 0 on success.
 */
static int aic325x_suspend(struct snd_soc_codec *codec)
{
	dprintk(KERN_INFO "Function: %s\n", __func__);
	aic325x_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}


/**
 * aic325x_resume: This function is to resume the AIC3256 driver
 *		 from off state to standby
 * @codec: pointer variable to codec having informaton related to codec,
 *
 * Return: Return 0 on success.
 */
static int aic325x_resume(struct snd_soc_codec *codec)
{
	aic325x_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

enum headset_accessory_state {
	BIT_NO_ACCESSORY = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADPHONE = (1 << 1),
};

static void aic3256_hs_jack_report(struct snd_soc_codec *codec,
					struct snd_soc_jack *jack, int report)
{
	//struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	int status, state = 0, switch_state = BIT_NO_ACCESSORY;

	//mutex_lock(&aic3256->io_lock);

	/* Sync status */
	status = snd_soc_read(codec, AIC3256_HEADSET_DETECT);
	/* We will check only stereo MIC and headphone */
	switch (status & AIC3256_JACK_TYPE_MASK) {
	case AIC3256_JACK_WITH_MIC:
		state |= SND_JACK_HEADSET;
		break;
	case AIC3256_JACK_WITHOUT_MIC:
		state |= SND_JACK_HEADPHONE;
	}

	//mutex_unlock(&aic3256->io_lock);

	snd_soc_jack_report(jack, state, report);

	if ((state & SND_JACK_HEADSET) == SND_JACK_HEADSET)
		switch_state |= BIT_HEADSET;
	else if (state & SND_JACK_HEADPHONE)
		switch_state |= BIT_HEADPHONE;

}

/**
 * aic3256_hs_jack_detect: Detect headphone jack during boot time
 * @codec: pointer variable to codec having information related to codec
 * @jack: Pointer variable to snd_soc_jack having information of codec
 *	     and pin number$
 * @report: Provides informaton of whether it is headphone or microphone
 *
*/
void aic3256_hs_jack_detect(struct snd_soc_codec *codec,
			struct snd_soc_jack *jack, int report)
{
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	struct aic3256_jack_data *hs_jack = &aic3256->hs_jack;

	hs_jack->jack = jack;
	hs_jack->report = report;
	aic3256_hs_jack_report(codec, hs_jack->jack, hs_jack->report);
}
EXPORT_SYMBOL_GPL(aic3256_hs_jack_detect);


/**
 * aic325x_probe: This is first driver function called by the SoC core driver.
 * @codec: pointer variable to codec having informaton related to codec,
 *
 * Return: Return 0 on success.
 */
static int aic325x_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec); //&codec->dapm;
	struct aic325x_priv *aic325x;
	struct aic3xxx *control;

	if (codec == NULL)
		dev_err(codec->dev, "codec pointer is NULL\n");
	codec->control_data = dev_get_drvdata(codec->dev->parent);
	control = codec->control_data;
	aic325x = kzalloc(sizeof(struct aic325x_priv), GFP_KERNEL);
	if (aic325x == NULL)
		return -ENOMEM;
	snd_soc_codec_set_drvdata(codec, aic325x);
	aic325x->codec = codec;
	
	aic325x->cur_fw = NULL;

	aic325x->cfw_p = &(aic325x->cfw_ps);

	aic3xxx_cfw_init(aic325x->cfw_p, &aic3256_cfw_codec_ops,
				aic325x->codec);
	snd_soc_dapm_new_controls(dapm, aic325x_dapm_widgets,
				ARRAY_SIZE(aic325x_dapm_widgets));
	switch (control->type) {
	case TLV320AIC3256:
		snd_soc_dapm_new_controls(dapm, aic325x_input_mixer_widgets,
				ARRAY_SIZE(aic325x_input_mixer_widgets));
		ret = snd_soc_dapm_add_routes(dapm, aic325x_input_dapm_routes,
				ARRAY_SIZE(aic325x_input_dapm_routes));
		if (!ret)
			dprintk("#Completed adding DAPM input routes = %d\n",
				ARRAY_SIZE(aic325x_input_dapm_routes));
		break;
	case TLV320ADC3001:
		snd_soc_dapm_new_controls(dapm, adc3x01_input_mixer_widgets,
				ARRAY_SIZE(adc3x01_input_mixer_widgets));
		ret = snd_soc_dapm_add_routes(dapm, adc3x01_input_dapm_routes,
				ARRAY_SIZE(adc3x01_input_dapm_routes));
		if (!ret)
			dprintk("#Completed adding DAPM input routes = %d\n",
				ARRAY_SIZE(adc3x01_input_dapm_routes));
		break;
	}
	ret = snd_soc_dapm_add_routes(dapm, aic325x_dapm_routes,
				ARRAY_SIZE(aic325x_dapm_routes));
	if (!ret)
		dprintk("#Completed adding DAPM routes = %d\n",
			ARRAY_SIZE(aic325x_dapm_routes));
	aic325x_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	/* firmware load */
/*
	if (!strcmp(codec->name, "tlv320aic325x-codec.0"))
		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				"tlv320aic3256_fw_v1.bin",
				codec->dev, GFP_KERNEL, codec,
				aic3256_firmware_load);
	else // slave device
*/
		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				"tlv320adc3x01_slave_v1.bin",
				codec->dev, GFP_KERNEL, codec,
				aic3256_firmware_load);
	
	mutex_init(&aic325x->io_lock);

	return ret;

}

/*
 *----------------------------------------------------------------------------
 * Function : aic325x_remove
 * Purpose  : to remove aic325x soc device
 *
 *----------------------------------------------------------------------------
 */
static int aic325x_remove(struct snd_soc_codec *codec)
{
	/* power down chip */
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);

	aic325x_set_bias_level(codec, SND_SOC_BIAS_OFF);

	if (aic3256->cur_fw != NULL)
		release_firmware(aic3256->cur_fw);

	kfree(aic3256);
	return 0;
}


static struct snd_soc_codec_driver soc_codec_driver_aic325x = {
	.probe = aic325x_probe,
	.remove = aic325x_remove,
	.suspend = aic325x_suspend,
	.resume = aic325x_resume,
	.read = aic325x_codec_read,
	.write = aic325x_codec_write,
	.set_bias_level = aic325x_set_bias_level,
	.controls = aic325x_snd_controls ,
	.num_controls = ARRAY_SIZE(aic325x_snd_controls),
	.reg_cache_size = 0,
	.reg_word_size = sizeof(u8),
	.reg_cache_default = NULL,
};

static int aic3256_probe(struct platform_device *pdev)
{
	int ret;
	struct aic3xxx *aic3xxx = dev_get_drvdata(pdev->dev.parent);
	
	switch (aic3xxx->type) {
	case TLV320AIC3256:
		ret = snd_soc_register_codec(
					&pdev->dev,
					&soc_codec_driver_aic325x,
					tlv320aic325x_dai_driver,
					ARRAY_SIZE(tlv320aic325x_dai_driver)
					);
		break;
	case TLV320ADC3001:
		ret = snd_soc_register_codec(
					&pdev->dev,
					&soc_codec_driver_aic325x,
					tlv320adc3x01_dai_driver,
					ARRAY_SIZE(tlv320adc3x01_dai_driver)
					);
		break;
	}
	
	return ret;
}

static int aic3256_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_device_id aic325x_id_table[] = {
#ifndef CONFIG_MFD_ADC3X01
	{ "tlv320aic325x-codec", 0 },
	{ "tlv320adc325x-codec", 1 },
#else
	{ "tlv320adc3x01-codec", 0},
#endif
	{},
};
MODULE_DEVICE_TABLE(platform, aic325x_id_table);

static struct platform_driver aic325x_codec_driver = {
	.driver = {
#ifndef CONFIG_MFD_ADC3X01
		.name = "tlv320aic325x-codec",
#else
		.name = "tlv320adc3x01-codec",
#endif
		.owner = THIS_MODULE,
	},
	.probe = aic3256_probe,
	//.remove = __devexit_p(aic3256_remove),
	.remove = aic3256_remove,
	.id_table = aic325x_id_table,
};

/*
 *----------------------------------------------------------------------------
 * Function : tlv320aic3256_modinit
 * Purpose  : module init function. First function to run.
 *
 *----------------------------------------------------------------------------
 */
static int __init tlv320aic325x_modinit(void)
{
	return platform_driver_register(&aic325x_codec_driver);
}
module_init(tlv320aic325x_modinit);

/*
 *----------------------------------------------------------------------------
 * Function : tlv320aic3256_exit
 * Purpose  : module init function. First function to run.
 *
 *----------------------------------------------------------------------------
 */
static void __exit tlv320aic325x_exit(void)
{
	platform_driver_unregister(&aic325x_codec_driver);
}

module_exit(tlv320aic325x_exit);


MODULE_ALIAS("platform:tlv320aic325x-codec");
MODULE_DESCRIPTION("ASoC TLV320AIC325x codec driver");
MODULE_AUTHOR("Aravindan Muthukumar");
MODULE_AUTHOR("Suresh Pm");
MODULE_LICENSE("GPL");
