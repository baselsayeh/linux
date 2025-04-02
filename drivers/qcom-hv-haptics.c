// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/vmalloc.h>
#include <linux/export.h>
#include <linux/workqueue.h>

/* status register definitions in HAPTICS_CFG module */
#define HAP_CFG_REVISION1_REG			0x00
#define HAP_CFG_V1				0x1
#define HAP_CFG_V2				0x2
#define HAP_CFG_V3				0x3
#define HAP_CFG_V4				0x4

#define HAP_CFG_STATUS_DATA_MSB_REG		0x09
/* STATUS_DATA_MSB definitions while MOD_STATUS_SEL is 0 */
#define AUTO_RES_CAL_DONE_BIT			BIT(5)
#define CAL_TLRA_CL_STS_MSB_MASK		GENMASK(4, 0)
/* STATUS_DATA_MSB definition while MOD_STATUS_SEL is 3 */
#define LAST_GOOD_TLRA_CL_MSB_MASK		GENMASK(4, 0)
/* STATUS_DATA_MSB definition while MOD_STATUS_SEL is 4 */
#define TLRA_CL_ERR_MSB_MASK			GENMASK(4, 0)
/* STATUS_DATA_MSB when MOD_STATUS_SEL is 5 and MOD_STATUS_XT.SEL is 1 */
#define FIFO_REAL_TIME_FILL_STATUS_MSB_MASK_V2	GENMASK(1, 0)
#define FIFO_REAL_TIME_FILL_STATUS_MSB_MASK_V3	GENMASK(2, 0)
#define FIFO_REAL_TIME_FILL_STATUS_MSB_MASK_V4	GENMASK(3, 0)
#define FIFO_EMPTY_FLAG_BIT			BIT(6)
#define FIFO_FULL_FLAG_BIT			BIT(5)

#define HAP_CFG_STATUS_DATA_LSB_REG		0x0A
/* STATUS_DATA_LSB definition while MOD_STATUS_SEL is 0 */
#define CAL_TLRA_CL_STS_LSB_MASK		GENMASK(7, 0)
/* STATUS_DATA_LSB when MOD_STATUS_SEL is 5 and MOD_STATUS_XT.SEL is 1 */
#define FIFO_REAL_TIME_FILL_STATUS_LSB_MASK	GENMASK(7, 0)
/* STATUS_DATA_LSB when MOD_STATUS_SEL is 5 and MOD_STATUS_XT.SEL is 0 */
#define HAP_DRV_PATTERN_SRC_STATUS_MASK		GENMASK(2, 0)

#define HAP_CFG_FAULT_STATUS_REG		0x0C
#define SC_FLAG_BIT				BIT(2)
#define AUTO_RES_ERROR_BIT			BIT(1)
#define HPRW_RDY_FAULT_BIT			BIT(0)

/* Only for HAP525_HV */
#define HAP_CFG_HPWR_INTF_REG                   0x0B
#define HPWR_INTF_STATUS_MASK                   GENMASK(1, 0)
#define HPWR_DISABLED                           0
#define HPWR_READY                              3

#define HAP_CFG_REAL_TIME_LRA_IMPEDANCE_REG	0x0E
#define LRA_IMPEDANCE_MOHMS_LSB			250

#define HAP_CFG_INT_RT_STS_REG			0x10
#define FIFO_EMPTY_BIT				BIT(1)

/* config register definitions in HAPTICS_CFG module */
#define HAP_CFG_EN_CTL_REG			0x46
#define HAPTICS_EN_BIT				BIT(7)

#define HAP_CFG_DRV_CTRL_REG			0x47
#define PSTG_DLY_MASK				GENMASK(7, 6)
#define DRV_SLEW_RATE_MASK			GENMASK(2, 0)

#define HAP_CFG_VMAX_REG			0x48
#define VMAX_HV_STEP_MV				50
#define VMAX_MV_STEP_MV				32
#define MAX_VMAX_MV				11000
#define MAX_HV_VMAX_MV				10000
#define MAX_MV_VMAX_MV				6000
#define CLAMPED_VMAX_MV				5000
#define DEFAULT_VMAX_MV				5000

#define HAP_CFG_DRV_WF_SEL_REG			0x49
#define DRV_WF_FMT_BIT				BIT(4)
#define DRV_WF_SEL_MASK				GENMASK(1, 0)

#define HAP_CFG_AUTO_SHUTDOWN_CFG_REG		0x4A

#define HAP_CFG_TRIG_PRIORITY_REG		0x4B
#define SWR_IGNORE_BIT				BIT(4)

#define HAP_CFG_SPMI_PLAY_REG			0x4C
#define PLAY_EN_BIT				BIT(7)
#define PATX_MEM_SEL_MASK			GENMASK(5, 4) /* This is only for HAP525_HV */
#define BRAKE_EN_BIT				BIT(3)
#define PAT_SRC_MASK				GENMASK(2, 0)

#define HAP_CFG_EXT_TRIG_REG			0x4D

#define HAP_CFG_SWR_ACCESS_REG			0x4E
#define SWR_PAT_CFG_EN_BIT			BIT(7)
#define SWR_PAT_INPUT_EN_BIT			BIT(6)
#define SWR_PAT_RES_N_BIT			BIT(5)

#define HAP_CFG_BRAKE_MODE_CFG_REG		0x50
#define BRAKE_MODE_MASK				GENMASK(7, 6)
#define BRAKE_MODE_SHIFT			6
#define BRAKE_SINE_GAIN_MASK			GENMASK(3, 2)
#define BRAKE_SINE_GAIN_SHIFT			2
#define BRAKE_WF_SEL_MASK			GENMASK(1, 0)

#define HAP_CFG_CL_BRAKE_CFG_REG		0x51
#define HAP_CFG_CL_BRAKE_CAL_PARAM_REG		0x52
#define HAP_CFG_CL_BRAKE_RSET_REG		0x53
#define HAP_CFG_PWM_CFG_REG			0x5A

#define HAP_CFG_TLRA_OL_HIGH_REG		0x5C
#define TLRA_OL_MSB_MASK			GENMASK(3, 0)
#define HAP_CFG_TLRA_OL_LOW_REG			0x5D
#define TLRA_OL_LSB_MASK			GENMASK(7, 0)
#define TLRA_STEP_US				5
#define TLRA_MAX_US				20475

#define HAP_CFG_RC_CLK_CAL_COUNT_MSB_REG	0x5E
#define RC_CLK_CAL_COUNT_MSB_MASK		GENMASK(1, 0)
#define HAP_CFG_RC_CLK_CAL_COUNT_LSB_REG	0x5F
#define RC_CLK_CAL_COUNT_LSB_MASK		GENMASK(7, 0)

#define HAP_CFG_DRV_DUTY_CFG_REG		0x60
#define ADT_DRV_DUTY_EN_BIT			BIT(7)
#define ADT_BRK_DUTY_EN_BIT			BIT(6)
#define DRV_DUTY_MASK				GENMASK(5, 3)
#define DRV_DUTY_62P5_PCT			2
#define DRV_DUTY_SHIFT				3
#define BRK_DUTY_MASK				GENMASK(2, 0)
#define BRK_DUTY_75_PCT			6

#define HAP_CFG_ADT_DRV_DUTY_CFG_REG		0x61
#define HAP_CFG_ZX_WIND_CFG_REG			0x62

#define HAP_CFG_AUTORES_CFG_REG			0x63
#define AUTORES_EN_BIT				BIT(7)
#define AUTORES_EN_DLY_MASK			GENMASK(5, 2)
#define AUTORES_EN_DLY(cycles)			((cycles) * 2)
#define AUTORES_EN_DLY_6_CYCLES			AUTORES_EN_DLY(6)
#define AUTORES_EN_DLY_7_CYCLES			AUTORES_EN_DLY(7)
#define AUTORES_EN_DLY_SHIFT			2
#define AUTORES_ERR_WINDOW_MASK			GENMASK(1, 0)
#define AUTORES_ERR_WINDOW_12P5_PERCENT		0x0
#define AUTORES_ERR_WINDOW_25_PERCENT		0x1
#define AUTORES_ERR_WINDOW_50_PERCENT		0x2
#define AUTORES_ERR_WINDOW_100_PERCENT		0x3

#define HAP_CFG_AUTORES_ERR_RECOVERY_REG	0x64
#define EN_HW_RECOVERY_BIT			BIT(1)
#define SW_ERR_DRV_FREQ_BIT			BIT(0)

#define HAP_CFG_ISC_CFG_REG			0x65
#define ILIM_CC_EN_BIT				BIT(7)
#define ILIM_CC_EN_BIT_VAL			1
/* Following bits are only for HAP525_HV */
#define EN_SC_DET_P_HAP525_HV_BIT		BIT(6)
#define EN_SC_DET_N_HAP525_HV_BIT		BIT(5)
#define EN_IMP_DET_HAP525_HV_BIT		BIT(4)
#define ILIM_PULSE_DENSITY_MASK			GENMASK(3, 2)
#define ILIM_DENSITY_8_OVER_64_CYCLES		0

#define HAP_CFG_FAULT_CLR_REG			0x66
#define SC_CLR_BIT				BIT(2)
#define AUTO_RES_ERR_CLR_BIT			BIT(1)
#define HPWR_RDY_FAULT_CLR_BIT			BIT(0)

#define HAP_CFG_VMAX_HDRM_REG			0x67
#define VMAX_HDRM_MASK				GENMASK(6, 0)
#define VMAX_HDRM_STEP_MV			50
#define VMAX_HDRM_MAX_MV			6350

#define HAP_CFG_VSET_CFG_REG			0x68
#define FORCE_VSET_ACK_BIT			BIT(1) /* This is only for HAP525_HV */
#define FORCE_VREG_RDY_BIT			BIT(0)

#define HAP_CFG_MOD_STATUS_SEL_REG		0x70
#define HAP_CFG_MOD_STATUS_XT_REG		0x71

#define HAP_CFG_CAL_EN_REG			0x72
#define CAL_RC_CLK_MASK				GENMASK(3, 2)
#define CAL_RC_CLK_SHIFT			2
#define CAL_RC_CLK_DISABLED_VAL			0
#define CAL_RC_CLK_AUTO_VAL			1
#define CAL_RC_CLK_MANUAL_VAL			2

/* For HAP520_MV and HAP525_HV */
#define HAP_CFG_ISC_CFG2_REG			0x77
#define EN_SC_DET_P_HAP520_MV_BIT		BIT(6)
#define EN_SC_DET_N_HAP520_MV_BIT		BIT(5)
#define ISC_THRESH_HAP520_MV_MASK		GENMASK(2, 0)
#define ISC_THRESH_HAP520_MV_140MA		0x01
#define ISC_THRESH_HAP525_HV_MASK		GENMASK(4, 0)
#define ISC_THRESH_HAP525_HV_125MA		0x11
#define ISC_THRESH_HAP525_HV_250MA		0x12

/* These registers are only applicable for HAP520_MV */
#define HAP_CFG_HW_CONFIG_REG			0x0D
#define HV_HAP_DRIVER_BIT			BIT(1)

#define HAP_CFG_HPWR_INTF_CTL_REG		0x80
#define INTF_CTL_MASK				GENMASK(1, 0)
#define INTF_CTL_BOB				1
#define INTF_CTL_BHARGER			2

#define HAP_CFG_VHPWR_REG			0x84
#define VHPWR_STEP_MV				32

/* Only for HAP525_HV */
#define HAP_CFG_RT_LRA_IMPD_MEAS_CFG_REG	0xA4
#define LRA_IMPEDANCE_MEAS_EN_BIT		BIT(7)
#define LRA_IMPEDANCE_MEAS_CURRENT_SEL_BIT	BIT(0)
#define CURRENT_SEL_VAL_125MA			0
#define CURRENT_SEL_VAL_250MA			1

/* version register definitions for HAPTICS_PATTERN module */
#define HAP_PTN_REVISION2_REG			0x01
#define HAP_PTN_V1				0x1
#define HAP_PTN_V2				0x2
#define HAP_PTN_V3				0x3
#define HAP_PTN_V4				0x4

/* status register definition for HAPTICS_PATTERN module */
#define HAP_PTN_FIFO_READY_STS_REG		0x08
#define FIFO_READY_BIT				BIT(0)

#define HAP_PTN_NUM_PAT_REG			0x09

/* config register definition for HAPTICS_PATTERN module */
#define HAP_PTN_FIFO_DIN_0_REG			0x20
#define HAP_PTN_FIFO_DIN_NUM			4

#define HAP_PTN_FIFO_PLAY_RATE_REG		0x24
#define PAT_MEM_PLAY_RATE_MASK			GENMASK(7, 4)
#define FIFO_PLAY_RATE_MASK			GENMASK(3, 0)

#define HAP_PTN_FIFO_EMPTY_CFG_REG		0x2A
#define EMPTY_THRESH_MASK			GENMASK(3, 0)
#define HAP_PTN_FIFO_THRESH_LSB			40

#define HAP_PTN_FIFO_DEPTH_CFG_REG		0x2B
#define HAP_PTN_FIFO_DIN_1B_REG			0x2C

#define HAP_PTN_DIRECT_PLAY_REG			0x26
#define DIRECT_PLAY_MAX_AMPLITUDE		0xFF

#define HAP_PTN_AUTORES_CAL_CFG_REG		0x28

#define HAP_PTN_PTRN1_TLRA_MSB_REG		0x30
#define HAP_PTN_PTRN1_TLRA_LSB_REG		0x31
#define HAP_PTN_PTRN2_TLRA_MSB_REG		0x32
#define HAP_PTN_PTRN2_TLRA_LSB_REG		0x33

#define HAP_PTN_PTRN1_CFG_REG			0x34
#define PTRN_FLRA2X_SHIFT			7
#define PTRN_SAMPLE_PER_MASK			GENMASK(2, 0)

#define PTRN_AMP_MSB_MASK			BIT(0)
#define PTRN_AMP_LSB_MASK			GENMASK(7, 0)

#define HAP_PTN_PTRN2_CFG_REG			0x50

#define HAP_PTN_BRAKE_AMP_REG			0x70

/* HAPTICS_PATTERN registers only present in HAP525_HV */
#define HAP_PTN_MEM_OP_ACCESS_REG		0x2D
#define MEM_PAT_ACCESS_BIT			BIT(7)
#define MEM_PAT_RW_SEL_MASK			GENMASK(5, 4)
#define MEM_FLUSH_RELOAD_BIT			BIT(0)

#define HAP_PTN_MMAP_FIFO_REG			0xA0
#define MMAP_FIFO_EXIST_BIT			BIT(7)
#define MMAP_FIFO_LEN_MASK			GENMASK(3, 0)
#define MMAP_FIFO_LEN_PER_LSB			128

#define HAP_PTN_MMAP_PAT1_REG			0xA1
#define MMAP_PAT_LEN_PER_LSB			32
#define MMAP_PAT1_LEN_MASK			GENMASK(6, 0)
#define MMAP_PAT2_LEN_MASK			GENMASK(5, 0)
#define MMAP_PAT3_PAT4_LEN_MASK			GENMASK(4, 0)

/* register in HBOOST module */
#define HAP_BOOST_REVISION1			0x00
#define HAP_BOOST_REVISION2			0x01
#define HAP_BOOST_V0P0				0x0000
#define HAP_BOOST_V0P1				0x0001
#define HAP_BOOST_V0P2				0x0002

#define HAP_BOOST_STATUS4_REG			0x0B
#define BOOST_DTEST1_STATUS_BIT			BIT(0)

#define HAP_BOOST_HW_CTRL_FOLLOW_REG		0x41
#define FOLLOW_HW_EN_BIT			BIT(7)
#define FOLLOW_HW_CCM_BIT			BIT(6)
#define FOLLOW_HW_VSET_BIT			BIT(5)

#define HAP_BOOST_VREG_EN_REG			0x46
#define VREG_EN_BIT				BIT(7)

#define HAP_BOOST_CLAMP_REG			0x70
#define CLAMP_5V_BIT				BIT(0)

/* haptics SDAM registers offset definition */
#define HAP_STATUS_DATA_MSB_SDAM_OFFSET		0x46
#define HAP_LRA_NOMINAL_OHM_SDAM_OFFSET		0x75
#define HAP_LRA_DETECTED_OHM_SDAM_OFFSET	0x76

/* constant parameters */
#define SAMPLES_PER_PATTERN			8
#define BRAKE_SAMPLE_COUNT			8
#define DEFAULT_ERM_PLAY_RATE_US		5000
#define MAX_EFFECT_COUNT			16
#define FIFO_READY_TIMEOUT_MS			1000
#define CHAR_PER_PATTERN_S			48
#define CHAR_PER_SAMPLE				8
#define CHAR_MSG_HEADER				16
#define CHAR_BRAKE_MODE				24
#define HW_BRAKE_MAX_CYCLES			16
#define F_LRA_VARIATION_HZ			5
#define NON_HBOOST_MAX_VMAX_MV			4000
/* below definitions are only for HAP525_HV */
#define MMAP_NUM_BYTES				2048
#define MMAP_FIFO_MIN_SIZE			640
#define FIFO_PRGM_INIT_SIZE			320

#define is_between(val, min, max)	\
	(((min) <= (max)) && ((min) <= (val)) && ((val) <= (max)))

#define SWEEP_MIN_FREQUENCY			50
#define SWEEP_MAX_FREQUENCY			500
#define DEFAULT_SWEEP_START_FREQUENCY		170
#define DEFAULT_SWEEP_END_FREQUENCY		230
#define DEFAULT_SWEEP_STEP_FREQUENCY		1
#define DEFAULT_SWEEP_STEP_DURATION_MS		100
#define DEFAULT_AMPLITUDE_FOR_SWEEP		0xFF

// b/311643110
// The spec of LRA's f0 is 200 ± 10%
// But there is a 5 Hz shift in SW value compared to the f0 measured by IMU.
#define LRA_SPEC_MIN_FREQUENCY 185
#define LRA_SPEC_MAX_FREQUENCY 215
#define LRA_SPEC_DEFAULT_FREQUENCY 205
#define LRA_SPEC_DEFAULT_PERIOD_US (USEC_PER_SEC / LRA_SPEC_DEFAULT_FREQUENCY)

struct effect_with_status {
	struct ff_effect	effect;
	bool			is_loaded;
	bool			is_playing;
};

struct haptics_chip {
	struct device			*dev;
	struct input_dev		*input_dev;

	struct effect_with_status	effects_loaded[MAX_EFFECT_COUNT];
	u32				combined_effect_amp;
	bool				is_combined_effect_playing;

	volatile bool			chip_effect_loaded;
	volatile bool			chip_is_playing;

	int 				external_val;
};








static int __chip_haptics_upload_effect(struct input_dev *dev, struct ff_effect *effect, struct ff_effect *old) {
	struct haptics_chip *chip = input_get_drvdata(dev);

	pr_info("[hardware] %s: enter (id=%d)\n", __func__, effect->id);

	if (effect->type == FF_CONSTANT)
		pr_info("[hardware] Effect (Constant) value: %u\n", effect->u.constant.level);
	else if (effect->type == FF_PERIODIC)
		pr_info("[hardware] Effect (Periodic) value: %u\n", effect->u.periodic.magnitude);

	chip->chip_effect_loaded = true;

	return 0;
}

static int __chip_haptics_erase(struct input_dev *dev, int effect_id) {
	struct haptics_chip *chip = input_get_drvdata(dev);

	pr_info("[hardware] %s: enter (id=%d)\n", __func__, effect_id);

	chip->chip_effect_loaded = false;
	chip->chip_is_playing = false;

	return 0;
}

static int __chip_haptics_playback(struct input_dev *dev, int effect_id, int val) {
	struct haptics_chip *chip = input_get_drvdata(dev);

	pr_info("[hardware] %s: enter (id=%d, val=%d)\n", __func__, effect_id, val);

	if (val)
		chip->chip_is_playing = true;
	else
		chip->chip_is_playing = false;

	return 0;
}

static void __chip_haptics_set_gain(struct input_dev *dev, u16 gain) {
	struct haptics_chip *chip = input_get_drvdata(dev);

	if (gain > 0x7fff)
		gain = 0x7fff;

	pr_info("[hardware] %s: enter (gain=%u)\n", __func__, (u32)gain);
}

/**/

static struct haptics_chip *global_haptics = NULL;

static bool __haptics_is_effect_supported(struct ff_effect *effect) {
	switch (effect->type) {
		case FF_CONSTANT:
			return true;

		case FF_PERIODIC:
			if (effect->u.periodic.waveform == FF_SINE)
				return true;

		fallthrough;
		default:
			return false;
	}
}
static int haptics_upload_effect(struct input_dev *dev, struct ff_effect *effect, struct ff_effect *old) {
	//TODO int ret;
	int ret;
	struct haptics_chip *chip = input_get_drvdata(dev);

	pr_info("%s: __enter, eid=%d, ef=%p, old=%p\n", __func__, effect->id, effect, old);

	if (effect->id > MAX_EFFECT_COUNT)
		return -EINVAL;

	//spin_lock_irq(&dev->event_lock);

	//Simulate uploading
	if (!__haptics_is_effect_supported(effect))
		return -ENOTSUPP;

	if (chip->effects_loaded[effect->id].is_loaded) { //stop and erase the old effect first
		/*if (old->id != effect->id)
			return -ENOTSUPP;*/

		//pr_info("%s: UPLOAD TO SAME SLOT, old slot =%d, new=%d\n", __func__, old->id, effect->id);
		pr_info("%s: UPLOAD TO SAME SLOT, slot=%d\n", __func__, effect->id, effect->id);

		ret = dev->ff->erase(dev, effect->id);
		if (ret) {
			pr_err("Error stopping old effect!!!!\n");

			return ret;
		}

		chip->effects_loaded[effect->id].is_loaded = true;
	}

	memcpy(&chip->effects_loaded[effect->id].effect, effect, sizeof(*effect));
	chip->effects_loaded[effect->id].is_loaded = true;

	//spin_unlock_irq(&dev->event_lock);

	return 0;
}

unsigned int __haptics_combine_effects(struct haptics_chip *chip, struct ff_effect *c) {
	unsigned int num;
	u32 level;
	struct ff_effect *eff;

	memset(c, 0x00, sizeof(c));

	//No delay
	c->type = FF_CONSTANT;
	c->id = 0; //Not used
	c->replay.length = 0x7FFF; //32s, should be infinite

	num = 0;
	for (unsigned int i=0; i<MAX_EFFECT_COUNT; i++) {
		eff = &chip->effects_loaded[i].effect;

		if (!chip->effects_loaded[i].is_playing)
			continue;

		num++;

		switch (eff->type) {
			case FF_CONSTANT:
				level += eff->u.constant.level;
				continue;

			case FF_PERIODIC:
				if (eff->u.periodic.waveform == FF_SINE)
					level += eff->u.periodic.magnitude;

			fallthrough;
			default:
				continue;
		}
	}

	c->u.constant.level = clamp(level, 1, 0x7FFF);

	return num;
}

static int __haptics_playback_start(struct input_dev *dev, struct ff_effect *eff) {
	int ret;

	//Upload the effect to chip
	ret = __chip_haptics_upload_effect(dev, eff, NULL);
	if (ret)
		return ret;

	//actually play it
	ret = __chip_haptics_playback(dev, 0, 1); //Start the playback
	if (ret)
		return ret;

	return 0;
}
static int __haptics_playback_stop(struct input_dev *dev) {
	int ret;

	ret = __chip_haptics_erase(dev, 0); //erase the only effect the chip supports
	if (ret) {
		pr_warn("Failed to stop chip\n");
		return ret;
	}

	return 0;
}

static int haptics_playback_start(struct input_dev *dev, int effect_id) {
	int ret;
	unsigned int eff_num;
	struct ff_effect combined_effect;
	struct ff_effect *eff;
	struct haptics_chip *chip = input_get_drvdata(dev);

	eff = &chip->effects_loaded[effect_id].effect;

	if (chip->effects_loaded[effect_id].is_playing) {
		pr_warn("Effect %u is already playing... skipping\n", effect_id);
		return 0;
	}

	if (chip->chip_is_playing || chip->is_combined_effect_playing) {
		//handle this case, that one (or more) effect is playing and another one wants to play
		pr_warn("request to start playing two effects!!!!, will combine them\n");

		//TODO: tidy & check
		chip->effects_loaded[effect_id].is_playing = true; //mark it as true so that the
		eff_num = __haptics_combine_effects(chip, &combined_effect);
		chip->effects_loaded[effect_id].is_playing = false;

		//Stop the old effect playback
		__haptics_playback_stop(dev);
		eff = &combined_effect;
		chip->is_combined_effect_playing = true;

		pr_info("Number of playing effects: %d\n", eff_num);

		//return -EBUSY;
	}

	ret = __haptics_playback_start(dev, eff);
	if (ret) {
		pr_err("Unable to start playback!\n");
		return ret;
	}

	//chip->combined_effect_amp += chip->effects_loaded[effect_id].effect.;
	chip->effects_loaded[effect_id].is_playing = true;

	return 0;
}
static int haptics_playback_stop(struct input_dev *dev, int effect_id) {
	int ret;
	unsigned int eff_num;
	struct ff_effect combined_effect;

	struct haptics_chip *chip = input_get_drvdata(dev);

	if (chip->is_combined_effect_playing) { //combined effect is playing, stop that effect
		chip->effects_loaded[effect_id].is_playing = false;
		eff_num = __haptics_combine_effects(chip, &combined_effect);
		chip->effects_loaded[effect_id].is_playing = true;

		//TODO: check out

		//Stop the old playback
		__haptics_playback_stop(dev);
		__haptics_playback_start(dev, &combined_effect);

		chip->effects_loaded[effect_id].is_playing = false;

		pr_info("Number of playing effects: %d\n", eff_num);

		if (eff_num <= 1)
			chip->is_combined_effect_playing = false;

		return 0;
	}

	ret = __haptics_playback_stop(dev);
	if (ret)
		return ret;

	chip->effects_loaded[effect_id].is_playing = false;

	return 0;
}
static int haptics_playback(struct input_dev *dev, int effect_id, int val) {
	struct haptics_chip *chip = input_get_drvdata(dev);

	pr_info("%s: __enter, eid=%d, val=%d\n", __func__, effect_id, val);

	if (effect_id > MAX_EFFECT_COUNT)
		return -EINVAL;

	//spin_lock_irq(&dev->event_lock);

	if (!chip->effects_loaded[effect_id].is_loaded)
		return -EINVAL;

	if (val) //start playback
		return haptics_playback_start(dev, effect_id);

	return haptics_playback_stop(dev, effect_id);
}


//TODO: maybe set the gain for combined effects?????
static int haptics_erase(struct input_dev *dev, int effect_id) {
	struct haptics_chip *chip = input_get_drvdata(dev);

	pr_info("%s: __enter\n", __func__);

	if (effect_id > MAX_EFFECT_COUNT)
		return -EINVAL;

	haptics_playback_stop(dev, effect_id); //Stop it and ignore the return value

	//chip->effects_loaded[effect_id].is_playing = false;
	chip->effects_loaded[effect_id].is_loaded = false;
	memset(&chip->effects_loaded[effect_id].effect, 0x00, sizeof(chip->effects_loaded[effect_id].effect));

	return 0;
}

static int __init haptics_init(void) {
	struct haptics_chip *chip;
	struct input_dev *input_dev;
	struct ff_device *ff_dev;
	int rc, count;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	input_dev->name = "qcom-hv-haptics";
	input_set_drvdata(input_dev, chip);
	chip->input_dev = input_dev;

	input_set_capability(input_dev, EV_FF, FF_CONSTANT);
	input_set_capability(input_dev, EV_FF, FF_PERIODIC);
	input_set_capability(input_dev, EV_FF, FF_GAIN);
	//if ((chip->effects_count != 0) || (chip->primitives_count != 0)) {
	//input_set_capability(input_dev, EV_FF, FF_CUSTOM); //Drop FF_CUSTOM
	//}

	memset(&chip->effects_loaded, 0x00, sizeof(chip->effects_loaded));
	//chip->playing_effect = NULL;

	count = MAX_EFFECT_COUNT;

	rc = input_ff_create(input_dev, count);
	if (rc < 0) {
		pr_err("create input FF device failed, rc=%d\n",
				rc);
		return rc;
	}

	ff_dev = input_dev->ff;
	ff_dev->upload = haptics_upload_effect;
	ff_dev->playback = haptics_playback;
	ff_dev->erase = haptics_erase;
	ff_dev->set_gain = __chip_haptics_set_gain;
	rc = input_register_device(input_dev);
	if (rc < 0) {
		pr_err("register input device failed, rc=%d\n",
				rc);
		goto destroy_ff;
	}

	/*INIT_WORK(&chip->external_upload_effect_work, __qcom_spmi_haptics_global_upload_work);
	INIT_WORK(&chip->external_playback_effect_work, __qcom_spmi_haptics_global_playback_work);
	INIT_WORK(&chip->external_erase_effect_work, __qcom_spmi_haptics_global_erase_work);*/

	global_haptics = chip;

	return 0;
destroy_ff:
	input_ff_destroy(chip->input_dev);
	return rc;
}

static void __exit haptics_remove(void) {
	struct haptics_chip *chip = global_haptics;

	input_unregister_device(chip->input_dev);

	kfree(global_haptics);

	global_haptics = NULL;

	return;
}

module_init(haptics_init);
module_exit(haptics_remove);

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. High-Voltage Haptics driver");
MODULE_LICENSE("GPL v2");
