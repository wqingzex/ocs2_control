/* SPDX-License-Identifier: GPL-2.0 */
/*
 * CAN-FD extension to Kunhong-Elec CAN products.
 *
 * Copyright (C) 2022-2026 Chengdu Kunhong Electronic Technology Co., Ltd <www.kunhong-elec.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Contact:      <support@cdkhdz.com>
 * Author:       Kunhong-Elec driver team <gene@cdkhdz.com>
 */
/*#define DEBUG*/
/*#undef DEBUG*/

#include "src/kcanfd_core.h"
#include "src/kcan_filter.h"

#ifdef DEBUG
#define DEBUG_WAIT_RD
#define DEBUG_WAIT_WR
#define DEBUG_OPEN
#define DEBUG_BITRATE
#else
//#define DEBUG_WAIT_RD
//#define DEBUG_WAIT_WR
//#define DEBUG_OPEN
//#define DEBUG_TRACE
//#define DEBUG_BITRATE
#endif

/* Timeout set to task waiting for room in the tx queue.
 * 0 means infinite.
 * != 0 implies that the wait might end with -ETIMEDOUT.
 */
//#define KCANFD_TIMEOUT_WAIT_FOR_WR	100
#define KCANFD_TIMEOUT_WAIT_FOR_WR	0

/* if defined, default init settings will be used in case any error in user
 * bittiming is found.
 */
//#define KCAN_USE_DEFBT_ON_ERROR

/* if defined, the controller is NOT reset if it is currently configured
 * with the same bittiming specs than the new one. This is especially useful
 * when calling ioctl(SET_INIT) next to open("/dev/kcanX").
 */
#define KCANFD_IGNORE_SAME_BITTIMING

extern u16 btr0btr1;
extern u32 kcan_def_dbitrate;

/*
 * Compute bitrate according to bittiming spec and Clock frequency
 */
static int kcan_bittiming_to_bitrate(struct kcan_bittiming *pbt, u32 clk_Hz)
{
	/* avoid div by 0 */
	if (pbt->brp) {
		u64 v64;

		if (!pbt->sjw)
			pbt->sjw = 1; /* ??? */

		pbt->sample_point = (KCAN_SAMPT_SCALE *
			(1 + pbt->tseg1)) / (1 + pbt->tseg1 + pbt->tseg2);

		pbt->bitrate = kcan_get_bps(clk_Hz, pbt);

		v64 = (u64 )pbt->brp * GHz;
		do_div(v64, clk_Hz);
		pbt->tq = (u32 )v64;

		return 0;
	}

	pr_warn(DEVICE_NAME
		": %s(): cannot compute bitrate from invalid brp=%d\n",
		__func__, pbt->brp);

	return -EINVAL;
}

int kcan_bittiming_normalize(struct kcan_bittiming *pbt,
			u32 clock_Hz, const struct kcanfd_bittiming_range *caps)
{
	int err = -EINVAL;

#ifdef DEBUG_BITRATE
	kcanfd_dump_bittiming(pbt, clock_Hz);
#endif

	/* NEW 8.2: always trust BRP/TEGx first:
	 * if brp valid, use these for computing the bitrate field */
	if (pbt->brp) {
		if (pbt->brp < caps->brp_min)
			pbt->brp = caps->brp_min;
		else if (pbt->brp > caps->brp_max)
			pbt->brp = caps->brp_max;

		if (pbt->tseg1 < caps->tseg1_min)
			pbt->tseg1 = caps->tseg1_min;
		else if (pbt->tseg1 > caps->tseg1_max)
			pbt->tseg1 = caps->tseg1_max;

		if (pbt->tseg2 < caps->tseg2_min)
			pbt->tseg2 = caps->tseg2_min;
		else if (pbt->tseg2 > caps->tseg2_max)
			pbt->tseg2 = caps->tseg2_max;

		err = kcan_bittiming_to_bitrate(pbt, clock_Hz);

	} else if (pbt->bitrate) {
		err = kcan_bitrate_to_bittiming(pbt, caps, clock_Hz);

#ifdef DEBUG_BITRATE
	/* else, if any of them is valid, it's an error! */
	} else {
		pr_info("%s: invalid bittiming specs: unable to normalize\n",
							DEVICE_NAME);
#endif
	}

	/* real bit-rate */
	if (!err)
		pbt->bitrate_real = clock_Hz /
		 	(pbt->brp * (pbt->tseg1 + pbt->tseg2 + 1));

#ifdef DEBUG_BITRATE
	kcanfd_dump_bittiming(pbt, clock_Hz);
#endif
	return err;
}

/*
 * Convert SJA1000 BTR0BTR1 16-bits value into a generic bittiming
 * representation
 */
struct kcan_bittiming *kcan_btr0btr1_to_bittiming(struct kcan_bittiming *pbt,
						  u16 btr0btr1)
{
	pbt->sjw = 1 + ((btr0btr1 & 0xc000) >> 14);
	pbt->brp = 1 + ((btr0btr1 & 0x3f00) >> 8);
	pbt->tsam = (btr0btr1 & 0x0080) >> 7;
	pbt->tseg2 = 1 + ((btr0btr1 & 0x0070) >> 4);
	pbt->tseg1 = 1 + (btr0btr1 & 0x000f);
	pbt->bitrate = 0;

	kcan_bittiming_to_bitrate(pbt, 8*MHz);

	return pbt;
}

/* Convert old CAN 2.0 init object into new-style CAN-FD init object. */
struct kcanfd_init *kcan_init_to_fd(struct kcandev *dev,
				    struct kcanfd_init *pfdi,
				    const TKCANInit *pi)
{
	/* DON'T memset('\0') the struct kcanfd_init since it may already 
	 * contain data (or other CANFD specific values). Caller HAS TO
	 * initialize the struct kcanfd_init by himself!
	 */
	memset(&pfdi->data, '\0', sizeof(struct kcan_bittiming));

	if (!pfdi->clock_Hz)
		pfdi->clock_Hz = dev->sysclock_Hz;

	if (!(pi->ucCANMsgType & MSGTYPE_EXTENDED))
		pfdi->flags |= KCANFD_INIT_STD_MSG_ONLY;

	if (pi->ucListenOnly)
		pfdi->flags |= KCANFD_INIT_LISTEN_ONLY;

	if (pi->wBTR0BTR1) {
		kcan_btr0btr1_to_bittiming(&pfdi->nominal, pi->wBTR0BTR1);

		if (pfdi->clock_Hz != 8*MHz) {

			/* compute new bittiming according to the real clock */
			kcan_bitrate_to_bittiming(&pfdi->nominal,
						  dev->bittiming_caps,
						  pfdi->clock_Hz);
		}
	}

#ifdef DEBUG_BITRATE
	pr_info(DEVICE_NAME ": %s(): 8xMHz btr0btr1=%04xh =>\n",
		__func__, pi->wBTR0BTR1);
	kcanfd_dump_bittiming(&pfdi->nominal, pfdi->clock_Hz);
#endif
	return pfdi;
}

/* Release a device: after that call, the device is no more opened.
 * WARNING: caller should normally wait for the output fifo to be empty
 *          before calling kcanfd_dev_reset()
 */
void __kcanfd_dev_reset(struct kcandev *dev)
{
	/* close Tx engine BEFORE device_release() so that device Tx resources
	 * will be able to be safety released from writing task.
	 */
	kcan_lock_irqsave_ctxt flags;

#if defined(DEBUG_TRACE) || defined(DEBUG_OPEN)
	pr_info(DEVICE_NAME ": %s(%s CAN%u bus=%u)\n", __func__,
		dev->adapter->name, kcan_idx(dev)+1, dev->bus_state);
#endif

	dev->lock_irq(dev, &flags);
	kcan_set_tx_engine(dev, TX_ENGINE_CLOSED);
	dev->unlock_irq(dev, &flags);

	/* release the device */
	if (dev->device_release)
		dev->device_release(dev);

	dev->flags &= ~KCAN_DEV_OPENED;
	kcan_set_bus_state(dev, KCANFD_UNKNOWN);

	if (dev->adapter->opened_count > 0)
		dev->adapter->opened_count--;

#if defined(DEBUG) || defined(DEBUG_OPEN)
	pr_info(DEVICE_NAME ": %s CAN%u rx=%u tx=%u\n",
			dev->adapter->name, kcan_idx(dev)+1,
			dev->rx_frames_counter, dev->tx_frames_counter);
#endif
}

/* default allowed msgs mask equals all messages except ERR_MSG */
#define KCANFD_ALLOWED_MSG_DEFAULT      (KCANFD_ALLOWED_MSG_CAN|\
					 KCANFD_ALLOWED_MSG_RTR|\
					 KCANFD_ALLOWED_MSG_EXT|\
					 KCANFD_ALLOWED_MSG_STATUS)

void kcanfd_dev_open_init(struct kcandev *dev)
{
	/* nofilter */
	dev->acc_11b.code = 0;
	dev->acc_11b.mask = CAN_MAX_STANDARD_ID;
	dev->acc_29b.code = 0;
	dev->acc_29b.mask = CAN_MAX_EXTENDED_ID;

	dev->tx_iframe_delay_us = 0;
	dev->allowed_msgs = KCANFD_ALLOWED_MSG_DEFAULT;

	/* Nope! Since kcan v8.6, time sync is handled as soon as the device
	 * hw is probed (mainly USB devices): starting/stopping CM when
	 * opening/closing the KCAN-Chip does not work, since ts in CM are not
	 * based like ts in CAN msgs...
	 * (see also: UCAN_USB_START_CM_AT_OPEN)
	 */
	//kcan_sync_init(dev);

	/* New: reset these counters too */
	dev->dwErrorCounter = 0;
	dev->rx_irq_counter = 0;
	dev->tx_irq_counter = 0;

	kcan_set_bus_state(dev, KCANFD_UNKNOWN);
}

static int kcanfd_fix_init_clock(struct kcandev *dev, struct kcanfd_init *pfdi)
{
	const struct kcanfd_available_clocks *pc = dev->clocks_list;
	int i, err;

	for (i = 0; i < pc->count; i++)
		if (pfdi->clock_Hz == pc->list[i].clock_Hz)
			break;

	/* user clock not found in device clocks list */
	if (i >= pc->count) {

		if (pfdi->nominal.brp) {

			/* to be compatible with old API, accept 8*MHz clocks
			 * and consider that BRP values and so on come from
			 * SJA1000 BTR0BTR1.
			 */
			if (pfdi->clock_Hz == 8*MHz)
				return 0;

			/* otherwise, try to convert bittiming into bitrate
			 * using the user (wrong) clock
			 */
			err = kcan_bittiming_to_bitrate(&pfdi->nominal,
							pfdi->clock_Hz);
			if (err) {
				pr_err(DEVICE_NAME ": unable to convert"
				       " user nominal bittiming with wrong clk"
				       "=%uMHz (err %d)\n",
				       pfdi->clock_Hz, err);

				return err;
			}

			/* rst BRP to force using bitrate value next */
			pfdi->nominal.brp = 0;
		}

		/* do the same for data bittiming */
		if (pfdi->flags & KCANFD_INIT_FD) {
			if (pfdi->data.brp) {
				err = kcan_bittiming_to_bitrate(&pfdi->data,
								pfdi->clock_Hz);
				if (err) {
					pr_err(DEVICE_NAME ": unable to convert"
					       " user data bittiming with wrong"
					       "clk=%uMHz (err %d)\n",
					       pfdi->clock_Hz, err);

					return err;
				}
			}
		}

		/* fix user clock with device default one to convert
		 * the bitrate value in valid bittiming
		 */
		pfdi->clock_Hz = dev->sysclock_Hz;
	}

	return 0;
}

/* consider pfdi ok */
static int __kcanfd_dev_open(struct kcandev *dev, struct kcanfd_init *pfdi)
{
	struct kcanfd_init tmp_init;
	int err;

#if defined(DEBUG_TRACE) || defined(DEBUG_OPEN)
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1);
#endif

	/* do this BEFORE calling open callbacks, to be ready to handle
	 * timestamps conversion if any msg is posted by them. These two init
	 * steps are made again at the end, as usual.
	 */
	kcan_gettimeofday(&dev->init_timestamp);

	kcanfd_copy_init(&tmp_init, &dev->init_settings);
	kcanfd_copy_init(&dev->init_settings, pfdi);

	kcanfd_dev_open_init(dev);

	/* use old APi entry (with wBTR0BTR1) if CAN controller clock
	 * is 8 MHz.
	 */
	if (pfdi->clock_Hz == 8*MHz) {
		TKCANInit init;

		pfdi->flags &= ~(KCANFD_INIT_FD|KCANFD_INIT_FD_NON_ISO);
		memset(&pfdi->data, '\0', sizeof(struct kcan_bittiming));

		init.ucCANMsgType = (pfdi->flags & KCANFD_INIT_STD_MSG_ONLY) ?
						0 : MSGTYPE_EXTENDED;
		init.ucListenOnly = !!(pfdi->flags & KCANFD_INIT_LISTEN_ONLY);

		/* we're sure that the bittiming are ok: no need to check nor
		 * convert them again.
		 */
		init.wBTR0BTR1 = kcan_bittiming_to_btr0btr1(&pfdi->nominal);
		if (!init.wBTR0BTR1) {
			init.wBTR0BTR1 = sja1000_bitrate(
				dev->def_init_settings.nominal.bitrate,
				dev->def_init_settings.nominal.sample_point,
				dev->def_init_settings.nominal.sjw);
#ifdef DEBUG
			pr_err(DEVICE_NAME
				": %s CAN%u using default BTR0BTR1\n",
				dev->adapter->name, kcan_idx(dev)+1);
#endif
		}

#ifdef DEBUG_BITRATE
		pr_info(DEVICE_NAME
			": %s(CAN%d): time=%u.%06us: opening with "
			"BTR0BTR1=%04xh (bitrate=%u sp=%u) flags=%08xh\n",
			dev->adapter->name, kcan_idx(dev)+1,
			(u32 )dev->init_timestamp.tv_sec,
			(u32 )dev->init_timestamp.tv_usec,
			init.wBTR0BTR1, pfdi->nominal.bitrate,
			pfdi->nominal.sample_point, pfdi->flags);
		pr_info(DEVICE_NAME
			": nominal [brp=%d tseg1=%d tseg2=%d sjw=%d]\n",
			pfdi->nominal.brp, pfdi->nominal.tseg1,
			pfdi->nominal.tseg2, pfdi->nominal.sjw);
#endif

		/* device is not CAN-FD capable: forward to (old) CAN 2.0 API */
		err = dev->device_open(dev, init.wBTR0BTR1,
						init.ucCANMsgType,
						init.ucListenOnly);

	/* use the new API entry point (erroneously called "open_fd") that 
	 * enable to setup the true bitimings according to a given clock, as
	 * well as defining a data bitrate (CAN-FD)
	 */
	} else {

#ifdef DEBUG_BITRATE
		pr_info(DEVICE_NAME
			": %s(CAN%d): time=%u.%06us: opening with "
			"clk=%u bitrate=%u dbitrate=%u (flags=%08xh)\n",
			dev->adapter->name, kcan_idx(dev)+1,
			(u32 )dev->init_timestamp.tv_sec,
			(u32 )dev->init_timestamp.tv_usec,
			pfdi->clock_Hz, pfdi->nominal.bitrate,
			pfdi->data.bitrate, pfdi->flags);
		pr_info(DEVICE_NAME
			": nominal [brp=%d tseg1=%d tseg2=%d sjw=%d sp=%u]\n",
			pfdi->nominal.brp, pfdi->nominal.tseg1,
			pfdi->nominal.tseg2, pfdi->nominal.sjw,
			pfdi->nominal.sample_point);
		pr_info(DEVICE_NAME
			": data [brp=%d tseg1=%d tseg2=%d sjw=%d sp=%u]\n",
			pfdi->data.brp, pfdi->data.tseg1, pfdi->data.tseg2,
			pfdi->data.sjw, pfdi->data.sample_point);
#endif
		err = dev->device_open_fd(dev, pfdi);
	}

	if (!err) {
		kcan_lock_irqsave_ctxt flags;

		dev->flags |= KCAN_DEV_OPENED;
		dev->opened_index = dev->adapter->opened_count++;

		kcan_gettimeofday(&dev->init_timestamp);

		/* remember the init settings for further usage */
		kcanfd_copy_init(&dev->init_settings, pfdi);

		/* default tx engine state: ready to start! */
		dev->lock_irq(dev, &flags);

		if (dev->locked_tx_engine_state == TX_ENGINE_CLOSED)
			kcan_set_tx_engine(dev, TX_ENGINE_STOPPED);

		dev->unlock_irq(dev, &flags);

		/* now bus load timer is started when bus state goes to
		 * ERROR_ACTIVE (see kcan_set_bus_state()) when not in
		 * NETDEV mode.
		 */
		return 0;
	}

	/* since these settings are bad, should undo the above 
	 * kcanfd_copy_init()
	 */
	kcanfd_copy_init(&dev->init_settings, &tmp_init);

	return err;
}

/*
 * Verify settings in struct kcanfd_init and fill defaults with their values
 */
static int kcanfd_check_init(struct kcandev *dev, struct kcanfd_init *pfdi)
{
	int err;

#ifdef DEBUG_BITRATE
	kcanfd_dump_bittiming(&pfdi->nominal, pfdi->clock_Hz);
	if (pfdi->flags & KCANFD_INIT_FD)
		kcanfd_dump_bittiming(&pfdi->data, pfdi->clock_Hz);
#endif

	/* check init settings: */
	if (pfdi->flags & KCANFD_INIT_FD)
		if (!dev->dbittiming_caps ||
		    !(dev->features & KCAN_DEV_FD_RDY)) {

			pr_err(DEVICE_NAME ": %s CAN%u: "
				"can't be opened in CAN FD mode\n",
				dev->adapter->name, kcan_idx(dev)+1);

			return -EINVAL;
		}

	/* if user has given no clock, use device current clock */
	if (!pfdi->clock_Hz)
		pfdi->clock_Hz = dev->sysclock_Hz;

	/* otherwise, check if user clock is valid. If not AND
	 * if bittiming are used, then convert them into values that
	 * match the device default clock.
	 */
	else {
		err = kcanfd_fix_init_clock(dev, pfdi);
		if (err)
			return err;
	}

	/* if user has not given any bitrate nor BRP, setup default
	 * settings for the nominal bitrate
	 */
	if (!pfdi->nominal.bitrate && !pfdi->nominal.brp)
		pfdi->nominal = dev->def_init_settings.nominal;

	/* be sure that bitrate and brp,tsegx,sjw are set */
	err = kcan_bittiming_normalize(&pfdi->nominal,
				pfdi->clock_Hz, dev->bittiming_caps);
	if (err) {
#ifdef KCAN_USE_DEFBT_ON_ERROR
		pr_err(DEVICE_NAME
			": %s CAN%u: error %d in user nominal "
			"bittiming: using default\n",
			dev->adapter->name, kcan_idx(dev)+1, err);
		pfdi->nominal = dev->def_init_settings.nominal;
#else
		pr_err(DEVICE_NAME
			": %s CAN%u: error %d in user nominal bittiming:\n",
			dev->adapter->name, kcan_idx(dev)+1, err);

		kcanfd_dump_bittiming(&pfdi->nominal, pfdi->clock_Hz);

		return err;
#endif
	}

#ifdef DEBUG_OPEN
	pr_info(DEVICE_NAME ": opening %s CAN%u with nominal bittiming:\n",
		dev->adapter->name, kcan_idx(dev)+1);

	kcanfd_dump_bittiming(&pfdi->nominal, pfdi->clock_Hz);
#endif

	/* do the same if CAN-FD */
	if (pfdi->flags & KCANFD_INIT_FD) {

		/* if user has not given any bitrate nor BRP,
		 * setup default settings for the data bitrate
		 */
		if (!pfdi->data.bitrate && !pfdi->data.brp)
			pfdi->data = dev->def_init_settings.data;

		/* be sure that bitrate and brp,tsegx,sjw are set */
		err = kcan_bittiming_normalize(&pfdi->data,
				pfdi->clock_Hz, dev->dbittiming_caps);
		if (err) {
#ifdef KCAN_USE_DEFBT_ON_ERROR
			pr_err(DEVICE_NAME
				": %s CAN%u: error %d in user data "
				"bittiming: using default\n",
				dev->adapter->name, kcan_idx(dev)+1,
				err);

			pfdi->data = dev->def_init_settings.data;
#else
			pr_err(DEVICE_NAME ": %s CAN%u: error %d in user data "
				"bittiming\n", dev->adapter->name,
				kcan_idx(dev)+1, err);

			kcanfd_dump_bittiming(&pfdi->data,
					      pfdi->clock_Hz);

			return err;
#endif
		}

		/* For CAN FD the data bitrate has to be >= the nominal
		 * bitrate
		 */
		if (pfdi->data.bitrate < pfdi->nominal.bitrate) {
			pr_err(DEVICE_NAME ": %s CAN%u data bitrate "
				"(%u bps) should be greater than "
				"nominal bitrate (%u bps)\n",
				dev->adapter->name, kcan_idx(dev)+1,
				pfdi->data.bitrate,
				pfdi->nominal.bitrate);

			return -EINVAL;
		}
#ifdef DEBUG_OPEN
		pr_info(DEVICE_NAME ": opening %s CAN%u with data bittiming:\n",
			dev->adapter->name, kcan_idx(dev)+1);

		kcanfd_dump_bittiming(&pfdi->data, pfdi->clock_Hz);
#endif
	}

	return 0;
}

int kcanfd_dev_open(struct kcandev *dev, struct kcanfd_init *pfdi)
{
	int err;

#if defined(DEBUG_TRACE) || defined(DEBUG_OPEN)
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1);
#endif

	if (pfdi->flags & KCANFD_INIT_BTR_NOK) {
		err = kcanfd_check_init(dev, pfdi);
		if (err)
			return err;

		/* no need to check them next */
		pfdi->flags &= ~KCANFD_INIT_BTR_NOK;
	}

	return __kcanfd_dev_open(dev, pfdi);
}

#ifdef KCANFD_IGNORE_SAME_BITTIMING
/*
 * Compare bitrate settings according to the following criteria:
 *
 * - clock MUST be the same. If clock_Hz is 0, then default device clock is
 *   used for comparison.
 *
 * Return:
 *
 * < 0 if error
 * == 0 if bittiming settings are equal
 * > 0 if bittiming settings are different
 */
static int kcanfd_are_bittiming_equal(struct kcandev *dev,
				      struct kcanfd_init *pfi1,
				      struct kcanfd_init *pfi2)
{
	int err;

	/* first, be sure that all settings are filled and correct */
	if (pfi1->flags & KCANFD_INIT_BTR_NOK) {
		err = kcanfd_check_init(dev, pfi1);
		if (err)
			return err;

		/* no need to check them next */
		pfi1->flags &= ~KCANFD_INIT_BTR_NOK;
	}

	if (pfi2->flags & KCANFD_INIT_BTR_NOK) {
		err = kcanfd_check_init(dev, pfi2);
		if (err)
			return err;

		/* no need to check them next */
		pfi2->flags &= ~KCANFD_INIT_BTR_NOK;
	}

#ifdef DEBUG_OPEN
	pr_info(DEVICE_NAME ": %s() comparing nominal:\n", __func__);
	kcanfd_dump_bittiming(&pfi1->nominal, pfi1->clock_Hz);
	kcanfd_dump_bittiming(&pfi1->nominal, pfi1->clock_Hz);
#endif

	/* now compare settings */
	if (pfi1->clock_Hz != pfi2->clock_Hz)
		return 1;

	if (pfi1->nominal.bitrate != pfi2->nominal.bitrate)
		return 2;

	if (pfi1->nominal.brp != pfi2->nominal.brp ||
	    pfi1->nominal.tseg1 != pfi2->nominal.tseg1 ||
	    pfi1->nominal.tseg2 != pfi2->nominal.tseg2 ||
	    pfi1->nominal.sjw != pfi2->nominal.sjw)
		return 3;

	if (pfi1->flags & KCANFD_INIT_FD) {

		if (!(pfi2->flags & KCANFD_INIT_FD))
			return 4;

		if (pfi1->flags & KCANFD_INIT_FD_NON_ISO) {
			if (!(pfi2->flags & KCANFD_INIT_FD_NON_ISO))
				return 11;
		} else if (pfi2->flags & KCANFD_INIT_FD_NON_ISO)
			return 12;

#ifdef DEBUG_OPEN
		pr_info(DEVICE_NAME ": %s() comparing data:\n", __func__);
		kcanfd_dump_bittiming(&pfi1->data, pfi1->clock_Hz);
		kcanfd_dump_bittiming(&pfi2->data, pfi2->clock_Hz);
#endif
		if (pfi1->data.bitrate != pfi2->data.bitrate)
			return 13;

		if (pfi1->nominal.brp != pfi2->nominal.brp ||
		    pfi1->nominal.tseg1 != pfi2->nominal.tseg1 ||
		    pfi1->nominal.tseg2 != pfi2->nominal.tseg2 ||
		    pfi1->nominal.sjw != pfi2->nominal.sjw)
			return 14;

	} else if (pfi2->flags & KCANFD_INIT_FD)
		return 4;

	if (pfi1->flags & KCANFD_INIT_LISTEN_ONLY) {
		if (!(pfi2->flags & KCANFD_INIT_LISTEN_ONLY))
			return 5;
	} else if (pfi2->flags & KCANFD_INIT_LISTEN_ONLY)
		return 6;

	return 0;
}
#endif

/*
 * int kcanfd_ioctl_set_init(struct kcandev *dev, struct kcanfd_init *pfdi)
 *
 * User-only.
 *
 * Called only from:
 *
 * - ioctl(KCAN_INIT)
 * - ioctl(KCANFD_SET_INIT)
 * - write("i xxx")
 */
int kcanfd_ioctl_set_init(struct kcandev *dev, struct kcanfd_init *pfdi)
{
	int err = 0;

#if defined(DEBUG_TRACE) || defined(DEBUG_OPEN)
	pr_info(DEVICE_NAME ": %s(%s CAN%u bus=%u): nOpenPaths=%d\n", __func__,
		dev->adapter->name, kcan_idx(dev)+1, dev->bus_state,
		dev->nOpenPaths);
#endif

	if ((dev->flags & KCAN_DEV_OPENED) && (dev->nOpenPaths > 1)) {
		pr_err(DEVICE_NAME
		       ": %s CAN%u can't be initialized when opened %d times\n",
		       dev->adapter->name, kcan_idx(dev)+1, dev->nOpenPaths);

		err = -EBUSY;
		goto lbl_exit;
	}

	/* sanitize */
	if (!(dev->features & KCAN_DEV_BUSLOAD_RDY))
		pfdi->flags &= ~KCANFD_INIT_BUS_LOAD_INFO;

	/* force bittiming checking and remember init comes from userland */
	pfdi->flags |= KCANFD_INIT_BTR_NOK|KCANFD_INIT_USER;

#ifdef KCANFD_IGNORE_SAME_BITTIMING
	/* if the device is not yet configured then do it */
	if (!(dev->flags & KCAN_DEV_OPENED))
		goto lbl_do_reset;

	/* ignoring same bittimings needs of course that the state of the bus
	 * is correct. Unfortunately, it might be UNKNOWN if ioctl(SET_INIT)
	 * is called next to open().
	 */
	if (dev->bus_state > KCANFD_ERROR_ACTIVE)
		goto lbl_do_reset;

	/* if bittiming are different, donot flush fifos but reset controller
	 * only
	 */
	err = kcanfd_are_bittiming_equal(dev, &dev->init_settings, pfdi);
	if (err)
		goto lbl_do_reset;

	/* bittiming are equal. If controller has been opened WITHOUT
	 * bus load option, and if new init wants them, then controller
	 * must be closed then open.
	 */
	if (!(dev->features & KCAN_DEV_BUSLOAD_RDY) ||
	    (dev->init_settings.flags & KCANFD_INIT_BUS_LOAD_INFO) ||
	    !(pfdi->flags & KCANFD_INIT_BUS_LOAD_INFO)) {

		/* Don't need to change the controller state. Just
		 * update the device init_settings fields with
		 * other initialization settings that don't deal with
		 * the controller itself.
		 */
		kcanfd_copy_init(&dev->init_settings, pfdi);

#ifndef NETDEV_SUPPORT
		/* in case controler is not changed, then donot reset rx fifo
		 * either so that any event that might have come in between
		 * won't be lost...
		 */

		/* send back a STATUS[bus_state] */
		kcan_post_bus_state(dev);
#endif
		return 0;
	}

lbl_do_reset:
#endif

	/* flush Tx fifo (only): Rx fifo may contain unread msgs) */
	kcan_fifo_reset(&dev->tx_fifo);

	dev->wCANStatus &= ~CAN_ERR_XMTFULL;

	/* do reset the device */
	kcanfd_dev_reset(dev);

#ifndef NETDEV_SUPPORT
#ifdef KCAN_USES_O_ACCMODE_HACK
	/* User sets a bitrate that is not the current/default one.
	 * Unfortunately, between open() and this initialization, one or several
	 * msgs may already have been pushed into rx fifo. The problem is
	 * if we don't reset it now, the same STATUS[ACTIVE] may be present
	 * twice (because all "dev->posted" fields have been reset by
	 * kcanfd_dev_reset() ->...-> kcan_init_session_counters()).
	 */
	kcan_fifo_reset(&dev->rx_fifo);

	dev->wCANStatus &= ~CAN_ERR_OVERRUN;
#else
	/* When O_ACCMODE is not handled by kcan, then donot reset Rx fifo:
	 * user may read twice the same STATUS but frames read between
	 * open() and the following new init won't be lost.
	 */
#endif
#endif

	/* then reopen it with the user settings */
	err = kcanfd_dev_open(dev, pfdi);

lbl_exit:
	return err;
}

int kcanfd_ioctl_get_init(struct kcandev *dev, struct kcanfd_init *pfdi)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	memcpy(pfdi, &dev->init_settings, sizeof(*pfdi));

	return 0;
}

int kcanfd_ioctl_reset(struct kcandev *dev, unsigned long flags)
{
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u, flgs=%08lxh)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1, flags);
#endif

	if (flags & KCANFD_RESET_RXFIFO) {
#ifndef NETDEV_SUPPORT
		err = kcan_fifo_reset(&dev->rx_fifo);
		if (err)
			return err;
#endif
		dev->wCANStatus &= ~CAN_ERR_OVERRUN;
	}

	if (flags & KCANFD_RESET_TXFIFO) {
		err = kcan_fifo_reset(&dev->tx_fifo);
		if (err)
			return err;

		dev->wCANStatus &= ~CAN_ERR_XMTFULL;
	}

	if (flags & KCANFD_RESET_CTRLR) {
		kcan_lock_irqsave_ctxt flags;

		if (!dev->device_reset)
			return -ENOTSUPP;

		dev->lock_irq(dev, &flags);

		dev->rx_frames_counter = 0;
		dev->tx_frames_counter = 0;
		dev->rx_error_counter = 0;
		dev->tx_error_counter = 0;
		dev->dwErrorCounter = 0;

		dev->time_sync.ts_fixed = 0;

		kcan_set_tx_engine(dev, TX_ENGINE_STOPPED);

		dev->unlock_irq(dev, &flags);

		kcan_set_bus_state(dev, KCANFD_UNKNOWN);

		err = dev->device_reset(dev);
	}

	return err;
}

/* add a message filter_element into the filter chain or delete all
 * filter_elements
 */
int kcanfd_ioctl_add_filter(struct kcandev *dev, struct kcanfd_msg_filter *pf)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, kcan_idx(dev)+1);
#endif

	/* filter == NULL -> delete the filter_elements in the chain */
	if (!pf) {
		kcan_delete_filter_all(dev->filter);
		return 0;
	}

	return kcan_add_filter(dev->filter,
		               pf->id_from, pf->id_to, pf->msg_flags);
}

/* add several message filter_element into the filter chain.
 */
int kcanfd_ioctl_add_filters(struct kcandev *dev,
			     struct kcanfd_msg_filters *pfl)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, kcan_idx(dev)+1);
#endif

	/* filter == NULL -> delete the filter_elements in the chain */
	if (!pfl) {
		kcan_delete_filter_all(dev->filter);
		return 0;
	}

	return kcan_add_filters(dev->filter, pfl->list, pfl->count);
}

/* get several message filter_element from the filter chain.
 */
int kcanfd_ioctl_get_filters(struct kcandev *dev,
			     struct kcanfd_msg_filters *pfl)
{
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, kcan_idx(dev)+1);
#endif

	/* filter == NULL -> return the current nb of filters in the chain */
	if (!pfl)
		return kcan_get_filters_count(dev->filter);

	err = kcan_get_filters(dev->filter, pfl->list, pfl->count);
	if (err < 0) {
		pfl->count = 0;
		return err;
	}

	pfl->count = err;
	return 0;
}

int kcanfd_ioctl_get_state(struct kcandev *dev, struct kcanfd_state *pfds)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, kcan_idx(dev)+1);
#endif

	pfds->ver_major = KCAN_VERSION_MAJOR;
	pfds->ver_minor = KCAN_VERSION_MINOR;
	pfds->ver_subminor = KCAN_VERSION_SUBMINOR;

	pfds->tv_init = dev->init_timestamp;

	pfds->bus_state = dev->bus_state;
	pfds->device_id = dev->device_alt_num;

	pfds->open_counter = dev->nOpenPaths;
	pfds->filters_counter = kcan_get_filters_count(dev->filter);

	pfds->hw_type = dev->wType;
	pfds->channel_number = kcan_idx(dev);

#ifdef USB_SUPPORT
	if (dev->wType == HW_USB_X6) {
		struct kcan_usb_interface *usb_if;

		usb_if = kcan_usb_get_if(dev);

		pfds->channel_number += usb_if->index * usb_if->can_count;
	}
#endif
 
	pfds->can_status = dev->wCANStatus;
	pfds->bus_load = dev->bus_load;

	pfds->tx_max_msgs = dev->tx_fifo.nCount;
	pfds->tx_pending_msgs = dev->tx_fifo.nStored;

#ifndef NETDEV_SUPPORT
	pfds->rx_max_msgs = dev->rx_fifo.nCount;
	pfds->rx_pending_msgs = dev->rx_fifo.nStored;
#else
	pfds->rx_max_msgs = 0;
	pfds->rx_pending_msgs = 0;
#endif
	pfds->tx_error_counter = dev->tx_error_counter;
	pfds->rx_error_counter = dev->rx_error_counter;
	pfds->tx_frames_counter = dev->tx_frames_counter;
	pfds->rx_frames_counter = dev->rx_frames_counter;

	pfds->host_time_ns = dev->time_sync.tv_ns;
	pfds->hw_time_ns = dev->time_sync.ts_us * 1000;

	return 0;
}

#ifndef NETDEV_SUPPORT
/*
 * This function is called while Rx FIFO is locked when reading into.
 */
static int kcan_rxfifo_get_completion(FIFO_MANAGER *anchor,
				      void *item,
				      void *arg,
				      int err)
{
	/* if read succeeded */
	if (err >= 0) {
		struct kcandev *dev = (struct kcandev *)arg;
#ifdef FIFO_PRE_ROUTINE
		struct kcanfd_rxmsg *rx = (struct kcanfd_rxmsg *)item;
#endif

#ifdef DEBUG_WAIT_RD
		pr_info(DEVICE_NAME
			": %s(%s CAN%u): still %u items in Rx queue\n",
			__func__, dev->adapter->name, kcan_idx(dev)+1, err);
#endif

#ifdef kcan_event_clear
		/* if rx fifo is now empty, then the corresponding event
		 * should be cleared now.
		 */
		if (!err)
			kcan_event_clear(&dev->in_event);
#endif

		/* sure that the FIFO is no more FULL. If it was, then notify
		 * application of a new STATUS[] just to clear kcanview taskbar!
		 */
		if (dev->wCANStatus & CAN_ERR_OVERRUN) {
			dev->wCANStatus &= ~CAN_ERR_OVERRUN;

			/* since kcan_fifo_get() returns count of items still
			 * in fifo, then force return value to be greater so
			 * that caller will be informed of that situation.
			 */
			err = kcan_fifo_items_max(anchor) + 1;
		}

#ifdef FIFO_PRE_ROUTINE
		/* keep a copy of msg read by user to prevent driver from
		 * posted it again
		 */
		switch (rx->msg.type) {
		case KCANFD_TYPE_STATUS:
			dev->posted.status = *rx;
			break;
		case KCANFD_TYPE_ERROR_MSG:
			dev->posted.error = *rx;
			break;
		}
#endif

	/* otherwise, change -ENODATA into chardev -EAGAIN */
	} else {
		err = -EAGAIN;
	}

	return err;
}
#endif

static int kcanfd_recv_msg(struct kcandev *dev, struct kcanfd_rxmsg *pf,
			   struct kcanusr *ctx)
{
#ifdef NETDEV_SUPPORT
	return -EAGAIN;		/* be compatible with old behaviour */
#else
	FIFO_MANAGER *rx_fifo = &dev->rx_fifo;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(kcan%u): is_plugged=%d nOpenPaths=%d\n",
		__func__, dev->nMinor, dev->is_plugged, dev->nOpenPaths);
#endif
	do {
		/* if the device has been plugged out while waiting,
		 * or if any task is closing it
		 */
		if (!dev->is_plugged || !dev->nOpenPaths) {
			err = -ENODEV;
			break;
		}

		/* get data from fifo */
		err = kcan_fifo_get_ex(rx_fifo, pf,
				       kcan_rxfifo_get_completion, dev);

		if (err > kcan_fifo_items_max(rx_fifo)) {

			/* fifo is no more full: since RX_OVERFLOW is pushed
			 * whatever the device open mode is blocking or not,
			 * unconditionnaly post a STATUS msg here when the fifo
			 * is no more full...
			 */
#ifndef NETDEV_SUPPORT
			kcan_post_bus_state(dev);
#endif
		}

		/* support nonblocking read if requested */
		else if (err == -EAGAIN) {
			if (!ctx || (ctx->open_flags & O_NONBLOCK)) {
#ifdef KCAN_NO_EWOULDBLOCK
				/* ioctl(KCAN_READ_MSG) = 0 */
				err = kcan_init_rxmsg(dev, pf,
						      KCANFD_TYPE_STATUS,
						      KCANFD_RX_EMPTY,
						      KCANFD_ERROR_INTERNAL);
#else
				/* ioctl(KCAN_READ_MSG) = -EWOULDBLOCK */
				break;
#endif
			}
		}

		/* got a msg from rx fifo */
		if (err >= 0) {

			kcan_sync_timestamps(dev, pf);

			err = 0;
			break;
		}

		/* check whether the task is able to wait:
		 * Linux: always!
		 * RT: depends on the RT context of the running task
		 */
		if (!kcan_task_can_wait()) {
			pr_info(DEVICE_NAME
				": %s(%u): ABNORMAL task unable to wait!\n",
				__func__, __LINE__);
			break;
		}

		/* sleep until some msg is available. */
#ifdef DEBUG_WAIT_RD
		pr_info("%s: %s(%u): waiting for some msgs to read...\n",
			DEVICE_NAME, __func__, __LINE__);
#endif

		/* task might go to sleep: unlock current dev */
		kcan_unlock_dev(dev);

		/* wait for some msg in the Rx queue.
		 *
		 * Note: ^C may occur while waiting. In RT, preemption can 
		 * schedule another task that might call close() while we're
		 * always waiting here.
		 * - If the event is destroyed by some other task, the below
		 *   call fails with err=-EIDRM(43).
		 * - if some other task deletes this waiting task, this tasks
		 *   is first unblocked, thus err=-EINTR(4).
		 */
		err = kcan_event_wait(dev->in_event,
					!dev->is_plugged ||
					!kcan_fifo_empty(rx_fifo));

		kcan_lock_dev(dev);

#ifdef DEBUG_WAIT_RD
		pr_info(DEVICE_NAME
			": end of waiting for rx fifo not empty: err=%d\n",
			err);
#endif

	} while (err >= 0);

	/* Note: ERESTARTSYS == 512 */
	return (err == -ERESTARTSYS) ? -EINTR : err;
#endif
}

/* this function SHOULD be used with dev->isr_lock locked */
int __kcan_dev_start_writing(struct kcandev *dev, struct kcanusr *ctx)
{
	int err = 0;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME
		": %s(%s CAN%u) flags=%08xh tx_engine_state=%u bus=%d\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1,
		dev->flags, dev->locked_tx_engine_state, dev->bus_state);
#endif

	/* no need to start anything in that context */
	if (!(dev->flags & KCAN_DEV_OPENED)) {
		return -ENETDOWN;
	}

	/* Hem... this should be tested here. But KCAN-USB takes up to ~900 ms
	 * to notify from ERROR_ACTIVE. See also handle_error_active() in
	 * src/kcan_main.c
	 */
	if (dev->bus_state == KCANFD_UNKNOWN) {
		return 0;
	}

	/* if we just put the 1st message (=the fifo was empty), we can start
	 * writing on hardware if it is ready for doing this.
	 */
	if (dev->locked_tx_engine_state == TX_ENGINE_STOPPED) {
		err = dev->device_write(dev, ctx);
	}

	/* since v8.8, device_write() should not return -ENODATA except if no
	 * data has been read from the Tx fifo. Since __kcan_dev_start_writing()
	 * is called after having put a frame into the Tx fifo, err
	 * cannot be -ENODATA.
	 */
	return (err == -ENODATA) ? 0 : err;
}

static int kcanfd_start_tx_engine(struct kcandev *dev, struct kcanusr *ctx)
{
	kcan_lock_irqsave_ctxt lck_ctx;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u bus_state=%u)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1, dev->bus_state);
#endif

	dev->lock_irq(dev, &lck_ctx);

	/* if can device ready to send, start writing */
	err = __kcan_dev_start_writing(dev, ctx);

	dev->unlock_irq(dev, &lck_ctx);

	return err;
}

/*
 * Called when user put a CAN frame in the device Tx fifo, while it is locked.
 *
 * @return:
 *	-EAGAIN	if tx fifo is full
 *	>= 0 if item successfully put.
 */
static int kcan_txfifo_put_completion(FIFO_MANAGER *anchor, void *item,
				      void *arg, int err)
{
	/* change -ENOSPC into -EAGAIN to map chardev interface */
	if (err == -ENOSPC)
		err = -EAGAIN;

#ifdef kcan_event_clear
	else {
		struct kcandev *dev = (struct kcandev *)arg;

		/* if tx fifo is now full, then the corresponding event
		 * should be cleared now.
		 */
		if (kcan_fifo_full(anchor))
			kcan_event_clear(&dev->out_event);
	}
#endif

	return err;
}

/*
 * Return:
 * > 0		Tx fifo number of items
 * 0		if nothing done in Tx fifo.
 * < 0		An error code:
 *		-EBADMSG	if sending CAN FD msg on CAN 2.0 settings
 *				if sending extended id while std msg allowed
 *		-ENODEV		if device no more plugged
 *		-ENETDOWN	if bus off
 *		-EAGAIN		if Tx fifo full
 *		-EINTR		if wait() has been interrupted
 */
static int kcanfd_send_msg(struct kcandev *dev, struct kcanfd_txmsg *ptx,
			   struct kcanusr *ctx)
{
	FIFO_MANAGER *tx_fifo = &dev->tx_fifo;
	int err;

	switch (ptx->msg.type) {

	case KCANFD_TYPE_CANFD_MSG:

		/* accept such messages for devices that have been initialized
		 * for CAN-FD.
		 */
		if ((dev->init_settings.flags & KCANFD_INIT_FD) &&
				(ptx->msg.data_len <= KCANFD_CANFD_MAXDATALEN))
			break;

		/* Ok to be permissive *BUT* force the message to be
		 * CAN 2.0 only, that is, CAN-FD specific flags won't be
		 * taken into account next.
		 */
		ptx->msg.type = KCANFD_TYPE_CAN20_MSG;

		/* fall through */
		fallthrough;
	case KCANFD_TYPE_CAN20_MSG:
		if (ptx->msg.data_len <= KCANFD_CAN20_MAXDATALEN)
			break;

		/* fall through */
		fallthrough;
	default:
		pr_err(DEVICE_NAME
			": trying to send invalid msg (type=%xh len=%d)\n",
			ptx->msg.type, ptx->msg.data_len);

		return -EBADMSG;
	}

	/* filter extended data if initialized to standard only
	 * SGr note: no need to wait for doing such a test...
	 */
	if ((dev->init_settings.flags & KCANFD_INIT_STD_MSG_ONLY)
	   && ((ptx->msg.flags & KCANFD_MSG_EXT) || (ptx->msg.id > 2047))) {

		pr_err(DEVICE_NAME
			": trying to send ext msg %xh while not setup for\n",
			ptx->msg.id);
		return -EBADMSG;
	}

	if (!(dev->features & KCAN_DEV_SLF_RDY)
	  && (ptx->msg.flags & KCANFD_MSG_SLF)) {
		pr_err(DEVICE_NAME
			": trying to send unsupported slf msg %xh\n",
			ptx->msg.id);
		return -EBADMSG;
	}

	if (!(dev->features & KCAN_DEV_SNG_RDY)
	  && (ptx->msg.flags & KCANFD_MSG_SNG)) {
		pr_err(DEVICE_NAME
			": trying to send unsupported single-shot msg %xh\n",
			ptx->msg.id);
		return -EBADMSG;
	}

	do {
		int xmtfull_was_not_set;

		/* if the device has been plugged out while waiting,
		 * or if any task is closing it
		 */
		if (!dev->is_plugged || !dev->nOpenPaths) {
			err = -ENODEV;
			break;
		}

		/* no need to write in case of BUS_OFF */
		if (dev->bus_state == KCANFD_ERROR_BUSOFF) {
			err = -ENETDOWN;
			break;
		}

		/* put data into fifo */
		err = kcan_fifo_put_ex(tx_fifo, ptx,
				       kcan_txfifo_put_completion, dev);
		if (err >= 0) {
			if (kcan_clear_status_bit(dev, CAN_ERR_XMTFULL)) {
#ifndef NETDEV_SUPPORT
				/* Do post a STATUS[] to indicate that Tx fifo
				 * is no more full. This is mainly useful to
				 * refesh kcanview status bar...
				 */
				if (!ctx || (ctx->open_flags & O_NONBLOCK))
					kcan_post_bus_state(dev);
#endif
			}
			break;
		}

		xmtfull_was_not_set = kcan_set_status_bit(dev, CAN_ERR_XMTFULL);

		/* support nonblocking write if requested */
		if (!ctx || (ctx->open_flags & O_NONBLOCK)) {
#ifndef NETDEV_SUPPORT
			/* do post KCANFD_TX_OVERFLOW only when device is opened
			 * in non blocking mode: user doesn't need to know when
			 * Tx fifo is full in blocking mode since his writing
			 * task is blocked...
			 */
			if (xmtfull_was_not_set) {
				struct kcanfd_rxmsg rx;

				kcan_handle_error_internal(dev, &rx,
							KCANFD_TX_OVERFLOW);

				if (kcan_chardev_rx(dev, &rx) > 0)
					kcan_event_signal(&dev->in_event);
			}
#endif
			break;
		}

		if (!kcan_task_can_wait()) {
			pr_info(DEVICE_NAME
				": %s(%u): ABNORMAL task unable to wait!\n",
				__func__, __LINE__);
			break;
		}

		/* check Tx engine whether it is running before going asleep
		 * (Note: useful only if one has sent more msgs than Tx fifo
		 * size, at once)
		 */
		kcanfd_start_tx_engine(dev, ctx);

		/* sleep until space is available. */
#ifdef DEBUG_WAIT_WR
		pr_info(DEVICE_NAME
			": %s CAN%u waiting %u ms. for some free space "
			"to write...\n",
			dev->adapter->name, kcan_idx(dev)+1,
			KCANFD_TIMEOUT_WAIT_FOR_WR);
#endif

		/* task might go to sleep: unlock current dev */
		kcan_unlock_dev(dev);

		/* wait up to 100 ms. for some room in the Tx queue.
		 *
		 * some logs:
		 *
[ 7977.396005] kcan: kcanfd_send_msg(359): waiting for some free space to write...
...
[ 7977.400974] kcan: CAN1 lnk=1 signaling writing task...
...
[ 7977.400977] kcan: end of waiting for tx fifo not full: err=0
		 *
		 * Note: ^C may occur while waiting. In RT, preemption can 
		 * schedule another task that might call close() while we're
		 * always waiting here.
		 * - If the event is destroyed by some other task, the below
		 *   call fails with err=-EIDRM(43).
		 * - if some other task deletes this waiting task, this tasks
		 *   is first unblocked, thus err=-EINTR(4).
		 */
		err = kcan_event_wait_timeout(dev->out_event,
					!dev->is_plugged ||
					!kcan_fifo_full(tx_fifo) ||
					dev->bus_state == KCANFD_ERROR_BUSOFF,
					KCANFD_TIMEOUT_WAIT_FOR_WR);

		/* lock the device again */
		kcan_lock_dev(dev);

#ifdef DEBUG_WAIT_WR
		pr_info(DEVICE_NAME
			": end of waiting for tx fifo not full: err=%d\n",
			err);
#endif

	} while (err >= 0);

	return (err == -ERESTARTSYS) ? -EINTR : err;
}

int kcanfd_ioctl_send_msg(struct kcandev *dev, struct kcanfd_txmsg *ptx,
			  struct kcanusr *ctx)
{
	int err = kcanfd_send_msg(dev, ptx, ctx);

	/* start Tx engine only if Tx fifo is not empty */
	if (err > 0)
		err = kcanfd_start_tx_engine(dev, ctx);

	return err >= 0 ? 0 : err;
}

int kcanfd_ioctl_send_msgs(struct kcandev *dev, struct kcanfd_txmsgs *pl,
			   struct kcanusr *ctx)
{
	struct kcanfd_txmsg *ptx;
	int err = 0, msgs_queued = 0, n = pl->count;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(count=%u)\n", __func__, n);
#endif

	ptx = pl->list;
	for (pl->count = 0; pl->count < n; pl->count++) {
		err = kcanfd_send_msg(dev, ptx, ctx);

		/* don't stop sending if error is related to the msg only */
		if (err < 0) {
			if (err != -EBADMSG)
				break;
		} else {
			msgs_queued += err;
		}

		ptx++;
	}

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(count=%u): queued %u msgs\n",
		__func__, n, msgs_queued);
#endif

	/* if at least ONE message has been enqueued */
	if (msgs_queued > 0) {

		/* if we just put the 1st message (=the fifo was empty),
		 * we can start writing on hardware if it is ready for doing
		 * this.
		 */
		err = kcanfd_start_tx_engine(dev, ctx);
	}

	return msgs_queued ? 0 : err;
}

int kcanfd_ioctl_recv_msg(struct kcandev *dev, struct kcanfd_rxmsg *prx,
			  struct kcanusr *ctx)
{
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	err = kcanfd_recv_msg(dev, prx, ctx);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(): returns %d\n", __func__, err);
#endif
	return err;
}

int kcanfd_ioctl_recv_msgs(struct kcandev *dev, struct kcanfd_rxmsgs *pl,
			   struct kcanusr *ctx)
{
	struct kcanfd_rxmsg *prx;
	int err = 0, n = pl->count, saved_flags = ctx->open_flags;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(count=%u)\n", __func__, n);
#endif

	prx = pl->list;

	for (pl->count = 0; pl->count < n; pl->count++) {
		err = kcanfd_recv_msg(dev, prx, ctx);
		if (err)
			break;

		/* the task won't block anymore since at least one msg has been
		 * read.
		 */
		ctx->open_flags |= O_NONBLOCK;
		prx++;
	}

	/* restore original flags asap */
	ctx->open_flags = saved_flags;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(count=%u): got %u msgs (err %d)\n",
			__func__, n, pl->count, err);
#endif
	return (pl->count > 0) ? 0 : err;
}
