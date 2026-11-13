/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_fops_linux.c - all file operation functions, exports only struct fops
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
 * Maintainer:   Kunhong-Elec driver team <gene@cdkhdz.com>
 */
/* #define DEBUG */
/* #undef DEBUG */
static struct kcandev *kcan_get_dev_lock(struct kcanusr *pud)
{
	struct kcandev *dev = kcan_get_dev(pud);
	if (dev)
		kcan_lock_dev(dev);

	return dev;
}

/* return a locked (mutex) device according to its (major, minor) */
static struct kcandev* kcan_search_dev_lock(int major, int minor)
{
	struct kcandev *dev = kcan_search_dev(major, minor);
	if (dev)
		kcan_lock_dev(dev);

	return dev;
}

/* put (unlock) a locked device */
static int kcan_put_dev_unlock(struct kcandev *dev, long err)
{
	if (dev)
		kcan_unlock_dev(dev);

	return kcan_put_dev(dev, err);
}

/* is called when the path is opened */
static int kcan_open(struct inode *inode, struct file *filep)
{
	struct kcandev *dev;
	struct kcanusr *dev_priv;
	int _major = MAJOR(inode->i_rdev);
	int _minor = minor(inode->i_rdev);
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(minor=%d, f_flags=%08xh (%o) f_mode=%08xh)\n",
		__func__, _minor, filep->f_flags, filep->f_flags,
		filep->f_mode);
#endif
	dev = kcan_search_dev_lock(_major, _minor);
	if (!dev)
		return -ENODEV;

	/* create file object */
	dev_priv = kcan_malloc(sizeof(struct kcanusr), GFP_KERNEL);
	if (!dev_priv) {
		pr_err(DEVICE_NAME ": %s(): memory allocation failed!\n",
		       __func__);
		return kcan_put_dev_unlock(dev, -ENOMEM);
	}

	/* fill file object and init read and write method buffers */
	dev_priv->dev = dev;

	/* O_RDONLY(0) =>
	 * - O_WRONLY(1) not set
	 * - O_RDWR(2) not set
	 */
	dev_priv->open_flags = filep->f_flags;
	dev_priv->filep = filep;

	if (filep->f_mode & FMODE_READ) {
		dev_priv->nReadRest = 0;
		dev_priv->nTotalReadCount = 0;
		dev_priv->pcReadPointer = dev_priv->pcReadBuffer;
	}

	/* F_MODE_WRITE not set when O_RDONLY used */
	if (filep->f_mode & FMODE_WRITE) {
		dev_priv->nWriteCount = 0;
		dev_priv->pcWritePointer = dev_priv->pcWriteBuffer;
	}

	filep->private_data = (void *)dev_priv;

	/* why? dev->init_settings are filled from:
	 * - dev->def_init_settings which are supposed to be ok
	 * - user settings (see KCANFD_SET_INIT) but ioctl_set_init() handler
	 *   does check them before copying them into dev->init_settings.
	 */
	err = kcan_open_path(dev, dev_priv);
	if (!err)
		return kcan_put_dev_unlock(dev, 0);

	kcan_free(dev_priv);

	return kcan_put_dev_unlock(dev, err);
}

static int kcan_release(struct inode *inode, struct file *filep)
{
	struct kcanusr *dev_priv = (struct kcanusr *)filep->private_data;
	struct kcandev *dev = kcan_get_dev_lock(dev_priv);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(dev=%p)\n", __func__, dev);
#endif

	/* free the associated irq and allocated memory */
	if (dev)
		kcan_release_path(dev, dev_priv);

	filep->private_data = kcan_free(dev_priv);

	return kcan_put_dev_unlock(dev, 0);
}

/*
 * is called at user ioctl() with cmd = KCAN_INIT
 */
static int kcan_ioctl_init(struct kcandev *dev, TKCANInit __user *pi)
{
	TKCANInit init;
	struct kcanfd_init fdi = {};
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, kcan_idx(dev)+1);
#endif

	err = copy_from_user(&init, pi, sizeof(init));
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	/* convert wBTR0BTR1 (8MHz) value into bitrate value. pfdi->clock_Hz
	 * is let 0 so that kcanfd_dev_open() will fill it with device default
	 * clock.
	 */
	return kcanfd_ioctl_set_init(dev, kcan_init_to_fd(dev, &fdi, &init));
}

/*
 * is called at user ioctl() with cmd = KCAN_WRITE_MSG
 */
static int kcan_ioctl_write(struct kcandev *dev, TKCANMsg __user *usr,
			    struct kcanusr *dev_priv)
{
	struct kcanfd_txmsg tx;
	TKCANMsg msg;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, kcan_idx(dev)+1);
#endif

	/* get from user space */
	if (copy_from_user(&msg, usr, sizeof(msg))) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		err = -EFAULT;
		goto fail;
	}

	/* do some minimal (but mandatory!) check */
	if (msg.LEN > 8) {
		pr_err(DEVICE_NAME
			": trying to send msg %xh  with invalid data len %d\n",
			msg.ID, msg.LEN);
		err = -EINVAL;
		goto fail;
	}

	/* convert old-style TKCANMsg into new-style struct kcanfd_rxmsg */
	kcan_msg_to_fd(&tx.msg, &msg);
	err = kcanfd_ioctl_send_msg(dev, &tx, dev_priv);
	if (err)
		goto fail;

	return 0;

fail:
#ifdef DEBUG
        pr_err(DEVICE_NAME ": failed to write CAN frame (err %d)\n", err);
#endif
	return err;
}

/*
 * is called at user ioctl() with cmd = KCAN_READ_MSG
 */
static int kcan_ioctl_read(struct kcandev *dev, TKCANRdMsg __user *usr,
						struct kcanusr *dev_priv)
{
	struct kcanfd_rxmsg msgfd;
	TKCANRdMsg msg;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, kcan_idx(dev)+1);
#endif

	do {
		err = kcanfd_ioctl_recv_msg(dev, &msgfd, dev_priv);
		if (err)
			return err;

		if (kcan_is_fd(&msgfd.msg)) {
			pr_err(DEVICE_NAME ": CAN-FD frame discarded "
				"(CAN 2.0 application)\n");
			err = -EINVAL;
		}
	} while (err);

	if (copy_to_user(usr, kcan_fd_to_msg(&msg, &msgfd.msg), sizeof(*usr))) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

	return err;
}

/*
 * is called at user ioctl() with cmd = KCAN_GET_STATUS
 */
static int kcan_ioctl_status(struct kcandev *dev, TPSTATUS __user *status)
{
	TPSTATUS local;
	int err = 0;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, kcan_idx(dev)+1);
#endif

	kcan_ioctl_status_common(dev, &local);

	if (copy_to_user(status, &local, sizeof(local))) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
		goto fail;
	}

	dev->wCANStatus = 0;
	dev->nLastError = 0;

fail:
	return err;
}

/*
 * is called at user ioctl() with cmd = KCAN_GET_EXT_STATUS
 */
static int kcan_ioctl_extended_status(struct kcandev *dev,
						TPEXTENDEDSTATUS __user *status)
{
	TPEXTENDEDSTATUS local;
	int err = 0;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1);
#endif

	kcan_ioctl_extended_status_common(dev, &local);

	if (copy_to_user(status, &local, sizeof(local))) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
		goto fail;
	}

	dev->wCANStatus = 0;
	dev->nLastError = 0;

fail:
	return err;
}

/*
 * is called at user ioctl() with cmd = KCAN_DIAG
 */
static int kcan_ioctl_diag(struct kcandev *dev, TPDIAG __user *diag)
{
	TPDIAG local;
	int err = 0;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1);
#endif

	kcan_ioctl_diag_common(dev, &local);

	if (copy_to_user(diag, &local, sizeof(local))) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

	return err;
}

/*
 * get BTR0BTR1 init values
 */
static int kcan_ioctl_BTR0BTR1(struct kcandev *dev, TPBTR0BTR1 __user *BTR0BTR1)
{
	TPBTR0BTR1 local;
	int err = 0;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1);
#endif

	if (copy_from_user(&local, BTR0BTR1, sizeof(local))) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		err = -EFAULT;
		goto fail;
	}

	/* this does not influence hardware settings, only BTR0BTR1 values
	 * are calculated
	 */
	local.wBTR0BTR1 = sja1000_bitrate(local.dwBitRate, 0, 0 /* TODO */);
	if (!local.wBTR0BTR1) {
		err = -EINVAL;
		goto fail;
	}

	if (copy_to_user(BTR0BTR1, &local, sizeof(*BTR0BTR1))) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

fail:
	return err;
}

/*
 * add a message filter_element into the filter chain or delete all
 * filter_elements
 */
static int kcan_ioctl_msg_filter(struct kcandev *dev,
						TPMSGFILTER __user *filter)
{
	TPMSGFILTER local_filter;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1);
#endif

	/* filter == NULL -> delete the filter_elements in the chain */
	if (!filter) {
		kcan_delete_filter_all(dev->filter);
		return 0;
	}

	if (copy_from_user(&local_filter, filter, sizeof(local_filter))) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return kcan_add_filter(dev->filter, local_filter.FromID,
				local_filter.ToID, local_filter.MSGTYPE);
}

/*
 * set or get extra parameters from the devices
 */
static int kcan_ioctl_extra_parameters(struct kcandev *dev,
				       TPEXTRAPARAMS __user *params,
				       int sizeof_extra_params)
{
	TPEXTRAPARAMS local;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u, s=%d)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1,
		sizeof_extra_params);
#endif

	if (copy_from_user(&local, params, sizeof_extra_params)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
		       __func__);
		err = -EFAULT;
		goto fail;
	}

	err = _kcan_ioctl_extra_parameters(dev, &local, sizeof_extra_params);
	if (!err)
		if (copy_to_user(params, &local, sizeof_extra_params)) {
			pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			       __func__);
			err = -EFAULT;
		}

fail:
	return err;
}

/* device is considered as locked when entering this function */
static long __kcan_ioctl(struct file *filep, unsigned int cmd, void __user *up,
			 struct kcandev *dev)
{
	struct kcanusr *dev_priv = (struct kcanusr *)filep->private_data;
	struct kcanfd_init fdi;
	struct kcanfd_state fds;
	struct kcanfd_rxmsg rx;
	struct kcanfd_txmsg tx;
	int err, l;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u, cmd=%u (NR=%u))\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1,
		cmd, _IOC_NR(cmd));
#endif

	switch (cmd) {
	case KCAN_INIT:
		err = kcan_ioctl_init(dev, (TKCANInit __user *)up);
		break;
	case KCAN_READ_MSG:
		/* support blocking and nonblocking IO */
		err = kcan_ioctl_read(dev, (TKCANRdMsg __user *)up, dev_priv);
		break;
	case KCAN_WRITE_MSG:
		/* support blocking and nonblocking IO */
		err = kcan_ioctl_write(dev, (TKCANMsg __user *)up, dev_priv);
		break;
	case KCAN_GET_STATUS:
		err = kcan_ioctl_status(dev, (TPSTATUS __user *)up);
		break;
	case KCAN_GET_EXT_STATUS:
		err = kcan_ioctl_extended_status(dev,
						(TPEXTENDEDSTATUS __user *)up);
		break;
	case KCAN_DIAG:
		err = kcan_ioctl_diag(dev, (TPDIAG __user *)up);
		break;
	case KCAN_BTR0BTR1:
		err = kcan_ioctl_BTR0BTR1(dev, (TPBTR0BTR1 __user *)up);
		break;
	case KCAN_MSG_FILTER:
		err = kcan_ioctl_msg_filter(dev, (TPMSGFILTER __user *)up);
		break;
	case KCAN_EXTRA_PARAMS_8_13:
	case KCAN_EXTRA_PARAMS:
		err = kcan_ioctl_extra_parameters(dev,
						(TPEXTRAPARAMS __user *)up,
						_IOC_SIZE(cmd));
		break;

	/* CAN-FD new API */
	case KCANFD_SET_INIT:
		err = copy_from_user(&fdi, up, sizeof(fdi));
		if (err) {
			pr_err(DEVICE_NAME
				": %s(%u): copy_from_user() failure\n",
				__func__, __LINE__);
			err = -EFAULT;
		}

		err = kcanfd_ioctl_set_init(dev, &fdi);
		break;

	case KCANFD_GET_INIT:
		err = kcanfd_ioctl_get_init(dev, &fdi);
		if (err)
			break;

		err = copy_to_user(up, &fdi, sizeof(fdi));
		if (err) {
			pr_err(DEVICE_NAME ": %s(%u): copy_to_user() failure\n",
				__func__, __LINE__);
			err = -EFAULT;
		}
		break;

	case KCANFD_RESET:
		err = kcanfd_ioctl_reset(dev, (unsigned long )up);
		break;

	case KCANFD_GET_STATE:
		err = kcanfd_ioctl_get_state(dev, &fds);
		if (err)
			break;

		err = copy_to_user(up, &fds, sizeof(fds));
		if (err) {
			pr_err(DEVICE_NAME ": %s(%u): copy_to_user() failure\n",
				__func__, __LINE__);
			err = -EFAULT;
		}	
		break;

#ifdef KCANFD_ADD_FILTER
	case KCANFD_ADD_FILTER:
		if (up) {
			struct kcanfd_msg_filter mf;

			err = copy_from_user(&mf, up, sizeof(mf));
			if (err) {
				pr_err(DEVICE_NAME
					": %s(%u): copy_from_user() failure\n",
					__func__, __LINE__);
				err = -EFAULT;
			}
			err = kcanfd_ioctl_add_filter(dev, &mf);
		} else {
			err = kcanfd_ioctl_add_filter(dev, NULL);
		}
		break;
#endif

	case KCANFD_ADD_FILTERS:
		if (up) {
			struct kcanfd_msg_filters mfl, *pfl;

			l = sizeof(struct kcanfd_msg_filters_0);
			err = copy_from_user(&mfl, up, l);
			if (err) {
				pr_err(DEVICE_NAME
					": %s(%u): copy_from_user() failure\n",
					__func__, __LINE__);
				err = -EFAULT;
				break;
			}

			if (!mfl.count) {
				err = 0;
				break;
			}

			l += mfl.count * sizeof(struct kcanfd_msg_filter);
			pfl = kcan_malloc(l, GFP_KERNEL);
			if (!pfl) {
				pr_err("%s: failed to alloc filter list\n",
						DEVICE_NAME);
				err = -ENOMEM;
				break;
			}

			if (copy_from_user(pfl, up, l)) {
				kcan_free(pfl);
				pr_err(DEVICE_NAME
					": %s(%u): copy_from_user() failure\n",
					__func__, __LINE__);
				err = -EFAULT;
			} else {
				/* re-sync with kernel space content */
				pfl->count = mfl.count;
				err = kcanfd_ioctl_add_filters(dev, pfl);
			}

			kcan_free(pfl);
		} else {
			err = kcanfd_ioctl_add_filters(dev, NULL);
		}
		break;

	case KCANFD_GET_FILTERS:
		if (up) {
			struct kcanfd_msg_filters mfl, *pfl;

			l = sizeof(struct kcanfd_msg_filters_0);
			err = copy_from_user(&mfl, up, l);
			if (err) {
				pr_err(DEVICE_NAME
					": %s(%u): copy_from_user() failure\n",
					__func__, __LINE__);
				err = -EFAULT;
				break;
			}

			if (!mfl.count) {
				err = 0;
				break;
			}

			l += mfl.count * sizeof(struct kcanfd_msg_filter);
			pfl = kcan_malloc(l, GFP_KERNEL);
			if (!pfl) {
				pr_err("%s: failed to alloc filter list\n",
						DEVICE_NAME);
				err = -ENOMEM;
				break;
			}

			pfl->count = mfl.count;
			err = kcanfd_ioctl_get_filters(dev, pfl);

			/* copy the count and the filter received */
			l = sizeof(struct kcanfd_msg_filters_0) +
				pfl->count * sizeof(struct kcanfd_msg_filter);

			if (copy_to_user(up, pfl, l)) {
				pr_err(DEVICE_NAME
					": %s(%u): copy_to_user() failure\n",
					__func__, __LINE__);
				err = -EFAULT;
			}

			kcan_free(pfl);
		} else {
			err = kcanfd_ioctl_get_filters(dev, NULL);
		}
		break;

	case KCANFD_SEND_MSG:
		err = copy_from_user(&tx.msg, up, sizeof(tx.msg));
		if (err) {
			pr_err(DEVICE_NAME
				": %s(%u): copy_from_user() failure\n",
				__func__, __LINE__);
			err = -EFAULT;
			break;
		}
		err = kcanfd_ioctl_send_msg(dev, &tx, dev_priv);
		break;

	case KCANFD_RECV_MSG:
		err = kcanfd_ioctl_recv_msg(dev, &rx, dev_priv);
		if (err)
			break;

		err = copy_to_user(up, &rx.msg, sizeof(rx.msg));
		if (err) {
			pr_err(DEVICE_NAME
				": %s(%u): copy_to_user() failure\n",
				__func__, __LINE__);
			err = -EFAULT;
			break;
		}
		break;

	case KCANFD_SEND_MSGS:
		err = handle_kcanfd_send_msgs(dev, up, dev_priv, NULL);
		break;

	case KCANFD_RECV_MSGS:
		err = handle_kcanfd_recv_msgs(dev, up, dev_priv, NULL);
		break;

	case KCANFD_GET_AVAILABLE_CLOCKS:
		err = handle_kcanfd_get_av_clocks(dev, up, dev_priv, NULL);
		break;

	case KCANFD_GET_BITTIMING_RANGES:
		err = handle_kcanfd_get_bittiming_ranges(dev, up,
							 dev_priv, NULL);
		break;

	case KCANFD_GET_OPTION:
		err = handle_kcanfd_get_option(dev, up, dev_priv, NULL);
		break;

	case KCANFD_SET_OPTION:
		err = handle_kcanfd_set_option(dev, up, dev_priv, NULL);
		break;

	default:
		pr_err(DEVICE_NAME ": %s(cmd=%u): unsupported cmd "
			"(dir=%u type=%u nr=%u size=%u)\n",
			__func__, cmd,
			_IOC_DIR(cmd), _IOC_TYPE(cmd),
			_IOC_NR(cmd), _IOC_SIZE(cmd));
		err = -ENOTTY;
		break;
	}

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(CAN%u, cmd=%u (NR=%u)): returns %d\n",
		__func__, kcan_idx(dev)+1, cmd, _IOC_NR(cmd), err);
#endif

	return err;
}

/* ioctl() entry point: lock the device then handle the ioctl cmd */
static long _kcan_ioctl(struct file *filep, unsigned int cmd, void __user *up)
{
	struct kcanusr *dev_priv = (struct kcanusr *)filep->private_data;
	long err;

	struct kcandev *dev = kcan_get_dev_lock(dev_priv);
	if (!dev)
		return -ENODEV;

	err = __kcan_ioctl(filep, cmd, up, dev);
	return kcan_put_dev_unlock(dev, err);
}

/*
 * is called at user ioctl() call
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int kcan_ioctl(struct inode *inode,
		struct file *filep, unsigned int cmd, unsigned long arg)
#else
static long kcan_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
#endif
{
	return _kcan_ioctl(filep, cmd, (void __user *)arg);
}

#ifdef KCAN_CONFIG_COMPAT
/*
 * 32-bit application using 64-bit driver
 */
#define __array_of_struct32(_n, _x)					\
	_n##s_##_x {							\
		__u32		count;					\
		struct _n	list[_x];				\
	} __aligned(4)

struct kcanfd_msg32 {
	__u16	type;
	__u16	data_len;
	__u32	id;
	__u32	flags;
	struct compat_timeval	timestamp;
	__u8	ctrlr_data[KCANFD_MAXCTRLRDATALEN];
	__u8	data[KCANFD_MAXDATALEN] __attribute__((aligned(8)));
} __aligned(4);

struct __array_of_struct32(kcanfd_msg32, 0);

#define kcanfd_msgs32		kcanfd_msg32s_0

struct kcanfd_state32 {
	__u16	ver_major, ver_minor, ver_subminor;

	struct compat_timeval	tv_init;

	compat_int_t	bus_state;

	__u32	device_id;

	__u32	open_counter;
	__u32	filters_counter;

	__u16	hw_type;
	__u16	channel_number;

	__u16	can_status;
	__u16	bus_load;

	__u32	tx_max_msgs;
	__u32	tx_pending_msgs;
	__u32	rx_max_msgs;
	__u32	rx_pending_msgs;
	__u32	tx_frames_counter;
	__u32	rx_frames_counter;
	__u32	tx_error_counter;
	__u32	rx_error_counter;

	__u64	host_time_ns;
	__u64	hw_time_ns;
} __aligned(4);

/* Device options */
struct kcanfd_option32 {
	compat_int_t	size;
	compat_int_t	name;
	compat_uptr_t 	value;
} __aligned(4);

#define KCANFD_GET_STATE32	_IOR(KCAN_MAGIC_NUMBER, KCANFD_SEQ_GET_STATE,\
					struct kcanfd_state32)

#define KCANFD_SEND_MSG32	_IOW(KCAN_MAGIC_NUMBER, KCANFD_SEQ_SEND_MSG,\
					struct kcanfd_msg32)

#define KCANFD_RECV_MSG32	_IOR(KCAN_MAGIC_NUMBER, KCANFD_SEQ_RECV_MSG,\
					struct kcanfd_msg32)

#define KCANFD_SEND_MSGS32	_IOWR(KCAN_MAGIC_NUMBER, KCANFD_SEQ_SEND_MSGS,\
					struct kcanfd_msgs32)

#define KCANFD_RECV_MSGS32	_IOWR(KCAN_MAGIC_NUMBER, KCANFD_SEQ_RECV_MSGS,\
					struct kcanfd_msgs32)

#define KCANFD_GET_OPTION32	_IOWR(KCAN_MAGIC_NUMBER, KCANFD_SEQ_GET_OPTION,\
					struct kcanfd_option32)

#define KCANFD_SET_OPTION32	_IOW(KCAN_MAGIC_NUMBER, KCANFD_SEQ_SET_OPTION,\
					struct kcanfd_option32)

/* because of the struct timeval different size, we must
 * do some manual copy... */
static void copy_from_msg32(struct kcanfd_msg *msgfd,
					const struct kcanfd_msg32 *msgfd32)
{
	/* note: copying msgs from userspace means that this is a message
	 * to send: no need to copy every fields...
	 */
	msgfd->type = msgfd32->type;
	msgfd->data_len = msgfd32->data_len;
	msgfd->id = msgfd32->id;
	msgfd->flags = msgfd32->flags;
	memcpy(msgfd->data, msgfd32->data, sizeof(msgfd->data));
}

static void copy_to_msg32(struct kcanfd_msg32 *msgfd32,
					const struct kcanfd_msg *msgfd)
{
	msgfd32->type = msgfd->type;
	msgfd32->data_len = msgfd->data_len;
	msgfd32->id = msgfd->id;
	msgfd32->flags = msgfd->flags;

	msgfd32->timestamp.tv_sec = msgfd->timestamp.tv_sec;
	msgfd32->timestamp.tv_usec = msgfd->timestamp.tv_usec;

	memcpy(msgfd32->ctrlr_data, msgfd->ctrlr_data, KCANFD_MAXCTRLRDATALEN);
	memcpy(msgfd32->data, msgfd->data, sizeof(msgfd32->data));
}

static int handle_kcanfd_send_msgs32(struct kcandev *dev, void __user *up,
						struct kcanusr *dev_priv)
{
	struct kcanfd_msg32s_0 __user *pl32 = (struct kcanfd_msg32s_0 *)up;
	struct kcanfd_txmsgs txs, *pl;
	int i, l, err;

	l = sizeof(*pl32);
	err = copy_from_user(&txs, up, l);
	if (err) {
		pr_err(DEVICE_NAME ": %s(%u): copy_from_user() failure\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	/* ok. Nothing to send. So nothing done. Perfect. */
	if (!txs.count)
		return 0;

	l += txs.count * sizeof(txs.list[0]);
	pl = kcan_malloc(l, GFP_KERNEL);
	if (!pl) {
		pr_err(DEVICE_NAME ": %s(): failed to alloc msgs list\n",
			__func__);
		return -ENOMEM;
	}

	for (i = 0; i < txs.count; i++) {
		struct kcanfd_msg32 m32;

		err = copy_from_user(&m32, &pl32->list[i], sizeof(m32));
		if (err) {
			pr_err(DEVICE_NAME
				": %s(%u): copy_from_user() failure\n",
				__func__, __LINE__);
			err = -EFAULT;
			goto lbl_free;
		}

		copy_from_msg32(&pl->list[i].msg, &m32);
	}

	pl->count = i;
	err = kcanfd_ioctl_send_msgs(dev, pl, dev_priv);

	/* copy the count of msgs really sent (= pl->count) */
	if (copy_to_user(pl32, pl, sizeof(*pl32))) {
		pr_err(DEVICE_NAME
			": %s(%u): copy_to_user() failure\n",
			__func__, __LINE__);
		err = -EFAULT;
	}

lbl_free:
	kcan_free(pl);

	return err;
}

static int handle_kcanfd_recv_msgs32(struct kcandev *dev, void __user *up,
						struct kcanusr *dev_priv)
{
	struct kcanfd_msg32s_0 __user *pl32 = (struct kcanfd_msg32s_0 *)up;
	struct kcanfd_rxmsgs rxs, *pl;
	int i, l, err;

	l = sizeof(*pl32);
	err = copy_from_user(&rxs, up, l);
	if (err) {
		pr_err(DEVICE_NAME
			": %s(%u): copy_from_user() failure\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	/* ok! no room for saving rcvd msgs!? Thus, nothing returned */
	if (!rxs.count)
		return 0;

	l += rxs.count * sizeof(rxs.list[0]);
	pl = kcan_malloc(l, GFP_KERNEL);
	if (!pl) {
		pr_err(DEVICE_NAME ": failed to alloc msgs list\n");
		return -ENOMEM;
	}

	pl->count = rxs.count;
	err = kcanfd_ioctl_recv_msgs(dev, pl, dev_priv);

	/* copy the count of msgs received */
	if (copy_to_user(pl32, pl, sizeof(*pl32))) {
		pr_err(DEVICE_NAME
			": %s(%u): copy_to_user() failure\n",
			__func__, __LINE__);
		err = -EFAULT;
		goto lbl_free;
	}

	/* copy the msgs received */
	for (i = 0; i < pl->count; i++) {
		struct kcanfd_msg32 m32;

		copy_to_msg32(&m32, &pl->list[i].msg);

		if (copy_to_user(&pl32->list[i], &m32, sizeof(m32))) {
			pr_err(DEVICE_NAME
				": %s(%u): copy_to_user() failure\n",
				__func__, __LINE__);
			err = -EFAULT;
			break;
		}
	}

lbl_free:
	kcan_free(pl);

	return err;
}

static int handle_kcanfd_get_option32(struct kcandev *dev, void __user *up,
					struct kcanusr *dev_priv, void *c)
{
	struct kcanfd_option32 opt32;
	const int l = sizeof(opt32);
	struct kcanfd_option opt;

	int err = kcan_copy_from_user(&opt32, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME
			": %s(%u): copy_from_user() failure\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	if (opt32.name >= KCANFD_OPT_MAX) {
		pr_err(DEVICE_NAME ": invalid option name %d to get\n",
			opt32.name);
		return -EINVAL;
	}

	if (!dev->option[opt32.name].get) {
		return -EOPNOTSUPP;
	}

	if (dev->option[opt32.name].req_size > 0)

		/* if user option buffer size is too small, return the 
		 * requested size with -ENOSPC
		 */
		if (opt32.size < dev->option[opt32.name].req_size) {
			pr_warn(DEVICE_NAME
				": invalid option size %d < %d for option %d\n",
				opt32.size, dev->option[opt32.name].req_size,
				opt32.name);
			opt.size = dev->option[opt32.name].req_size;
			err = -ENOSPC;
			goto lbl_cpy_size;
		}

	opt.name = opt32.name;
	opt.size = opt32.size;
	opt.value = compat_ptr(opt32.value);

	err = dev->option[opt32.name].get(dev, &opt, c);
	if (err && err != -ENOSPC)
		return err;

lbl_cpy_size:
	/* update 'size' field */
	if (kcan_copy_to_user(up+offsetof(struct kcanfd_option32, size),
			 &opt.size, sizeof(opt.size), c)) {
		pr_err(DEVICE_NAME
			": %s(%u): copy_to_user() failure\n",
			__func__, __LINE__);
		err = -EFAULT;
	}

	return err;
}

static int handle_kcanfd_set_option32(struct kcandev *dev, void __user *up,
					struct kcanusr *dev_priv, void *c)
{
	struct kcanfd_option32 opt32;
	struct kcanfd_option opt;
	int l = sizeof(opt32);

	int err = kcan_copy_from_user(&opt32, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME
			": %s(%u): copy_from_user() failure\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	if (opt32.name >= KCANFD_OPT_MAX) {
		pr_err(DEVICE_NAME ": invalid option name %d to get\n",
			opt32.name);
		return -EINVAL;
	}

	if (!dev->option[opt32.name].set) {
		return -EOPNOTSUPP;
	}

	opt.name = opt32.name;
	opt.size = opt32.size;
	opt.value = compat_ptr(opt32.value);

	return dev->option[opt.name].set(dev, &opt, c);
}

static long kcan_compat_ioctl(struct file *filep, unsigned int cmd,
						unsigned long arg)
{
	struct kcanusr *dev_priv = (struct kcanusr *)filep->private_data;
	void __user *argp = compat_ptr(arg);
	struct kcanfd_msg32 msgfd32;
	struct kcanfd_rxmsg rx;
	struct kcanfd_txmsg tx;
	struct kcanfd_state fds;
	struct kcanfd_state32 fds32;
	struct kcandev *dev;
	void *ps, *pd;
	long err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(cmd=%u) NR=%u SIZE=%u\n",
		__func__, cmd, _IOC_NR(cmd), _IOC_SIZE(cmd));
#endif

	dev = kcan_get_dev_lock(dev_priv);
	if (!dev)
		return -ENODEV;

	switch (cmd) {

	case KCANFD_GET_STATE32:
		err = kcanfd_ioctl_get_state(dev, &fds);
		if (err)
			break;

		/* because of the struct timeval different size, we must
		 * do some manual copy...
		 */
		ps = &fds;
		pd = &fds32;

		memcpy(pd, ps, offsetof(struct kcanfd_state, tv_init));

		fds32.tv_init.tv_sec = fds.tv_init.tv_sec;
		fds32.tv_init.tv_usec = fds.tv_init.tv_usec;

		memcpy(pd+offsetof(struct kcanfd_state32, bus_state),
			ps+offsetof(struct kcanfd_state, bus_state),
			sizeof(fds32) - 
				offsetof(struct kcanfd_state32, bus_state));

		err = copy_to_user(argp, &fds32, sizeof(fds32));
		if (err) {
			pr_err(DEVICE_NAME
				": %s(%u): copy_to_user() failure: err %ld\n",
				__func__, __LINE__, err);
			err = -EFAULT;
		}
		break;

	case KCANFD_SEND_MSG32:
		err = copy_from_user(&msgfd32, argp, sizeof(msgfd32));
		if (err) {
			pr_err(DEVICE_NAME
				": %s(%u): copy_from_user() failure\n",
				__func__, __LINE__);
			err = -EFAULT;
			break;
		}
		copy_from_msg32(&tx.msg, &msgfd32);

		err = kcanfd_ioctl_send_msg(dev, &tx, dev_priv);
		break;

	case KCANFD_RECV_MSG32:
		err = kcanfd_ioctl_recv_msg(dev, &rx, dev_priv);
		if (err)
			break;

		copy_to_msg32(&msgfd32, &rx.msg),

		err = copy_to_user(argp, &msgfd32, sizeof(msgfd32));
		if (err) {
			pr_err(DEVICE_NAME
				": %s(%u): copy_to_user() failure\n",
				__func__, __LINE__);
			err = -EFAULT;
		}
		break;

	case KCANFD_SEND_MSGS32:
		err = handle_kcanfd_send_msgs32(dev, argp, dev_priv);
		break;

	case KCANFD_RECV_MSGS32:
		err = handle_kcanfd_recv_msgs32(dev, argp, dev_priv);
		break;

	case KCANFD_GET_OPTION32:
		err = handle_kcanfd_get_option32(dev, argp, dev_priv, NULL);
		break;

	case KCANFD_SET_OPTION32:
		err = handle_kcanfd_set_option32(dev, argp, dev_priv, NULL);
		break;

	default:
		/* call the unlocked device entry point */
		err = __kcan_ioctl(filep, cmd, argp, dev);
	}

	return kcan_put_dev_unlock(dev, err);
}
#endif

/*
 * is called when read from the path
 */
static ssize_t kcan_read(struct file *filep, char *buf, size_t count,
								loff_t *f_pos)
{
	struct kcanusr *dev_priv = (struct kcanusr *)filep->private_data;
	struct kcanfd_rxmsg rx;
	int err, len = 0;
	struct kcandev *dev = kcan_get_dev_lock(dev_priv);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__,
		(dev) ? kcan_idx(dev)+1 : 0xff);
#endif
	if (!dev)
		return -ENODEV;

	if (dev_priv->nReadRest <= 0) {
		err = kcanfd_ioctl_recv_msg(dev, &rx, dev_priv);
		if (err)
			return kcan_put_dev_unlock(dev, err);

		dev_priv->nReadRest =
			kcan_make_output(dev_priv->pcReadBuffer, &rx.msg);
		dev_priv->pcReadPointer = dev_priv->pcReadBuffer;
	}

	/* give the data to the user */
	if (count > dev_priv->nReadRest) {
		/* put all data to user */
		len = dev_priv->nReadRest;
		dev_priv->nReadRest = 0;
		if (copy_to_user(buf, dev_priv->pcReadPointer, len)) {
			pr_err(DEVICE_NAME
				": %s(%u): copy_to_user() failure\n",
				__func__, __LINE__);
			return kcan_put_dev_unlock(dev, -EFAULT);
		}
		dev_priv->pcReadPointer = dev_priv->pcReadBuffer;
	} else {
		/* put only partial data to user */
		len = count;
		dev_priv->nReadRest -= count;
		if (copy_to_user(buf, dev_priv->pcReadPointer, len)) {
			pr_err(DEVICE_NAME
				": %s(%u): copy_to_user() failure\n",
				__func__, __LINE__);
			return kcan_put_dev_unlock(dev, -EFAULT);
		}
		dev_priv->pcReadPointer =
				(u8 *)((u8*)dev_priv->pcReadPointer + len);
	}

	*f_pos += len;
	dev_priv->nTotalReadCount += len;

	return kcan_put_dev_unlock(dev, len);
}

static int kcan_write_line(struct kcanusr *dev_priv, u8 *ptr, size_t count)
{
	struct kcandev *dev = dev_priv->dev;
	u32 amount, offset;
	int err = 0;

	amount = (u32)(dev_priv->pcWritePointer - ptr - 1);
	offset = (u32)(ptr - dev_priv->pcWriteBuffer + 1);

	if ((amount > WRITEBUFFER_SIZE) || (offset > WRITEBUFFER_SIZE)) {
		pr_err(DEVICE_NAME ": %s() fault: %zu %u, %u: \n",
			__func__, count, amount, offset);
		err = -EFAULT;
		goto lbl_err;
	}

	if (kcan_parse_input_idle(dev_priv->pcWriteBuffer)) {
		struct kcanfd_txmsg tx;

		if (kcan_parse_input_message(dev_priv->pcWriteBuffer,
					     &tx.msg)) {
			struct kcanfd_init fdi;

			err = kcan_parse_input_init(dev_priv->pcWriteBuffer,
									&fdi);
			if (err)
				goto lbl_err;

			/* init the associated chip and the fifos again
			 * with new parameters
			 */
			err = kcanfd_ioctl_set_init(dev, &fdi);
			if (err)
				goto lbl_err;
		} else {
			err = kcanfd_ioctl_send_msg(dev, &tx, dev_priv);
			if (err)
				if (err != -ENODATA)
					goto lbl_err;
		}
	}

	/* move rest of amount data in buffer offset steps to left */
	memmove(dev_priv->pcWriteBuffer, ptr + 1, amount);
	dev_priv->pcWritePointer -= offset;

lbl_err:
	return err;
}

static ssize_t kcan_write(struct file *filep, const char *buf, size_t count,
								loff_t *f_pos)
{
	struct kcanusr *dev_priv = (struct kcanusr *)filep->private_data;
	int err = 0;
	u32 dwRest;
	u8 *ptr;

	struct kcandev *dev = kcan_get_dev_lock(dev_priv);
	if (!dev)
		return -ENODEV;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, kcan_idx(dev)+1);
#endif

	/* calculate remaining buffer space */
	dwRest = WRITEBUFFER_SIZE -
		(dev_priv->pcWritePointer - dev_priv->pcWriteBuffer);
	count  = (count > dwRest) ? dwRest : count;

	if (copy_from_user(dev_priv->pcWritePointer, buf, count)) {
		pr_err(DEVICE_NAME ": %s(%u): copy_from_user() failure\n",
			__func__, __LINE__);
		return kcan_put_dev_unlock(dev, -EFAULT);
	}

	/* adjust working pointer to end */
	dev_priv->pcWritePointer += count;

	/* iterate search blocks ending with '\n' */
	while (1) {

		/* search first '\n' from begin of buffer */
		ptr = dev_priv->pcWriteBuffer;
		while ((*ptr != '\n') && (ptr < dev_priv->pcWritePointer))
			ptr++;

		/* parse input when a CR was found */
		if ((*ptr == '\n') && (ptr < dev_priv->pcWritePointer)) {
			err = kcan_write_line(dev_priv, ptr, count);
			if (err)
				return kcan_put_dev_unlock(dev, err);
		} else
			break; /* no CR found */
	}

	if (dev_priv->pcWritePointer >=
				(dev_priv->pcWriteBuffer + WRITEBUFFER_SIZE)) {
		/* reject all */
		dev_priv->pcWritePointer = dev_priv->pcWriteBuffer;
		return kcan_put_dev_unlock(dev, -EINVAL);
	}

	return kcan_put_dev_unlock(dev, count);
}

/*
 * is called at poll or select
 */
static unsigned int kcan_poll(struct file *filep, poll_table *wait)
{
	struct kcanusr *dev_priv = (struct kcanusr *)filep->private_data;
	struct kcandev *dev = kcan_get_dev_lock(dev_priv);
	unsigned int mask = 0;

	if (!dev)
		return POLLERR;

	/* return on ops that could be performed without blocking */

#ifndef NETDEV_SUPPORT
	poll_wait(filep, &dev->in_event, wait);

	if (!kcan_fifo_empty(&dev->rx_fifo))
		mask |= POLLIN | POLLRDNORM;
#endif
	poll_wait(filep, &dev->out_event, wait);

	if (kcan_fifo_ratio(&dev->tx_fifo) < txqhiwat)
		mask |= POLLOUT | POLLWRNORM;

	return kcan_put_dev_unlock(dev, mask);
}

/*
 * this structure is used in init_module(void)
 */
struct file_operations kcan_fops = {
	/*
	 * marrs:  added owner, which is used to implement a use count that
	 *         disallows rmmod calls when the driver is still in use (as
	 *         suggested by Duncan Sands on the linux-kernel mailinglist)
	 */
	owner:      THIS_MODULE,
	open:       kcan_open,
	release:    kcan_release,
	read:       kcan_read,
	write:      kcan_write,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	ioctl:      kcan_ioctl,
#else
	unlocked_ioctl: kcan_ioctl,
#endif
#ifdef KCAN_CONFIG_COMPAT
	compat_ioctl: kcan_compat_ioctl,
#endif
	poll:       kcan_poll,
};
