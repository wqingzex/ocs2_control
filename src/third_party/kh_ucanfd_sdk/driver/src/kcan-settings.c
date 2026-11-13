/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * kcan-settings.c - a small program to get and set various parameters of
 *                   KCAN USB devices
 *
 * Ported into KH-UCANFD_Linux_SDK_dev from KH-UCANFD_LinuxSDK-v1.2.2.
 */

#define CURRENT_RELEASE "Release_20181220_n"
#define PRGNAME "kcan-settings"

#define USES_32BITS_DEVICENO

#include <ctype.h>
#include <errno.h>
#include <err.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <kcanfd.h>

#if !defined(NO_RT) && !defined(__COBALT__)
#include <rtdm/rtdm.h>

#define __open(x, y)             rt_dev_open(x, y)
#define __errno_ioctl(x, y, z)   -rt_dev_ioctl(x, y, z)
#define __ioctl(x, y, z)         rt_dev_ioctl(x, y, z)
#define __close(fp)              rt_dev_close(fp)
#define __fprintf                rt_fprintf
#else
static inline int __open(char *pfn, int flg)
{
	int fd = open(pfn, flg);

	return (fd >= 0) ? fd : -errno;
}

#define __errno_ioctl(x, y, z)   (ioctl(x, y, z) ? errno : 0)
#define __ioctl(x, y, z)         ioctl(x, y, z)
#define __close(fp)              close(fp)
#define __fprintf                fprintf
#endif

#define DEFAULT_NODE "/dev/kcan32"

#ifndef bool
#define bool int
#define true 1
#define false 0
#endif

#define SERIALNO 0x00000001
#define DEVICENO 0x00000002
#define VERBOSE  0x00000004
#define QUIET    0x00000008
#define MSD      0x00000010

static int normalize_errno(int err)
{
	if (!err)
		return 0;

	return (err < 0) ? -err : err;
}

static void print_errno(const char *message, int err)
{
	err = normalize_errno(err);
	__fprintf(stderr, PRGNAME ": %s: %s (errno=%d)\n",
		  message, strerror(err), err);
}

static int redirect_standard_streams_to_null(void)
{
	int err = 0;
	int null_fd;

	fflush(stdout);
	fflush(stderr);

	null_fd = open("/dev/null", O_WRONLY);
	if (null_fd < 0)
		return errno;

	if (dup2(null_fd, STDOUT_FILENO) < 0)
		err = errno;
	else if (dup2(null_fd, STDERR_FILENO) < 0)
		err = errno;

	if (null_fd != STDOUT_FILENO && null_fd != STDERR_FILENO)
		close(null_fd);

	return err;
}

static int get_serial_number(int fd, unsigned int *serial_no)
{
	int err;
	TPEXTRAPARAMS params;

	params.nSubFunction = SF_GET_SERIALNUMBER;
	err = __errno_ioctl(fd, KCAN_EXTRA_PARAMS, &params);
	if (!err)
		*serial_no = params.func.dwSerialNumber;

	return normalize_errno(err);
}

static int get_device_number(int fd, __uint32_t *device_no)
{
	int err;
	TPEXTRAPARAMS params;

	params.nSubFunction = SF_GET_HCDEVICENO;
	err = __errno_ioctl(fd, KCAN_EXTRA_PARAMS, &params);
	if (!err)
#ifndef USES_32BITS_DEVICENO
		*device_no = (__uint32_t)params.func.ucHCDeviceNo;
#else
		*device_no = (__uint32_t)params.func.dwSerialNumber;
#endif
	return normalize_errno(err);
}

static int set_device_number(int fd, __uint32_t device_no)
{
	TPEXTRAPARAMS params;

	params.nSubFunction = SF_SET_HCDEVICENO;
#ifndef USES_32BITS_DEVICENO
	params.func.ucHCDeviceNo = (BYTE)device_no;
#else
	params.func.dwSerialNumber = (DWORD)device_no;
#endif
	return normalize_errno(__errno_ioctl(fd, KCAN_EXTRA_PARAMS, &params));
}

static int set_mass_storage_mode(int fd)
{
	__u32 onoff = 1;
	struct kcanfd_option opt = {
		.name = KCANFD_OPT_MASS_STORAGE_MODE,
		.size = sizeof(onoff),
		.value = &onoff,
	};

	return normalize_errno(__errno_ioctl(fd, KCANFD_SET_OPTION, &opt));
}

static void print_usage(FILE *stream, const char *progname)
{
	__fprintf(stream,
		  "Usage: %s [OPTIONS]\n"
		  "  -f, --deviceNode PATH   Set path to KCAN device (default: %s)\n"
		  "  -s, --SerialNo          Get serial number\n"
		  "  -d, --DeviceNo[=NUM]    Get or set device number\n"
		  "  -v, --verbose           Make it verbose\n"
		  "  -q, --quiet             No display at all\n"
		  "  -M, --MSD               Switch device to Mass Storage Device mode\n"
		  "  -?, --help              Show this help\n",
		  progname, DEFAULT_NODE);
}

static char *skip_optional_equal(char *arg)
{
	return (arg && arg[0] == '=') ? arg + 1 : arg;
}

int main(int argc, char *argv[])
{
	int ret = 0;
	char *dev_node = DEFAULT_NODE;
	unsigned int serial_no = 0;
	__uint32_t device_no = 0;
	int result = 0;
	int verbose = false;
	int device_arg_set = false;
	int opt;
	static const struct option long_opts[] = {
		{ "deviceNode", required_argument, NULL, 'f' },
		{ "SerialNo", no_argument, NULL, 's' },
		{ "DeviceNo", optional_argument, NULL, 'd' },
		{ "verbose", no_argument, NULL, 'v' },
		{ "quiet", no_argument, NULL, 'q' },
		{ "MSD", no_argument, NULL, 'M' },
		{ "help", no_argument, NULL, '?' },
		{ NULL, 0, NULL, 0 }
	};

	errno = 0;

	while ((opt = getopt_long(argc, argv, "?f:sd::vqM", long_opts, NULL)) != -1) {
		switch (opt) {
		case 'f':
			dev_node = skip_optional_equal(optarg);
			break;
		case 's':
			result |= SERIALNO;
			break;
		case 'd':
			result |= DEVICENO;
			if (optarg) {
				device_no = strtoul(skip_optional_equal(optarg),
						    NULL, 0);
				device_arg_set = true;
			} else if ((optind < argc) && argv[optind] &&
				   (argv[optind][0] != '-')) {
				device_no = strtoul(skip_optional_equal(argv[optind]),
						    NULL, 0);
				device_arg_set = true;
				optind++;
			}
			break;
		case 'v':
			result |= VERBOSE;
			verbose = true;
			break;
		case 'q':
			result |= QUIET;
			ret = redirect_standard_streams_to_null();
			if (ret)
				return ret;
			break;
		case 'M':
			result |= MSD;
			break;
		case '?':
			print_usage(stderr, PRGNAME);
			return opt == '?' && (optopt == 0) ? 0 : EINVAL;
		default:
			print_usage(stderr, PRGNAME);
			return EINVAL;
		}
	}

	errno = EINVAL;
	result &= ~(VERBOSE | QUIET);

	if (!result) {
		print_usage(stderr, PRGNAME);
		err(errno, "more than a device node required");
	}

	ret = 0;
	if (result) {
		int fd;

		fd = __open(dev_node, O_RDWR);
		if (fd < 0) {
			errno = -fd;
			err(-fd, "open(%s) failed", dev_node);
		}

		if ((result & SERIALNO) && !ret) {
			ret = get_serial_number(fd, &serial_no);
			if (ret)
				print_errno("Failed to get serial number", ret);
			else if (verbose)
				printf(PRGNAME ": Serial Number is 0x%08x\n", serial_no);
			else
				printf("0x%08x\n", serial_no);
		}

		if ((result & DEVICENO) && !ret) {
			if (device_arg_set) {
				ret = set_device_number(fd, device_no);
				if (ret)
					print_errno("Failed to set device number", ret);
				else if (verbose)
					printf(PRGNAME ": Device Number set to %u\n", device_no);
			} else {
				ret = get_device_number(fd, &device_no);
				if (ret)
					print_errno("Failed to get device number", ret);
				else if (verbose)
					printf(PRGNAME ": Device Number is %u\n", device_no);
				else
					printf("%u\n", device_no);
			}
		}

		if ((result & MSD) && !ret) {
			ret = set_mass_storage_mode(fd);
			if (ret) {
				__fprintf(stderr,
					  PRGNAME ": failed to switch \"%s\" in Mass Storage Device mode: %s (errno=%d)\n",
					  dev_node, strerror(ret), ret);
			} else {
				printf(PRGNAME ": Mass Storage mode successfully set\n");
				if (verbose) {
					printf("\n");
					printf("The device node \"%s\" doesn't exist anymore.\n",
					       dev_node);
				}

				printf("Please wait for the LED(s) of the USB device to flash, then, if not\n");
				printf("automatically done by the system, mount a VFAT filesystem on the newly\n");
				printf("detected USB Mass Storage Device \"/dev/sdX\".\n");
				if (verbose) {
					printf("\n");
					printf("For example:\n");
					printf("# mkdir -p /mnt/kcan-usb\n");
					printf("# mount -t vfat /dev/sdX /mnt/kcan-usb\n");
					printf("# ls -al /mnt/kcan-usb\n");
				}
			}
		}

		__close(fd);
	}

	return ret;
}
