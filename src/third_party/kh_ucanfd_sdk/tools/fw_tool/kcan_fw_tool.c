/*
 * KCAN Firmware Tool
 * Copyright (C) 2022-2026 Chengdu Kunhong Electronic Technology Co., Ltd. All rights reserved.
 * Auto-generated release build. Do not edit manually.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/netlink.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>

#include <stdint.h>

struct _st_a {
    uint8_t f0;
    uint8_t f1;
    uint8_t f2;
    uint8_t f3;
    uint8_t f4;
    uint8_t f5;
    uint8_t _r[2];
} __attribute__((packed));

struct _st_b {
    uint8_t f0;
    uint8_t f1;
    uint8_t _r[6];
} __attribute__((packed));

struct _st_c {
    uint32_t f0;
    uint8_t _r[4];
} __attribute__((packed));

struct _st_d {
    uint8_t f0;
    uint8_t _r[7];
} __attribute__((packed));

static const uint16_t _ct[] = {0x15EB, 0x1215, 0x1210, 0x1211, 0x1216, 0x1217, 0x1223, 0x122C, 0x1514, 0x1515};
static inline uint32_t _cid(int i) { return _ct[i] ^ 0x1214; }

static const uint8_t _bt[] = {0x5A, 0x57, 0x76, 0x76, 0x99, 0x98, 0x45, 0x44, 0x31, 0x30};
static inline int _btv(int i) { return _bt[i*2] ^ _bt[i*2+1]; }


static int _gv = 0;
static int _gs = 0;
static int _gc = -1;
static char _gi[IFNAMSIZ] = "can0";


#define _LI(fmt, ...)  do { printf("[INFO] " fmt "\n", ##__VA_ARGS__); } while(0)
#define _LE(fmt, ...) do { fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__); } while(0)
#define _LV(fmt, ...) do { if (_gv) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__); } while(0)

static void _ph(const char *prefix, const uint8_t *data, size_t len)
{
    if (!_gv) return;

    printf("%s: ", prefix);
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}


#define PCAN_DEVICE_MODE_PRO_FD 0u
#define PCAN_DEVICE_MODE_FD     1u
#define PCAN_DEVICE_MODE_X6     2u
#define PCAN_DEVICE_MODE_KCAN_X4 3u
#define PCAN_DEVICE_MODE_MAX   4u

static const char *device_mode_name(uint8_t mode)
{
    switch (mode) {
    case PCAN_DEVICE_MODE_PRO_FD:  return "PRO_FD";
    case PCAN_DEVICE_MODE_FD:      return "FD";
    case PCAN_DEVICE_MODE_X6:      return "X6";
    case PCAN_DEVICE_MODE_KCAN_X4:return "KCAN_X4";
    default:                        return "UNKNOWN";
    }
}

static int parse_device_channels(const char *s, uint8_t *out_mode)
{
    if (!s || !out_mode) return (-5);
    if (s[0] == '\0') return (-5);


    char *end = NULL;
    unsigned long channels = strtoul(s, &end, 0);
    if (!(end && *end == '\0')) {
        return (-5);
    }

    if (channels == 1) {
        *out_mode = PCAN_DEVICE_MODE_FD;
    } else if (channels == 2) {
        *out_mode = PCAN_DEVICE_MODE_PRO_FD;
    } else if (channels == 4) {
        *out_mode = PCAN_DEVICE_MODE_KCAN_X4;
    } else {
        return (-5);
    }

    return 0;
}


static int _fn_a(const char *ifname)
{
    int sock;
    struct ifreq ifr;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        _LE("Failed to create socket: %s", strerror(errno));
        return (-2);
    }

    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0) {
        _LE("Failed to get interface flags: %s", strerror(errno));
        close(sock);
        return (-2);
    }

    ifr.ifr_flags &= ~IFF_UP;

    if (ioctl(sock, SIOCSIFFLAGS, &ifr) < 0) {
        _LE("Failed to set interface down: %s", strerror(errno));
        close(sock);
        return (-2);
    }

    close(sock);
    _LV("Interface %s set down", ifname);
    return 0;
}


static int _fn_b(const char *ifname)
{
    int sock;
    struct ifreq ifr;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        _LE("Failed to create socket: %s", strerror(errno));
        return (-2);
    }

    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0) {
        _LE("Failed to get interface flags: %s", strerror(errno));
        close(sock);
        return (-2);
    }

    ifr.ifr_flags |= IFF_UP;

    if (ioctl(sock, SIOCSIFFLAGS, &ifr) < 0) {
        _LE("Failed to set interface up: %s", strerror(errno));
        close(sock);
        return (-2);
    }

    close(sock);
    _LV("Interface %s set up", ifname);
    return 0;
}


static int _fn_c(const char *ifname, int _fm)
{
    char cmd[512];
    int ret;


    ret = _fn_a(ifname);
    if (ret != 0) {
        return ret;
    }

    if (_fm) {

        snprintf(cmd, sizeof(cmd),
                 "ip link set %s type can tq %d prop-seg %d phase-seg1 %d phase-seg2 %d sjw %d fd off",
                 ifname,
                 _btv(0),
                 _btv(1),
                 _btv(2),
                 _btv(3),
                 _btv(4));
    } else {

        snprintf(cmd, sizeof(cmd),
                 "ip link set %s type can bitrate 1000000 dbitrate 8000000 fd on",
                 ifname);
    }

    _LV("Executing: %s", cmd);
    ret = system(cmd);
    if (ret != 0) {
        _LE("Failed to set CAN bittiming (exit code: %d)", ret);
        _LE("Command was: %s", cmd);
        return (-7);
    }


    ret = _fn_b(ifname);
    if (ret != 0) {
        return ret;
    }

    usleep(100000);

    return 0;
}


static int _fn_d(const char *ifname)
{
    int sock;
    struct ifreq ifr;
    struct sockaddr_can addr;
    struct timeval tv;

    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        _LE("Failed to create CAN socket: %s", strerror(errno));
        return (-2);
    }


    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        _LE("Failed to get interface index for %s: %s", ifname, strerror(errno));
        close(sock);
        return (-2);
    }


    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));


    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        _LE("Failed to bind CAN socket: %s", strerror(errno));
        close(sock);
        return (-2);
    }

    _LV("CAN socket opened on %s (ifindex=%d)", ifname, ifr.ifr_ifindex);
    return sock;
}


static void _fn_e(int sock)
{
    if (sock >= 0) {
        close(sock);
        _LV("CAN socket closed");
    }
}


static int _fn_f(int sock, uint32_t can_id, const uint8_t *data, uint8_t len)
{
    struct can_frame frame;
    ssize_t nbytes;

    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = (len > 8) ? 8 : len;

    if (data && len > 0) {
        memcpy(frame.data, data, frame.can_dlc);
    }

    _ph("TX", frame.data, frame.can_dlc);
    _LV("Sending CAN frame: ID=0x%03X, DLC=%d", can_id, frame.can_dlc);

    nbytes = write(sock, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        _LE("Failed to send CAN frame: %s", strerror(errno));
        return (-3);
    }

    return 0;
}


static int _fn_g(int sock, uint32_t *can_id, uint8_t *data, uint8_t *len, int timeout_ms)
{
    struct can_frame frame;
    ssize_t nbytes;
    struct timeval tv;
    fd_set readfds;
    int ret;


    FD_ZERO(&readfds);
    FD_SET(sock, &readfds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    ret = select(sock + 1, &readfds, NULL, NULL, &tv);
    if (ret < 0) {

        if (errno == ENETDOWN) {
            _LV("Network is down (device resetting - this is expected)");
            return (-1);
        }
        _LE("Select failed: %s", strerror(errno));
        return (-4);
    }
    if (ret == 0) {
        _LV("Receive timeout (%d ms)", timeout_ms);
        return (-1);
    }

    nbytes = read(sock, &frame, sizeof(frame));
    if (nbytes < 0) {

        if (errno == ENETDOWN) {
            _LV("Network is down (device resetting - this is expected)");
            return (-1);
        }
        _LE("Failed to receive CAN frame: %s", strerror(errno));
        return (-4);
    }
    if (nbytes != sizeof(frame)) {
        _LE("Incomplete CAN frame received (%zd bytes)", nbytes);
        return (-4);
    }

    *can_id = frame.can_id & CAN_SFF_MASK;
    *len = frame.can_dlc;
    if (data) {
        memcpy(data, frame.data, frame.can_dlc);
    }

    _LV("Received CAN frame: ID=0x%03X, DLC=%d", *can_id, *len);
    _ph("RX", frame.data, frame.can_dlc);

    return 0;
}


static int _fn_h(int sock, uint32_t cmd_id, const uint8_t *tx_data,
                                     uint8_t tx_len, uint8_t *rx_data, int timeout_ms)
{
    uint32_t rx_id;
    uint8_t rx_len;
    int ret;
    int retries = 3;

    while (retries-- > 0) {

        ret = _fn_f(sock, cmd_id, tx_data, tx_len);
        if (ret != 0) {
            continue;
        }


        ret = _fn_g(sock, &rx_id, rx_data, &rx_len, timeout_ms);
        if (ret == (-1)) {
            _LV("Retry %d...", 3 - retries);
            continue;
        }
        if (ret != 0) {
            continue;
        }


        if (rx_id == cmd_id) {
            return 0;
        }

        _LV("Unexpected response ID: 0x%03X (expected 0x%03X)", rx_id, cmd_id);
    }

    return ret;
}


static int _fn_i(const char *ifname)
{
    int ret;

    _LV("FM+ %s", ifname);


    if (_gc >= 0) {
        _fn_e(_gc);
        _gc = -1;
    }


    ret = _fn_c(ifname, 1);
    if (ret != 0) {
        _LE("FM+ bt fail");
        return ret;
    }


    _gc = _fn_d(ifname);
    if (_gc < 0) {
        _LE("FM+ sk fail");
        return (-2);
    }

    _LV("FM+OK");
    return 0;
}


static int _fn_j(const char *ifname)
{
    int ret;

    _LV("FM- %s", ifname);


    if (_gc >= 0) {
        _fn_e(_gc);
        _gc = -1;
    }


    ret = _fn_c(ifname, 0);
    if (ret != 0) {
        _LE("FM- bt fail");
        return ret;
    }

    _LV("FM-OK");
    return 0;
}


static int cmd_get_boot_version(void);

static int _fn_k(void)
{
    uint8_t response[8] = {0};
    int ret;

    _LI("Getting firmware version...");

    ret = _fn_h(_gc, _cid(1),
                                     NULL, 0, response, 1000);
    if (ret != 0) {
        _LE("Failed to get version");
        return ret;
    }

    struct _st_a *ver = (struct _st_a *)response;

    printf("=== Firmware Version ===\n");
    printf("  Version: %d.%d.%d\n", ver->f0, ver->f1, ver->f2);
    printf("  Build Date: 20%02d-%02d-%02d\n", ver->f3, ver->f4, ver->f5);
    printf("=== BL Version ===\n");
    (void)cmd_get_boot_version();
    printf("========================\n");

    return 0;
}


static int cmd_get_boot_version(void)
{
    uint8_t response[8] = {0};
    int ret;

    _LI("Getting bootloader version...");

    ret = _fn_h(_gc, _cid(2),
                                     NULL, 0, response, 1000);
    if (ret != 0) {
        _LE("Failed to get boot version");
        return ret;
    }

    uint8_t header_version = response[0];
    uint8_t sw_major = response[1];
    uint8_t sw_minor = response[2];
    uint8_t sw_patch = response[3];
    uint8_t sw_build = response[4];


    (void)header_version;
    printf("  Version: %u.%u.%u.%u\n",
           (unsigned)sw_major, (unsigned)sw_minor,
           (unsigned)sw_patch, (unsigned)sw_build);

    return 0;
}


static int _fn_v(uint8_t req_mode)
{
    uint8_t tx_data[8] = {0};
    uint8_t rx_data[8] = {0};
    int ret;

    if (req_mode >= PCAN_DEVICE_MODE_MAX) {
        _LE("Invalid device mode: %u", (unsigned)req_mode);
        return (-5);
    }

    tx_data[0] = req_mode;

    _LI("Setting device mode: %s (%u)", device_mode_name(req_mode), (unsigned)req_mode);

    ret = _fn_h(_gc,
                                     _cid(3),
                                     tx_data, 8,
                                     rx_data, 1000);
    if (ret != 0) {
        _LE("Failed to set device mode (no/invalid ACK)");
        return ret;
    }


    if (rx_data[0] == 0xFF && rx_data[1] == 0xFF &&
        rx_data[2] == 0xFF && rx_data[3] == 0xFF) {
        _LE("Set device mode failed");
        return (-6);
    }

    printf("Device mode: %s (%u)\n",
           device_mode_name(rx_data[0]),
           (unsigned)rx_data[0]);
    printf("Device is rebooting to apply mode...\n");


    usleep(500 * 1000);
    _fn_e(_gc);
    _gc = -1;

    return 0;
}


static int _fn_l(int enable)
{
    uint8_t tx_data[8] = {0};
    uint8_t rx_data[8] = {0};
    int ret;

    tx_data[0] = enable ? 1 : 0;

    _LI("Setting termination resistor: %s", enable ? "ON" : "OFF");

    ret = _fn_h(_gc, _cid(4),
                                     tx_data, 8, rx_data, 1000);
    if (ret != 0) {
        _LE("Failed to set termination");
        return ret;
    }

    printf("Termination resistor set to: %s\n", enable ? "ON (120Ω)" : "OFF");

    return 0;
}


static int _fn_m(void)
{
    uint8_t response[8] = {0};
    int ret;

    _LI("Getting termination status...");

    ret = _fn_h(_gc, _cid(5),
                                     NULL, 0, response, 1000);
    if (ret != 0) {
        _LE("Failed to get termination status");
        return ret;
    }

    struct _st_d *term = (struct _st_d *)response;

    printf("Termination resistor: %s\n", term->f0 ? "ON (120Ω)" : "OFF");

    return 0;
}


static int _fn_n(void)
{
    uint8_t response[8] = {0};
    int ret;

    _LV("SC...");

    ret = _fn_h(_gc, _cid(6),
                                     NULL, 0, response, 1000);
    if (ret != 0) {
        _LE("SC fail");
        return ret;
    }

    struct _st_b *sec = (struct _st_b *)response;

    printf("SC:U%d,C%d\n", sec->f0, sec->f1);

    return 0;
}


static int _fn_o(void)
{
    uint8_t response[8] = {0};
    int ret;

    _LV("CC...");

    ret = _fn_h(_gc, _cid(7),
                                     NULL, 0, response, 1000);
    if (ret != 0) {
        _LE("CC fail");
        return ret;
    }

    struct _st_c *crc = (struct _st_c *)response;

    printf("C32:0x%08X\n", crc->f0);

    return 0;
}


static int _fn_p(void)
{
    uint8_t tx_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int ret;

    _LV("BL...");

    if (!_gs) {
        printf("BL? [y/N]: ");
        char ch = getchar();
        if (ch != 'y' && ch != 'Y') {
            printf("N\n");
            return 0;
        }

        while (getchar() != '\n');
    }


    ret = _fn_f(_gc, _cid(0), tx_data, 2);
    if (ret != 0) {
        _LE("BL fail");
        return ret;
    }

    printf("BL+RST\n");


    usleep(500 * 1000);


    _fn_e(_gc);
    _gc = -1;

    _LV("BL+OK");

    return 0;
}


static int _fn_q(uint32_t _aa)
{
    uint8_t tx_data[8] = {0};
    uint8_t rx_data[8] = {0};
    uint32_t rx_id;
    uint8_t rx_len;
    int ret;

    _LI("Switching to APP at address 0x%08X...", _aa);
    printf("WARNING: Device will reset and switch to specified APP!\n");

    if (!_gs) {
        printf("Continue? [y/N]: ");
        char ch = getchar();
        if (ch != 'y' && ch != 'Y') {
            printf("Aborted.\n");
            return 0;
        }

        while (getchar() != '\n');
    }


    tx_data[0] = (_aa >> 0) & 0xFF;
    tx_data[1] = (_aa >> 8) & 0xFF;
    tx_data[2] = (_aa >> 16) & 0xFF;
    tx_data[3] = (_aa >> 24) & 0xFF;


    ret = _fn_f(_gc, _cid(8), tx_data, 4);
    if (ret != 0) {
        _LE("Failed to send switch APP command");
        return ret;
    }


    ret = _fn_g(_gc, &rx_id, rx_data, &rx_len, 500);
    if (ret == 0) {
        if (rx_id == _cid(8)) {
            uint32_t error_pattern = rx_data[0] | (rx_data[1] << 8) |
                                     (rx_data[2] << 16) | (rx_data[3] << 24);
            if (error_pattern == ((uint32_t)0xFFFFFFFF)) {
                _LE("APP switch failed - invalid address or APP not found");
                return (-6);
            }
        }
    }


    printf("APP switch command sent. Device will reset to APP at 0x%08X...\n", _aa);


    usleep(500 * 1000);
    _fn_e(_gc);
    _gc = -1;

    return 0;
}


static int _fn_r(void)
{
    uint8_t rx_data[8] = {0};
    uint32_t rx_id;
    uint8_t rx_len;
    int ret;

    _LI("Switching to next APP...");
    printf("WARNING: Device will reset and switch to next APP slot!\n");

    if (!_gs) {
        printf("Continue? [y/N]: ");
        char ch = getchar();
        if (ch != 'y' && ch != 'Y') {
            printf("Aborted.\n");
            return 0;
        }

        while (getchar() != '\n');
    }


    ret = _fn_f(_gc, _cid(9), NULL, 0);
    if (ret != 0) {
        _LE("Failed to send next APP command");
        return ret;
    }


    ret = _fn_g(_gc, &rx_id, rx_data, &rx_len, 500);
    if (ret == 0) {
        if (rx_id == _cid(9)) {
            uint32_t error_pattern = rx_data[0] | (rx_data[1] << 8) |
                                     (rx_data[2] << 16) | (rx_data[3] << 24);
            if (error_pattern == ((uint32_t)0xFFFFFFFF)) {
                _LE("Next APP command failed - no next APP available");
                return (-6);
            }
        }
    }

    printf("Next APP command sent. Device will reset...\n");


    usleep(500 * 1000);
    _fn_e(_gc);
    _gc = -1;

    return 0;
}


static void _fn_s(void)
{
    printf("\n");
    printf("╔════════════════════════════════════════════╗\n");
    printf("║   KCAN Factory Mode Tool - Main Menu       ║\n");
    printf("╠════════════════════════════════════════════╣\n");
    printf("║  1. Get Firmware Version                   ║\n");
    printf("║  2. Get Termination Status                 ║\n");
    printf("║  3. Set Termination ON (120Ω)              ║\n");
    printf("║  4. Set Termination OFF                    ║\n");
    printf("║  5. Switch to APP (specify address)        ║\n");
    printf("║  6. Switch to Next APP                     ║\n");
    printf("║  7. Set Device Mode (by channels)          ║\n");
    printf("║  0. Exit                                   ║\n");
    printf("╚════════════════════════════════════════════╝\n");
    printf("Select option: ");
}

static int _fn_t(void)
{
    char input[64];
    int choice;

    while (1) {
        _fn_s();

        if (fgets(input, sizeof(input), stdin) == NULL) {
            break;
        }

        choice = atoi(input);


        if (_gc < 0 && choice != 0) {
            _gc = _fn_d(_gi);
            if (_gc < 0) {
                _LE("CAN socket not available. Try re-entering factory mode.");
                printf("Press Enter to continue...");
                getchar();
                continue;
            }
        }

        switch (choice) {
        case 1:
            (void)_fn_k();
            break;
        case 2:
            (void)_fn_m();
            break;
        case 3:
            (void)_fn_l(1);
            break;
        case 4:
            (void)_fn_l(0);
            break;
        case 5:
            printf("Enter APP address (hex, e.g., 0x10000): ");
            if (fgets(input, sizeof(input), stdin) != NULL) {
                uint32_t addr = strtoul(input, NULL, 0);
                if (addr > 0) {
                    (void)_fn_q(addr);
                } else {
                    _LE("Invalid address");
                }
            }
            break;
        case 6:
            (void)_fn_r();
            break;
        case 7:
            printf("Enter channel count (1:FD, 2:FDPRO, 4:KCAN_X4): ");
            if (fgets(input, sizeof(input), stdin) != NULL) {
                input[strcspn(input, "\r\n")] = '\0';
                uint8_t mode;
                if (parse_device_channels(input, &mode) == 0) {
                    (void)_fn_v(mode);
                } else {
                    _LE("Invalid channel count (supported: 1/2/4)");
                }
            }
            break;
        case 0:
            printf("Exiting...\n");
            return 0;
        default:
            printf("Invalid option\n");
            break;
        }

        printf("\nPress Enter to continue...");
        getchar();
    }

    return 0;
}


static void _fn_u(const char *prog)
{
    printf("KCAN Factory Mode Firmware Updater Tool\n\n");
    printf("Usage: %s <interface> <command> [options]\n\n", prog);
    printf("Commands:\n");
    printf("  version       Get firmware version\n");
    printf("  term-get      Get termination status\n");
    printf("  term-on       Enable termination resistor (120Ω)\n");
    printf("  term-off      Disable termination resistor\n");
    printf("  switch-app    Switch to APP at address (requires -a option)\n");
    printf("  next-app      Switch to next APP\n");
    printf("  device-mode   Set device mode by channel count (requires -c option)\n");
    printf("  menu          Interactive menu mode\n");
    printf("\n");
    printf("Options:\n");
    printf("  -a, --address ADDR   APP address for switch-app command (hex)\n");
    printf("  -c, --channels N    Channel count: 1->FD, 2->FDPRO, 4->KCAN_X4\n");
    printf("  -v, --verbose        Verbose output\n");
    printf("  -y, --yes            Skip confirmation prompts (auto-answer yes)\n");
    printf("  -h, --help           Show this help\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s can0 version          # Get firmware version\n", prog);
    printf("  %s can0 term-on          # Enable termination\n", prog);
    printf("  %s can0 switch-app -a 0x10000  # Switch to APP at 0x10000\n", prog);
    printf("  %s can0 device-mode -c 4 # Set KCAN_X4 mode by channels\n", prog);
    printf("  %s can0 switch-app -a 0x40000 -y  # Switch without confirmation\n", prog);
    printf("  %s can0 menu             # Interactive menu\n", prog);
}

int main(int argc, char *argv[])
{
    int opt;
    int ret = 0;
    uint32_t _aa = 0;
    const char *device_channels_arg = NULL;
    int _nfm = 0;

    static struct option long_opts[] = {
        {"address", required_argument, 0, 'a'},
        {"channels", required_argument, 0, 'c'},
        {"verbose", no_argument, 0, 'v'},
        {"yes", no_argument, 0, 'y'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "a:c:vyh", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'a':
            _aa = strtoul(optarg, NULL, 0);
            break;
        case 'c':
            device_channels_arg = optarg;
            break;
        case 'v':
            _gv = 1;
            break;
        case 'y':
            _gs = 1;
            break;
        case 'h':
            _fn_u(argv[0]);
            return 0;
        default:
            _fn_u(argv[0]);
            return 1;
        }
    }

    if (optind >= argc) {
        _fn_u(argv[0]);
        return 1;
    }


    strncpy(_gi, argv[optind], IFNAMSIZ - 1);
    _gi[IFNAMSIZ - 1] = '\0';
    optind++;

    if (optind >= argc) {
        _fn_u(argv[0]);
        return 1;
    }

    const char *command = argv[optind];


    if (strcmp(command, "enter") != 0 && strcmp(command, "exit") != 0) {
        _nfm = 1;
    }


    if (strcmp(command, "enter") == 0) {
        ret = _fn_i(_gi);
        if (ret == 0) {
            printf("FM+OK\n");
        }
    }
    else if (strcmp(command, "exit") == 0) {
        ret = _fn_j(_gi);
    }
    else if (strcmp(command, "menu") == 0) {
        ret = _fn_i(_gi);
        if (ret == 0) {
            ret = _fn_t();
            _fn_j(_gi);
        }
    }
    else {

        if (_nfm) {
            ret = _fn_i(_gi);
            if (ret != 0) {
                _LE("Failed to enter factory mode");
                return 1;
            }
        }


        if (strcmp(command, "version") == 0) {
            ret = _fn_k();
        }
        else if (strcmp(command, "term-get") == 0) {
            ret = _fn_m();
        }
        else if (strcmp(command, "term-on") == 0) {
            ret = _fn_l(1);
        }
        else if (strcmp(command, "term-off") == 0) {
            ret = _fn_l(0);
        }
        else if (strcmp(command, "device-mode") == 0) {
            if (!device_channels_arg) {
                _LE("Channel count required (-c option)");
                ret = 1;
            } else {
                uint8_t mode;
                if (parse_device_channels(device_channels_arg, &mode) != 0) {
                    _LE("Invalid channel count: %s (supported: 1/2/4)", device_channels_arg);
                    ret = 1;
                } else {
                    ret = _fn_v(mode);
                }
            }
        }
        else if (strcmp(command, "sc") == 0) {
            ret = _fn_n();
        }
        else if (strcmp(command, "cc") == 0) {
            ret = _fn_o();
        }
        else if (strcmp(command, "bl") == 0) {
            ret = _fn_p();
        }
        else if (strcmp(command, "switch-app") == 0) {
            if (_aa == 0) {
                _LE("APP address required (-a option)");
                ret = 1;
            } else {
                ret = _fn_q(_aa);
            }
        }
        else if (strcmp(command, "next-app") == 0) {
            ret = _fn_r();
        }
        else {
            _LE("Unknown command: %s", command);
            _fn_u(argv[0]);
            ret = 1;
        }


        if (_nfm && _gc >= 0) {
            _fn_j(_gi);
        }
    }


    if (_gc >= 0) {
        _fn_e(_gc);
    }

    return (ret == 0) ? 0 : 1;
}
