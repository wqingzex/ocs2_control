#define _DEFAULT_SOURCE

#include <arpa/inet.h>
#include <dlfcn.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>

int bind(int fd, const struct sockaddr *address, socklen_t length)
{
    typedef int (*BindFn)(int, const struct sockaddr *, socklen_t);
    BindFn system_bind;
    void *symbol = dlsym(RTLD_NEXT, "bind");
    memcpy(&system_bind, &symbol, sizeof(system_bind));
    if (!system_bind) {
        errno = ENOSYS;
        return -1;
    }

    const char *port = getenv("FA_XRT_PICO_TCP_PORT");
    if (!port || !address || length < sizeof(address->sa_family)) {
        return system_bind(fd, address, length);
    }

    struct sockaddr_in ipv4;
    struct sockaddr_in6 ipv6;
    unsigned int socket_port;
    if (address->sa_family == AF_INET && length >= sizeof(ipv4)) {
        memcpy(&ipv4, address, sizeof(ipv4));
        socket_port = ntohs(ipv4.sin_port);
    } else if (address->sa_family == AF_INET6 && length >= sizeof(ipv6)) {
        memcpy(&ipv6, address, sizeof(ipv6));
        socket_port = ntohs(ipv6.sin6_port);
    } else {
        return system_bind(fd, address, length);
    }

    int socket_type;
    socklen_t type_length = sizeof(socket_type);
    if (socket_port != strtoul(port, NULL, 10) ||
        getsockopt(fd, SOL_SOCKET, SO_TYPE, &socket_type, &type_length) != 0 ||
        socket_type != SOCK_STREAM) {
        return system_bind(fd, address, length);
    }

    const char *interface = getenv("FA_XRT_LAN_INTERFACE");
    const char *ip = getenv("FA_XRT_LAN_IP");
    struct in_addr lan_address;
    if (!interface || !*interface || !ip || inet_pton(AF_INET, ip, &lan_address) != 1) {
        errno = EINVAL;
        return -1;
    }
    if (setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, interface, strlen(interface) + 1) != 0) {
        return -1;
    }

    if (address->sa_family == AF_INET) {
        ipv4.sin_addr = lan_address;
        return system_bind(fd, (const struct sockaddr *)&ipv4, sizeof(ipv4));
    }

    // Qt uses a dual-stack socket for the wildcard Pico listener.
    memset(&ipv6.sin6_addr, 0, sizeof(ipv6.sin6_addr));
    ipv6.sin6_addr.s6_addr[10] = 0xff;
    ipv6.sin6_addr.s6_addr[11] = 0xff;
    memcpy(&ipv6.sin6_addr.s6_addr[12], &lan_address, sizeof(lan_address));
    ipv6.sin6_scope_id = 0;
    return system_bind(fd, (const struct sockaddr *)&ipv6, sizeof(ipv6));
}
