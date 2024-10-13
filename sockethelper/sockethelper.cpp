#include "sockethelper.h"

#include <arpa/inet.h>
#include <execinfo.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

bool openTcpSendSocket(int& sock, struct sockaddr_in& saddr, const char* ip, uint16_t port) {
    static const int on = 1;
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Error creating socket: ");
        fprintf(stderr, "Error: creating socket in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        return false;
    }

    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on))) {
        perror("setsockopt reuseaddr:");
        fprintf(stderr, "Error: setsockopt failed in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        close(sock);
        return false;
    }

    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = INADDR_ANY;
    saddr.sin_port = htons(port);

    if (ip != nullptr) {
        if (inet_aton(ip, &(saddr.sin_addr)) == 0) {
            fprintf(stderr, "Error: can't convert ascii ip address %s to int %s line %d. Exiting.\n", ip, __FILE__, __LINE__);
            close(sock);
            return false;
        }
    } else {
        fprintf(stderr, "Error: ip address is NULL %s line %d. Exiting.\n", __FILE__, __LINE__);
        close(sock);
        return false;
    }

    if (connect(sock, (struct sockaddr*)&saddr, sizeof(saddr)) < 0) {
        perror("Error connect socket: ");
        fprintf(stderr, "Error: bind to port %d on %s failed in file %s line %d. Exiting.\n", port, ip, __FILE__, __LINE__);
        close(sock);

        void* array[10];
        size_t size;
        char** strings;
        size_t i;

        printf("Visualizer: Transaction was changed after it was commit! Change was from:\n");

        size = backtrace(array, 10);
        strings = backtrace_symbols(array, size);

        for (i = 0; i < size; i++) {
            printf("%s\n", strings[i]);
        }

        free(strings);
        fflush(stdout);

        return false;
    }

    return true;
}

void openTcpRecvSocket(int& sock, struct sockaddr_in& saddr, const char* ip, uint16_t port, int backLogLength) {
    static const int on = 1;
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Error creating socket: ");
        fprintf(stderr, "Error: creating socket in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        exit(EXIT_FAILURE);
    }

    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on))) {
        perror("setsockopt reuseaddr:");
        fprintf(stderr, "Error: setsockopt failed in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        close(sock);
        exit(EXIT_FAILURE);
    }

    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = INADDR_ANY;
    saddr.sin_port = htons(port);

    if (ip != nullptr) {
        if (inet_aton(ip, &(saddr.sin_addr)) == 0) {
            fprintf(stderr, "Error: can't convert ascii ip address %s to int %s line %d. Exiting.\n", ip, __FILE__, __LINE__);
            close(sock);
            exit(EXIT_FAILURE);
        }
    }

    if (bind(sock, (struct sockaddr*)&saddr, sizeof(saddr)) < 0) {
        perror("Error bind socket: ");
        fprintf(stderr, "Error: bind failed in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        close(sock);
        exit(EXIT_FAILURE);
    }

    if (listen(sock, backLogLength) < 0) {
        perror("Error listen to socket");
        fprintf(stderr, "Error listen to socket in file: %s line %d. Existing", __FILE__, __LINE__);
        close(sock);
        exit(1);
    }
}

void openRecvBroadcastSocket(struct sockaddr_in& saddr, int& sock, uint16_t port) {
    static const int on = 1;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Error creating socket: ");
        fprintf(stderr, "Error: creating socket in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        exit(EXIT_FAILURE);
    }

    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on))) {
        perror("setsockopt reuseaddr:");
        fprintf(stderr, "Error: setsockopt failed in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        close(sock);
        exit(EXIT_FAILURE);
    }

    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = INADDR_ANY;
    saddr.sin_port = htons(port);

    if (bind(sock, (struct sockaddr*)&saddr, sizeof(saddr)) < 0) {
        perror("Error bind socket: ");
        fprintf(stderr, "Error: bind failed in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        close(sock);
        exit(EXIT_FAILURE);
    }
}

void sendBroadcast(uint16_t port, uint8_t* data, size_t size) {
    int status;
    static const int on = 1;

    sockaddr_in bc_saddr;
    memset(&bc_saddr, 0, sizeof(bc_saddr));
    bc_saddr.sin_family = AF_INET;
    bc_saddr.sin_addr.s_addr = INADDR_BROADCAST;
    bc_saddr.sin_port = htons(port);

    struct ifaddrs* addrs = nullptr;
    int result = getifaddrs(&addrs);
    if (result < 0) {
        perror("Couldn't get network interfaces!");
        fprintf(stderr, "Error: getting iface list in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        exit(1);
    }

    const struct ifaddrs* cursor = addrs;
    while (cursor != nullptr) {
        if (cursor->ifa_addr != nullptr && cursor->ifa_addr->sa_family == AF_INET && !(cursor->ifa_flags & IFF_POINTOPOINT) &&
            !(cursor->ifa_flags & IFF_LOOPBACK) && (cursor->ifa_flags & IFF_BROADCAST) && (cursor->ifa_flags & IFF_UP)) {

            int sock = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock < 0) {
                perror("Error creating socket");
                fprintf(stderr, "Error: creating socket in file %s line %d. Exiting.\n", __FILE__, __LINE__);
                exit(EXIT_FAILURE);
            }

            sockaddr_in saddr;
            memset(&saddr, 0, sizeof(saddr));
            saddr.sin_family = AF_INET;
            saddr.sin_port = 0;
            saddr.sin_addr = ((struct sockaddr_in*)cursor->ifa_addr)->sin_addr;

            if (bind(sock, (struct sockaddr*)&saddr, sizeof(saddr)) < 0) {
                perror("bind:");
                fprintf(stderr, "Error: bind failed in file %s line %d. Exiting.\n", __FILE__, __LINE__);
                close(sock);
                cursor = cursor->ifa_next;
                continue;
            }

            status = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
            if (status) {
                perror("Couldn't set  broadcast");
                fprintf(stderr, "Error: setsockopt failed in file %s line %d. Exiting.\n", __FILE__, __LINE__);
                close(sock);
                cursor = cursor->ifa_next;
                continue;
            }

            bc_saddr.sin_addr = ((struct sockaddr_in*)cursor->ifa_ifu.ifu_broadaddr)->sin_addr;

            if (sendto(sock, data, size, MSG_DONTWAIT, (struct sockaddr*)&bc_saddr, sizeof(bc_saddr)) < 0) {
                perror("Couldn't send");
                fprintf(stderr, "Error: sending udp bc failed in file %s line %d. Exiting.\n", __FILE__, __LINE__);
            }

            close(sock);
        }
        cursor = cursor->ifa_next;
    }
    freeifaddrs(addrs);
}

void sendBroadcastByIface(const char* iface, uint16_t port, uint8_t* data, size_t size) {
    int status;
    static const int on = 1;

    sockaddr_in bc_saddr;
    memset(&bc_saddr, 0, sizeof(bc_saddr));
    bc_saddr.sin_family = AF_INET;
    bc_saddr.sin_addr.s_addr = INADDR_BROADCAST;
    bc_saddr.sin_port = htons(port);

    struct ifaddrs* addrs = nullptr;
    int result = getifaddrs(&addrs);
    if (result < 0) {
        perror("Couldn't get network interfaces!");
        fprintf(stderr, "Error: getting iface list in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        exit(1);
    }

    const struct ifaddrs* cursor = addrs;
    while (cursor != nullptr) {
        if(std::strcmp(iface, cursor->ifa_name)) {
            cursor = cursor->ifa_next;
            continue;
        }

        if (cursor->ifa_addr != nullptr && cursor->ifa_addr->sa_family == AF_INET && !(cursor->ifa_flags & IFF_POINTOPOINT) &&
            !(cursor->ifa_flags & IFF_LOOPBACK) && (cursor->ifa_flags & IFF_BROADCAST) && (cursor->ifa_flags & IFF_UP)) {

            int sock = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock < 0) {
                perror("Error creating socket");
                fprintf(stderr, "Error: creating socket in file %s line %d. Exiting.\n", __FILE__, __LINE__);
                exit(EXIT_FAILURE);
            }

            sockaddr_in saddr;
            memset(&saddr, 0, sizeof(saddr));
            saddr.sin_family = AF_INET;
            saddr.sin_port = 0;
            saddr.sin_addr = ((struct sockaddr_in*)cursor->ifa_addr)->sin_addr;

            if (bind(sock, (struct sockaddr*)&saddr, sizeof(saddr)) < 0) {
                perror("bind:");
                fprintf(stderr, "Error: bind failed in file %s line %d. Exiting.\n", __FILE__, __LINE__);
                close(sock);
                cursor = cursor->ifa_next;
                continue;
            }

            status = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
            if (status) {
                perror("Couldn't set  broadcast");
                fprintf(stderr, "Error: setsockopt failed in file %s line %d. Exiting.\n", __FILE__, __LINE__);
                close(sock);
                cursor = cursor->ifa_next;
                continue;
            }

            bc_saddr.sin_addr = ((struct sockaddr_in*)cursor->ifa_ifu.ifu_broadaddr)->sin_addr;

            if (sendto(sock, data, size, MSG_DONTWAIT, (struct sockaddr*)&bc_saddr, sizeof(bc_saddr)) < 0) {
                perror("Couldn't send");
                fprintf(stderr, "Error: sending udp bc failed in file %s line %d. Exiting.\n", __FILE__, __LINE__);
            }

            close(sock);
        }
        cursor = cursor->ifa_next;
    }
    freeifaddrs(addrs);
}

bool socketReadBytes(int sock, uint32_t len, void* buffer) {
    unsigned int bytesRead = 0;
    int result;
    uint8_t* ptr = (uint8_t*)buffer;

    while (bytesRead < len) {
        result = read(sock, ptr + bytesRead, len - bytesRead);
        if (result < 1) {
            return false;
        }

        bytesRead += result;
    }
    return true;
}

bool socketWriteBytes(int sock, uint32_t sendLeft, const void* buffer) {
    uint8_t* ptr = (uint8_t*)buffer;
    while (sendLeft > 0) {
        int rc = send(sock, ptr, sendLeft, MSG_NOSIGNAL);
        if (rc == -1) {
            perror("socketWriteBytes: send");
            return false;
        }

        sendLeft -= rc;
        ptr += rc;
    }

    return true;
}

void getMacAddress(const char* device, unsigned char* mac) {
    int sock;
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Error creating socket");
        fprintf(stderr, "Error: creating socket in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        exit(EXIT_FAILURE);
    }

    /* I want to get an IPv4 IP address */
    ifr.ifr_addr.sa_family = AF_INET;

    /* I want IP address attached to "eth0" */
    strncpy(ifr.ifr_name, device, IFNAMSIZ - 1);

    if (mac != nullptr) {
        if (ioctl(sock, SIOCGIFHWADDR, &ifr) == -1) {
            perror("get_mac_address: Error calling ioctl. Couldn't get a MAC address.");
            mac[0] = mac[1] = mac[2] = mac[3] = mac[4] = mac[5] = 0xff;
            close(sock);
            return;
        }
        memcpy(mac, ifr.ifr_hwaddr.sa_data, 6);
    }

    close(sock);
}

void getIpAddress(const char* device, uint32_t& ip, char* ipAddrStr) {
    int sock;
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Error creating socket");
        fprintf(stderr, "Error: creating socket in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        exit(EXIT_FAILURE);
    }

    /* I want to get an IPv4 IP address */
    ifr.ifr_addr.sa_family = AF_INET;

    /* I want IP address attached to "eth0" */
    strncpy(ifr.ifr_name, device, IFNAMSIZ - 1);

    if (ioctl(sock, SIOCGIFADDR, &ifr) == -1) {
        // perror("get_ip_address: Error calling ioctl. Couldn't get an ip address. Is one assigned to the device?");
        // printf("Device was: %s\n", device);
        ip = std::numeric_limits<uint32_t>::max();
        if (ipAddrStr != nullptr) {
            snprintf(ipAddrStr, 16, "error");
        }
        close(sock);
        return;
    }

    ip = ((struct sockaddr_in*)&ifr.ifr_addr)->sin_addr.s_addr;

    if (ipAddrStr != nullptr) {
        inet_ntop(AF_INET, &(((struct sockaddr_in*)&ifr.ifr_addr)->sin_addr), ipAddrStr, 16);
    }

    close(sock);
}

void sendUnicast(std::string& dest, uint16_t port, uint8_t* data, size_t size) {
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service);
    auto remote = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(dest), port);
    try {
        socket.open(boost::asio::ip::udp::v4());
        socket.send_to(boost::asio::buffer(data, size), remote);
    } catch (const boost::system::system_error& e) {
        printf("sendUnicast: Failed to send -- %s\n", e.what());
    }
}
