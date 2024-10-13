#pragma once

#include <cstdint>
#include <string>

#include <netinet/in.h>

/**
 * @brief openTcpRecvSocket opens a receiving TCP socket.
 * @param sock  After the call the created socket.
 * @param saddr After the call contains metainformation about the socket.
 * @param ip    The ip where we later listen on. Can be NULL -> Default interface is used.
 * @param port  The port where we listen.
 */
void openTcpRecvSocket(int& sock, struct sockaddr_in& saddr, const char* ip, uint16_t port, int backLogLength);

/**
 * @brief openTcpSendSocket opens a send TCP socket.
 * @param sock  After the call the created socket.
 * @param saddr After the call contains metainformation about the socket.
 * @param ip    The ip where we want connect to
 * @param port  The port we want connect to
 * @return      A bool that indicates that the connection was successfull
 */
bool openTcpSendSocket(int& sock, struct sockaddr_in& saddr, const char* ip, uint16_t port);

/**
 * @brief openRecvBroadcastSocket Open a broadcast port where we can receive broadcast messages.
 * @param saddr Contains later metainformation about the socket.
 * @param sock  Contains later the opend socket.
 * @param port  To which port we want to open a broadcast port.
 */
void openRecvBroadcastSocket(struct sockaddr_in& saddr, int& sock, uint16_t port);

/**
 * @brief socketReadBytes Read a number of bytes of a TCP socket.
 * @param socket The socket to read from.
 * @param len The number of bytes we want to read.
 * @param buffer The buffer we write the read bytes to.
 * @return True if everything was ok, false otherwise.
 */
bool socketReadBytes (int sock, uint32_t len, void* buffer);

/**
 * @brief socketWriteBytes Write a number of bytes of a TCP socket.
 * @param socket The socket to write to.
 * @param len The number of bytes we want to write.
 * @param buffer The buffer we read the bytes from we want to write.
 * @return True if everything was ok, false otherwise.
 */
bool socketWriteBytes(int sock, uint32_t sendLeft, const void *buffer);


/**
 * @brief Get the MAC address of an interface.
 *
 * macAddress if it is not NULL.
 */
void getMacAddress(const char* device, unsigned char* mac);

/**
  @brief Get the IP address of an interface.
 *
 * ipAddrStr will only be filled if it is not NULL.
 * ipAddrStr must be at least 16 char long when it is not NULL.
 */
void getIpAddress(const char* device, uint32_t& ip, char * ipAddrStr);

/**
 * @brief sendBroadcast Sends a broadcast out of every capable interface.
 * @param port The port we want to send the broadcast to.
 * @param data The data we want to send in the broadcast
 * @param size The size of the data we want to send.
 */
void sendBroadcast(uint16_t port, uint8_t* data, size_t size);

/**
 * @brief sendBroadcastByIface Sends a broadcast only via a specify interface
 * @param iface The interface the broadcast should be send e.g. "eth0" or "wlan0".
 * @param port The port to which we send the broadcast.
 * @param data The data we want to send via broadcast.
 * @param size The number of bytes in data.
 */
void sendBroadcastByIface(const char* iface, uint16_t port, uint8_t* data, size_t size);

/**
 * @brief sendUnicast Sends a unicast udp packet to a destination address.
 * @param dest The destination we want to send the unicast to.
 * @param port The port we want to send the unicast to.
 * @param data The data we want to send in the unicast
 * @param size The size of the data we want to send.
 */
void sendUnicast(std::string& dest, uint16_t port, uint8_t* data, size_t size);
