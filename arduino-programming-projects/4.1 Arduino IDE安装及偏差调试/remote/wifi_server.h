#ifndef WIFI_SERVER_H_
#define WIFI_SERVER_H_

#include <esp_wifi.h>
#include "WiFi.h"
#include "global.h"

#define WIFI_SSID             "hiwonder"
#define WIFI_PASSWORD         "hiwonder"

#define UDP_LISTENER_PORT     9027
#define UDP_SEND_PORT         9025
#define TCP_LISTENER_PORT     9023

#ifdef __cplusplus
extern "C" 
{
#endif

#ifdef __cplusplus
} // extern "C"
#endif

class WiFiServerManager
{
public:
  enum TCPState {DISCONNECTED, CONNECTED, RECEIVEDATA};

  RecData_t rec;
  TCPState tcp_state;
  
  WiFiClient tcpClient;     // TCP configuration

  WiFiServerManager();
  bool begin();
  bool tcp_server();
  bool udp_server();
  void parameters_reset();

private:
  IPAddress clientIP;       // used to store the TCP client IP address extracted from the broadcast
  IPAddress selfIP;      // used to store this device's own IP address
  String clientAddress;
  const char* cClientAddress;  

  WiFiUDP udpClient;
  uint32_t tick_start = 0;
  char incomingPacket[255];  // buffer the received broadcast data   
  bool udp_pairing_state; 

  
  WiFiServer tcpServer;  // create a TCP server listening on port 9023
  bool tcp_connection_state;     // connection flag state

  bool wifi_connection_state;

  char rec_buffer[256]; // assume the maximum data length per read is 1024 bytes

  static RecData_t read_data(String data);
};

#endif