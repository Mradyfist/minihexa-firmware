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
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

class WiFiServerManager {
public:
  enum TCPState {DISCONNECTED, CONNECTED, RECEIVEDATA};

  RecData_t rec;
  RecData_t pose_stream;
  RecData_t rgb_stream;
  bool pose_stream_pending = false;
  bool rgb_stream_pending = false;
  TCPState tcp_state;
  bool tcp_authenticated = false;

  WiFiClient tcpClient;

  WiFiServerManager();
  static void station_begin();
  void station_reconnect(const String &ssid, const String &password);
  String station_status_frame() const;
  bool begin();
  bool tcp_server();
  bool udp_server();
  bool is_cortex_active() const;
  void parameters_reset();

private:
  IPAddress clientIP;
  IPAddress selfIP;
  String clientAddress;
  const char *cClientAddress;

  WiFiUDP udpClient;
  uint32_t tick_start = 0;
  char incomingPacket[255];
  bool udp_pairing_state;

  WiFiServer tcpServer;
  bool tcp_connection_state;
  bool wifi_connection_state;
  uint32_t station_reconnect_ms = 0;

  static constexpr uint32_t STATION_JOIN_GRACE_MS = 20000;

  char rec_buffer[256];

  bool handle_auth_frame(const String &frame);
  void apply_parsed_command(const RecData_t &parsed);
};

#endif
