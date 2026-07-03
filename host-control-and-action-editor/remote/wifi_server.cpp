#include "wifi_server.h"
#include "cortex_config.h"
#include "command_parser.h"

WiFiServerManager::WiFiServerManager() {
  wifi_connection_state = false;
  tcp_connection_state = false;
  tcp_state = DISCONNECTED;
  udp_pairing_state = false;
  tcp_authenticated = false;
  pose_stream_pending = false;
  rgb_stream_pending = false;
  memset(incomingPacket, 0, sizeof(incomingPacket));
}

bool WiFiServerManager::is_cortex_active() const {
  return tcp_connection_state && tcp_authenticated;
}

bool WiFiServerManager::handle_auth_frame(const String &frame) {
  if (!frame.startsWith("P|")) {
    return false;
  }
  String password_raw = frame.substring(2);
  String password;
  if (!CortexConfig::decode_wire_secret(password_raw, password)) {
    tcpClient.printf("P|0&");
    tcpClient.stop();
    tcp_connection_state = false;
    tcp_state = DISCONNECTED;
    return false;
  }
  bool ok = cortex_config.check_cortex_password(password.c_str());
  tcpClient.printf("P|%d&", ok ? 1 : 0);
  if (ok) {
    tcp_authenticated = true;
    return true;
  }
  tcpClient.stop();
  tcp_connection_state = false;
  tcp_state = DISCONNECTED;
  return false;
}

void WiFiServerManager::apply_parsed_command(const RecData_t &parsed) {
  if (parsed.mode == MINIHEXA_POSE_CONTROL) {
    pose_stream = parsed;
    pose_stream_pending = true;
  }
  else if (parsed.mode == MINIHEXA_RGB_ADJUST) {
    rgb_stream = parsed;
    rgb_stream_pending = true;
  }
  else {
    rec = parsed;
  }
}

void WiFiServerManager::station_begin() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  // BLE is enabled on this board; WiFi modem sleep must stay on.
  WiFi.setSleep(true);
}

void WiFiServerManager::station_reconnect(const String &ssid, const String &password) {
  wifi_connection_state = false;
  udp_pairing_state = false;
  parameters_reset();
  udpClient.stop();
  station_reconnect_ms = millis();

  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(250);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(true);
  WiFi.begin(ssid.c_str(), password.c_str());
  ESP_LOGI("WIFI", "Reconnecting (ssid_len=%u)\n", (unsigned)ssid.length());
}

String WiFiServerManager::station_status_frame() const {
  char buf[80];
  wl_status_t status = WiFi.status();

  if(status == WL_CONNECTED) {
    snprintf(buf, sizeof(buf), "W|3|2|%s&", WiFi.localIP().toString().c_str());
  }
  else if((status == WL_NO_SSID_AVAIL || status == WL_CONNECT_FAILED) &&
          (station_reconnect_ms == 0 ||
           millis() - station_reconnect_ms >= STATION_JOIN_GRACE_MS)) {
    snprintf(buf, sizeof(buf), "W|3|3|%d&", (int)status);
  }
  else if(status == WL_NO_SSID_AVAIL || status == WL_CONNECT_FAILED) {
    snprintf(buf, sizeof(buf), "W|3|1|%d&", (int)status);
  }
  else if(status == WL_DISCONNECTED || status == WL_IDLE_STATUS) {
    snprintf(buf, sizeof(buf), "W|3|0|&");
  }
  else {
    snprintf(buf, sizeof(buf), "W|3|1|%d&", (int)status);
  }
  return String(buf);
}

bool WiFiServerManager::begin() {
  if(WiFi.status() != WL_CONNECTED) {
    if(wifi_connection_state) {
      wifi_connection_state = false;
      udp_pairing_state = false;
      udpClient.stop();
    }
    return false;
  }

  if(wifi_connection_state == false) {
    ESP_LOGI("WIFI", "Connected to WIFI!\n");
    String macAddress = WiFi.macAddress();
    macAddress.replace(":", "");
    selfIP = WiFi.localIP();
    clientAddress = "MINIHEXA:" + macAddress + "|" + selfIP.toString();
    cClientAddress = clientAddress.c_str();
    udpClient.begin(UDP_LISTENER_PORT);
    tcpServer.begin(TCP_LISTENER_PORT);
    tcpClient.setTimeout(0);
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    wifi_connection_state = true;
    ESP_LOGI("WIFI", "Local IPAddress: %s\n", selfIP.toString().c_str());
  }
  return wifi_connection_state;
}

bool WiFiServerManager::udp_server() {
  int packetSize = 0;
  static uint32_t tick_start = 0;

  if (millis() - tick_start > 1000) {
    packetSize = udpClient.parsePacket();
    tick_start = millis();
  }

  if (packetSize) {
    int len = udpClient.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    if (strcmp(incomingPacket, "LOBOT_NET_DISCOVER") == 0 ||
        strcmp(incomingPacket, "MINIHEXA_NET_DISCOVER") == 0) {
      clientIP = udpClient.remoteIP();
      ESP_LOGI("WIFI", "Client IPAddress: %d.%d.%d.%d\n", clientIP[0], clientIP[1], clientIP[2], clientIP[3]);
      udpClient.beginPacket(clientIP, UDP_SEND_PORT);
      udpClient.print(clientAddress);
      udpClient.endPacket();
      udp_pairing_state = true;
    }
  }
  return udp_pairing_state;
}

bool WiFiServerManager::tcp_server() {
  if (!tcpClient && (tcp_connection_state == false)) {
    tcpClient = tcpServer.available();

    if (tcpClient) {
      tcp_connection_state = true;
      tcp_authenticated = !cortex_config.auth_required();
      ESP_LOGI("WIFI", "TCP Client connected!\n");
    }
    else {
      tcp_state = DISCONNECTED;
      return false;
    }
  }

  if (tcp_connection_state == true) {
    if (!tcpClient.connected()) {
      ESP_LOGI("WIFI", "Client disconnected!\n");
      parameters_reset();
      return false;
    }

    if (tcpClient.available()) {
      size_t bytesRead = tcpClient.readBytesUntil('&', rec_buffer, sizeof(rec_buffer) - 1);
      rec_buffer[bytesRead] = '\0';
      String fullData = String(rec_buffer);
      ESP_LOGI("Received from client", "%s", fullData.c_str());

      if (!tcp_authenticated) {
        handle_auth_frame(fullData);
        tcp_state = CONNECTED;
        return false;
      }

      RecData_t parsed = parse_command_frame(fullData);
      apply_parsed_command(parsed);
      tcp_state = RECEIVEDATA;
      return true;
    }

    tcp_state = CONNECTED;
    return is_cortex_active();
  }

  return false;
}

void WiFiServerManager::parameters_reset() {
  tcp_connection_state = false;
  udp_pairing_state = false;
  tcp_authenticated = false;
  tcp_state = DISCONNECTED;
  tcpClient.stop();
  memset(incomingPacket, 0, sizeof(incomingPacket));
}
