#include "uart_server.h"
#include "cortex_config.h"
#include "command_parser.h"
#include "wifi_server.h"

extern WiFiServerManager wifi_server;

void UartServerManager::begin() {
  pose_stream_pending = false;
  rgb_stream_pending = false;
  memset(&rec, 0, sizeof(rec));
  memset(&pose_stream, 0, sizeof(pose_stream));
  memset(&rgb_stream, 0, sizeof(rgb_stream));
  Serial.setRxBufferSize(2048);
  Serial.begin(115200);
}

void UartServerManager::on_receive(void (*callback)(void)) {
  Serial.onReceive(callback, true);
}

void UartServerManager::receive_message() {
  while (Serial.available()) {
    String fullData = Serial.readStringUntil('&');

    if (fullData.startsWith("W|")) {
      if (fullData == "W|3") {
        Serial.print(wifi_server.station_status_frame().c_str());
        continue;
      }
      bool ok = cortex_config.handle_provision_frame(fullData);
      if (fullData.indexOf("|1|") == 1) {
        Serial.printf("W|1|%d&", ok ? 1 : 0);
        if (ok) {
          wifi_server.station_reconnect(
            cortex_config.wifi_ssid(),
            cortex_config.wifi_password());
        }
      }
      else if (fullData.indexOf("|2|") == 1) {
        Serial.printf("W|2|%d&", ok ? 1 : 0);
      }
      continue;
    }

    RecData_t parsed = parse_command_frame(fullData);
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
}
