#ifndef CORTEX_CONFIG_H_
#define CORTEX_CONFIG_H_

#include "Arduino.h"

class CortexConfig {
public:
  void begin();
  String wifi_ssid() const;
  String wifi_password() const;
  String cortex_password() const;
  bool auth_required() const;
  bool check_cortex_password(const char *password) const;
  static bool decode_wire_secret(const String &raw, String &out);
  bool set_wifi_credentials(const char *ssid, const char *password);
  bool set_cortex_password(const char *password);
  bool handle_provision_frame(const String &frame);

private:
  String _wifi_ssid;
  String _wifi_password;
  String _cortex_password;
  void load();
  void save_wifi();
  void save_cortex();
};

extern CortexConfig cortex_config;

#endif
