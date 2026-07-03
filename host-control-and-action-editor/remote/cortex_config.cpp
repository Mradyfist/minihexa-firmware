#include "cortex_config.h"
#include <Preferences.h>
#include "mbedtls/base64.h"

CortexConfig cortex_config;

static Preferences prefs;

static bool decode_provision_field(const String &raw, String &out) {
  String encoded = raw;
  if(raw.startsWith("b64:")) {
    encoded = raw.substring(4);
  }
  else {
    out = raw;
    return true;
  }

  String padded = encoded;
  while(padded.length() % 4 != 0) {
    padded += "=";
  }

  unsigned char buf[96];
  size_t olen = 0;
  int rc = mbedtls_base64_decode(
    buf, sizeof(buf) - 1, &olen,
    (const unsigned char *)padded.c_str(), padded.length());
  if(rc != 0) {
    return false;
  }
  buf[olen] = '\0';
  out = String((char *)buf);
  return true;
}

void CortexConfig::begin() {
  load();
}

void CortexConfig::load() {
  prefs.begin("cortex", true);
  _wifi_ssid = prefs.getString("wifi_ssid", "");
  _wifi_password = prefs.getString("wifi_pass", "");
  _cortex_password = prefs.getString("cortex_pass", "");
  prefs.end();
}

String CortexConfig::wifi_ssid() const {
  return _wifi_ssid;
}

String CortexConfig::wifi_password() const {
  return _wifi_password;
}

String CortexConfig::cortex_password() const {
  return _cortex_password;
}

bool CortexConfig::auth_required() const {
  return _cortex_password.length() > 0;
}

bool CortexConfig::decode_wire_secret(const String &raw, String &out) {
  return decode_provision_field(raw, out);
}

bool CortexConfig::check_cortex_password(const char *password) const {
  if (!auth_required()) {
    return true;
  }
  return _cortex_password.equals(password);
}

bool CortexConfig::set_wifi_credentials(const char *ssid, const char *password) {
  if (ssid == nullptr || strlen(ssid) == 0 || strlen(ssid) > 32) {
    return false;
  }
  if (password == nullptr || strlen(password) > 64) {
    return false;
  }
  _wifi_ssid = ssid;
  _wifi_password = password;
  save_wifi();
  return true;
}

bool CortexConfig::set_cortex_password(const char *password) {
  if (password == nullptr || strlen(password) > 32) {
    return false;
  }
  _cortex_password = password;
  save_cortex();
  return true;
}

void CortexConfig::save_wifi() {
  prefs.begin("cortex", false);
  prefs.putString("wifi_ssid", _wifi_ssid);
  prefs.putString("wifi_pass", _wifi_password);
  prefs.end();
}

void CortexConfig::save_cortex() {
  prefs.begin("cortex", false);
  prefs.putString("cortex_pass", _cortex_password);
  prefs.end();
}

bool CortexConfig::handle_provision_frame(const String &frame) {
  int first = frame.indexOf('|');
  if (first < 0) {
    return false;
  }
  int second = frame.indexOf('|', first + 1);
  if (second < 0) {
    return false;
  }

  String op = frame.substring(first + 1, second);
  if (op == "1") {
    int third = frame.indexOf('|', second + 1);
    if (third < 0) {
      return false;
    }
    String ssid_raw = frame.substring(second + 1, third);
    String pass_raw = frame.substring(third + 1);
    String ssid;
    String password;
    if (!decode_provision_field(ssid_raw, ssid) ||
        !decode_provision_field(pass_raw, password)) {
      return false;
    }
    return set_wifi_credentials(ssid.c_str(), password.c_str());
  }
  if (op == "2") {
    String pass_raw = frame.substring(second + 1);
    String password;
    if (!decode_provision_field(pass_raw, password)) {
      return false;
    }
    return set_cortex_password(password.c_str());
  }
  return false;
}
