#pragma once
// Case-sensitivity shim: kinematics.h includes "arduino.h" (lowercase), which
// resolves fine on Windows/macOS but not on case-sensitive Linux/WSL filesystems
// where the ESP32 core ships "Arduino.h". This local header (found first for a
// quoted include) forwards to the real one.
#include <Arduino.h>
