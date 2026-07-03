#!/usr/bin/env bash
# Install clang/clangd, arduino-cli, ESP32 core, and MiniHexa libraries on Debian/Raspberry Pi OS.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${ROOT}/arduino-cli.yaml"
ARDUINO_USER="${HOME}/Arduino"
CLI_VERSION="${ARDUINO_CLI_VERSION:-1.2.2}"
ESP32_CORE_VERSION="${ESP32_CORE_VERSION:-2.0.17}"
SENSORLIB_VERSION="${SENSORLIB_VERSION:-0.2.2}"
BIN_DIR="${HOME}/.local/bin"
CLI="${BIN_DIR}/arduino-cli"

echo "==> Installing system packages (clang, build tools)..."
sudo apt-get update
sudo apt-get install -y \
  clang \
  clangd \
  curl \
  git \
  python3 \
  unzip

mkdir -p "${BIN_DIR}" "${ARDUINO_USER}/libraries"

if [[ ! -x "${CLI}" ]]; then
  echo "==> Installing arduino-cli ${CLI_VERSION} (linux arm64)..."
  tmp="$(mktemp -d)"
  trap 'rm -rf "${tmp}"' EXIT
  curl -fsSL \
    "https://github.com/arduino/arduino-cli/releases/download/v${CLI_VERSION}/arduino-cli_${CLI_VERSION}_Linux_ARM64.tar.gz" \
    -o "${tmp}/arduino-cli.tar.gz"
  tar -xzf "${tmp}/arduino-cli.tar.gz" -C "${tmp}"
  install -m 0755 "${tmp}/arduino-cli" "${CLI}"
fi

export PATH="${BIN_DIR}:${PATH}"

echo "==> Initializing arduino-cli..."
"${CLI}" config init --overwrite --dest-dir "$(dirname "${CONFIG}")" 2>/dev/null || true
# Keep our tracked config (board URL, paths).
cp "${CONFIG}" "${HOME}/.arduino15/arduino-cli.yaml"

echo "==> Installing ESP32 board core ${ESP32_CORE_VERSION}..."
"${CLI}" core update-index
"${CLI}" core install "esp32:esp32@${ESP32_CORE_VERSION}"

echo "==> Installing SensorLib ${SENSORLIB_VERSION}..."
if ! "${CLI}" lib list SensorLib 2>/dev/null | grep -q "SensorLib"; then
  "${CLI}" lib install "SensorLib@${SENSORLIB_VERSION}"
fi

echo "==> Linking Hiwonder kinematics library as ~/Arduino/libraries/library ..."
kinematics_link="${ARDUINO_USER}/libraries/library"
if [[ -L "${kinematics_link}" ]]; then
  rm "${kinematics_link}"
elif [[ -d "${kinematics_link}" ]]; then
  echo "WARNING: ${kinematics_link} exists and is not a symlink; leaving it in place." >&2
else
  ln -s "${ROOT}/kinematics" "${kinematics_link}"
fi

if [[ ! -f "${ROOT}/kinematics/src/esp32/kinematics_lib.a" ]]; then
  echo "ERROR: missing precompiled kinematics_lib.a" >&2
  exit 1
fi

cat <<EOF

Setup complete.

  arduino-cli: ${CLI}
  config:      ${CONFIG}
  sketch:      ${ROOT}/host-control-and-action-editor/remote
  fqbn:        esp32:esp32:esp32:PartitionScheme=huge_app

Add to ~/.bashrc if needed:
  export PATH="${BIN_DIR}:\$PATH"

Next:
  ${ROOT}/scripts/build-remote.sh compile
  ${ROOT}/scripts/build-remote.sh upload /dev/ttyUSB0

EOF
