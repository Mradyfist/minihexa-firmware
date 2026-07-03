#!/usr/bin/env bash
# Compile and/or upload the patched MiniHexa remote firmware.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${ROOT}/arduino-cli.yaml"
SKETCH="${ROOT}/host-control-and-action-editor/remote"
FQBN="esp32:esp32:esp32:PartitionScheme=huge_app"
CLI="${HOME}/.local/bin/arduino-cli"
PORT="${PORT:-/dev/ttyUSB0}"

if [[ ! -x "${CLI}" ]]; then
  echo "arduino-cli not found. Run: ${ROOT}/scripts/setup-arduino.sh" >&2
  exit 1
fi

export PATH="${HOME}/.local/bin:${PATH}"

cmd="${1:-compile}"
shift || true

case "${cmd}" in
  compile)
  echo "Compiling ${SKETCH} ..."
  "${CLI}" compile --config-file "${CONFIG}" --fqbn "${FQBN}" "${SKETCH}"
  echo "Build OK."
  ;;
  upload)
  port="${1:-${PORT}}"
  echo "Compiling ${SKETCH} ..."
  "${CLI}" compile --config-file "${CONFIG}" --fqbn "${FQBN}" "${SKETCH}"
  echo "Uploading to ${port} ..."
  "${CLI}" upload --config-file "${CONFIG}" -p "${port}" --fqbn "${FQBN}" "${SKETCH}"
  echo "Upload OK."
  ;;
  monitor)
  port="${1:-${PORT}}"
  "${CLI}" monitor --config-file "${CONFIG}" -p "${port}" -c baudrate=115200
  ;;
  *)
  echo "Usage: $0 {compile|upload [port]|monitor [port]}" >&2
  exit 1
  ;;
esac
