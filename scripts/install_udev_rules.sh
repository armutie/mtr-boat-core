#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this installer as root: sudo $0" >&2
  exit 1
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

install -m 0644 "${repo_root}"/config/udev/*.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
udevadm trigger --subsystem-match=video4linux
udevadm settle

echo "Installed MTR hardware aliases:"
for alias in mtr_camera mtr_esp32 mtr_gnss; do
  if [[ -e "/dev/${alias}" ]]; then
    echo "  /dev/${alias} -> $(readlink -f "/dev/${alias}")"
  else
    echo "  /dev/${alias} (device not currently connected)"
  fi
done
