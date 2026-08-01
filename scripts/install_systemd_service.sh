#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this installer as root: sudo $0" >&2
  exit 1
fi

service_user="${SUDO_USER:-}"
if [[ -z "${service_user}" || "${service_user}" == "root" ]]; then
  echo "Run with sudo from the account that should own the boat runtime." >&2
  exit 1
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
template="${repo_root}/config/systemd/mtr-boat.service.in"
service_group="$(id -gn "${service_user}")"
service_path="/etc/systemd/system/mtr-boat.service"
rendered="$(mktemp)"
trap 'rm -f "${rendered}"' EXIT

for hardware_group in dialout video i2c; do
  if ! getent group "${hardware_group}" >/dev/null; then
    echo "Required hardware group does not exist: ${hardware_group}" >&2
    exit 1
  fi
  usermod -aG "${hardware_group}" "${service_user}"
done

sed \
  -e "s|@MTR_USER@|${service_user}|g" \
  -e "s|@MTR_GROUP@|${service_group}|g" \
  -e "s|@REPO_ROOT@|${repo_root}|g" \
  "${template}" >"${rendered}"

install -m 0644 "${rendered}" "${service_path}"
systemctl daemon-reload
systemctl enable mtr-boat.service

echo "Installed and enabled mtr-boat.service."
echo "It will start automatically on the next boot in off/neutral mode."
echo
echo "Start now:  sudo systemctl start mtr-boat"
echo "View logs:  journalctl -u mtr-boat -f"
echo "Stop:       sudo systemctl stop mtr-boat"
