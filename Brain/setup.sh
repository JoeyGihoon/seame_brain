#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
# Jetson은 전체 upgrade가 가끔 NVIDIA 패키지 꼬이게 할 때가 있어 필요할 때만 권장
sudo apt-get upgrade -y

sudo apt-get install -y \
  python3-pip python3-venv python3-dev build-essential pkg-config \
  libgl1 libglib2.0-0 libssl-dev libffi-dev \
  xdg-utils curl ca-certificates


# Node.js
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt-get install -y nodejs
sudo npm i -g npm@10.8.2 @angular/cli@20.1.4

# Python deps (venv 권장)
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt

# Frontend
pushd src/dashboard/frontend >/dev/null
if [ -f package-lock.json ]; then npm ci; else npm install; fi
popd >/dev/null

echo "Install complete."
