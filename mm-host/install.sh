#!/bin/bash

# Adapted root check from https://stackoverflow.com/a/28776100
if [ $(id -u) -ne 0 ]
  then echo Please run this script as root or using sudo!
fi

# cd to script dir
cd "${0%/*}" || exit

echo "Installing system dependencies"
apt update
apt install -y python3-picamera2

echo "Cleaning up any old virtualenvs"
rm -rf .venv/

echo "Creating new virtualenv"
python3 -mvenv --system-site-packages .venv
source .venv/bin/activate

echo "Installing python dependencies"
pip install -r requirements.txt

echo "Creating systemd unit"
cat > /etc/systemd/system/mm-host.service << EOF
[Unit]
Description=MatrixMirror Host Service

[Service]
ExecStart=$(pwd)/mm-host
User=$(stat -c '%U' install.sh)
RestartSec=2
Restart=on-failure
TimeoutSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "Enabling service"
systemctl daemon-reload
systemctl enable mm-host
systemctl start mm-host

echo "MatrixMirror Host service installed and started"
echo "Check the current status using 'sudo systemctl status mm-host'"
echo "Manually restart (e.g. after updating) using 'sudo systemctl restart mm-host'"
echo "Disable using 'sudo systemctl disable mm-host'"
