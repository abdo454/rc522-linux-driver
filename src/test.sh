#!/bin/bash

# source ~/.bash_aliases

RC522_DTBO="rc522-overlay.dtbo "
MODULE="rc522.ko"
MODULE_NAME="rc522"
SCPI_CMD="scppi"
# SSH_CMD="ssh pi@192.168.0.23"
SSH_CMD="ssh pi@192.168.1.2"

cpy2pi() {
    # scp "$1" pi@192.168.0.23:~ || error_exit "Failed to copy file to Raspberry Pi."
    scp "$1" pi@192.168.1.2:~ || error_exit "Failed to copy file to Raspberry Pi."
}
error_exit() {
    echo "Error: $1" >&2
    exit 1
}

# if [ "$1" == "dtbo" ]; then
# cpy2pi ${RC522_DTBO} || error_exit "Failed to copy file to Raspberry Pi."
# ${SSH_CMD} "sudo chmod +x ${RC522_DTBO}"
# ${SSH_CMD} "sudo chown root:root ${RC522_DTBO}"
# ${SSH_CMD} "sudo mv ${RC522_DTBO} /boot/firmware/overlays"
# exit 1
# if
# Check if the module file exists

make
if [ ! -f "$MODULE" ]; then
    error_exit "The module '$MODULE' does not exist. Please build it first."
fi

echo " copying the module '$MODULE' to Raspberry Pi..."
cpy2pi "$MODULE"

echo "Clearing kernel messages on Raspberry Pi..."
$SSH_CMD "sudo dmesg -c" >>/dev/null || error_exit "Failed to clear kernel messages."

echo "Removing module $MODULE on Raspberry Pi..."
$SSH_CMD "sudo rmmod $MODULE" >>/dev/null

echo "Inserting module $MODULE on Raspberry Pi..."
$SSH_CMD "sudo insmod $MODULE" || error_exit "Failed to insert module."

echo "Checking if the module is loaded..."
$SSH_CMD "lsmod | grep $MODULE_NAME" >/dev/null

$SSH_CMD "lsmod | grep $MODULE_NAME" >/dev/null

if [ $? -eq 0 ]; then
    echo "Module $MODULE_NAME loaded successfully on Raspberry Pi."
else
    error_exit "Module $MODULE_NAME failed to load."
fi

$SSH_CMD "dmesg"
