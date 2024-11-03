#!/bin/bash

# source ~/.bash_aliases
RASP_IP="192.168.1.2"
RASP_USER="pi"

RC522_DTBO="rc522-overlay.dtbo "
MODULE="rc522_driver.ko"
MODULE_NAME="rc522_driver"
SCPI_CMD="scppi"
# SSH_CMD="ssh pi@192.168.0.23"
SSH_CMD="ssh ${RASP_USER}@${RASP_IP}"

cpy2pi() {
    # scp "$1" pi@192.168.0.23:~ || error_exit "Failed to copy file to Raspberry Pi."
    scp "$1" "${RASP_USER}@${RASP_IP}:~" || error_exit "Failed to copy file to Raspberry Pi."
}
error_exit() {
    echo "Error: $1" >&2
    exit 1
}
case "$1" in
"help")
    echo "Usage: $0 [option]"
    echo ""
    echo "Options:"
    echo "  help         Display this help message."
    echo "  db-install   Copy and install the device tree overlay to the Raspberry Pi."
    echo "  build        Build the kernel module."
    echo "  insert       Insert the kernel module into the Raspberry Pi kernel."
    echo "  remove       Remove the kernel module from the Raspberry Pi kernel."
    echo "  (no option)  Default action: build, copy, and insert the module."
    exit 0
    ;;
"db-install")
    echo "Copying device tree overlay to Raspberry Pi..."
    cpy2pi "${RC522_DTBO}" || error_exit "Failed to copy device tree overlay."
    ${SSH_CMD} "sudo chmod +x ${RC522_DTBO}"
    ${SSH_CMD} "sudo chown root:root ${RC522_DTBO}"
    ${SSH_CMD} "sudo mv ${RC522_DTBO} /boot/firmware/overlays"
    exit 0
    ;;
"build")
    echo "Building the module..."
    make || error_exit "Failed to make the module file."
    exit 0
    ;;
"insert")
    echo "Installing the module..."
    $SSH_CMD "sudo rmmod $MODULE" >>/dev/null
    $SSH_CMD "sudo insmod $MODULE" || error_exit "Failed to insert module."
    $SSH_CMD "lsmod | grep $MODULE_NAME" >/dev/null
    if [ $? -eq 0 ]; then
        echo "Module $MODULE_NAME loaded successfully on Raspberry Pi."
    else
        error_exit "Module $MODULE_NAME failed to load."
    fi
    $SSH_CMD "sudo chmod 666 /dev/rc522" || error_exit "Failed to set permissions."
    exit 0
    ;;
"remove")
    echo "Removing the module..."
    $SSH_CMD "sudo rmmod $MODULE" || error_exit "Failed to remove module."
    exit 0
    ;;
*)
    make || error_exit "Failed to Make the module file"
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
    $SSH_CMD "lsmod | grep ${MODULE_NAME}" >/dev/null
    if [ $? -eq 0 ]; then
        echo "Module ${MODULE_NAME} loaded successfully on Raspberry Pi."
    else
        error_exit "Module ${MODULE_NAME} failed to load."
    fi

    $SSH_CMD "dmesg"

    $SSH_CMD "sudo  chmod 666 /dev/rc522 "
    exit 0
    ;;
esac
