#!/bin/bash

source ~/.bash_aliases

MODULE="hello.ko"
MODULE_NAME="hello"
SCPI_CMD="scppi"
SSH_CMD="ssh sshpi"

error_exit() {
    echo "Error: $1" >&2
    exit 1
}

# Check if the module file exists
if [ ! -f "$MODULE" ]; then
    error_exit "The module '$MODULE' does not exist. Please build it first."
fi

echo " copying the module '$MODULE' to Raspberry Pi..."
$SCPI_CMD "$MODULE" ||  error_exit "Failed to copy module to Raspberry Pi."


echo "Clearing kernel messages on Raspberry Pi..."
$SSH_CMD "sudo dmesg -c" >> /dev/null || error_exit "Failed to clear kernel messages."

echo "Removing module $MODULE on Raspberry Pi..."
$SSH_CMD "sudo rmmod $MODULE" >> /dev/null 


echo "Inserting module $MODULE on Raspberry Pi..."
$SSH_CMD "sudo insmod $MODULE" || error_exit "Failed to insert module."

echo "Checking if the module is loaded..."
$SSH_CMD "lsmod | grep $MODULE_NAME" > /dev/null

$SSH_CMD "lsmod | grep $MODULE_NAME" > /dev/null

if [ $? -eq 0 ]; then
    echo "Module $MODULE_NAME loaded successfully on Raspberry Pi."
else
    error_exit "Module $MODULE_NAME failed to load."
fi

