# README

## RC522 RFID Reader Linux Driver

### Project Overview

This project provides a Linux kernel driver for the **RC522 RFID reader module**, along with a C++ application to interact with it. The driver enables communication with the RC522 module on a Raspberry Pi, allowing you to read RFID cards and retrieve their UIDs. A Bash script (`rc522_manager.sh`) is included to simplify building, deploying, and managing the kernel module on the Raspberry Pi.

---

### Features

- **Linux Kernel Driver**: Custom driver for the RC522 RFID reader.
- **Bash Script (`rc522_manager.sh`)**: Automates building, copying, inserting, and removing the kernel module.
- **Device Tree Overlay**: Includes `rc522-overlay.dtbo` for proper device tree configuration.

---


### Hardware Connections

Connect the RC522 module to the Raspberry Pi as follows:

| RC522 Pin | Raspberry Pi Pin |
|-----------|------------------|
| **VCC**   | 3.3V             |
| **GND**   | GND              |
| **MISO**  | GPIO 9 (MISO)    |
| **MOSI**  | GPIO 10 (MOSI)   |
| **SCK**   | GPIO 11 (SCLK)   |
| **NSS/SDA** | GPIO 8 (CE0)   |
| **RST**   | GPIO 24          |

---

### Script Usage

The `rc522_manager.sh` script simplifies managing the kernel module.

#### Available Options

- **`help`**: Display usage information.
- **`db-install`**: Copy and install the device tree overlay to the Raspberry Pi.
- **`build`**: Build the kernel module.
- **`insert`**: Insert the kernel module into the Raspberry Pi kernel.
- **`remove`**: Remove the kernel module from the Raspberry Pi kernel.
- **No Option**: Default action (build, copy, and insert the module).

### License

This project is licensed under the **MIT License**. See the `LICENSE` file for details.

---

**Enjoy using the RC522 RFID Reader Linux Driver and feel free to contribute or report issues!**


## ref

1. https://datasheets.raspberrypi.com/bcm2835/bcm2835-peripherals.pdf
2. https://datasheets.raspberrypi.com/bcm2711/bcm2711-peripherals.pdf
3. https://github.com/raspberrypi/documentation/blob/develop/documentation/asciidoc/computers/raspberry-pi/spi-bus-on-raspberry-pi.adoc#spi-software#

#debugging device tree

1. https://github.com/raspberrypi/documentation/blob/develop/documentation/asciidoc/computers/configuration/device-tree.adoc#debugging

## used Commands :

```bash
abdo:src$> fdtdump rc522-overlay.dtbo
abdo:src$> dtc -@ -Hepapr  -I dts -O dtb -o rc522-overlay.dtbo  rc522-overlay.dts
sudo dtoverlay -d . /boot/firmware/overlay/rc522-overlay.dtbo
```
