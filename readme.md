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
