# Module name
MODULE_NAME := rc522


obj-m += $(MODULE_NAME).o


CROSS_COMPILE := aarch64-linux-gnu-

# Kernel directory for the target architecture
KDIR := /home/abdo/tutorials/DevideDriver/rpi/lib/modules/6.6.56-v8_BY_ABDO_DAOOD+/build/


PWD := $(shell pwd)


all: module dt
	
module:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=arm64 CROSS_COMPILE=$(CROSS_COMPILE) modules
dt: rc522-overlay.dts
	dtc -@ -I dts -O dtb -o rc522-overlay.dtbo rc522-overlay.dts
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=arm64 CROSS_COMPILE=$(CROSS_COMPILE) clean
	rm -rf rc522-overlay.dtbo
