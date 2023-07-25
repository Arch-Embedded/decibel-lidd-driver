.PHONY:all clean modules_install
obj-m := lidd_fb.o

ARCH=arm
CROSS_COMPILE=arm-linux-gnueabihf-
KERNELDIR := ../linux
PWD := $(shell pwd)
WARN :=-Wall -Wstrict-prototypes -Wno-trigraphs -Wmissing-prototypes
ccflags-y += "-Wno-error=date-time"
all: modules

modules modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) $(WARN) $@

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
