obj-m+=llid_fb.o
CFLAGS_llid_fb.o := -DDEBUG
KDIR ?= /usr/embedded_build/linux
CROSS_COMPILE = /usr/embedded_build/buildroot-2016.02/output/host/opt/ext-toolchain/bin/arm-linux-gnueabihf-
all:
	make -C $(KDIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) SUBDIRS=$(PWD) modules 
clean:
	make -C $(KDIR) M=$(PWD) clean
	
