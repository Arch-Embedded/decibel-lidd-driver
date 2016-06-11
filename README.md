# lcd_lidd_ssd1289
Developing linux driver and device tree for ssd1289 display using
Beaglebone Black and AM335x LIDD interface

This module is based on Christopher Mitchell SSD1289 Framebuffer driver:
https://www.cemetech.net/forum/viewtopic.php?t=7814

This module is modified to using newer kernel with devicetree overlays.

Installation
Modify KDIR and CROSS_COMPILE variables in Makefile to your kernel sources dir
and cross-compiler

After make lidd_fb.ko file is ready to copy to BeagleBone Black

Modify uEnv.txt file on BeagleBone Black:
Choose options:
##BeagleBone Black: HDMI (Audio/Video) disabled:
dtb=am335x-boneblack-emmc-overlay.dtb

and disable cape universal:
cmdline=coherent_pool=1M quiet init=/lib/systemd/systemd cape_universal=disable

Reboot your BeagleBone

Operations from BeagleBone Black:
Next build dts overlay file.
Download bb.overlays to your Beaglebone Black
git clone https://github.com/beagleboard/bb.org-overlays.git

and copy BB-LIDD_LCD-01-00A0.dts to src directory

Run install.sh script from bb.org-overlay directory

In /lib/firmware BB-LIDD_LCD-01-00A0.dtbo should be build correctly

cd /lib/firmware
echo BB-LIDD_LCD-01 > /sys/devices/platform/bone_capemgr/slots

Install previously builded lidd_fb.ko:
insmod lidd_fb.ko

Test your SSD1289:
cat /dev/urandom > /dev/fb0
ssd1289 display should be filled random pixels
cat /dev/zero > /dev/fb0
ssd1289 display should be blanked

Refer to:
https://github.com/notro/fbtft/wiki/Framebuffer-use

for examples of usage

