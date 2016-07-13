General Information
===================

Linux driver for Realtek PCI-Express card reader chip.

Linux already has rtsx_pci driver which theoretically should work with
rtl8411 chip. But in fact it can only detect the chip and nothing more.

rts_bpp works fine with rtl8411 card reader and probably some others.


Build Steps
===========

1) make
2) make install
3) depmod
4) reboot your computer

Note: Root privilege is required in step 2 and 3
