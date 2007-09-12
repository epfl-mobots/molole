all:
	$(MAKE) -C can-bootloader builddir=pic30-24hj128gp506 cpu=24hj128gp506 flashend=0x15800 nodeid=1 prefix=pic30-elf-
	$(MAKE) -C timer builddir=pic30-33fj256gp710 cpu=33fj256gp710 prefix=pic30-elf-

clean:
	$(MAKE) -C can-bootloader builddir=pic30-24hj128gp506 clean
	$(MAKE) -C timer builddir=pic30-33fj256gp710 clean

distclean:
	$(MAKE) -C can-bootloader builddir=pic30-24hj128gp506 distclean
	$(MAKE) -C timer builddir=pic30-33fj256gp710 distclean
