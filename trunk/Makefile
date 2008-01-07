all:
	$(MAKE) -C can-bootloader builddir=pic30-24hj128gp506 cpu=24hj128gp506 flashend=0x15800 nodeid=1 prefix=pic30-elf-
	$(MAKE) -C error builddir=pic30-generic prefix=pic30-elf-
	$(MAKE) -C clock builddir=pic30-33fj256gp710 cpu=33fj256gp710 prefix=pic30-elf-
	$(MAKE) -C timer builddir=pic30-33fj256gp710 cpu=33fj256gp710 prefix=pic30-elf-
	$(MAKE) -C adc builddir=pic30-33fj256gp710 cpu=33fj256gp710 prefix=pic30-elf-
	$(MAKE) -C i2c builddir=pic30-33fj256gp710 cpu=33fj256gp710 prefix=pic30-elf-
	$(MAKE) -C uart builddir=pic30-33fj256gp710 cpu=33fj256gp710 prefix=pic30-elf-
	$(MAKE) -C oc builddir=pic30-33fj256gp710 cpu=33fj256gp710 prefix=pic30-elf-
	$(MAKE) -C pwm builddir=pic30-33fj256mc510 cpu=33fj256mc510 prefix=pic30-elf-

clean:
	$(MAKE) -C can-bootloader builddir=pic30-24hj128gp506 clean
	$(MAKE) -C error builddir=pic30-generic clean
	$(MAKE) -C clock builddir=pic30-33fj256gp710 clean
	$(MAKE) -C timer builddir=pic30-33fj256gp710 clean
	$(MAKE) -C adc builddir=pic30-33fj256gp710 clean
	$(MAKE) -C i2c builddir=pic30-33fj256gp710 clean
	$(MAKE) -C uart builddir=pic30-33fj256gp710 clean
	$(MAKE) -C oc builddir=pic30-33fj256gp710 clean
	$(MAKE) -C pwm builddir=pic30-33fj256mc510 cpu=33fj256mc510 prefix=pic30-elf-