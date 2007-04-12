ifeq (,$(filter build-%,$(notdir $(CURDIR))))
include target.mk
else
#----- End Boilerplate

VPATH = $(SRCDIR)

sources = can-bootloader.s
objects = $(patsubst %.s,%.o,$(sources))
target-elf = can-bootloader.elf
target-hex = $(patsubst %.elf,%.hex,$(target-elf))

PIC30SUPPORTROOT = /usr/share/pic30-support
AS = $(prefix)as
LD = $(prefix)ld
BIN2HEX = $(prefix)bin2hex
ASFLAGS = -p $(cpu) -I$(PIC30SUPPORTROOT)/inc --defsym cpuinc=\"p$(cpu).inc\"
LDFLAGS = -p $(cpu) --stack 16 --no-isr --no-data-init --warn-section-align --script $(PIC30SUPPORTROOT)/gld/p$(cpu).gld

$(target-hex): $(target-elf)
	$(BIN2HEX) $<

$(target-elf): $(objects)
	$(LD) $(objects) $(LDFLAGS) -o $@

#----- Begin Boilerplate
endif