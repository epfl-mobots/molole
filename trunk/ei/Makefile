ifeq (,$(filter build-%,$(notdir $(CURDIR))))
include target.mk
else
#----- End Boilerplate

VPATH = $(SRCDIR)

sources = ei.c
objects = $(patsubst %.c,%.o,$(sources))
target = ei.a

CFLAGS +=-g -Wall -mcpu=$(cpu)
CC = $(prefix)gcc

$(target): $(objects)
	$(prefix)ar rsc $@ $(objects)

%.d: %.c
	set -e; $(CC) -MM $(CFLAGS) $< \
		| sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@; \
		[ -s $@ ] || rm -f $@

include $(sources:.c=.d)

#----- Begin Boilerplate
endif
