
PREFIX = /usr/local
BINDIR = $(DESTDIR)$(PREFIX)/bin

CC = gcc
CFLAGS = -O3 -Wall -I../modules/mqnic

BIN = mqnic-config mqnic-dump mqnic-fw mqnic-bmc perout

GENDEPFLAGS = -MD -MP -MF .$(@F).d

ALL_CFLAGS = $(CFLAGS) $(GENDEPFLAGS)

all: $(BIN)

%.o: %.c
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

mqnic-config: mqnic-config.o mqnic.o timespec.o
	$(CC) $(ALL_CFLAGS) $^ -o $@

mqnic-dump: mqnic-dump.o mqnic.o
	$(CC) $(ALL_CFLAGS) $^ -o $@

mqnic-fw: mqnic-fw.o mqnic.o flash.o flash_spi.o flash_bpi.o fpga_id.o bitfile.o
	$(CC) $(ALL_CFLAGS) $^ -o $@

mqnic-bmc: mqnic-bmc.o mqnic.o
	$(CC) $(ALL_CFLAGS) $^ -o $@

perout: perout.o timespec.o
	$(CC) $(ALL_CFLAGS) $^ -o $@

install:
	install -d $(BINDIR)
	install -m 0755 $(BIN) $(BINDIR)

clean:
	rm -f $(BIN)
	rm -f *.o
	rm -f .*.d

-include $(wildcard .*.d)

.PHONY: all install clean
