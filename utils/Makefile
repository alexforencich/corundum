# SPDX-License-Identifier: BSD-2-Clause-Views
# Copyright (c) 2019-2023 The Regents of the University of California

PREFIX ?= /usr/local
BINDIR = $(DESTDIR)$(PREFIX)/bin

CC ?= gcc
CFLAGS ?= -O3

CFLAGS += -Wall
CPPFLAGS += -Ilib -Iinclude
LDFLAGS += -Llib/mqnic
LDLIBS += -lmqnic

LIBMQNIC = lib/mqnic/libmqnic.a

BIN = mqnic-config
BIN += mqnic-dump
BIN += mqnic-fw
BIN += mqnic-bmc
BIN += mqnic-bert
BIN += mqnic-xcvr
BIN += perout

GENDEPFLAGS = -MD -MP -MF .$(@F).d

ALL_CFLAGS = $(CFLAGS) $(CPPFLAGS) $(GENDEPFLAGS)

all: $(BIN)

FORCE:

$(LIBMQNIC): FORCE
	$(MAKE) -C $(dir $@) $(notdir $@)

%.o: %.c
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

mqnic-config: mqnic-config.o timespec.o $(LIBMQNIC)
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

mqnic-dump: mqnic-dump.o $(LIBMQNIC)
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

mqnic-fw: mqnic-fw.o flash.o flash_spi.o flash_bpi.o bitfile.o $(LIBMQNIC)
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

mqnic-bmc: mqnic-bmc.o $(LIBMQNIC)
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

mqnic-bert: mqnic-bert.o timespec.o $(LIBMQNIC)
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

mqnic-xcvr: mqnic-xcvr.o $(LIBMQNIC) drp.o xcvr_gt.o xcvr_gthe3.o xcvr_gtye3.o xcvr_gthe4.o xcvr_gtye4.o
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

perout: perout.o timespec.o
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) $^ -o $@

install:
	install -d $(BINDIR)
	install -m 0755 $(BIN) $(BINDIR)

clean:
	rm -f $(BIN)
	rm -f *.o
	rm -f .*.d
	$(MAKE) -C $(dir $(LIBMQNIC))/ $@

-include $(wildcard .*.d)

.PHONY: all install clean FORCE
