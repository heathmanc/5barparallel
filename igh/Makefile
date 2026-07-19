# Build the IgH DC go/no-go test. Needs the IgH EtherCAT master installed
# (headers in /usr/include or /opt/etherlab/include, lib in /usr/lib or
# /opt/etherlab/lib). Override paths if your install prefix differs:
#   make ETHERLAB=/opt/etherlab

ETHERLAB ?= /usr
CFLAGS  += -Wall -O2 -I$(ETHERLAB)/include
LDFLAGS += -L$(ETHERLAB)/lib -Wl,-rpath,$(ETHERLAB)/lib
LDLIBS  += -lethercat

igh_test: igh_test.c
	$(CC) $(CFLAGS) -o $@ igh_test.c $(LDFLAGS) $(LDLIBS)

clean:
	rm -f igh_test
