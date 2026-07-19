# Build the IgH DC go/no-go test. Needs the IgH EtherCAT master installed
# (headers in /usr/include or /opt/etherlab/include, lib in /usr/lib or
# /opt/etherlab/lib). Override paths if your install prefix differs:
#   make ETHERLAB=/opt/etherlab

ETHERLAB ?= /usr
CFLAGS  += -Wall -O2 -I$(ETHERLAB)/include
LDFLAGS += -L$(ETHERLAB)/lib -Wl,-rpath,$(ETHERLAB)/lib
LDLIBS  += -lethercat
# shm_open/mlockall need -lrt, the SDO worker needs -lpthread
DAEMON_LIBS = -lethercat -lrt -lpthread

all: igh_test ec_master_daemon

igh_test: igh_test.c
	$(CC) $(CFLAGS) -o $@ igh_test.c $(LDFLAGS) $(LDLIBS)

ec_master_daemon: ec_master_daemon.c
	$(CC) $(CFLAGS) -o $@ ec_master_daemon.c $(LDFLAGS) $(DAEMON_LIBS)

clean:
	rm -f igh_test ec_master_daemon

.PHONY: all clean
