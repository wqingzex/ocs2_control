# Makefile for kcanfdtest.c


TARGETS = kcanfdtest cantx_1KHz
SCRIPTS = kcanfd_test
SRCS = kcanfdtest.c cantx_1KHz.c
OBJS = $(SRCS:.c=.o)
CC = gcc
CFLAGS = -Wall -O2
LDFLAGS = 
INSTALL_DIR = /usr/local/bin
SCRIPT_DIR = $(INSTALL_DIR)

.PHONY: all clean install uninstall

all: $(TARGETS)

kcanfdtest: kcanfdtest.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

cantx_1KHz: cantx_1KHz.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) -lrt -lpthread -lm

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TARGETS) $(OBJS)

install:
	sudo install -m 0755 $(TARGETS) $(INSTALL_DIR)
	sudo install -m 0755 $(SCRIPTS) $(SCRIPT_DIR)

uninstall:
	sudo rm -f $(addprefix $(INSTALL_DIR)/, $(TARGETS))
	sudo rm -f $(addprefix $(SCRIPT_DIR)/, $(SCRIPTS))
	sudo rm -f $(SCRIPT_DIR)/kcanfd_test.sh