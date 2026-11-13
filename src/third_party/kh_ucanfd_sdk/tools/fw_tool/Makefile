# Makefile for KCAN Firmware Tool
#
# Build: make
# Clean: make clean
# Install: sudo make install

CC = gcc
CFLAGS = -Wall -Wextra -O2
LDFLAGS =

# Target
TARGET = kcan_fw_tool

# Source files
SRCS = kcan_fw_tool.c
OBJS = $(SRCS:.c=.o)

# Install directory
PREFIX ?= /usr/local
BINDIR = $(PREFIX)/bin
FWDIR = /usr/lib/firmware

# Firmware upgrade script and images
FW_SCRIPT = kcan_fw_upgrade.sh
FW_PY_SCRIPT = kcan_fw_upgrade.py
FW_SCRIPT_INSTALL_NAME = kcan_fw_upgrade_bash
FW_PY_SCRIPT_INSTALL_NAME = kcan_fw_upgrade
FW_IMAGE_DIR = ../fw_images
FW_IMAGES = $(wildcard $(FW_IMAGE_DIR)/*.enc)
FW_MANIFEST_SRC = $(FW_IMAGE_DIR)/fw_manifest.json
FW_MANIFEST_INSTALL_NAME = kcan_fw_manifest.json

.PHONY: all clean install uninstall help

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)

install: $(TARGET)
	install -d $(DESTDIR)$(BINDIR)
	install -m 755 $(TARGET) $(DESTDIR)$(BINDIR)/
	install -m 755 $(FW_SCRIPT) $(DESTDIR)$(BINDIR)/$(FW_SCRIPT_INSTALL_NAME)
	install -m 755 $(FW_PY_SCRIPT) $(DESTDIR)$(BINDIR)/$(FW_PY_SCRIPT_INSTALL_NAME)
	install -d $(DESTDIR)$(FWDIR)
	@if [ -n "$(FW_IMAGES)" ]; then \
		install -m 644 $(FW_IMAGES) $(DESTDIR)$(FWDIR)/; \
		echo "Installed firmware images to $(DESTDIR)$(FWDIR)/"; \
	else \
		echo "Warning: No firmware images found in $(FW_IMAGE_DIR)/"; \
	fi
	@if [ -f "$(FW_MANIFEST_SRC)" ]; then \
		install -m 644 "$(FW_MANIFEST_SRC)" "$(DESTDIR)$(FWDIR)/$(FW_MANIFEST_INSTALL_NAME)"; \
		echo "Installed firmware manifest to $(DESTDIR)$(FWDIR)/$(FW_MANIFEST_INSTALL_NAME)"; \
	else \
		echo "Warning: Manifest not found: $(FW_MANIFEST_SRC)"; \
	fi

uninstall:
	rm -f $(DESTDIR)$(BINDIR)/$(TARGET)
	rm -f $(DESTDIR)$(BINDIR)/$(FW_SCRIPT_INSTALL_NAME)
	rm -f $(DESTDIR)$(BINDIR)/$(FW_PY_SCRIPT_INSTALL_NAME)
	# Backward compatibility: remove old install names if they exist.
	rm -f $(DESTDIR)$(BINDIR)/kcan_fw_upgrade.sh
	rm -f $(DESTDIR)$(BINDIR)/kcan_fw_upgrade.py
	rm -f $(DESTDIR)$(BINDIR)/$(basename $(FW_SCRIPT))
	rm -f $(DESTDIR)$(BINDIR)/$(basename $(FW_PY_SCRIPT))
	rm -f $(DESTDIR)$(FWDIR)/KH-UCANFD-*.enc
	rm -f $(DESTDIR)$(FWDIR)/$(FW_MANIFEST_INSTALL_NAME)

# Static build (for distribution)
static: LDFLAGS += -static
static: clean all

help:
	@echo "KCAN Firmware Tool - Build System"
	@echo ""
	@echo "Targets:"
	@echo "  all       - Build the firmware tool (default)"
	@echo "  clean     - Remove build artifacts"
	@echo "  install   - Install to $(BINDIR) and firmware to $(FWDIR)"
	@echo "  uninstall - Remove from $(BINDIR) and $(FWDIR)"
	@echo "  static    - Build statically linked binary"
	@echo "  help      - Show this help"
