CC      ?= clang
ARCHS   ?= -arch arm64 -arch x86_64
CFLAGS  ?= -O2 -g
CFLAGS  += -std=c11 -Wall -Wextra -Wno-unused-parameter -Iinclude -Isrc $(ARCHS) -mmacosx-version-min=11.0
LDLIBS  += -framework IOKit -framework CoreFoundation -framework ApplicationServices -framework ImageIO -lm
PREFIX  ?= /usr/local

LIB_SRC = src/avision.c src/avlib.c src/calib.c src/scanner.c src/image.c src/usb_macos.c src/sim.c src/escl.c
LIB_OBJ = $(LIB_SRC:src/%.c=build/%.o)

all: build/libdsdual4.a build/dsdual4

build:
	mkdir -p build

build/%.o: src/%.c src/dsd4_internal.h src/transport.h src/escl.h include/dsdual4.h | build
	$(CC) $(CFLAGS) -c $< -o $@

build/libdsdual4.a: $(LIB_OBJ)
	rm -f $@
	libtool -static -o $@ $^ 2>/dev/null

build/dsdual4: tools/dsdual4.c build/libdsdual4.a
	$(CC) $(CFLAGS) $< build/libdsdual4.a $(LDLIBS) -o $@

test: build/dsdual4
	./tests/run_sim_tests.sh

# needs: pip install unicorn; ORIGINAL_LIB=<extracted MFSBaseLib2891>
difftest:
	./tests/run_diff.sh

# FIRMWARE=path/to/DS_Dual4.BIN installs the firmware image used for the
# (scanner-requested) firmware check next to the binary.
install: all
	install -d $(PREFIX)/bin $(PREFIX)/lib $(PREFIX)/include $(PREFIX)/share/dsdual4
	install -m 755 build/dsdual4 $(PREFIX)/bin/
	install -m 644 build/libdsdual4.a $(PREFIX)/lib/
	install -m 644 include/dsdual4.h $(PREFIX)/include/
	if [ -n "$(FIRMWARE)" ]; then install -m 644 "$(FIRMWARE)" $(PREFIX)/share/dsdual4/DS_Dual4.BIN; fi

# Run the AirScan bridge in the background at login (scanner appears in
# Image Capture / Preview whenever it is connected).
AGENT = $(HOME)/Library/LaunchAgents/org.dsdual4.airscan.plist
install-agent: install
	sed "s|@BIN@|$(PREFIX)/bin/dsdual4|; s|@LOG@|$(HOME)/Library/Logs/dsdual4.log|" packaging/org.dsdual4.airscan.plist > $(AGENT)
	launchctl bootout gui/$$(id -u) $(AGENT) 2>/dev/null || true
	launchctl bootstrap gui/$$(id -u) $(AGENT)

uninstall-agent:
	launchctl bootout gui/$$(id -u) $(AGENT) 2>/dev/null || true
	rm -f $(AGENT)

VERSION ?= 1.0.0
PKGROOT  = build/pkgroot

# macOS installer package (unsigned). FIRMWARE=... optionally bundles your
# copy of DS_Dual4.BIN under /usr/local/share/dsdual4.
pkg: all
	rm -rf $(PKGROOT)
	$(MAKE) install PREFIX=$(CURDIR)/$(PKGROOT)/usr/local FIRMWARE="$(FIRMWARE)"
	install -m 644 packaging/org.dsdual4.airscan.plist $(PKGROOT)/usr/local/share/dsdual4/
	cat README.md > $(PKGROOT)/usr/local/share/dsdual4/README.md
	cat docs/PROTOCOL.md > $(PKGROOT)/usr/local/share/dsdual4/PROTOCOL.md
	xattr -cr $(PKGROOT)
	COPYFILE_DISABLE=1 pkgbuild --root $(PKGROOT) --identifier org.dsdual4.driver --version $(VERSION) \
	    --install-location / build/dsdual4-$(VERSION).pkg

clean:
	rm -rf build

.PHONY: all pkg test difftest install install-agent uninstall-agent clean
