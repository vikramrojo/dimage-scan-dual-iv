CC      ?= clang
ARCHS   ?= -arch arm64 -arch x86_64
CFLAGS  ?= -O2 -g
CFLAGS  += -ffp-contract=off -std=c11 -Wall -Wextra -Wno-unused-parameter -Iinclude -Isrc $(ARCHS) -mmacosx-version-min=11.0
LDLIBS  += -framework IOKit -framework CoreFoundation -framework ApplicationServices -framework ImageIO -framework Accelerate -lm
PREFIX  ?= /usr/local

# version: 1.1.<commits>, plus the commit id for --version
GIT_COUNT := $(shell git rev-list --count HEAD 2>/dev/null || echo 0)
GIT_SHA   := $(shell git rev-parse --short HEAD 2>/dev/null || echo unknown)
GIT_DIRTY := $(shell test -n "$$(git status --porcelain --untracked-files=no 2>/dev/null)" && echo +dirty)
VERSION   ?= 1.1.$(GIT_COUNT)

LIB_SRC = src/avision.c src/avlib.c src/calib.c src/scanner.c src/image.c src/usb_macos.c src/sim.c src/escl.c src/minolta.c src/minolta_ae.c src/minolta_lut.c src/dng.c src/merge.c src/superres.c src/research.c src/deblur.c
LIB_OBJ = $(LIB_SRC:src/%.c=build/%.o)

all: build/libdsdual4.a build/dsdual4

build:
	mkdir -p build

build/%.o: src/%.c src/dsd4_internal.h src/transport.h src/escl.h src/minolta_ae.h src/minolta_lut.h src/superres.h include/dsdual4.h | build
	$(CC) $(CFLAGS) -c $< -o $@

build/libdsdual4.a: $(LIB_OBJ)
	rm -f $@
	libtool -static -o $@ $^ 2>/dev/null

# regenerated only when the version text changes
build/version.h: FORCE | build
	@echo '#define DSD4_VERSION "$(VERSION)"' > $@.tmp
	@echo '#define DSD4_COMMIT "$(GIT_SHA)$(GIT_DIRTY)"' >> $@.tmp
	@cmp -s $@.tmp $@ && rm $@.tmp || mv $@.tmp $@

build/dsdual4: tools/dsdual4.c build/libdsdual4.a build/version.h
	$(CC) $(CFLAGS) -Ibuild $< build/libdsdual4.a $(LDLIBS) -o $@

version:
	@echo $(VERSION) $(GIT_SHA)$(GIT_DIRTY)

FORCE:

test: build/dsdual4
	./tests/run_sim_tests.sh

# needs: pip install unicorn; ORIGINAL_LIB=<extracted MFSBaseLib2891>
difftest:
	./tests/run_diff.sh

# Installs the binary, library, header and Konica Minolta's colour profiles.
# FIRMWARE=path/to/DS_Dual4.BIN additionally installs the firmware image used
# only by the opt-in --firmware-update.
install: all
	install -d $(PREFIX)/bin $(PREFIX)/lib $(PREFIX)/include $(PREFIX)/share/dsdual4
	install -m 755 build/dsdual4 $(PREFIX)/bin/
	install -m 644 build/libdsdual4.a $(PREFIX)/lib/
	install -m 644 include/dsdual4.h $(PREFIX)/include/
	install -m 644 profiles/MLTF3200p.icc profiles/MLTF3200n.icc profiles/NOTICE $(PREFIX)/share/dsdual4/
	install -m 755 packaging/uninstall.sh $(PREFIX)/share/dsdual4/
	if [ -n "$(FIRMWARE)" ]; then install -m 644 "$(FIRMWARE)" $(PREFIX)/share/dsdual4/DS_Dual4.BIN; fi

# Image Capture service for the current user only (the package installs it for everyone)
AGENT = $(HOME)/Library/LaunchAgents/org.dsdual4.airscan.plist
install-agent: install
	sed "s|@BIN@|$(PREFIX)/bin/dsdual4|" packaging/org.dsdual4.airscan.plist > $(AGENT)
	launchctl bootout gui/$$(id -u)/org.dsdual4.airscan 2>/dev/null || true
	launchctl bootstrap gui/$$(id -u) $(AGENT)

uninstall-agent:
	launchctl bootout gui/$$(id -u)/org.dsdual4.airscan 2>/dev/null || true
	rm -f $(AGENT)

# ---- installer package ------------------------------------------------------
# make pkg       unsigned build/dsdual4-$(VERSION).pkg
# make release   signed + notarised package, tagged and published on GitHub.
# Signing (optional for pkg, required for release):
#   SIGN_APP="Developer ID Application: Name (TEAMID)"
#   SIGN_INSTALLER="Developer ID Installer: Name (TEAMID)"
#   NOTARY_PROFILE=<name given to xcrun notarytool store-credentials>
PKGROOT = build/pkgroot
PKG     = build/dsdual4-$(VERSION).pkg

pkg: all
	rm -rf $(PKGROOT) build/pkgres build/dsdual4-component.pkg $(PKG)
	$(MAKE) install PREFIX=$(CURDIR)/$(PKGROOT)/usr/local
	@if [ -n "$(SIGN_APP)" ]; then \
	    codesign --force --options runtime --timestamp --sign "$(SIGN_APP)" $(PKGROOT)/usr/local/bin/dsdual4; \
	else echo "note: SIGN_APP not set, binary is ad-hoc signed"; fi
	install -d $(PKGROOT)/Library/LaunchAgents
	sed "s|@BIN@|/usr/local/bin/dsdual4|" packaging/org.dsdual4.airscan.plist > $(PKGROOT)/Library/LaunchAgents/org.dsdual4.airscan.plist
	cat README.md > $(PKGROOT)/usr/local/share/dsdual4/README.md
	cat LICENSE > $(PKGROOT)/usr/local/share/dsdual4/LICENSE
	xattr -cr $(PKGROOT)
	COPYFILE_DISABLE=1 pkgbuild --root $(PKGROOT) --identifier org.dsdual4.driver --version $(VERSION) \
	    --scripts packaging/scripts --install-location / build/dsdual4-component.pkg
	mkdir -p build/pkgres
	cp packaging/resources/*.html build/pkgres/
	cat LICENSE > build/pkgres/LICENSE.txt
	sed "s|@VERSION@|$(VERSION)|" packaging/distribution.xml > build/distribution.xml
	productbuild --distribution build/distribution.xml --resources build/pkgres --package-path build \
	    $(if $(SIGN_INSTALLER),--sign "$(SIGN_INSTALLER)" --timestamp) $(PKG)
	@if [ -n "$(NOTARY_PROFILE)" ]; then \
	    xcrun notarytool submit $(PKG) --keychain-profile "$(NOTARY_PROFILE)" --wait && xcrun stapler staple $(PKG); \
	else echo "note: NOTARY_PROFILE not set, package not notarised"; fi
	@echo "built $(PKG)"

release:
	@test -n "$(SIGN_APP)" -a -n "$(SIGN_INSTALLER)" -a -n "$(NOTARY_PROFILE)" || \
	    { echo "release needs SIGN_APP, SIGN_INSTALLER and NOTARY_PROFILE"; exit 1; }
	@test -z "$(GIT_DIRTY)" || { echo "commit your changes first"; exit 1; }
	$(MAKE) test
	$(MAKE) pkg
	git tag -a v$(VERSION) -m "dsdual4 $(VERSION)"
	git push origin v$(VERSION)
	gh release create v$(VERSION) $(PKG) --title "dsdual4 $(VERSION)" --generate-notes

clean:
	rm -rf build

.PHONY: all pkg release test version FORCE difftest install install-agent uninstall-agent clean
