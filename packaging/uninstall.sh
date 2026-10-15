#!/bin/sh
# Removes dsd4: sudo /usr/local/share/dsd4/uninstall.sh
[ "$(id -u)" = 0 ] || exec sudo "$0" "$@"
USER_ID=$(stat -f %u /dev/console 2>/dev/null)
[ -n "$USER_ID" ] && launchctl bootout "gui/$USER_ID/org.dsd4.airscan" 2>/dev/null
rm -f /Library/LaunchAgents/org.dsd4.airscan.plist /usr/local/bin/dsd4 \
      /usr/local/lib/libdsd4.a /usr/local/include/dsd4.h
rm -rf /usr/local/share/dsd4
# and an install from before the rename to dsd4 (dsdual4), if any
[ -n "$USER_ID" ] && launchctl bootout "gui/$USER_ID/org.dsdual4.airscan" 2>/dev/null
rm -f /Library/LaunchAgents/org.dsdual4.airscan.plist /usr/local/bin/dsdual4 \
      /usr/local/lib/libdsdual4.a /usr/local/include/dsdual4.h
rm -rf /usr/local/share/dsdual4
pkgutil --forget org.dsd4.driver >/dev/null 2>&1
pkgutil --forget org.dsdual4.driver >/dev/null 2>&1
echo "dsd4 removed (settings and calibration cache remain in ~/Library/Application Support/dsd4)"
