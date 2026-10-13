#!/bin/sh
# Removes dsdual4: sudo /usr/local/share/dsdual4/uninstall.sh
[ "$(id -u)" = 0 ] || exec sudo "$0" "$@"
USER_ID=$(stat -f %u /dev/console 2>/dev/null)
[ -n "$USER_ID" ] && launchctl bootout "gui/$USER_ID/org.dsdual4.airscan" 2>/dev/null
rm -f /Library/LaunchAgents/org.dsdual4.airscan.plist /usr/local/bin/dsdual4 \
      /usr/local/lib/libdsdual4.a /usr/local/include/dsdual4.h
rm -rf /usr/local/share/dsdual4
pkgutil --forget org.dsdual4.driver >/dev/null 2>&1
echo "dsdual4 removed (settings and calibration cache remain in ~/Library/Application Support/dsdual4)"
