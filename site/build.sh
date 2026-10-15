#!/bin/sh
# Builds the GitHub Pages site into _site/: site/ with the latest release's
# download filled in.  Runs the same locally and in .github/workflows/pages.yml
# (needs gh, logged in or with GH_TOKEN; without a release, or without gh, the
# button links the Releases page).
set -eu
cd "$(dirname "$0")/.."
REPO=${GITHUB_REPOSITORY:-vikramrojo/dimage-scan-dual-iv}
RELEASES="https://github.com/$REPO/releases"

href="$RELEASES/latest"
label="Download the latest release"
file="the latest dsd4 package"
note="latest release"
if command -v gh >/dev/null 2>&1 &&
   info=$(gh release view --repo "$REPO" --json tagName,publishedAt,assets \
          -q '[.tagName, (.publishedAt | split("T")[0]), ([.assets[] | select(.name | endswith(".pkg"))][0] | .name, .url)] | @tsv' 2>/dev/null) &&
   [ -n "$info" ]; then
    tag=$(printf '%s' "$info" | cut -f1)
    date=$(printf '%s' "$info" | cut -f2)
    name=$(printf '%s' "$info" | cut -f3)
    url=$(printf '%s' "$info" | cut -f4)
    if [ -n "$name" ] && [ -n "$url" ]; then
        href=$url
        label="Download $name"
        file=$name
        note="$tag, $date"
    fi
else
    echo "note: no release found (or gh unavailable): linking the Releases page" >&2
fi

# emptied in place, not replaced: a server running in _site keeps working
mkdir -p _site
find _site -mindepth 1 -delete
cp -R site/. _site/
rm -f _site/build.sh
# placeholders; values contain no '|' (URLs, file names, dates)
sed -i.bak \
    -e "s|@DOWNLOAD_HREF@|$href|g" \
    -e "s|@DOWNLOAD_LABEL@|$label|g" \
    -e "s|@DOWNLOAD_FILE@|$file|g" \
    -e "s|@RELEASE_NOTE@|$note|g" \
    -e "s|@RELEASES_URL@|$RELEASES|g" \
    _site/index.html
rm -f _site/index.html.bak
if grep -q '@[A-Z_]*@' _site/index.html; then
    echo "error: unfilled placeholder in _site/index.html" >&2
    exit 1
fi
echo "built _site/ ($label)"
