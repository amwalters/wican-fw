#!/usr/bin/env sh
set -e

minify-html --output main/web/src/homepage.html \
  --keep-closing-tags \
  --minify-css \
  --minify-js \
  main/web/homepage_full.html

echo "HTML minification completed successfully!"
