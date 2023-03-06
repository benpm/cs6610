#!/usr/bin/env bash

set -e
set -x

cd $( dirname "${BASH_SOURCE[0]}" )

./package_assets.sh

scp -P 5522 assets.zip benpm@shell.cs.utah.edu:'~/public_html/assets.zip'

rm assets.zip