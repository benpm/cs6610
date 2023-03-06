#!/usr/bin/env bash

set -e
set -x

cd $( dirname "${BASH_SOURCE[0]}" )

wget https://cs.utah.edu/~benpm/assets.zip
unzip assets.zip
rm assets.zip