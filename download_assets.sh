#!/usr/bin/env bash

set -e
set -x

cd $( dirname "${BASH_SOURCE[0]}" )

ftp ftp://ftp.cs.utah.edu/users/benpm/assets.zip
unzip assets.zip
rm assets.zip