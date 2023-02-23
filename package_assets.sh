#!/usr/bin/env bash

set -e
set -x

cd $( dirname "${BASH_SOURCE[0]}" )

zip -ru assets.zip resources/models resources/textures