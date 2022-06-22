#!/bin/bash

SCRIPT=$(readlink -f "$0")
BASEDIR=$(dirname "$SCRIPT")
cd $BASEDIR


cd result
sh build_and_run.sh
cd ..
cd test
sh build_and_run.sh