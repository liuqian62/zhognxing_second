#!/bin/bash

SCRIPT=$(readlink -f "$0")
BASEDIR=$(dirname "$SCRIPT")
cd $BASEDIR

if [ ! -d result ]
then
    echo "ERROR: $BASEDIR is not a valid directory named result."
    echo "  Please run this script in a regular directory of SDK_C++."
    exit -1
fi

cd result
rm -rf bin
rm -rf build
rm *.png
cd ..
cd test
rm -rf bin
rm -rf build
rm *.png
cd ..

rm -f result.zip
zip -r result.zip result

rm -f test.zip
zip -r test.zip test
