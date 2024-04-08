#!/bin/bash

mkdir build

# build exec for cpp

cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build -j