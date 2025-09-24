#!/bin/bash

PREFIX=/home/taste/tool-inst
SOURCES=$(dirname $0)

mkdir -p "${PREFIX}/include/TASTE-SAMV71-RTEMS-Drivers/src"
rm -rf "${PREFIX}/include/TASTE-SAMV71-RTEMS-Drivers/src/*"
cp -r "${SOURCES}/src/samv71_rtems_serial" "${PREFIX}/include/TASTE-SAMV71-RTEMS-Drivers/"
cp -r "${SOURCES}/configurations" "${PREFIX}/include/TASTE-SAMV71-RTEMS-Drivers/configurations"
