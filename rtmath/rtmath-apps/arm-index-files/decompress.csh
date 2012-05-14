#!/bin/tcsh

find /data/rhoneyag/arm/ftp.archive.arm.gov -type f \( -name '*.cdf.bz2' \) -exec bunzip2 {} \;

