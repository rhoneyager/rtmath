#!/bin/tcsh
echo "Running pre-build"
#set RTMATHCONF = $HOME/.rtmath
#set RTMATHCONF = /home/rhoneyag/.rtmath
echo "Getting revision information"
# Set the subversion revision and repository information
#set file = build/debug_subversion.h
set file = $argv[1]/debug_subversion.h
echo "Outputting to $file"
# Get information
set rev = `/usr/bin/svn info . | /bin/grep Revision: |/usr/bin/cut -c11-`
set url = `/usr/bin/svn info . | /bin/grep URL: |/usr/bin/cut -c6-`
set dt = `/usr/bin/svn info . | /bin/grep Date: |/usr/bin/cut -c20-`
# Create file
/bin/rm -f $file
echo "#pragma once" > $file
echo "#define SUB_REV $rev" >> $file
echo "#define SUB_DATE "\""$dt"\"" " >> $file
echo "#define SUB_SOURCE "\""$url"\"" " >> $file
#echo "#define RTC "\""$RTMATHCONF"\""" >> $file

