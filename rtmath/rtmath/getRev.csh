#!/bin/tcsh

echo "Getting revision information"

# Set the subversion revision and repository information

set file = build/debug_subversion.h

# Get information
set rev = `svn info . | grep Revision: |cut -c11-`
set url = `svn info . | grep URL: |cut -c6-`
set dt = `svn info . | grep Date: |cut -c20-`

# Create file

#cp debug_subversion.h.template $file

rm -f $file

echo "#pragma once" > $file
echo "#define SUB_REV $rev" >> $file
echo "#define SUB_DATE "\""$dt"\"" " >> $file
echo "#define SUB_SOURCE "\""$url"\"" " >> $file

#rpline $file 3 "#define SUB_REV $rev"
#rpline $file 4 "#define SUB_DATE \"$dt\""
#rpline $file 6 "#define SUB_SOURCE \"$url\""

touch src/debug.cpp

