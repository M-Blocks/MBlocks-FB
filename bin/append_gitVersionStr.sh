#!/bin/bash

# Check for two input arguments
if [ $# -ne 2 ]; then
	echo "$(basename $0) expects two arguments: the file to rename and the file defining \"const char gitVersionStr[]\""
	exit 1
fi

# Split the name of the that we are renaming at the first period.  
# Everything before is the baseName; everything after is the extension.
baseName=$(echo $1 |\
	sed -e 's/\..*//g')
	
extension=$(echo $1 |\
	sed -e 's/^[^\.]*\.//g')
	
# Use sed to do three things:
# 1) Remove all lines except those (hope the only) that contain gitVersionStr = "..."
# 2) Filter out all of the text on the remaning line except what is between the quotes
# 3) Replace all spaces with underscores
gitString=$(sed \
	-e '/gitVersionStr.*=.*\".*\"/!d' \
	-e 's/.*\"\(.*\)\".*/\1/g' \
	-e 's/ /_/g' \
	< $2)
	
# Rename the file	
mv $1 ${baseName}-${gitString}.${extension}