#!/bin/sh

STYLE="AStyle.exe --align-pointer=name --align-reference=name --suffix=none --break-blocks --pad-oper --pad-header --break-blocks --keep-one-line-blocks --indent-switches --indent=spaces"

find src/hal src/nvs src/modem src/unittest src/mqtt -regex '.*/.*\.\(c\|h\)' | xargs $STYLE

for file in src/main.c src/bme/bme680_helper.* src/icm/icm42605_helper.*
do
	$STYLE $file
done
