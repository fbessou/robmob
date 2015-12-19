#!/bin/sh

version=`rosversion -d`
sed -s "s/hydro/$version/g" $1 > $2
sed -s "s/indigo/$version/g" $1 > $2
