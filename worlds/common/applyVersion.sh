#!/bin/sh

version=`rosversion -d`
sed -s "s/ROSVERSION/$version/g" $1 > $2
