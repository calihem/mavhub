#!/bin/sh

# set date on memoryless embedded systems

if test -z $1 ; then
	echo Usage: $0 hostname
	exit 1
fi

HOST=$1

cmd="ssh ${HOST} date `date +%m%d%H%M%Y`"
echo Executing $cmd

($cmd)

