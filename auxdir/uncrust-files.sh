#! /bin/sh

if [ -z "$1" ]; then
	echo "specify the src directory"
	exit
fi

files=`find $1 \( -name \*.cpp -o -name \*.h \) -printf "%p "`

for file in $files ; do
	uncrustify -c $1../uncrustify.cfg -l CPP --replace $file
done

