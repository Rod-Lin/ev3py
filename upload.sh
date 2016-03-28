#! /bin/bash

git status
echo "Is this OK? (Y/n):"
read ans
case $ans in
	N|n )
		exit
		;;
	* );;
esac

echo "Your message:"
LFS="\n"
read ans

date=`date +%d%m%y`

if [ -f "upload.log" ]; then
	if [ -f "date.log" ]; then
		if [ $date = `cat date.log` ]; then
			count=`cat upload.log`
			rm -r upload.log
		else
			rm -r date.log
			rm -r upload.log
			echo $date > date.log
			count=1
		fi
	else
		echo $date > date.log
		rm -r upload.log
		count=1
	fi
else
	count=1
fi

echo `expr $count + 1` > upload.log

if [ $count -lt 10 ]; then
	count="0$count"
fi

git add -A
git commit -m "$date#$count: $ans"
git push -u origin master