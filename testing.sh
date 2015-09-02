#!/bin/bash

echo "adns test data collection" 

path=$1
if [ ! -d "$path" ]; then
	$(mkdir $path)
else
	echo "abort: folder $path already exists"
#	exit
fi

# set adns to automatic frame and shutter period
./adns-connect -a

#servo stuff
speeds=(0x00 0x01 0x02 0x04 0x06 0x08 0x0A 0x0C 0x0E 0x10 0x12 0x14 0x16 0x18 0x1A 0x1C 0x1E 0x20)
#speeds=(0x04)

# set servo max value
$(i2cset -y 0 0x18 0x36 6000 w)

for run in {1..10}
do
	echo "run: $run"

	for speed in "${!speeds[@]}"
	do
		s=${speeds[$speed]}
		t=10
	
		# setup adns to max shutter period - original
		./adns-connect -S 9100
		sleep 0.5

		#set servo initial position and speed
		$(i2cset -y 0 0x18 0x32 0x00 w)
		$(i2cset -y 0 0x18 0x39 $s b)

		d=$(date +%H%M%S)
		id="speed$s""_$d""_orig"

		program="./adns-connect -t $t -i /dev/i2c-0 -f $path/$id.dat"

		echo "$program"
		eval $program

		# setup adns to max shutter period
		./adns-connect -S 60000
		sleep 0.5

		#set servo initial position and speed
		$(i2cset -y 0 0x18 0x32 0x00 w)
		$(i2cset -y 0 0x18 0x39 $s b)

		id="speed$s""_$d"

		program="./adns-connect -t $t -i /dev/i2c-0 -f $path/$id.dat"

		echo "$program"
		eval $program
	done
done

# stop servo
$(i2cset -y 0 0x18 0x39 0x00 b)

