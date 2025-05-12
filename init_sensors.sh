
executable_found=`ls | grep "CALIBRATION_EXECUTABLE" | wc -l`
executable_running=`ps aux | grep "./CALIBRATION_EXECUTABLE" | grep -v grep | wc -l`
if [ $executable_running -gt 0 ]; then
	echo "The executable is running"
	exit 0
elif [ $executable_found -gt 0 ]; then
	echo "I have seen the executable"
	./CALIBRATION_EXECUTABLE > /dev/null 2>&1 &
else
	echo "I could not find the executable. Making it and running it";
	make main && mv main CALIBRATION_EXECUTABLE && ./CALIBRATION_EXECUTABLE > /dev/null 2>&1 &
fi
