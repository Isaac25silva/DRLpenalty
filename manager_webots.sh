#Program to manager webots

cd

cd webots/

while true
do
	#Open the xterm and execute the webots whitout wait finalize program
	xterm -e ./webots --mode=fast &
	echo "init webots"
	sleep 14400 #wait 4 hous to execute the program
	echo "kill webots"
	kill -9 $(pidof xterm) #kill the xterm and webots because the memory is full
	sleep 10
done
