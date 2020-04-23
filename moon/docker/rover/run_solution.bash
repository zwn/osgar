#!/usr/bin/env bash

while getopts hr: arg; do
    case $arg in
	r)
	    case $OPTARG in
		"1" )
		    JSONFILE="scout1.json"
		    ROVERSCRIPT="rospy_rover.py"
		    ;;
		"2" )
		    JSONFILE="excavator1.json"
		    ROVERSCRIPT="rospy_excavator.py"
		    ;;
		"3" )
		;;
		
	    esac
	    ;;
	h)
	    echo "Use -r [1|2|3] to choose the run to execute"
	    exit
	    ;;
	*)
	    exit
	    ;;
    esac
done

echo "Unpause simulation"
rosservice call /gazebo/unpause_physics "{}"
echo "wait a moment"
sleep 5

echo "Start robot solution"
export OSGAR_LOGS=`pwd`
cd osgar
python3 -m osgar.record --duration 2700 moon/$JSONFILE --note "collect some ROS data" &
ROBOT_PID=$!
cd ..

# get directory where this bash script lives
samedir=$(dirname $(readlink -f "${BASH_SOURCE[0]}"))

# enable ROS DEBUG output to see if messages are being dropped
export ROSCONSOLE_CONFIG_FILE="${samedir}/rosconsole.config"

# Run your solution and wait for ROS master
# http://wiki.ros.org/roslaunch/Commandline%20Tools#line-45
## roslaunch subt_seed x1.launch --wait &

python ./osgar/moon/$ROVERSCRIPT &
ROS_PID=$!

# Turn everything off in case of CTRL+C and friends.
function shutdown {
       kill ${ROBOT_PID}
       kill ${ROS_PID}
       wait
       exit
}
trap shutdown SIGHUP SIGINT SIGTERM


# Wait for the controllers to finish.
wait ${ROBOT_PID}

echo "Sleep and finish"
sleep 30

# Take robot simulation down.
kill ${ROS_PID}
wait

