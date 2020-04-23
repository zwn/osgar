#!/usr/bin/env bash

while getopts hr: arg; do
    case $arg in
	r)
	    case $OPTARG in
		"1" )
		    JSONFILES=("scout1.json")
		    ROVERSCRIPTS=("rospy_rover.py")
		    ;;
		"2" )
		    JSONFILES=("excavator1.json" "hauler1.json")
		    ROVERSCRIPTS=("rospy_excavator.py" "rospy_hauler.py")
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

ROBOT_PIDS=()
ROS_PIDS=()
echo "Start robot solution"
export OSGAR_LOGS=`pwd`
cd osgar
for s in ${JSONFILES[@]}; do
    echo "starting recording of $s"
    python3 -m osgar.record --duration 2700 moon/$s --note "collect some ROS data" &
    ROBOT_PIDS+=($!)
done

cd ..

# get directory where this bash script lives
samedir=$(dirname $(readlink -f "${BASH_SOURCE[0]}"))

# enable ROS DEBUG output to see if messages are being dropped
export ROSCONSOLE_CONFIG_FILE="${samedir}/rosconsole.config"

# Run your solution and wait for ROS master
# http://wiki.ros.org/roslaunch/Commandline%20Tools#line-45
## roslaunch subt_seed x1.launch --wait &

for s in ${ROVERSCRIPTS[@]}; do
    echo "Starting script $s"
    python ./osgar/moon/$s &
    ROS_PIDS+=($!)
done

# Turn everything off in case of CTRL+C and friends.
function shutdown {
    for p in ${ROBOT_PIDS[@]}; do
	kill $p
    done
    for p in ${ROS_PIDS[@]}; do
	kill $p
    done
    
    wait
    exit
}
trap shutdown SIGHUP SIGINT SIGTERM


# Wait for the controllers to finish.
for r in ${ROBOT_PIDS[@]}; do
    wait $r
done


echo "Sleep and finish"
sleep 30

# Take robot simulation down.
for r in ${ROS_PIDS[@]}; do
    wait $r
done
wait

