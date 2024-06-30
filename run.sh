if [ "$#" -ne 4 ]; then
    echo "Error: You must provide exactly four arguments. Usage: ./run.sh start_x start_y end_x end_y"
    exit 1
fi
source devel/setup.bash
roslaunch AMPER simulation.launch start_x:="$1" start_y:="$2" end_x:="$3" end_y:="$4"