if [[ "$#" -ne 2 && "$#" -ne 3 ]]; then
    echo "Error: You must provide two or three arguments. Usage: ./run.sh width height (min_dist)"
    exit 1
fi
source devel/setup.bash
if [[ "$#" -eq 2 ]]; then
    echo "Running command: python3 src/AMPER/scripts/world_launch_config.py $1 $2"
    python3 src/AMPER/scripts/world_launch_config.py "$1" "$2"
else
    echo "Running command: python3 src/AMPER/scripts/world_launch_config.py $1 $2 $3"
    python3 src/AMPER/scripts/world_launch_config.py "$1" "$2" "$3"
fi
catkin_make
python3 src/AMPER/scripts/world_launch.py