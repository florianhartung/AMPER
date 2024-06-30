import roslaunch
import sys
import os
import yaml

# Find the config file
config_path = os.path.join(os.path.dirname(__file__), '../../../config.yaml')
config = None

# Load the config file
with open(config_path, 'r') as f:
    config = yaml.load(f, Loader=yaml.FullLoader)

if config is None:
    print("Config file not found or can't be opened")
    sys.exit(1)

# Get the path to the launch file
path_to_launch_file = os.path.join(os.path.dirname(__file__), '../launch/simulation.launch')

# Get the start and end coordinates
start_x = int(config["start"]["x"])
start_y = int(config["start"]["y"])
end_x = int(config["end"]["x"])
end_y = int(config["end"]["y"])

# Launch the simulation with the given coordinates
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
cli_args = [path_to_launch_file,  f"start_x:={start_x}", f"start_y:={start_y}", f"end_x:={end_x}", f"end_y:={end_y}"]
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
launch.start()

# Spin the node until the user interrupts
try:
    launch.spin()
except KeyboardInterrupt:
    sys.exit(0)
finally:
    launch.shutdown()