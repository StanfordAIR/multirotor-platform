### telemetry.py
Takes mavros inputs and posts telemetry data to ground server.  Independent of other functionalities.

### px4.py
Interplay between waypoints and low-level px4 commands.  Takes waypoints and flies to the waypoints.  Currently loads waypoints from file rather than taking them in from another ROS node.

### pathplanning.py
Functionalities to create path from (lat, lng) point A to B given set of static obstacles.  Currently a standalone file and is imported for use.

### interop_data.py
 Pulls mission data from mission definition API, plans path from mission data, and sends resulting list of waypoints to px4.py node.

### planning_util/
All the stuff needed for path planning.  Not totally sure what it specifically does, but it works.

### Testing
To test path planning, simply run pathplanning.py and view the output in `/results/test_environment_display/env.png`.
