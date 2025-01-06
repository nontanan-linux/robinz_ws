# launch turtlebot simulation
````
export TURTLEBOT3_MODEL=burger
<!-- TurtleBot3 World -->
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
<!-- TurtleBot3 House -->
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
````

# launch nav2 node
- map server
- amcl localizer
- rviz
- initialpose
````
ros2 launch robinz_vehicle_launch robinz_simulation.launch.py
````

# py purepursuit node
````
ros2 run py_path_generator path_generator
ros2 run py_purepursuit bb_purepursuit
````
to publish path
```
ros2 topic pub -1 /AGV1/mode std_msgs/msg/String "data: 'path|A1|A2'"
```