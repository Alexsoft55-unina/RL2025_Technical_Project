# RL2025_Progetto_Finale
go to the workspace directory and build the packages
```
colcon build && source install/setup.bash
```

```
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/user/ros2_ws/src/panda_execution/model
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/user/ros2_ws/src/panda_configuration
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/user/ros2_ws/src/PX4-Autopilot/Tools/simulation/gz/models
export GZ_VERSION=harmonic
```


then open two different terminals, one must launch the drone simulation, the other the panda simulation
```
cd src/PX4-Autopilot \
make px4_sitl gz_x500
```
on another terminal launch the bridge between uORB and DDS
```
cd .. \
./DDS_run.sh
```
```
cd .. \
ros2 launch panda_execution panda_spawn.launch.py
```
then you must separetely launch the single node for the management of the drone
```
ros2 run offboard_rl go_to_point_server
ros2 run offboard_rl trajectory_drone_manager
```
to make the simulation start you must envoy a message on the panda control manager
```
ros2 topic pub --once /panda_command std_msgs/msg/Int32 "{data: 100}"
```

# RL2025_Technical_Project
