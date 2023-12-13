# ddg_bringup

commands to run the system: 

1. Run the docker:

```sudo docker exec -it ddg-humble-container bash```

(docker image: ddg-humblesim)
(docker container: ddg-humble-container)

Inside the docker in every terminal ensure to run: ```export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp```
This has to be done in order to establish reliable communication between nodes.

2. To run the planner + mission controller run: 

```export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp```

```cd /home/mrsd_teamh/mfi_multiagent_sim/ws```

```source install/setup.bash```

```ros2 launch ddg_bringup ddg_bringup.launch.py ```

3. To send a goal to a robot - on a new terminal run:

```export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp```

```cd /home/mrsd_teamh/mfi_multiagent_sim/ws```

```source install/setup.bash```

- just for robot 1: ```ros2 action send_goal /MissionControl robot_action_interfaces/action/MissionControl "{robot_specific_dock_ids: [1,2, 100, 100], robot_specific_undock_flags: [1,1]}"```

- just for robot 2: ```ros2 action send_goal /MissionControl robot_action_interfaces/action/MissionControl "{robot_specific_dock_ids: [100,100, 1, 2], robot_specific_undock_flags: [1,1]}"```

(the robot_specific_undock_flags are boolean flags to enable disable undocking only behaviour)

- both the robots: ```ros2 action send_goal /MissionControl robot_action_interfaces/action/MissionControl "{robot_specific_dock_ids: [1,2,2,1], robot_specific_undock_flags: [1,1]}"```
    

4. To stop the robots -  on a new terminal run:

```export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp```

```cd /home/mrsd_teamh/mfi_multiagent_sim/ws```

```source install/setup.bash```

```ros2 action send_goal /MissionControl robot_action_interfaces/action/MissionControl "{robot_specific_dock_ids: [0,0,0,0], robot_specific_undock_flags: [1,1]}"```

- to stop just one robot and ignore the other: 
```ros2 action send_goal /MissionControl robot_action_interfaces/action/MissionControl "{robot_specific_dock_ids: [100,100,0,0], robot_specific_undock_flags: [1,1]}"```


All the robot behaviour specific configs are exposed in the ddg_bringup.launch.py 
and all the docking location / ID specific stuff can be updated and changed in the `config/params.yaml` file in 
`/home/mrsd_teamh/mfi_multiagent_sim/ws/src/robot_misson_control`

Some useful stuff/hints from the MRSD Team:
1. ensure you are exporting cyclone_dds!
2. ensure the robots are localized well! 
3. Swapping docking locations that are close by (within 1.5 meters ish of radius) MAY (not 100% but sometimes) result in a crash due to downsampling and waypoint follower tuning issues. - if the locations are at sufficient distance they will go via non conflicting paths.
4. For the Aruco docking procedure to work - we need the Aruco nodes to launch which runs on ORIN - for the ORIN to boot - it HAS to be connected to a display via a display cable and login - it will launch a docker and run all the necessary nodes.
the password for the ORIN is `0` (the number zero).