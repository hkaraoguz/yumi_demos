# yumi_moveit_demos


This package acts as an interface for controlling YuMi via MoveIt!

For testing this package on an actual Yumi:

Bringup YuMi:
```
roslaunch yumi_launch yumi_traj_vel_control.launch
```
Run the demo_online:
```
roslaunch yumi_moveit_config demo_online.launch
```
Execute the following command in a terminal
```
rosrun yumi_moveit_demos main.py --planning_frame "move_it_planning_frame"
```
where `planning_frame` parameter defines the planning frame for MoveIt!. The default value is `/world`.


## Credits

Guðmundur F. Hallgrímsson (gudhal@kth.se) and Junxun Luo (junxun@kth.se) heavily contributed to the development of the Python scripts presented, as part of their work at KTH-RPL.
