# yumi_moveit_demos package


This package acts as an interface for controlling Yumi via MoveIt!

For testing this package on an actual Yumi:

First start the Yumi script using the Yumi terminal.

Bringup YuMi:
```
roslaunch yumi_launch yumi_traj_vel_control.launch
```
Run the demo_online:
```
roslaunch yumi_moveit_config demo_online.launch
```
Run the following command in a terminal
```
rosrun yumi_moveit_demos main.py --planning_frame "move_it_planning_frame"
```
where the `planning_frame` parameter defines the planning frame for MoveIt!. The default value is `/world`.

### Helper scripts
In order to simply move the arm to a 6D pose, you can run the following command:
```
rosrun yumi_demos go_to_pose.py x y z -roll roll_angle -pitch pitch_angle -yaw yaw_angle -planning_frame planning_frame
```
The arm will be automatically chosen based on the given `y` coordinate. The `xyz` parameters are mandatory while the others are optional. So in the simplest case you can just run the script with three parameters to reach `{0.3 0.3 1.2}`:
```
rosrun yumi_demos go_to_pose.py 0.3 0.3 1.2
```

In order to make the robot return to home position you can run:
```
rosrun yumi_demos reset_robot_pose.py
```


## Credits

Guðmundur F. Hallgrímsson (gudhal@kth.se) and Junxun Luo (junxun@kth.se) heavily contributed to the development of the Python scripts presented, as part of their work at KTH-RPL.
