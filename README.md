# yumi_moveit_demos


This package contains a set of scripts for controlling YuMi via MoveIt! Python interface

For running them, bringup YuMi:
```
roslaunch yumi_traj_vel_control.launch
```
Then run the demo_online launch file from yumi_moveit_config:
```
roslaunch yumi_moveit_config demo_online.launch
```
and execute the following command in a terminal
```
rosrun yumi_moveit_demos main.py
```


## Credits

Guðmundur F. Hallgrímsson (gudhal@kth.se) and Junxun Luo (junxun@kth.se) heavily contributed to the development of the Python scripts presented, as part of their work at KTH-RPL.
