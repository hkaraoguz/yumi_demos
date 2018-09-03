#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
from yumi_demos import yumi_moveit_utils as yumi
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty


def close_grippers(arm):
    """Closes the grippers.

    Closes the grippers with an effort of 15 and then relaxes the effort to 0.

    :param arm: The side to be closed (moveit_utils LEFT or RIGHT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    yumi.gripper_effort(arm, 15.0)
    yumi.gripper_effort(arm, 0.0)


def open_grippers(arm):
    """Opens the grippers.

    Opens the grippers with an effort of -15 and then relaxes the effort to 0.

    :param arm: The side to be opened (moveit_utils LEFT or RIGHT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    yumi.gripper_effort(arm, -10.0)
    yumi.gripper_effort(arm, 0.0)


def move_and_grasp(arm, pose_ee, grip_effort):
    # if (grip_effort <= 20 and grip_effort >= -20):
    #    yumi.gripper_effort(arm, grip_effort)
    # else:
    #    print("The gripper effort values should be in the range [-20, 20]")
    # try:
    #     yumi.traverse_path([pose_ee], arm, 10)
    # except Exception as ex:
    #     rospy.logerr(str(ex))
    #     return

    if (arm == yumi.LEFT):
        yumi.plan_and_move(yumi.group_l,
                           yumi.create_pose_euler(pose_ee[0], pose_ee[1], pose_ee[2], pose_ee[3], pose_ee[4],
                                                  pose_ee[5]))
    elif (arm == yumi.RIGHT):
        yumi.plan_and_move(yumi.group_r,
                           yumi.create_pose_euler(pose_ee[0], pose_ee[1], pose_ee[2], pose_ee[3], pose_ee[4],
                                                  pose_ee[5]))


def run():
    """Starts the node

    Runs to start the node and initialize everthing. Runs forever via Spin()

    :returns: Nothing
    :rtype: None
    """
    rospy.init_node('moveit_yumi_resetpose')
    # Start by connecting to ROS and MoveIt!
    yumi.init_Moveit("/yumi_pedestal")

    # Print current joint angles
    yumi.print_current_joint_states(yumi.RIGHT)
    yumi.print_current_joint_states(yumi.LEFT)

    # Reset YuMi joints to "home" position
    yumi.reset_pose()

if __name__ == '__main__':
    try:
        run()

        print "####################################     Program finished     ####################################"
    except rospy.ROSInterruptException:
        pass
