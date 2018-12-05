#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
from yumi_demos import yumi_moveit_utils as yumi
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty



def run():
    """Starts the node

    Runs to start the node and initialize everthing. Runs forever via Spin()

    :returns: Nothing
    :rtype: None
    """
    rospy.init_node('moveit_yumi_resetpose')
    # Start by connecting to ROS and MoveIt!
    yumi.init_Moveit("/world")

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
