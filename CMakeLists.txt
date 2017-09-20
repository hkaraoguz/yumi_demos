cmake_minimum_required(VERSION 2.8.3)
project(yumi_demos)

#add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS moveit_msgs moveit_core moveit_ros_planning geometry_msgs rospy std_msgs yumi_hw

)

catkin_python_setup()


# Declare catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


