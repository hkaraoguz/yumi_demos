cmake_minimum_required(VERSION 2.8.3)
project(yumi_demos)

#add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS moveit_msgs moveit_core moveit_ros_planning geometry_msgs rospy std_msgs yumi_hw message_generation trajectory_msgs

)

catkin_python_setup()


## Generate services in the 'srv' folder
 add_service_files(
  FILES
  PlanforAction.srv
#   Service2.srv
)

generate_messages(
  DEPENDENCIES
  moveit_msgs   geometry_msgs   std_msgs
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
