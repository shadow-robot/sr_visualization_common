cmake_minimum_required(VERSION 2.8.3)
project(sr_gui_joint_slider)
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py controller_manager_msgs sr_robot_msgs std_msgs sr_hand sr_visualization_common_icons)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy rqt_gui rqt_gui_py controller_manager_msgs sr_robot_msgs std_msgs sr_hand sr_visualization_common_icons
    INCLUDE_DIRS
    LIBRARIES
)
catkin_python_setup()

install( DIRECTORY uis DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install( FILES sr_joint_slider_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
