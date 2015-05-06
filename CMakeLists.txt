cmake_minimum_required(VERSION 2.8.3)
project(tactile_marker_publisher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  rostopic
  visualization_msgs
  dynamic_reconfigure
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# create classes for dynamic configuration
generate_dynamic_reconfigure_options(src/tactile_marker_publisher/TactileValue.cfg)

catkin_package()

catkin_install_python(PROGRAMS scripts/tactile_marker_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
