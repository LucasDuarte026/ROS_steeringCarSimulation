# Specify the minimum version of CMake required to build this package.
cmake_minimum_required(VERSION 3.0.2)

# Declare the project name. This should match the package name.
project(steering_car)

# Find catkin and other required ROS packages (components).
# These components are necessary for integrating with the Gazebo simulator and its control plugins.
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros_control
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
)

# This macro generates the necessary CMake configuration files for this package.
# It declares dependencies that other packages will need if they depend on this one.
catkin_package(
  # CATKIN_DEPENDS: Lists ROS packages needed at runtime by dependent packages.
  # We list the Gazebo dependencies here so any package using this one knows about them.
  CATKIN_DEPENDS
    gazebo_ros_control
    gazebo_msgs
    gazebo_plugins
    gazebo_ros
)

# Specify additional directories where header files can be found.
# ${catkin_INCLUDE_DIRS} includes the standard ROS include directories.
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Since this package only contains Python scripts, launch files, and models,
# there are no C++ libraries or executables to build.
# The 'install' section is also not needed as our launch files will find the Python
# scripts in the 'scripts' folder automatically, provided they are executable.