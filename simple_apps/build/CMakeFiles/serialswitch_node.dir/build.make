# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robot/ros/elektron_ballcollector/simple_apps

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/ros/elektron_ballcollector/simple_apps/build

# Include any dependencies generated for this target.
include CMakeFiles/serialswitch_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/serialswitch_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/serialswitch_node.dir/flags.make

CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: CMakeFiles/serialswitch_node.dir/flags.make
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: ../src/serialswitch_node.cpp
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: ../manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/diagnostic_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/stacks/diagnostics/diagnostic_updater/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/rosbag/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/stacks/joystick_drivers/joy/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/nav_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /home/robot/ros/elektron_robot/elektron_base/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/std_srvs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/orocos_kdl/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/python_orocos_kdl/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/kdl/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /home/robot/ros/RCPRG_laser_drivers/libLMS1xx/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /home/robot/ros/RCPRG_laser_drivers/LMS1xx/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /home/robot/ros/elektron_robot/elektron_sensors/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/actionlib_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/actionlib/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/stacks/navigation/move_base_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o: /opt/ros/fuerte/stacks/navigation/move_base_msgs/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robot/ros/elektron_ballcollector/simple_apps/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o -c /home/robot/ros/elektron_ballcollector/simple_apps/src/serialswitch_node.cpp

CMakeFiles/serialswitch_node.dir/src/serialswitch_node.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serialswitch_node.dir/src/serialswitch_node.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/robot/ros/elektron_ballcollector/simple_apps/src/serialswitch_node.cpp > CMakeFiles/serialswitch_node.dir/src/serialswitch_node.i

CMakeFiles/serialswitch_node.dir/src/serialswitch_node.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serialswitch_node.dir/src/serialswitch_node.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/robot/ros/elektron_ballcollector/simple_apps/src/serialswitch_node.cpp -o CMakeFiles/serialswitch_node.dir/src/serialswitch_node.s

CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o.requires:
.PHONY : CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o.requires

CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o.provides: CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o.requires
	$(MAKE) -f CMakeFiles/serialswitch_node.dir/build.make CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o.provides.build
.PHONY : CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o.provides

CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o.provides.build: CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o

CMakeFiles/serialswitch_node.dir/src/serialswitch.o: CMakeFiles/serialswitch_node.dir/flags.make
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: ../src/serialswitch.cpp
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: ../manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/diagnostic_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/stacks/diagnostics/diagnostic_updater/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/rosbag/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/stacks/joystick_drivers/joy/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/nav_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /home/robot/ros/elektron_robot/elektron_base/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/std_srvs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/orocos_kdl/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/python_orocos_kdl/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/kdl/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /home/robot/ros/RCPRG_laser_drivers/libLMS1xx/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /home/robot/ros/RCPRG_laser_drivers/LMS1xx/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /home/robot/ros/elektron_robot/elektron_sensors/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/actionlib_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/actionlib/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/stacks/navigation/move_base_msgs/manifest.xml
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
CMakeFiles/serialswitch_node.dir/src/serialswitch.o: /opt/ros/fuerte/stacks/navigation/move_base_msgs/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robot/ros/elektron_ballcollector/simple_apps/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/serialswitch_node.dir/src/serialswitch.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/serialswitch_node.dir/src/serialswitch.o -c /home/robot/ros/elektron_ballcollector/simple_apps/src/serialswitch.cpp

CMakeFiles/serialswitch_node.dir/src/serialswitch.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serialswitch_node.dir/src/serialswitch.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/robot/ros/elektron_ballcollector/simple_apps/src/serialswitch.cpp > CMakeFiles/serialswitch_node.dir/src/serialswitch.i

CMakeFiles/serialswitch_node.dir/src/serialswitch.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serialswitch_node.dir/src/serialswitch.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/robot/ros/elektron_ballcollector/simple_apps/src/serialswitch.cpp -o CMakeFiles/serialswitch_node.dir/src/serialswitch.s

CMakeFiles/serialswitch_node.dir/src/serialswitch.o.requires:
.PHONY : CMakeFiles/serialswitch_node.dir/src/serialswitch.o.requires

CMakeFiles/serialswitch_node.dir/src/serialswitch.o.provides: CMakeFiles/serialswitch_node.dir/src/serialswitch.o.requires
	$(MAKE) -f CMakeFiles/serialswitch_node.dir/build.make CMakeFiles/serialswitch_node.dir/src/serialswitch.o.provides.build
.PHONY : CMakeFiles/serialswitch_node.dir/src/serialswitch.o.provides

CMakeFiles/serialswitch_node.dir/src/serialswitch.o.provides.build: CMakeFiles/serialswitch_node.dir/src/serialswitch.o

# Object files for target serialswitch_node
serialswitch_node_OBJECTS = \
"CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o" \
"CMakeFiles/serialswitch_node.dir/src/serialswitch.o"

# External object files for target serialswitch_node
serialswitch_node_EXTERNAL_OBJECTS =

../bin/serialswitch_node: CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o
../bin/serialswitch_node: CMakeFiles/serialswitch_node.dir/src/serialswitch.o
../bin/serialswitch_node: CMakeFiles/serialswitch_node.dir/build.make
../bin/serialswitch_node: CMakeFiles/serialswitch_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/serialswitch_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serialswitch_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/serialswitch_node.dir/build: ../bin/serialswitch_node
.PHONY : CMakeFiles/serialswitch_node.dir/build

CMakeFiles/serialswitch_node.dir/requires: CMakeFiles/serialswitch_node.dir/src/serialswitch_node.o.requires
CMakeFiles/serialswitch_node.dir/requires: CMakeFiles/serialswitch_node.dir/src/serialswitch.o.requires
.PHONY : CMakeFiles/serialswitch_node.dir/requires

CMakeFiles/serialswitch_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/serialswitch_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/serialswitch_node.dir/clean

CMakeFiles/serialswitch_node.dir/depend:
	cd /home/robot/ros/elektron_ballcollector/simple_apps/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/ros/elektron_ballcollector/simple_apps /home/robot/ros/elektron_ballcollector/simple_apps /home/robot/ros/elektron_ballcollector/simple_apps/build /home/robot/ros/elektron_ballcollector/simple_apps/build /home/robot/ros/elektron_ballcollector/simple_apps/build/CMakeFiles/serialswitch_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/serialswitch_node.dir/depend

