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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/fri/ros/rosbuild_ws/class-code/friproject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fri/ros/rosbuild_ws/class-code/friproject

# Include any dependencies generated for this target.
include CMakeFiles/QRCodeFinder.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/QRCodeFinder.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/QRCodeFinder.dir/flags.make

CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: CMakeFiles/QRCodeFinder.dir/flags.make
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: src/QRCodeFinder.cpp
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: manifest.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/geometry_msgs/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/sensor_msgs/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/catkin/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/opencv2/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/cv_bridge/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/message_filters/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/console_bridge/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/class_loader/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/rospack/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/roslib/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/pluginlib/package.xml
CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o: /opt/ros/groovy/share/image_transport/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/fri/ros/rosbuild_ws/class-code/friproject/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o -c /home/fri/ros/rosbuild_ws/class-code/friproject/src/QRCodeFinder.cpp

CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/fri/ros/rosbuild_ws/class-code/friproject/src/QRCodeFinder.cpp > CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.i

CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/fri/ros/rosbuild_ws/class-code/friproject/src/QRCodeFinder.cpp -o CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.s

CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o.requires:
.PHONY : CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o.requires

CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o.provides: CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o.requires
	$(MAKE) -f CMakeFiles/QRCodeFinder.dir/build.make CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o.provides.build
.PHONY : CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o.provides

CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o.provides.build: CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o

# Object files for target QRCodeFinder
QRCodeFinder_OBJECTS = \
"CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o"

# External object files for target QRCodeFinder
QRCodeFinder_EXTERNAL_OBJECTS =

bin/QRCodeFinder: CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o
bin/QRCodeFinder: CMakeFiles/QRCodeFinder.dir/build.make
bin/QRCodeFinder: CMakeFiles/QRCodeFinder.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/QRCodeFinder"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/QRCodeFinder.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/QRCodeFinder.dir/build: bin/QRCodeFinder
.PHONY : CMakeFiles/QRCodeFinder.dir/build

CMakeFiles/QRCodeFinder.dir/requires: CMakeFiles/QRCodeFinder.dir/src/QRCodeFinder.cpp.o.requires
.PHONY : CMakeFiles/QRCodeFinder.dir/requires

CMakeFiles/QRCodeFinder.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/QRCodeFinder.dir/cmake_clean.cmake
.PHONY : CMakeFiles/QRCodeFinder.dir/clean

CMakeFiles/QRCodeFinder.dir/depend:
	cd /home/fri/ros/rosbuild_ws/class-code/friproject && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fri/ros/rosbuild_ws/class-code/friproject /home/fri/ros/rosbuild_ws/class-code/friproject /home/fri/ros/rosbuild_ws/class-code/friproject /home/fri/ros/rosbuild_ws/class-code/friproject /home/fri/ros/rosbuild_ws/class-code/friproject/CMakeFiles/QRCodeFinder.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/QRCodeFinder.dir/depend

