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
CMAKE_SOURCE_DIR = /nishome/ejennings/ros/rosbuild_ws/class-code/friproject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /nishome/ejennings/ros/rosbuild_ws/class-code/friproject

# Include any dependencies generated for this target.
include CMakeFiles/SIFTDetector.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SIFTDetector.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SIFTDetector.dir/flags.make

CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: CMakeFiles/SIFTDetector.dir/flags.make
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: src/SIFTDetector.cpp
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: manifest.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/geometry_msgs/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/sensor_msgs/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/catkin/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/opencv2/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/cv_bridge/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/message_filters/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/console_bridge/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/class_loader/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/rospack/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/roslib/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/pluginlib/package.xml
CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o: /opt/ros/groovy/share/image_transport/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /nishome/ejennings/ros/rosbuild_ws/class-code/friproject/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o -c /nishome/ejennings/ros/rosbuild_ws/class-code/friproject/src/SIFTDetector.cpp

CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /nishome/ejennings/ros/rosbuild_ws/class-code/friproject/src/SIFTDetector.cpp > CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.i

CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /nishome/ejennings/ros/rosbuild_ws/class-code/friproject/src/SIFTDetector.cpp -o CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.s

CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o.requires:
.PHONY : CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o.requires

CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o.provides: CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/SIFTDetector.dir/build.make CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o.provides.build
.PHONY : CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o.provides

CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o.provides.build: CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o

# Object files for target SIFTDetector
SIFTDetector_OBJECTS = \
"CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o"

# External object files for target SIFTDetector
SIFTDetector_EXTERNAL_OBJECTS =

bin/SIFTDetector: CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o
bin/SIFTDetector: CMakeFiles/SIFTDetector.dir/build.make
bin/SIFTDetector: CMakeFiles/SIFTDetector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/SIFTDetector"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SIFTDetector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SIFTDetector.dir/build: bin/SIFTDetector
.PHONY : CMakeFiles/SIFTDetector.dir/build

CMakeFiles/SIFTDetector.dir/requires: CMakeFiles/SIFTDetector.dir/src/SIFTDetector.cpp.o.requires
.PHONY : CMakeFiles/SIFTDetector.dir/requires

CMakeFiles/SIFTDetector.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SIFTDetector.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SIFTDetector.dir/clean

CMakeFiles/SIFTDetector.dir/depend:
	cd /nishome/ejennings/ros/rosbuild_ws/class-code/friproject && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /nishome/ejennings/ros/rosbuild_ws/class-code/friproject /nishome/ejennings/ros/rosbuild_ws/class-code/friproject /nishome/ejennings/ros/rosbuild_ws/class-code/friproject /nishome/ejennings/ros/rosbuild_ws/class-code/friproject /nishome/ejennings/ros/rosbuild_ws/class-code/friproject/CMakeFiles/SIFTDetector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SIFTDetector.dir/depend

