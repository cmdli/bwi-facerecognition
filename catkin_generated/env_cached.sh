#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/nishome/cmdli/ros/rosbuild_ws/class-code/friproject/catkin_generated', type 'exit' to leave"
  . "/nishome/cmdli/ros/rosbuild_ws/class-code/friproject/catkin_generated/setup_cached.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/nishome/cmdli/ros/rosbuild_ws/class-code/friproject/catkin_generated'"
else
  . "/nishome/cmdli/ros/rosbuild_ws/class-code/friproject/catkin_generated/setup_cached.sh"
  exec "$@"
fi
