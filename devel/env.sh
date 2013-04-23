#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/nishome/ejennings/ros/rosbuild_ws/class-code/friproject/devel', type 'exit' to leave"
  . "/nishome/ejennings/ros/rosbuild_ws/class-code/friproject/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/nishome/ejennings/ros/rosbuild_ws/class-code/friproject/devel'"
else
  . "/nishome/ejennings/ros/rosbuild_ws/class-code/friproject/devel/setup.sh"
  exec "$@"
fi
