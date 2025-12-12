#!/usr/bin/env bash

OpenCV_VERSION=4.5.4
OpenCV_VERSION_MAJOR=4
OpenCV_VERSION_MINOR=5
OpenCV_VERSION_PATCH=4
OpenCV_VERSION_STATUS=

_contains() {
  [ `echo $1 | grep -c "$2"` -gt 0 ]
}

if _contains "/usr/include/opencv4" "/ros/"; then
  ROS_VERSION=$(rosversion -d)
  OpenCV_VERSION=ros-$ROS_VERSION
fi
