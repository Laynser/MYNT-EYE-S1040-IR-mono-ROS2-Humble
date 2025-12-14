cmake_minimum_required(VERSION 3.8)
project(mynt_eye)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# Force OpenCV 3.4 that matches /usr/local/lib/libmynteye.so.2
set(OPENCV34_ROOT "/opt/opencv-3.4.3")
include_directories(BEFORE ${OPENCV34_ROOT}/include)

find_library(OPENCV34_CORE NAMES opencv_core PATHS ${OPENCV34_ROOT}/lib NO_DEFAULT_PATH)
if(NOT OPENCV34_CORE)
  message(FATAL_ERROR "OpenCV 3.4 core not found in ${OPENCV34_ROOT}/lib (needed to match libmynteye)")
endif()

find_library(OPENCV34_IMGPROC NAMES opencv_imgproc PATHS ${OPENCV34_ROOT}/lib NO_DEFAULT_PATH)
# IMGPROC is optional here; core is usually enough for cv::Mat

add_executable(depth_imu_publisher src/depth_imu_publisher.cpp)

set_target_properties(depth_imu_publisher PROPERTIES
  BUILD_RPATH "${OPENCV34_ROOT}/lib"
  INSTALL_RPATH "${OPENCV34_ROOT}/lib"
)

ament_target_dependencies(depth_imu_publisher
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
)

target_link_libraries(depth_imu_publisher
  ${OPENCV34_CORE}
  ${PCL_LIBRARIES}
  mynteye
)

install(TARGETS depth_imu_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
