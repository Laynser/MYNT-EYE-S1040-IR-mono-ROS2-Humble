cmake_minimum_required(VERSION 3.8)
project(mynt_eye)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# MYNT EYE SDK должен быть установлен в системе (libmynteye.so + headers)
# При необходимости добавьте include/library dirs вручную.
# include_directories(/usr/local/include)
# link_directories(/usr/local/lib)

add_executable(depth_imu_publisher src/depth_imu_publisher.cpp)

ament_target_dependencies(depth_imu_publisher
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  pcl_conversions
)

target_link_libraries(depth_imu_publisher
  ${OpenCV_LIBS}
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
