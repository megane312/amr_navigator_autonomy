cmake_minimum_required(VERSION 3.8)
project(amr_navigator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(om_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters octree)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(odom_to_pose src/odom_to_pose.cpp)
ament_target_dependencies(odom_to_pose rclcpp nav_msgs geometry_msgs)

add_executable(waypoint_recorder src/waypoint_recorder.cpp)
ament_target_dependencies(waypoint_recorder rclcpp geometry_msgs visualization_msgs)

add_executable(costmap_load src/costmap_load.cpp)
ament_target_dependencies(costmap_load rclcpp)

add_executable(draw_line src/draw_line.cpp)
ament_target_dependencies(draw_line rclcpp geometry_msgs std_msgs visualization_msgs)

add_executable(compare_pcd src/compare_pcd.cpp)
ament_target_dependencies(compare_pcd
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(compare_pcd ${PCL_LIBRARIES})

add_executable(detection_close_point src/detection_close_point.cpp)
ament_target_dependencies(detection_close_point
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(detection_close_point ${PCL_LIBRARIES})

add_executable(evalute_safety src/evalute_safety.cpp)
ament_target_dependencies(evalute_safety rclcpp geometry_msgs std_msgs)

add_executable(twist_to_radius src/twist_to_radius.cpp)
ament_target_dependencies(twist_to_radius rclcpp geometry_msgs std_msgs)

install(TARGETS
  odom_to_pose
  waypoint_recorder
  costmap_load
  draw_line
  compare_pcd
  detection_close_point
  evalute_safety
  twist_to_radius
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/init_pose_rclpy.py
  scripts/om_odom_tf.py
  scripts/joy_idshare_drive.py
  scripts/waypoint_mission.py
  scripts/waypoint_follower.py
  scripts/obstacle_stop_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY path
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
