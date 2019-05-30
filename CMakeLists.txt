cmake_minimum_required(VERSION 3.5)
project(interactive_markers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/interactive_marker_server.cpp
  src/tools.cpp
  src/menu_handler.cpp
  src/interactive_marker_client.cpp
  src/single_client.cpp
  src/message_context.cpp
)
set(dependencies
  rclcpp
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include/)

# C++ Unit Tests
#
#if(CATKIN_ENABLE_TESTING)
#  include_directories(${GTEST_INCLUDE_DIRS})
#
#  add_executable(server_test EXCLUDE_FROM_ALL src/test/server_test.cpp)
#  target_link_libraries(server_test ${PROJECT_NAME} ${GTEST_LIBRARIES})
#  add_dependencies(tests server_test)
#  add_rostest(test/cpp_server.test)

#  add_executable(client_test EXCLUDE_FROM_ALL src/test/client_test.cpp)
#  target_link_libraries(client_test ${PROJECT_NAME} ${GTEST_LIBRARIES})
#  add_dependencies(tests client_test)
#  add_rostest(test/cpp_client.test)

#  add_executable(server_client_test EXCLUDE_FROM_ALL src/test/server_client_test.cpp)
#  target_link_libraries(server_client_test ${PROJECT_NAME} ${GTEST_LIBRARIES})
#  add_dependencies(tests server_client_test)
#  add_rostest(test/cpp_server_client.test)

#  # Test program to simulate Interactive Marker with missing tf information
#  add_executable(bursty_tf EXCLUDE_FROM_ALL src/test/bursty_tf.cpp)
#  target_link_libraries(bursty_tf ${PROJECT_NAME})
#  add_dependencies(tests bursty_tf)

#  # Test program to simulate Interactive Marker with wrong tf information
#  add_executable(missing_tf EXCLUDE_FROM_ALL src/test/missing_tf.cpp)
#  target_link_libraries(missing_tf ${PROJECT_NAME})
#  add_dependencies(tests missing_tf)
#endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()