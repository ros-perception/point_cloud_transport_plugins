cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)

project(draco_point_cloud_transport)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(point_cloud_interfaces REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Draco REQUIRED)

set(dependencies
  pluginlib::pluginlib
  ${point_cloud_interfaces_TARGETS}
  point_cloud_transport::point_cloud_transport
  rclcpp::rclcpp
  rcpputils::rcpputils
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

include_directories(include)

add_library(${PROJECT_NAME}
  SHARED
  src/cloud.cpp
  src/draco_publisher.cpp
  src/draco_subscriber.cpp
  src/manifest.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${dependencies}
)

# draco 1.5.3 dropped the DRACO_LIBRARIES variable in favor of
# draco::draco imported target, but debian introduced DRACO_LIBRARIES
# back, but that is not present in other distributions. For maximum
# compatibility, we link DRACO_LIBRARIES if defined, otherwise draco::draco
# See:
# * https://github.com/google/draco/commit/7109fbee87c6d932cdc0ac7394bc5d485db26d44
# * https://sources.debian.org/patches/draco/1.5.6+dfsg-3/0004-Set-DRACO_LIBRARIES-for-backwards-compatibility.patch/
if(DEFINED DRACO_LIBRARIES)
  target_link_libraries(${PROJECT_NAME} PRIVATE
    ${DRACO_LIBRARIES}
  )
else()
  target_link_libraries(${PROJECT_NAME} PRIVATE
    draco::draco
  )
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${DRACO_INCLUDE_DIR}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(point_cloud_transport draco_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
