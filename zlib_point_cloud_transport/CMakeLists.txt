cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)

project(zlib_point_cloud_transport)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(point_cloud_interfaces REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ZLIB REQUIRED)

set(dependencies
  pluginlib
  point_cloud_interfaces
  point_cloud_transport
  rclcpp
)


include_directories(include)

add_library(${PROJECT_NAME}
  SHARED
  src/zlib_publisher.cpp
  src/zlib_subscriber.cpp
  src/zlib_cpp.cpp
  src/manifest.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  pluginlib::pluginlib
  ${point_cloud_interfaces_TARGETS}
  point_cloud_transport::point_cloud_transport
  rclcpp::rclcpp
  ZLIB::ZLIB
)

target_include_directories(${PROJECT_NAME} PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
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

pluginlib_export_plugin_description_file(point_cloud_transport zlib_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
