cmake_minimum_required(VERSION 3.8)
project(net_ft_diagnostic_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    diagnostic_msgs
    diagnostic_updater
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/net_ft_diagnostic_broadcaster.cpp
)
target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
    include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
