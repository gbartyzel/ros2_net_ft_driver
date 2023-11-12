cmake_minimum_required(VERSION 3.8)
project(net_ft_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ASIO
  CURLPP
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
  TinyXML2
)

find_package(ament_cmake REQUIRED)
find_package(asio_cmake_module REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  if(${Dependency} STREQUAL "CURLPP")
    include(FindPkgConfig)
    pkg_check_modules(${Dependency} REQUIRED curlpp)
  else()
    find_package(${Dependency} REQUIRED)
  endif()
endforeach()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/hardware_interface.cpp
  src/interfaces/net_ft_interface.cpp
  src/interfaces/ati_ft_interface.cpp
  src/interfaces/ati_axia_ft_interface.cpp
  src/interfaces/onrobot_ft_interface.cpp
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
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)


add_executable(tared_ft_broadcaster src/tared_ft_broadcaster_node.cpp src/tared_ft_broadcaster_ros.cpp)
ament_target_dependencies(tared_ft_broadcaster ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(
  tared_ft_broadcaster
  PRIVATE
  include
)

install(
  TARGETS tared_ft_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
