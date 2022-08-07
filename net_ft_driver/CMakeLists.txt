cmake_minimum_required(VERSION 3.8)
project(net_ft_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ASIO
  CURLPP 
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
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
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface net_ft_driver_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
