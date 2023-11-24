cmake_minimum_required(VERSION 3.5)
project(mw_ahrs_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(
  DEPENDENCIES
  rclcpp
  rclcpp_lifecycle
  tf2
  geometry_msgs
  sensor_msgs
  std_srvs
  realtime_tools
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add serial library
add_library(serial
  src/serial/serial.cc
  include/serial/serial.h
  include/serial/v8stdint.h
)

if(APPLE) # macOS
  find_library(IOKIT_LIBRARY IOKit)
  find_library(FOUNDATION_LIBRARY Foundation)
  target_sources(serial PRIVATE
    src/serial/impl/unix.cc
    src/serial/impl/list_ports/list_ports_osx.cc
  )
  target_link_libraries(serial ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX) # .*nix
  target_sources(serial PRIVATE
    src/serial/impl/unix.cc
    src/serial/impl/list_ports/list_ports_linux.cc
  )
  target_link_libraries(serial rt pthread)
elseif(WIN32) # Windows
  target_sources(serial PRIVATE
    src/serial/impl/win.cc
    src/serial/impl/list_ports/list_ports_win.cc
  )
  target_link_libraries(serial setupapi)
  ament_export_libraries(setupapi)
endif()

target_include_directories(serial PRIVATE include)

add_executable(
  mw_ahrs_driver_node 
  src/mw_ahrs_driver.cpp
)
target_link_libraries(mw_ahrs_driver_node serial)
target_include_directories(mw_ahrs_driver_node PRIVATE include)
ament_target_dependencies(mw_ahrs_driver_node ${DEPENDENCIES})

install(
  DIRECTORY launch rules
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS mw_ahrs_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_package()