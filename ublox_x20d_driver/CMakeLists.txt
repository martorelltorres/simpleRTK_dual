cmake_minimum_required(VERSION 3.0.2)
project(ublox_x20d_driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  geometry_msgs
  nmea_msgs
  roscpp
  rtcm_msgs
  sensor_msgs
  ublox_x20d_msgs
)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core ${PROJECT_NAME}_ros
  CATKIN_DEPENDS diagnostic_updater geometry_msgs nmea_msgs roscpp rtcm_msgs sensor_msgs ublox_x20d_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Protocol core and serial transport, no ROS dependencies.
add_library(${PROJECT_NAME}_core
  src/raw_logger.cpp
  src/serial_transport.cpp
  src/ubx_framer.cpp
)
target_link_libraries(${PROJECT_NAME}_core ${Boost_LIBRARIES} Threads::Threads)

# ROS message publishing shared by the driver and the .ubx file player.
add_library(${PROJECT_NAME}_ros
  src/nav_publisher.cpp
)
add_dependencies(${PROJECT_NAME}_ros ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_ros ${catkin_LIBRARIES})

add_executable(ublox_x20d_node src/ublox_x20d_node.cpp)
target_link_libraries(ublox_x20d_node ${PROJECT_NAME}_ros ${PROJECT_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ubx_file_player_node src/ubx_file_player_node.cpp)
target_link_libraries(ubx_file_player_node ${PROJECT_NAME}_ros ${PROJECT_NAME}_core ${catkin_LIBRARIES})

add_executable(heading_imu_node src/heading_imu_node.cpp)
add_dependencies(heading_imu_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(heading_imu_node ${catkin_LIBRARIES})

install(TARGETS ublox_x20d_node heading_imu_node ubx_file_player_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/check_recording.py scripts/make_test_ubx.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/replay_ubx.test)
  add_rostest(test/replay_bag.test)
  catkin_add_nosetests(test/test_check_recording.py)

  catkin_add_gtest(test_heading_conversion test/test_heading_conversion.cpp)
  catkin_add_gtest(test_ubx_framer test/test_ubx_framer.cpp)
  target_link_libraries(test_ubx_framer ${PROJECT_NAME}_core)
  catkin_add_gtest(test_ubx_parsers test/test_ubx_parsers.cpp)
  catkin_add_gtest(test_cfg_valset_valget test/test_cfg_valset_valget.cpp)
  catkin_add_gtest(test_config_engine test/test_config_engine.cpp)
  catkin_add_gtest(test_serial_transport test/test_serial_transport.cpp)
  target_link_libraries(test_serial_transport ${PROJECT_NAME}_core util)
  catkin_add_gtest(test_nav_conversion test/test_nav_conversion.cpp)
  catkin_add_gtest(test_heading_filter test/test_heading_filter.cpp)
  catkin_add_gtest(test_nmea_gga test/test_nmea_gga.cpp)
  catkin_add_gtest(test_raw_logger test/test_raw_logger.cpp)
  target_link_libraries(test_raw_logger ${PROJECT_NAME}_core)
endif()
