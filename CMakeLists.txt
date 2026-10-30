cmake_minimum_required(VERSION 3.16)
project(prior_map_localization)

# colcon passes no build type, and an unoptimized registration is several times slower.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(flux_cpp REQUIRED)
find_package(flux_std_adapters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration search)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(small_gicp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

# ndt_omp is built here from the submodule; only the matcher links it.
add_library(ndt_omp STATIC
  thirdparty/ndt_omp/src/pclomp/gicp_omp.cpp
  thirdparty/ndt_omp/src/pclomp/ndt_omp.cpp
  thirdparty/ndt_omp/src/pclomp/voxel_grid_covariance_omp.cpp)
target_include_directories(ndt_omp SYSTEM PUBLIC thirdparty/ndt_omp/include ${PCL_INCLUDE_DIRS})
target_link_libraries(ndt_omp PUBLIC ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
set_target_properties(ndt_omp PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Every node is a component with a standalone <node>_node executable. The launch files load them
# into one capa_component_container; a standalone matcher or bbs_initialpose serves dds only.
add_library(prior_map_matcher_component SHARED
  src/prior_map_matcher.cpp
  src/registration.cpp
  src/scan.cpp)
target_include_directories(prior_map_matcher_component PRIVATE ${flux_std_adapters_INCLUDE_DIRS})
target_link_libraries(prior_map_matcher_component flux_cpp::flux_cpp ndt_omp small_gicp::small_gicp)
ament_target_dependencies(prior_map_matcher_component diagnostic_msgs geometry_msgs nav_msgs rclcpp
  rclcpp_components sensor_msgs std_msgs tf2_eigen tf2_ros)
rclcpp_components_register_node(prior_map_matcher_component
  PLUGIN "prior_map_localization::PriorMapMatcher" EXECUTABLE prior_map_matcher_node)

add_library(bbs_initialpose_component SHARED
  src/bbs_initialpose.cpp
  src/corner_features.cpp)
target_include_directories(bbs_initialpose_component PRIVATE thirdparty
  ${flux_std_adapters_INCLUDE_DIRS})
target_link_libraries(bbs_initialpose_component flux_cpp::flux_cpp ${PCL_LIBRARIES})
ament_target_dependencies(bbs_initialpose_component geometry_msgs nav_msgs rclcpp rclcpp_components
  sensor_msgs std_msgs tf2_eigen tf2_ros)
rclcpp_components_register_node(bbs_initialpose_component
  PLUGIN "prior_map_localization::BbsInitialPose" EXECUTABLE bbs_initialpose_node)

add_library(lio_twist_relay_component SHARED
  src/gyro_bias.cpp
  src/lio_twist_relay.cpp)
ament_target_dependencies(lio_twist_relay_component geometry_msgs nav_msgs rclcpp rclcpp_components
  sensor_msgs tf2_ros)
rclcpp_components_register_node(lio_twist_relay_component
  PLUGIN "prior_map_localization::LioTwistRelay" EXECUTABLE lio_twist_relay_node)

add_library(wheel_twist_relay_component SHARED src/wheel_twist_relay.cpp)
ament_target_dependencies(wheel_twist_relay_component geometry_msgs rclcpp rclcpp_components std_msgs)
rclcpp_components_register_node(wheel_twist_relay_component
  PLUGIN "prior_map_localization::WheelTwistRelay" EXECUTABLE wheel_twist_relay_node)

add_library(gps_cov_floor_component SHARED src/gps_cov_floor.cpp)
ament_target_dependencies(gps_cov_floor_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(gps_cov_floor_component
  PLUGIN "prior_map_localization::GpsCovFloor" EXECUTABLE gps_cov_floor_node)

set(components prior_map_matcher_component bbs_initialpose_component lio_twist_relay_component
  wheel_twist_relay_component gps_cov_floor_component)
foreach(target ${components})
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()
install(TARGETS ${components}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(PROGRAMS scripts/prepare_map.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_registration test/test_registration.cpp src/registration.cpp)
  target_include_directories(test_registration PRIVATE src)
  target_link_libraries(test_registration ndt_omp small_gicp::small_gicp)

  ament_add_gtest(test_scan test/test_scan.cpp src/scan.cpp)
  target_include_directories(test_scan PRIVATE src)
  target_link_libraries(test_scan ${PCL_LIBRARIES})
  target_include_directories(test_scan SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

  ament_add_gtest(test_corner_features test/test_corner_features.cpp src/corner_features.cpp)
  target_include_directories(test_corner_features PRIVATE src)
  target_include_directories(test_corner_features SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

  ament_add_gtest(test_gyro_bias test/test_gyro_bias.cpp src/gyro_bias.cpp)
  target_include_directories(test_gyro_bias PRIVATE src)
endif()

ament_package()
