cmake_minimum_required(VERSION 3.16)
project(prior_map_localization)

# colcon passes no build type, and an unoptimized registration is several times slower.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(capa_core REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(flux_cpp REQUIRED)
find_package(flux_std_adapters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration search)
find_package(rclcpp REQUIRED)
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

add_executable(prior_map_matcher
  src/main.cpp
  src/prior_map_matcher.cpp
  src/registration.cpp
  src/scan.cpp)
target_include_directories(prior_map_matcher PRIVATE include ${flux_std_adapters_INCLUDE_DIRS})
target_link_libraries(prior_map_matcher
  capa_core::capa_core
  flux_cpp::flux_cpp
  ndt_omp
  small_gicp::small_gicp)
ament_target_dependencies(prior_map_matcher
  diagnostic_msgs geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2_eigen tf2_ros)
target_compile_features(prior_map_matcher PRIVATE cxx_std_17)
target_compile_options(prior_map_matcher PRIVATE -Wall -Wextra -Wpedantic)

add_executable(bbs_initialpose
  src/bbs_initialpose.cpp
  src/corner_features.cpp)
target_include_directories(bbs_initialpose PRIVATE include thirdparty ${flux_std_adapters_INCLUDE_DIRS})
target_link_libraries(bbs_initialpose flux_cpp::flux_cpp ${PCL_LIBRARIES})
ament_target_dependencies(bbs_initialpose
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2_eigen tf2_ros)
target_compile_features(bbs_initialpose PRIVATE cxx_std_17)
target_compile_options(bbs_initialpose PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS prior_map_matcher bbs_initialpose DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
foreach(node
    gps_cov_floor lio_twist_relay wheel_twist_relay)
  install(PROGRAMS ${PROJECT_NAME}/${node}.py DESTINATION lib/${PROJECT_NAME} RENAME ${node})
endforeach()
install(PROGRAMS scripts/prepare_map.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_registration test/test_registration.cpp src/registration.cpp)
  target_include_directories(test_registration PRIVATE include)
  target_link_libraries(test_registration ndt_omp small_gicp::small_gicp)
  ament_target_dependencies(test_registration rclcpp)
endif()

ament_package()
