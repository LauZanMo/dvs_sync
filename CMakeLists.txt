cmake_minimum_required(VERSION 3.0.2)
project(dvs_sync)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
if(CMAKE_BUILD_TYPE MATCHES "Release")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# build dvs_sync library
include_directories(
    ${PROJECT_SOURCE_DIR}/src/lib
)
file(GLOB_RECURSE DVS_SYNC_SOURCE src/lib/*.cpp)
add_library(${PROJECT_NAME} SHARED
    ${DVS_SYNC_SOURCE}
)

# yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

# abseil
set(ABSL_PROPAGATE_CXX_STD true)
add_subdirectory(thirdparty/abseil-cpp abseil-cpp)
target_link_libraries(${PROJECT_NAME}
    absl::strings
    absl::str_format
)

# TBB
find_package(TBB REQUIRED)
target_link_libraries(${PROJECT_NAME} TBB::tbb)

# boost
find_package(Boost REQUIRED COMPONENTS filesystem)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# MetavisionSDK
find_package(MetavisionSDK COMPONENTS driver REQUIRED)
target_link_libraries(${PROJECT_NAME} MetavisionSDK::driver)

# ROS
find_package(catkin REQUIRED COMPONENTS
    dvs_msgs
    sensor_msgs
    roscpp
    serial
    rosbag
    geometry_msgs
)
catkin_package(
    INCLUDE_DIRS src/lib
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS dvs_msgs sensor_msgs roscpp serial rosbag geometry_msgs
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# build ros node
add_executable(${PROJECT_NAME}_node src/dvs_sync_node.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# build utils
add_executable(rosbag_imu_fix utils/rosbag_imu_fix.cpp)
target_link_libraries(rosbag_imu_fix ${PROJECT_NAME})
add_dependencies(rosbag_imu_fix ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})