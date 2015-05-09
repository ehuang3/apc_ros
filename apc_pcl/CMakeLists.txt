cmake_minimum_required(VERSION 2.8.3)
project(apc_pcl)

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostime
)

find_package(PCL 1.7 REQUIRED)

catkin_python_setup()

set(catkin_LIBRARIES 
    ${catkin_LIBRARIES}
)

catkin_package(
 CATKIN_DEPENDS roscpp
 DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory (src)

# add_executable(test_icp src/test/test_icp.cpp)
# target_link_libraries(test_icp ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

# add_executable(icp_node nodes/icp_node.cpp)
# target_link_libraries(icp_node ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(icp_node apc_msgs)

add_executable(frustum_server nodes/frustum_cull_server.cpp
    ./src/pcl_tools/transform.cpp
    ./src/pcl_tools/loading.cpp
    ./src/pcl_tools/registration.cpp
    ./src/pcl_tools/visualization.cpp
)
target_link_libraries(frustum_server ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(frustum_server apc_msgs)



install(
  TARGETS
)

