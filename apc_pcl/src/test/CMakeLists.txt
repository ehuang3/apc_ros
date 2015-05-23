cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# find_package(PCL 1.3 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
set(CMAKE_BUILD_TYPE Debug)
add_executable(test_transform test_transform.cpp 
    ../pcl_tools/transform.cpp
    ../pcl_tools/loading.cpp
    ../pcl_tools/registration.cpp
    ../pcl_tools/visualization.cpp
)
target_link_libraries(test_transform ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_closest_pt test_closest_point.cpp
    ../pcl_tools/transform.cpp
    ../pcl_tools/loading.cpp
    ../pcl_tools/registration.cpp
    ../pcl_tools/visualization.cpp
)

target_link_libraries(test_closest_pt ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_sac_icp test_sac_icp.cpp
../pcl_tools/transform.cpp
../pcl_tools/loading.cpp
../pcl_tools/registration.cpp
../pcl_tools/adv_registration.cpp
../pcl_tools/visualization.cpp
)
target_link_libraries(test_sac_icp ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_apr alp_2.cpp
../pcl_tools/transform.cpp
../pcl_tools/loading.cpp
../pcl_tools/registration.cpp
../pcl_tools/visualization.cpp
)
target_link_libraries(test_apr ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_corresp_grp corresp_grouping.cpp
../pcl_tools/transform.cpp
../pcl_tools/loading.cpp
../pcl_tools/registration.cpp
../pcl_tools/visualization.cpp
)
target_link_libraries(test_corresp_grp ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
