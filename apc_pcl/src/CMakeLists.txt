cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(MY_GRAND_PROJECT)


# find_package(PCL 1.7 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

add_subdirectory (pcl_tools)
add_subdirectory (test)
