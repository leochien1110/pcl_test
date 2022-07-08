# cmake_minimum_required(VERSION 3.10.2)
# project(pcl_test)

add_compile_options(-std=c++14)


# set(PCL_DIR //home/baoyuanjin/pcl-pcl-1.9.1)
find_package(PCL 1.9 REQUIRED)
if(NOT PCL_FOUND)
  message(FATAL_ERROR "PCL not found")
endif()
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
if(NOT Boost_FOUND)
  message(FATAL_ERROR "Boost not found")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_test pcl_test.cpp)
target_link_libraries (pcl_test ${PCL_LIBRARIES})

add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})

add_executable(pc_align pc_align.cpp)
target_link_libraries (pc_align ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pcd_filter pcd_filter.cpp)
target_link_libraries (pcd_filter ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(normal_distributions_transform normal_distributions_transform.cpp)
target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES})

add_executable (iterative_closest_point iterative_closest_point.cpp)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})

install(TARGETS pcl_test RUNTIME DESTINATION bin)