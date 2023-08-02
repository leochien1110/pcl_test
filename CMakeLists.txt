cmake_minimum_required (VERSION 3.5)

set (CMAKE_CXX_STANDARD 17)

# The project name
project (pcl_test)

add_subdirectory(res)

include_directories(include)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5Widgets 5.15 REQUIRED)
set(QTX Qt5)
# find_package(Qt6 QUIET COMPONENTS Concurrent OpenGL Widgets)
# if (NOT Qt6_FOUND)
#   find_package(Qt5 COMPONENTS Concurrent OpenGL Widgets REQUIRED)
#   set(QTX Qt5)
# else()
#   set(QTX Qt6)
# endif()
message(STATUS "Found QtX: ${QTX}")

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(VTK 9.2.6 REQUIRED)
find_package(PCL 1.13.1 REQUIRED)
message(STATUS "VTK version: ${VTK_VERSION}")
message(STATUS "PCL version: ${PCL_VERSION}")

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# executables
add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries (pcl_test ${PCL_LIBRARIES})

add_executable (pcl_visualizer_demo src/pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})

add_executable(pcl_coordinate src/pcl_coordinate.cpp)
target_link_libraries(pcl_coordinate ${PCL_LIBRARIES})

add_executable(pc_align src/pc_align.cpp)
target_link_libraries (pc_align ${PCL_LIBRARIES} Boost::program_options)

add_executable(pcd_filter src/pcd_filter.cpp)
target_link_libraries (pcd_filter ${PCL_LIBRARIES} Boost::program_options)

add_executable(normal_distributions_transform src/normal_distributions_transform.cpp)
target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES})

add_executable (iterative_closest_point src/iterative_closest_point.cpp)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})

# add_executable(pclviewer_qt src/pclviewer_qt.cpp src/pclviewer.cpp include/pclviewer.h)
# target_link_libraries(pclviewer_qt 
#     ${PCL_LIBRARIES} 
#     ${QTX}::Widgets)

# put the executable into the bin folder
set_target_properties(pcl_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set_target_properties(pcl_visualizer_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set_target_properties(pcl_coordinate PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set_target_properties(pc_align PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set_target_properties(pcd_filter PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set_target_properties(normal_distributions_transform PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set_target_properties(iterative_closest_point PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# set_target_properties(pclviewer_qt PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)