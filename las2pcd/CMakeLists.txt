cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(las2pcd)
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters)
# find_package(LAS REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

include_directories(/usr/include)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(las2pcd las2pcd.cpp)

target_link_libraries(las2pcd 
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  # ${LAS_LIBRARIES}
  /usr/local/lib/liblas.so
  /usr/local/lib/liblas_c.so
)

add_executable(las2pcd_intensity las2pcd_intensity.cpp)

target_link_libraries(las2pcd_intensity 
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  # ${LAS_LIBRARIES}
  /usr/local/lib/liblas.so
  /usr/local/lib/liblas_c.so
)

add_executable(pcd_enhancer pcd_enhancer.cpp)

target_link_libraries(pcd_enhancer 
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

