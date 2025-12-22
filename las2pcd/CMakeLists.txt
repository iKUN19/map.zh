cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(las2pcd)
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters)
find_package(PkgConfig)
pkg_check_modules(LIBLAS liblas)

include_directories(${PCL_INCLUDE_DIRS})

include_directories(/usr/include)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(las2pcd las2pcd.cpp)

target_link_libraries(las2pcd
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  las
  las_c
)

add_executable(las2pcd_intensity las2pcd_intensity.cpp)

target_link_libraries(las2pcd_intensity
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  las
  las_c
)

add_executable(pcd_enhancer pcd_enhancer.cpp)

target_link_libraries(pcd_enhancer 
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

