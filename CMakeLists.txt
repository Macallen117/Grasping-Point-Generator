cmake_minimum_required(VERSION 3.16)
project(pgfat)

add_compile_options(-std=c++14)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
message("${CMAKE_INSTALL_PREFIX}/include")
message("${CMAKE_MODULE_PATH}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib 
  std_msgs
)
find_package(PCL REQUIRED)
if (PCL_FOUND)
  message("pcl found!")
  include_directories(${PCL_INCLUDE_DIR})
endif (PCL_FOUND)

find_package(Eigen3 REQUIRED)
if (EIGEN3_FOUND)
  message("eigen found!")
  include_directories(${EIGEN3_INCLUDE_DIR})
endif (EIGEN3_FOUND)

set(ENV{PKG_CONFIG_PATH} "$ENV{HOME}/local/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
find_package(PkgConfig)
pkg_check_modules(pkg_fcl REQUIRED fcl>=0.6.0)

#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#find_package(FCL REQUIRED)
#if (FCL_FOUND)
  #message("fcl found!")
  #include_directories(${pkg_fcl_INCLUDE_DIRS})
#endif (FCL_FOUND)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}   
  CATKIN_DEPENDS roscpp std_msgs   
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}_lib
  src/visualization.cpp
  src/mesh_sampling.cpp 
  src/grasp_point_generator.cpp
  src/geometrics.cpp
  src/data_transform.cpp
  src/Mesh_preprocessing.cpp 
)

add_executable(${PROJECT_NAME} 
  src/pgfat.cpp 
)

add_dependencies(${PROJECT_NAME}
 ${${PROJECT_NAME}_EXPORTED_TARGETS} 
 ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_lib
 ${${PROJECT_NAME}_EXPORTED_TARGETS} 
 ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${pkg_fcl_LIBRARIES}
  #fcl
  yaml-cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${pkg_fcl_LIBRARIES}
  #fcl
  yaml-cpp
  ${PROJECT_NAME}_lib
)


install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)
