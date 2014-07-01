cmake_minimum_required(VERSION 2.8.3)
project(pandora_geotiff)
find_package(catkin REQUIRED COMPONENTS cmake_modules pluginlib roscpp std_msgs)

find_package(Qt4 4.6 COMPONENTS QtCore QtGui REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${Eigen_DEFINITIONS})

include(${QT_USE_FILE})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  pluginlib roscpp std_msgs
  DEPENDS Eigen QT
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include include/pandora_geotiff)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(GeotiffCreator src/GeotiffCreator.cpp)
target_link_libraries(GeotiffCreator ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(GeotiffCreator ${catkin_EXPORTED_TARGETS})

#~ add_executable(geotiff_saver src/geotiff_saver.cpp)
#~ target_link_libraries(geotiff_saver geotiff_writer)
#~ add_dependencies(geotiff_saver ${catkin_EXPORTED_TARGETS})


