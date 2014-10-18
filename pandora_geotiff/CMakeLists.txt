cmake_minimum_required(VERSION 2.8.3)
project(pandora_geotiff)
find_package(catkin REQUIRED COMPONENTS cmake_modules pluginlib roscpp std_msgs message_generation)

find_package(Qt4 4.6 COMPONENTS QtCore QtGui  REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${Eigen_DEFINITIONS})

include(${QT_USE_FILE})



add_service_files(
   FILES
   SaveMission.srv
 )
 
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  pluginlib roscpp std_msgs
  LIBRARIES
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


add_library(geotiff_creator src/geotiff_creator.cpp)
target_link_libraries(geotiff_creator ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(geotiff_creator  ${catkin_EXPORTED_TARGETS})

#~ add_library( map_generator src/map_generator.cpp)
#~ target_link_libraries(map_generator  GeotiffCreator ${catkin_LIBRARIES} ${QT_LIBRARIES})
#~ add_dependencies( map_generator   GeotiffCreator${catkin_EXPORTED_TARGETS})
#~ 
#~ add_executable(geotiff_node src/pandora_geotiff_node.cpp)
#~ target_link_libraries(geotiff_node   
      #~ ${catkin_LIBRARIES}
      #~ GeotiffCreator
      #~ map_generator
      #~ )
#~ 
#~ add_executable(DataFusionSrvMock src/DataFusionSrvMock.cpp)
#~ target_link_libraries(DataFusionSrvMock ${catkin_LIBRARIES})
#~ 
