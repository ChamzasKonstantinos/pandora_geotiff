cmake_minimum_required(VERSION 2.8.3)
project(pandora_geotiff_plugins)
find_package(catkin REQUIRED COMPONENTS pandora_geotiff pandora_data_fusion_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pandora_geotiff_plugins
#  CATKIN_DEPENDS pandora_geotiff pandora_nav_msgs
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_library(pandora_geotiff_plugins
  src/geotiff_map_plugin.cpp
  src/geotiff_coverage_plugin.cpp
  src/geotiff_path_plugin.cpp
  src/geotiff_objects_plugin.cpp
)
add_dependencies(pandora_geotiff_plugins   ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(pandora_geotiff_plugins 
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS pandora_geotiff_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  pandora_geotiff_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

