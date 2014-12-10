find_package(catkin REQUIRED COMPONENTS
  rostest
  roslib 
  map_server
  )

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

link_directories(${catkin_LIBRARY_DIRS})
include_directories(map_loader/include ${catkin_INCLUDE_DIRS})

########################  map loader  ###############################

add_library(map_loader map_loader/src/map_loader.cpp)
target_link_libraries(map_loader 
  image_loader
  yaml-cpp
  ${catkin_LIBRARIES}
  )
########################  Unit Tests  ###############################

########################  geotiff_creator  ####################################

catkin_add_gtest(geotiff_creator_test unit/geotiff_creator_test.cpp)
target_link_libraries(geotiff_creator_test 
  ${catkin_LIBRARIES} 
  geotiff_creator
  map_loader
  gtest_main
  )


