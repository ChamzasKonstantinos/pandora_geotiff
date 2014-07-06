#include "GeotiffCreator.h"
#include "map_generator.h"


int main(int argc, char **argv){
  
  ros::init(argc, argv, "pandora_geotiff_node");
  ROS_INFO("PANDORA_GEOTIFF_NODE_STARTED");
  QApplication a(argc, argv);
  MapGenerator gc;
  ros::spin();

}
