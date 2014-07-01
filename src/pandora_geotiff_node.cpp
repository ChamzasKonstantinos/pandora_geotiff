
#include "GeotiffCreator.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "pandora_geotiff_node");
  InterfaceDiagnostics id;
  NodeDiagnostics nd;
  ros::spin();
}
