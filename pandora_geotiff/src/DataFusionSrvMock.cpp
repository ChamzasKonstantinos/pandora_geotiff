#include "ros/ros.h"
#include <pandora_data_fusion_msgs/DatafusionGeotiffSrv.h>
#include "std_msgs/String.h"


bool data_fusion_geotiff(pandora_data_fusion_msgs::DatafusionGeotiffSrv::Request &req,
    pandora_data_fusion_msgs::DatafusionGeotiffSrv::Response &res)
{

     res.victimsx.push_back(10);
     res.victimsy.push_back(10);
     res.hazmatx.push_back(80);
     res.hazmaty.push_back(20);
     res.pattern.push_back(5);

     std::vector<std::string>  QRCONTENT;
     QRCONTENT.push_back("Eleana");
     QRCONTENT.push_back("Eleana2");


     std::vector<ros::Time>  QRTIME;
     QRTIME.push_back(ros::Time(10));
     QRTIME.push_back(ros::Time(20));

     res.qrx.push_back(30);
     res.qry.push_back(80);
     res.qrworldx.push_back(40);
     res.qrworldy.push_back(40);
     res.qrx.push_back(50);
     res.qry.push_back(10);
     res.qrworldx.push_back(50);
     res.qrworldy.push_back(50);
     res.qrcontent = QRCONTENT;
     res.qrtimestamp = QRTIME;
     ROS_ERROR("Mock DataFusion was requested succesfully");
     return true;
   
 }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "DataFusionSrvMock");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("data_fusion_geotiff", data_fusion_geotiff);
  ROS_INFO("Mock DatafusionGeotiffSrv node initialized");
  ros::spin();

  return 0;
}

