#include "ros/ros.h"
#include <pandora_data_fusion_msgs/DatafusionGeotiffSrv.h>
#include <tf/transform_broadcaster.h>


bool data_fusion_geotiff(pandora_data_fusion_msgs::DatafusionGeotiffSrv::Request &req,
    pandora_data_fusion_msgs::DatafusionGeotiffSrv::Response &res)
{


     geometry_msgs::PoseStamped victim1;
     geometry_msgs::PoseStamped hazmat1;
     geometry_msgs::PoseStamped qr1;

     geometry_msgs::PoseStamped victim2;
     geometry_msgs::PoseStamped hazmat2;
     geometry_msgs::PoseStamped qr2;

     geometry_msgs::PoseStamped victim3;
     geometry_msgs::PoseStamped hazmat3;
     geometry_msgs::PoseStamped qr3;
     
     victim1.pose.position.x = 14;
     victim1.pose.position.y = -0.76;

     victim2.pose.position.x = 16.24;
     victim2.pose.position.y = -1.68;
     
     victim3.pose.position.x = 18.58;
     victim3.pose.position.y = -4.90998;

     hazmat1.pose.position.x = 0.65;
     hazmat1.pose.position.y = -1.54;

     hazmat2.pose.position.x = 8.335711;
     hazmat2.pose.position.y = 0.2478;

     hazmat3.pose.position.x = 14.1463;
     hazmat3.pose.position.y = 1.75368;

     qr1.pose.position.x = 4.338;
     qr1.pose.position.y = 6.22851;


     qr2.pose.position.x = 3.17607;
     qr2.pose.position.y = 6.92356;
     
     qr3.pose.position.x = 4.3;
     qr3.pose.position.y = 6;

     
     res.victims.push_back(victim1);
     res.victims.push_back(victim2);
     res.victims.push_back(victim3);
     res.hazmats.push_back(hazmat1);
     res.hazmats.push_back(hazmat2);
     res.hazmats.push_back(hazmat3);
     res.qrs.push_back(qr1);
     res.qrs.push_back(qr2);
     res.qrs.push_back(qr3);
     
     ROS_INFO("Mock DataFusion was requested succesfully");
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

