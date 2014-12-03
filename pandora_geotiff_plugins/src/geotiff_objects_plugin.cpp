#include <map_creator_interface.h>
#include <map_writer_plugin_interface.h>
#include <pandora_data_fusion_msgs/DatafusionGeotiffSrv.h>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_loader.h>

std::string homeFolderString("/home/konstantinos");

namespace pandora_geotiff_plugins
{

using namespace pandora_geotiff;

class ObjectsWriter : public MapWriterPluginInterface
{
public:
  ObjectsWriter();
  virtual ~ObjectsWriter();

  virtual void initialize(const std::string& name);
  virtual void draw(MapWriterInterface *interface);
  void getObjectsData();
  std::string getDateAndTime();
  std::string getQrTime(time_t qrTime);
  void generateQrCsv(std::string missionName);

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient service_client_;

  bool initialized_;
  bool gotData_;
  std::string name_;
  bool draw_all_objects_;
  std::string class_id_;
  std::string QRS_COLOR;
  std::string VICTIMS_COLOR;
  std::string TXT_COLOR;
  std::string HAZMATS_COLOR;
  std::string HAZMATS_SHAPE;
  std::string QRS_SHAPE;
  std::string VICTIMS_SHAPE;
  int HAZMATS_SIZE;
  int QRS_SIZE;
  int VICTIMS_SIZE;
  
private:

  std::vector<geometry_msgs::PoseStamped> victims_;
  std::vector<geometry_msgs::PoseStamped> qrs_;
  std::vector<geometry_msgs::PoseStamped> hazmats_;

};

ObjectsWriter::ObjectsWriter()
    : initialized_(false),gotData_(false),VICTIMS_COLOR("SOLID_RED"),HAZMATS_COLOR("SOLID_ORANGE"),QRS_COLOR("SOLID_BLUE"),
        VICTIMS_SHAPE("CIRCLE"),HAZMATS_SHAPE("DIAMOND"),QRS_SHAPE("CIRCLE"),TXT_COLOR("WHITE_MAX"),
        VICTIMS_SIZE(35),HAZMATS_SIZE(42),QRS_SIZE(35)
{}

ObjectsWriter::~ObjectsWriter()
{}

void ObjectsWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  std::string service_name_;

  plugin_nh.param("service_name", service_name_, std::string("data_fusion_geotiff"));

  service_client_ = nh_.serviceClient<pandora_data_fusion_msgs::DatafusionGeotiffSrv>(service_name_);

  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff  ObjectsWriter plugin %s.", name_.c_str());
}

std::string ObjectsWriter::getDateAndTime(){

  time_t t = time(0); // get time now
  struct tm * now = localtime( & t );
  char buf[20];
  std::stringstream ss;
  
  strftime(buf, sizeof(buf), "%F", now);
  
  ss << buf << "; ";
  
  strftime(buf, sizeof(buf), "%T", now);
  
  ss << buf;
  
  std::string str = ss.str();

  return str;

}

std::string ObjectsWriter::getQrTime(time_t qrTime){
  
  struct tm * now = localtime( & qrTime );
  char buf[10];
  std::stringstream ss;
  strftime(buf, sizeof(buf), "%T", now);
  ss << buf;
  
  std::string str = ss.str();

  return str;
}


void ObjectsWriter::generateQrCsv(std::string missionName){

  std::string filenameString("/RC_2014_PANDORA_");
  filenameString.append(missionName);
  filenameString.append("_qr.csv");
  
  std::string filepath = homeFolderString;
  filepath = filepath.append("/Desktop/");
  filepath.append(filenameString);
}

void ObjectsWriter::getObjectsData()
{
    pandora_data_fusion_msgs::DatafusionGeotiffSrv dataFusionSrv;

    if (!service_client_.call(dataFusionSrv)) {
      ROS_ERROR_NAMED(name_, "Cannot draw Objects, service %s failed", service_client_.getService().c_str());
      return;
    }
     
    victims_ = dataFusionSrv.response.victims;
    ROS_INFO("VICTIMS_SAVED SUCCESEFULLY");
    qrs_ = dataFusionSrv.response.qrs;
    ROS_INFO("QRS_SAVED SUCCESEFULLY");
    hazmats_ = dataFusionSrv.response.hazmats; 
    ROS_INFO("HAZMATS_SAVED SUCCESFULLY");
    gotData_ = true;
  }


void ObjectsWriter::draw(MapWriterInterface *interface)
{
    this->getObjectsData();

    if(!initialized_||!gotData_)
    {
      ROS_WARN_NAMED("OBjectsWriter","ObjectWriter plugin not initilized or no data has been received /n ABORTING DRAWING..");
      return;

  }
    ROS_INFO("DRAWING THE AWESOME OBJECTS");
    
    Eigen::Vector2f coords;
    std::string txt;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    tfScalar pitch, roll, yaw;
     
    try{

      listener.waitForTransform("/map", "/data", ros::Time(), ros::Duration(1));
      listener.lookupTransform("/map", "/data", ros::Time(), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    transform.getBasis().getRPY(roll, pitch, yaw);
    tf::Vector3 origin = transform.getOrigin();
    for (int i = 0 ; i< qrs_.size(); i++)
    {
      float x  =  qrs_[i].pose.position.x + origin.x();
      float y  = qrs_[i].pose.position.y + origin.y();
      coords = Eigen::Vector2f(x*cos(yaw) + y*sin(yaw),y*cos(yaw) +x*sin(yaw));
      txt = boost::lexical_cast<std::string>(i+1);
     
     interface->drawObjectOfInterest(coords,QRS_COLOR ,TXT_COLOR, QRS_SHAPE,txt,QRS_SIZE);
    }

    for (int i = 0 ; i< victims_.size(); i++)
    {
      float x  = victims_[i].pose.position.x + origin.x();
      float y  = victims_[i].pose.position.y + origin.y();
      coords = Eigen::Vector2f(x*cos(yaw) + y*sin(yaw),y*cos(yaw) +x*sin(yaw));
      txt = boost::lexical_cast<std::string>(i+1);
     
     interface->drawObjectOfInterest(coords,VICTIMS_COLOR ,TXT_COLOR, VICTIMS_SHAPE,txt,VICTIMS_SIZE);
    }
    
    for (int i = 0 ; i< hazmats_.size(); i++)
    {
      
      float x  = hazmats_[i].pose.position.x + origin.x();
      float y  = hazmats_[i].pose.position.y + origin.y();
      coords = Eigen::Vector2f(x*cos(yaw) + y*sin(yaw),y*cos(yaw) +x*sin(yaw));
     txt = boost::lexical_cast<std::string>(i+1);
     
     interface->drawObjectOfInterest(coords,HAZMATS_COLOR ,TXT_COLOR, HAZMATS_SHAPE,txt,HAZMATS_SIZE);
    }

    ROS_INFO("DRAWING THE AWESOME OBJECTS SUCCEEDED");
   
}

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::ObjectsWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, ObjectsWriter, pandora_geotiff_plugins::ObjectsWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
