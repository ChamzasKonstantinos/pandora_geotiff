#include <map_writer_interface.h>
#include <map_writer_plugin_interface.h>
#include <pandora_data_fusion_msgs/DatafusionGeotiffSrv.h>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>

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
  std::string name_;
  bool draw_all_objects_;
  std::string class_id_;
  std::string QRS_COLOR;
  std::string VICTIMS_COLOR;
  std::string HAZMATS_COLOR;
  //~ std::string HAZMATS_SHAPE;
  //~ std::string QRS_SHAPE
  //~ std::string VICTIMS_SHAPE
  //~ 
  
private:

  std::vector<geometry_msgs::PoseStamped> victims_;
  std::vector<geometry_msgs::PoseStamped> qrs_;
  std::vector<geometry_msgs::PoseStamped> hazmats_;

};

ObjectsWriter::ObjectsWriter()
    : initialized_(false),gotData(false),VICTIMS_COLOR("BLACK"),HAZMATS_COLOR("MAGENTA"),QRS_COLOR("RED")
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

  //~ std::cout << filepath << "\n";
  //~ 
  //~ std::ofstream csvFile;
  //~ csvFile.open(filepath.c_str());
  //~ 
  //~ // ---csvFile---
  //~ // Resko Koblenz, Germany
  //~ // 2013-06-23; 14:37:03
  //~ // Semi1
  //~ //
  //~ // 1;14:28:01;Y_1_1_chair_yoked;-8.29994;-2.29014
  //~ 
  //~ csvFile << "PANDORA AUTh, Greece" << std::endl;
  //~ csvFile << getDateAndTime() << std::endl;
  //~ csvFile << missionName.toUtf8().constData() << std::endl << std::endl;
  //~ std::string qrWorldTime[qrSize];
  //~ 
  //~ for(int i=0; i<qrSize; i++){
    //~ qrWorldTime[i] = getQrTime(qrTime[i]);
    //~ }
  //~ 
  //~ for(int i=0; i<qrSize; i++){
    //~ csvFile << i+1 << ";" << qrWorldTime[i] << ";" << qrType[i] << ";" << qrWorldX[i] << ";" << qrWorldY[i] << std::endl;
  //~ }
  //~ csvFile.close();
//~ 
//~ }

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
    hamzats_ = dataFusionSrv.response.hazmats; 
    ROS_INFO("HAZMATS_SAVED SUCCESFULLY");


void ObjectsWriter::draw(MapWriterInterface *interface)
{
    this->getObjectsData();

    if(!initialized_||!gotData)
    {
      ROS_WARN_NAMED("OBjectsWriter","ObjectWriter plugin not initilized or no data has been received /n ABORTING DRAWING..");
      return;

    
    ROS_INFO("DRAWING THE AWESOME OBJECTS");
    
    Eigen::Vector2f coords;
    std:string txt;
    
    for (int i = 0 ; i< qrs_->size(); i++)
    {
     coords = Eigen::Vector2f((*qrs_[i].pose.position.x,*qrs_[i].pose.position.y);
     txt = boost::lexical_cast<std::string>(i+1);
     
     interface->drawObjectOfInterest(coords,QRSCOLOR ,TXTCOLOR, SHAPE,txt,QRSIZE);
    }

    for (int i = 0 ; i< victims_->size(); i++)
    {
     coords = Eigen::Vector2f((*victims_[i].pose.position.x,*victims_[i].pose.position.y);
     txt = boost::lexical_cast<std::string>(i+1);
     
     interface->drawObjectOfInterest(coords,VICTIMSCOLOR ,TXTCOLOR, SHAPE,txt,VICTIMSIZE);
    }
    
    for (int i = 0 ; i< hamzats_->size(); i++)
    {
     coords = Eigen::Vector2f((*hamzats_[i].pose.position.x,*hamzats_[i].pose.position.y);
     txt = boost::lexical_cast<std::string>(i+1);
     
     interface->drawObjectOfInterest(coords,HAZMATS_COLOR ,TXT_COLOR, SHAPE,txt,HAZMATS_SIZE);
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
