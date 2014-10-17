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
  
  
private:

  std::vector<int> *qrx;
  std::vector<int> *qry;
  std::vector<float> *qrworldx;
  std::vector<float> *qrworldy;
  std::vector<ros::Time> *qrtimestamp;
  std::vector<std::string>  *qrcontent;
  
  std::vector<int> *victimsx;
  std::vector<int> *victimsy;
  
  std::vector<int> *hazmatx;
  std::vector<int> *hazmaty;
  std::vector<int> *pattern;

};

ObjectsWriter::ObjectsWriter()
    : initialized_(false)
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




//~ 
//~ void ObjectsWriter::saveMissionName(std::string missionName)
//~ {
  //~ this->missionName = (missionName);
//~ }

void ObjectsWriter::getObjectsData()
{
    pandora_data_fusion_msgs::DatafusionGeotiffSrv dataFusionSrv;

    if (!service_client_.call(dataFusionSrv)) {
      ROS_ERROR_NAMED(name_, "Cannot draw Objects, service %s failed", service_client_.getService().c_str());
      return;
    }


    
    //~ if(qrx!=0) delete[] qrx;
    //~ if(qry!=0) delete[] qry;
    //~ if(qrcontent!=0) delete[] qrcontent;
    //~ if(qrtimestamp!=0) delete[] qrtimestamp;
    //~ if(qrworldx!=0) delete[] qrworldx;
    //~ if(qrworldy=0) delete[] qrworldy;
    //~ if(victimsx!=0) delete[] victimsx;
    //~ if(victimsy!=0) delete[] victimsy;
    //~ if(hazmatx!=0) delete[] hazmatx;
    //~ if(hazmaty!=0) delete[] hazmaty;
    //~ if(pattern!=0) delete[] pattern;
    
    
 
    int qrSize = dataFusionSrv.response.qrx.size();
    
    qrx = new std::vector<int>();
    qry = new std::vector<int>();
    qrcontent = new std::vector<std::string>();
    qrworldx = new std::vector<float>();
    qrworldy = new std::vector<float>();
    qrtimestamp = new std::vector<ros::Time>();

    
    for(int i =0 ; i < qrSize ; i ++){
       qrx->push_back(dataFusionSrv.response.qrx[i]);
       qry->push_back(dataFusionSrv.response.qry[i]);
       //~ qrcontent->push_back(dataFusionSrv.response.qrcontent[i]);
       qrworldx->push_back(dataFusionSrv.response.qrworldx[i]);
       qrworldy->push_back(dataFusionSrv.response.qrworldy[i]);
       //~ qrtimestamp->push_back(dataFusionSrv.response.qrtimestamp[i]);
       ROS_INFO("QRS_SAVED SUCCESEFULLY");
     }
    
    int victimsSize = dataFusionSrv.response.qrx.size();
    
    victimsx = new std::vector<int>();
    victimsy = new std::vector<int>();
     
    for(int i = 0; i <victimsSize; i++)
    {
      victimsx->push_back(dataFusionSrv.response.victimsx[i]);
      victimsy->push_back(dataFusionSrv.response.victimsy[i]);
      ROS_INFO("VICTIMS_SAVED SUCCESEFULLY");
    }
    
    int hazmatsSize = dataFusionSrv.response.qrx.size(); 
    hazmatx = new std::vector<int>();
    hazmaty = new std::vector<int>();
    pattern = new std::vector<int>();
     
    for(int i = 0; i < hazmatsSize; i++)
    {
      hazmatx->push_back(dataFusionSrv.response.hazmatx[i]);
      hazmaty->push_back(dataFusionSrv.response.hazmaty[i]);
      pattern->push_back(dataFusionSrv.response.pattern[i]);
      ROS_INFO("HAZMATS_SAVED SUCCESFULLY");
    }
  
  }
void ObjectsWriter::draw(MapWriterInterface *interface)
{
    if(!initialized_) return;

    
    this->getObjectsData();
    
    ROS_INFO("DRAWING THE AWESOME OBJECTS");
    
    Eigen::Vector2f coords;
    std::string txt;

    
    MapWriterInterface::Color qrsColor(10,10,240);
    MapWriterInterface::Color victimsColor(240,10,10);
    MapWriterInterface::Color hazmatsColor(255,100,30);

    for (int i = 0 ; i< qrx->size(); i++)
    {
     coords = Eigen::Vector2f((*qrx)[i] ,(*qry)[i]);
     txt = boost::lexical_cast<std::string>(i+1);
     interface->drawObjectOfInterest(coords ,txt, qrsColor);
    }

    for (int i = 0 ; i< victimsx->size(); i++)
    {
     coords = Eigen::Vector2f((*victimsx)[i] ,(*victimsy)[i]);
     txt = boost::lexical_cast<std::string>(i+1);
     interface->drawObjectOfInterest(coords ,txt, victimsColor);
    }

    for (int i = 0 ; i< hazmatx->size(); i++)
    {
     coords = Eigen::Vector2f((*hazmatx)[i] ,(*hazmaty)[i]);
     txt = boost::lexical_cast<std::string>(i+1);
     interface->drawObjectOfInterest(coords ,txt, hazmatsColor);
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
