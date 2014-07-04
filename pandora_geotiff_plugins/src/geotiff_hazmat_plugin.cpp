#include <pandora_geotiff/map_writer_interface.h>
#include <pandora_geotiff/map_writer_plugin_interface.h>

#include <ros/ros.h>

#include <pluginlib/class_loader.h>
#include <fstream>

namespace pandora_geotiff_plugins
{

using namespace pandora_geotiff;

class HazmatWriter : public MapWriterPluginInterface
{
public:
  HazmatWriter();
  virtual ~HazmatWriter();

  virtual void initialize(const std::string& name);
  virtual void draw(MapWriterInterface *interface);
  virtual void getGeotiffData();

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient service_client_;

  bool initialized_;
  std::string name_;
  bool draw_all_objects_;
  std::string class_id_;
  
  
private:

  int *hazmatx;
  int *hazmaty;
  int *hazmatType;
  int hazmatSize;
  bool gotData;
  

};

HazmatWriter::HazmatWriter()
    : initialized_(false)
{}

HazmatWriter::~HazmatWriter()
{}

void HazmatWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  std::string service_name_;

  plugin_nh.param("service_name", service_name_, std::string("hazmat"));

  service_client_ = nh_.serviceClient<pandora_nav_msgs::GetRobotPath>(service_name_);

  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff MapWriter plugin %s.", name_.c_str());
}

HazmatWriter::getGeotiffData():
{
 pandora_nav_msgs::GetRobotPath hazmat_srv;
    if (!service_client_.call(map_coverage_srv)) {
      ROS_ERROR_NAMED(name_, "Cannot draw hazmat, service %s failed", service_client_.getService().c_str());
      return;
    }
    
  if(hazmatx!=0) delete[] hazmatx;
  if(hazmaty!=0) delete[] hazmaty;
  if(hazmatType!=0) delete[] hazmatType;
  
  
  hazmatSize = hazmat_srv.response.hazmatx.size();
  hazmatx = new int[hazmatSize];
  hazmaty = new int[hazmatSize];
  hazmatType = new int[hazmatSize];

  for(int i=0; i<hazmatSize; i++){
  hazmatx[i] = hazmat_srv.response.hazmatx[i];
  hazmaty[i] = hazmat_srv.response.hazmaty[i];
  hazmatType[i] = hazmat_srv.response.pattern[i];
}

  
}
  
  
}
void HazmatWriter::draw(MapWriterInterface *interface)
{
    if(!initialized_) return;
    
    //draw hazmats
for(int i=0; i<hazmatSize; i++){
  
  
  QColor hazmatColor;
  hazmatColor.setRgb(255,100,30);
  QRgb hazRgb = hazmatColor.rgb();

  QImage hazmat(40, 40, QImage::Format_ARGB32);
  QPainter hazPainter;

  hazPainter.begin(&hazmat);

  hazPainter.setCompositionMode(QPainter::CompositionMode_Source);
  hazPainter.fillRect(hazmat.rect(), Qt::transparent);
  hazPainter.setCompositionMode(QPainter::CompositionMode_SourceOver);

  hazPainter.setPen(hazmatColor);

  int x = 20;
  int y = 20;

  for(int iter=0; iter<10; iter++){
    for(int k=x-iter; k<=x+iter; k++){
      hazPainter.drawPoint(k, y-(10-iter));
    }
  }
  for(int iter=0; iter<=10; iter++){
    for(int k=x-iter; k<=x+iter; k++){
      hazPainter.drawPoint(k, y+(10-iter));
    }
  }

  QPen penWhite(Qt::white);
  penWhite.setWidth(2);
  hazPainter.setPen(penWhite);
  int hazId = i+1;
  QString hazIdStr = QString::number(hazId);
  hazPainter.drawText(x-4, y+5, hazIdStr);
  hazPainter.end();


  QTransform transform90DegTmp;
  transform90DegTmp.rotate(90);
  QTransform transform90Deg = hazmat.trueMatrix(transform90DegTmp, 40, 40);

  hazmat = hazmat.transformed(transform90Deg);

//flags indicating occupancy of a particular area by other victims or hazmats
  int flag0 = 0;
  int flag1 = 0;
  int exitloop = 0;

  QPen blackPen(Qt::black);


  //check area "0"
  for(int k=hazmatx[i]-14.14-15; k<hazmatx[i]-14.14+15; k++){
    for(int l=ysize-1-hazmaty[i]-14.14-15; l<ysize-1-hazmaty[i]-14.14+15; l++){
      if( (mapIm.pixel(k,l) == victRgb)
       || (mapIm.pixel(k,l) == hazRgb) 
       ||(mapIm.pixel(k,l) == qrRgb) ){
        flag0=1;
        exitloop = 1;
        break;
       }
      }
   if(exitloop == 1){
     exitloop = 0;
     break;
    }
  }
  
  if(!flag0){
    QPoint vicP(hazmatx[i]-14.14, ysize-1-hazmaty[i]-14.14);
    mapPainter.begin(&mapIm);
    mapPainter.drawImage(vicP,hazmat);
    mapPainter.end();
   }
   else {
     for(int k=hazmatx[i]-14.14-45; k<hazmatx[i]-14.14-15; k++){
       for(int l=ysize-1-hazmaty[i]-14.14-15; l<ysize-1-hazmaty[i]-14.14+15; l++){
         if( (mapIm.pixel(k,l) == victRgb) 
           || (mapIm.pixel(k,l) == hazRgb) 
           || (mapIm.pixel(k,l) == qrRgb)){
           flag1=1;
           exitloop = 1;
           break;
         }
       }
    if(exitloop == 1){
      exitloop = 0;
      break;
    }
   }


}

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::HazmatWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, HazmatWriter, pandora_geotiff_plugins::HazmatWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
