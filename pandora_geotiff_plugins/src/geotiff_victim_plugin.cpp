#include <pandora_geotiff/map_writer_interface.h>
#include <pandora_geotiff/map_writer_plugin_interface.h>

#include <ros/ros.h>

#include <pluginlib/class_loader.h>
#include <fstream>

namespace pandora_geotiff_plugins
{

using namespace pandora_geotiff;

class VictimWriter : public MapWriterPluginInterface
{
public:
  VictimWriter();
  virtual ~VictimWriter();

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
  int *victimsx;
  int *victimsy;
  int victimsSize;
  
  bool gotData;

};

VictimWriter::VictimWriter()
    : initialized_(false)
{}

VictimWriter::~VictimWriter()
{}

void VictimWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  std::string service_name_;

  plugin_nh.param("service_name", service_name_, std::string("Victim_srv"));

  service_client_ = nh_.serviceClient<pandora_nav_msgs::GetRobotPath>(service_name_);

  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff MapWriter plugin %s.", name_.c_str());
}

VictimWriter::getGeotiffData():
{
 pandora_nav_msgs::GetRobotPath victim_srv;
    if (!service_client_.call(victim_srv)) {
      ROS_ERROR_NAMED(name_, "Cannot draw victims, service %s failed", service_client_.getService().c_str());
      return;
    }
    
    if(victimsx!=0) delete[] victimsx;
    if(victimsy!=0) delete[] victimsy;
    
    victimsSize = victim_srv.response.victimsx.size();
    victimsx = new int[victimsSize];
    victimsy = new int[victimsSize];
    for(int i=0; i<victimsSize; i++){
      
      victimsx[i] = victim_srv.response.victimsx[i];
      victimsy[i] = victim_srv.response.victimsy[i];
      }

}
  
  
}
void VictimWriter::draw(MapWriterInterface *interface)
{
    if(!initialized_) return;
    
    QColor victimsColor;
    victimsColor.setRgb(240,10,10);
    QRgb victRgb = victimsColor.rgb();

    //draw victims
    for(int i=0; i<victimsSize; i++){

      QImage victim(40, 40, QImage::Format_ARGB32);
      QPainter victPainter;
      victPainter.begin(&victim);
      victPainter.setCompositionMode(QPainter::CompositionMode_Source);
      victPainter.fillRect(victim.rect(), Qt::transparent);
      victPainter.setCompositionMode(QPainter::CompositionMode_SourceOver);


      victPainter.setPen(victimsColor);
      victPainter.setBrush(victimsColor);

      QPoint circleCenter(20, 20);
      victPainter.drawEllipse(circleCenter, 10, 10);
      QPen penWhite(Qt::white);
      penWhite.setWidth(2);
      victPainter.setPen(penWhite);
      int victId = i+1;
      QString victIdStr = QString::number(victId);
      if(i+1>=10)
        victPainter.drawText(13, 25, victIdStr);
      else
        victPainter.drawText(16, 25, victIdStr);
      victPainter.end();

      QTransform transform90DegTmp;
      transform90DegTmp.rotate(90);
      QTransform transform90Deg = victim.trueMatrix(transform90DegTmp, 40, 40);

      victim = victim.transformed(transform90Deg);

      //flags indicating occupancy of a particular area by other victims or hazmats
      int flag0 = 0;
      int flag1 = 0;
      int exitloop = 0;

      QPen blackPen(Qt::black);


      //check area "0"
      for(int k=victimsx[i]-14.14-15; k<victimsx[i]-14.14+15; k++){
        for(int l=ysize-1-victimsy[i]-14.14-15; l<ysize-1-victimsy[i]-14.14+15; l++){
          if( (mapIm.pixel(k,l) == victRgb) || (mapIm.pixel(k,l) == hazRgb) || (mapIm.pixel(k,l) == qrRgb)){
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
        QPoint vicP(victimsx[i]-14.14, ysize-1-victimsy[i]-14.14);
        mapPainter.begin(&mapIm);
        mapPainter.drawImage(vicP,victim);
        mapPainter.end();
      }
      else {
        for(int k=victimsx[i]-14.14-45; k<victimsx[i]-14.14-15; k++){
          for(int l=ysize-1-victimsy[i]-14.14-15; l<ysize-1-victimsy[i]-14.14+15; l++){
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
        if(!flag1){

          QPoint vicPnorm(victimsx[i]-14.14-30, ysize-1-victimsy[i]-14.14);
          QPoint vicP(victimsx[i]-20, ysize-1-victimsy[i]+3);
          mapPainter.begin(&mapIm);
          mapPainter.drawImage(vicPnorm,victim);
          blackPen.setWidth(1);
          mapPainter.setPen(blackPen);
          QPoint smallCircleCenter(victimsx[i], ysize-1-victimsy[i]-4);
          mapPainter.drawEllipse(smallCircleCenter,2,2);
          mapPainter.drawLine(smallCircleCenter, vicP);
          mapPainter.end();
        }
        else{	
          flag1 = 0;	

          QPoint vicPnorm(victimsx[i]-14.14+30, ysize-1-victimsy[i]-14.14);
          QPoint vicP(victimsx[i]+28, ysize-1-victimsy[i]+3);
          mapPainter.begin(&mapIm);
          mapPainter.drawImage(vicPnorm,victim);
          blackPen.setWidth(1);
          mapPainter.setPen(blackPen);
          QPoint smallCircleCenter(victimsx[i], ysize-1-victimsy[i]-4);
          mapPainter.drawEllipse(smallCircleCenter,2,2);
          mapPainter.drawLine(smallCircleCenter, vicP);
          mapPainter.end();
       }
      }

  }


}

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::VictimWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, VictimWriter, pandora_geotiff_plugins::VictimWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
