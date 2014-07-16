/*
* Copyright (C) 2011 by Pandora Robotics Team, Aristotle Univeristy of Thessaloniki, Greece
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/


#include "GeotiffCreator.h"
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>


QString homeFolderString("/home/konstantinos");



std::string getDateAndTime(){

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

std::string getQrTime(time_t qrTime){

struct tm * now = localtime( & qrTime );
char buf[10];
std::stringstream ss;
strftime(buf, sizeof(buf), "%T", now);
ss << buf;

std::string str = ss.str();

    return str;
}

GeotiffCreator::GeotiffCreator() 
{

  geotiff_= new QImage(finalSizeX,finalSizeY, QImage::Format_RGB32);
  mapIm_= new QImage(finalSizeX,finalSizeY, QImage::Format_RGB32);


}


//~ void GeotiffCreator::generateQrCsv(){
//~ 
//~ QString filenameString("/RC_2014_PANDORA_");
//~ filenameString.append(missionName);
//~ filenameString.append("_qr.csv");
//~ 
//~ QString filepath = homeFolderString;
//~ filepath = filepath.append("/Desktop/");
//~ filepath.append(filenameString);
//~ std::string filepathStr = filepath.toUtf8().constData();
//~ 
//~ std::cout << filepathStr << "\n";
//~ 
//~ std::ofstream csvFile;
//~ csvFile.open (filepathStr.c_str());
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
//~ for(int i=0; i<qrSize; i++)
//~ csvFile << i+1 << ";" << qrWorldTime[i] << ";" << qrType[i] << ";" << qrWorldX[i] << ";" << qrWorldY[i] << std::endl;
//~ 
//~ 
//~ csvFile.close();
//~ 
//~ }


void GeotiffCreator::saveMissionName(std::string missionName)
{
  this->missionName = QString::fromStdString(missionName);
  
}


void GeotiffCreator::onCreateGeotiffClicked()
{
  QPainter geotiffPainter;
  geotiffPainter.begin(geotiff_);

  drawCheckers(finalSizeX, finalSizeY, &geotiffPainter);
  drawFileName(finalSizeX, finalSizeY, &geotiffPainter);
  drawMapOrientation(finalSizeX, finalSizeY, &geotiffPainter);
  drawMapScale(finalSizeX, finalSizeY, &geotiffPainter);

  }
  
void GeotiffCreator::inCreateGeotiffClicked()
{
  QPainter geotiffPainter;
  geotiffPainter.begin(geotiff_);
  
  int ioffset = 100;
  int joffset = 100;

  QPoint mapP(ioffset, joffset);
  geotiffPainter.drawImage(mapP, *mapIm_);
  
  drawExploredArea(finalSizeX, finalSizeY, &geotiffPainter);
}
  
void GeotiffCreator::saveGeotiff()
{
  //save geotiff_ in Desktop
   QString filepath = homeFolderString;
   filepath = filepath.append("/Desktop/");
   //~ filepath.append(filenameString);
   (*geotiff_).save("/home/konstantinos/Desktop/image.jpg");
}

void GeotiffCreator::drawCheckers ( int xsize ,int ysize ,  QPainter* geotiffPainter) {

  QColor colorLightGreyGrid;
  QColor colorDarkGreyGrid;
  colorLightGreyGrid.setRgb(226,226,227);
  colorDarkGreyGrid.setRgb(237,237,238);

  QBrush gridBrush(colorLightGreyGrid);
    //draw (checkerboard) grid
    bool dark = true;
        for (int i = 0; i < xsize/50 ; i++) {
          dark = !dark;
          for (int j = 0; j < ysize/50 ; j++) {
              dark = !dark;
              if (dark) {
                  gridBrush.setColor(colorLightGreyGrid);
                   geotiffPainter->setPen(colorLightGreyGrid);
              } 
             else 
             {
                gridBrush.setColor(colorDarkGreyGrid);
                geotiffPainter->setPen(colorDarkGreyGrid);
            }
            int x = 50 * i;
            int y = 50 * j;
            int width = 50;
            int height = 50;
            // Draw rectangle to screen.
            geotiffPainter->drawRect(x, y, width, height);
            geotiffPainter->fillRect(x, y, width, height, gridBrush);
            
          }
    }
}

void GeotiffCreator::drawFileName( int xsize ,int ysize ,  QPainter* geotiffPainter ){
  
  
   QColor colorBlueText;
   colorBlueText.setRgb(0, 44, 207);
   QPen filenamePen = QPen(colorBlueText);
   filenamePen.setWidth(4);
   geotiffPainter->setPen(filenamePen);

   QPointF filenamePoint(25 ,25);
   QString filenameString("/RRL_2013_PANDORA_");
   filenameString.append(missionName);
   filenameString.append(".tiff");

   QPen colorBlueTextPen(colorBlueText);
   colorBlueTextPen.setWidth(2);
   geotiffPainter->setPen(colorBlueTextPen);

   geotiffPainter->drawText(filenamePoint, filenameString);
}
void GeotiffCreator::drawMapScale( int xsize ,int ysize ,  QPainter* geotiffPainter ){
       QColor colorMapScale;
   colorMapScale.setRgb(0, 50, 140);

   QPen colorMapScalePen = QPen(colorMapScale);
   colorMapScalePen.setWidth(4);
   geotiffPainter->setPen(colorMapScalePen);

   geotiffPainter->setPen(colorMapScale);
   QPointF lengthUnitPoint1(25, 50);
   QPointF lengthUnitPoint2(25, 100);

   geotiffPainter->drawLine(lengthUnitPoint1, lengthUnitPoint2);

   QPointF lengthUnitPoint1Up(21, 50);
   QPointF lengthUnitPoint2Up(29, 50);
   geotiffPainter->drawLine(lengthUnitPoint1Up, lengthUnitPoint2Up);

   QPointF lengthUnitPoint1Down(21, 100);
   QPointF lengthUnitPoint2Down(29, 100);
   geotiffPainter->drawLine(lengthUnitPoint1Down, lengthUnitPoint2Down);

   QPointF lengthUnitTextPoint(28, 80);
   QString lengthUnitText("1m");
   geotiffPainter->drawText(lengthUnitTextPoint, lengthUnitText);
 }
void GeotiffCreator::drawMapOrientation( int xsize ,int ysize , QPainter* geotiffPainter ){
       QColor colorOrientation;
   colorOrientation.setRgb(0, 50, 140);

   QPointF pointX1(100, 50);
   QPointF pointX2(100, 100);
   QPointF pointY1(50, 100);
   QPointF pointY2(100, 100);

   geotiffPainter->drawLine(pointX1, pointX2);
   geotiffPainter->drawLine(pointY1, pointY2);

   QPointF pointYup(55, 95);
   QPointF pointYdown(55, 105);
   geotiffPainter->drawLine(pointY1, pointYup);
   geotiffPainter->drawLine(pointY1, pointYdown);

   QPointF pointXright(105, 55);
   QPointF pointXleft(95, 55);
   geotiffPainter->drawLine(pointX1, pointXright);
   geotiffPainter->drawLine(pointX1, pointXleft);

   QPointF pointXText(110, 52);
   QString XString("x");
   geotiffPainter->drawText(pointXText, XString);

   QPointF pointYText(60, 93);
   QString YString("y");
   geotiffPainter->drawText(pointYText, YString);
 }
 
void GeotiffCreator::drawExploredArea(int xsize , int ysize , QPainter* geotiffPainter) {

  QColor exploredAreaColor;
  exploredAreaColor.setRgb(190,190,191);
  QPen exploredAreaPen(exploredAreaColor);
  geotiffPainter->setPen(exploredAreaPen);
  
  for(int i=0; i<ysize-50; i++){
    for(int j=15; j<xsize-50; j+=25){
      QRgb rgb = geotiff_->pixel(i,j);
      int r = qRed(rgb);
      int g = qGreen(rgb);
      int b = qBlue(rgb);
      if( ((r==g)&&(g==b)&&(r>=140)) || ((g==230)&&(r==b)&&(r>=130)&&(r<=180)) ){
        geotiffPainter->drawPoint(i,j);
      }
    }
  }

  for(int i=15; i<xsize-50; i+=25){
    for(int j=0; j<ysize-50; j++){
      QRgb rgb = geotiff_->pixel(i,j);
        int r = qRed(rgb);
        int g = qGreen(rgb);
        int b = qBlue(rgb);
        if( ((r==g)&&(g==b)&&(r>=140)) || ((g==230)&&(r==b)&&(r>=130)&&(r<=180)) ){
          geotiffPainter->drawPoint(i,j);
        }
    }
  }
}


void GeotiffCreator::drawMap(const nav_msgs::OccupancyGrid* map)
{

  int xsize = map->info.height;
  int ysize = map->info.width;
  
  
  ROS_ERROR("SIZEX:%d",xsize);
  ROS_ERROR("SIZEX:%d",ysize);
  
  finalSizeY = (xsize/100)*100 +200;
  finalSizeX = (ysize/100)*100 +200;
  
  mapIm_ = new QImage(xsize, ysize, QImage::Format_ARGB32);
  QPainter mapPainter;
  mapPainter.begin(mapIm_);

  mapPainter.setCompositionMode(QPainter::CompositionMode_Source);
  mapPainter.fillRect(mapIm_->rect(), Qt::transparent);
  mapPainter.setCompositionMode(QPainter::CompositionMode_SourceOver);


  QColor wallsObstaclesColor;
  wallsObstaclesColor.setRgb(0, 40, 120);

  for(int i=0; i<xsize; i++){
    for(int j=0; j<ysize; j++){
      if(map->data[j*xsize + i] < 127 && map->data[j*xsize + i] > 0){

        QPen wallsObstaclesPen(wallsObstaclesColor);
        wallsObstaclesPen.setWidth(3);
        mapPainter.setPen(wallsObstaclesPen);
        mapPainter.drawPoint(i,ysize-1-j);
      }
      else if(map->data[j*xsize + i] > 126 ){
       QColor searchedAreaColor;
       searchedAreaColor.setRgb(map->data[j*xsize + i], map->data[j*xsize + i], map->data[j*xsize + i]);
       mapPainter.setPen(searchedAreaColor);
       mapPainter.drawPoint(i,ysize-1-j);
      }
    }
  }
}

void GeotiffCreator::drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points)
{
  QPainter mapPainter;
  mapPainter.begin(mapIm_);

  int robotx = points[0].x();
  int roboty = points[0].y();
  QColor initPoseColor;
  initPoseColor.setRgb(255, 200, 0);
  QPen initPosePen(initPoseColor); 
  mapPainter.setPen(initPosePen);
  QBrush initPoseBrush(initPoseColor);
  mapPainter.setBrush(initPoseBrush);
  QPoint *initPosePoints = new QPoint[5];
  initPosePoints[0].setX(robotx+15);
  initPosePoints[0].setY(roboty);
  initPosePoints[1].setX(robotx-10);
  initPosePoints[1].setY(roboty-5);
  initPosePoints[2].setX(robotx-5);
  initPosePoints[2].setY(roboty);
  initPosePoints[3].setX(robotx-10);
  initPosePoints[3].setY(roboty+5);
  mapPainter.drawPolygon(initPosePoints, 4);

  QColor pathColor;
  pathColor.setRgb(120,0,140);
  QPen pathPen(pathColor);
  pathPen.setWidth(2);
  mapPainter.setPen(pathPen);

  for(int i=0; i<points.size()-1; i++){
   mapPainter.drawLine(points[i].x(), points[i].y(), points[i+1].x(), points[i+1].y());
  }
}


void GeotiffCreator::drawObjectOfInterest(const Eigen::Vector2f& coords, const std::string& txt, const Color& color)
{

    QPainter mapPainter;
    mapPainter.begin(mapIm_);

    QImage objectIm(40, 40, QImage::Format_ARGB32);
    QPainter objectPainter;

    QColor objectColor;
    objectColor.setRgb(color.r,color.g,color.b);

    objectPainter.begin(&objectIm);

    objectPainter.setCompositionMode(QPainter::CompositionMode_Source);
    objectPainter.fillRect(objectIm.rect(), Qt::transparent); 
    objectPainter.setCompositionMode(QPainter::CompositionMode_SourceOver);

    objectPainter.setPen(objectColor);

    int x = 20;
    int y = 20;

    for(int iter=0; iter<10; iter++){
      for(int k=x-iter; k<=x+iter; k++){
        objectPainter.drawPoint(k, y-(10-iter));
      }
    }
    for(int iter=0; iter<=10; iter++){
      for(int k=x-iter; k<=x+iter; k++){
        objectPainter.drawPoint(k, y+(10-iter));
        }
    }

    QPen penWhite(Qt::white);
    penWhite.setWidth(2);
    objectPainter.setPen(penWhite);
    objectPainter.drawText(x-4, y+5, QString::fromStdString(txt));
    objectPainter.end();

    QPoint ObjectPoint(coords.x(), coords.y());
    mapPainter.drawImage(ObjectPoint, objectIm); 
}










