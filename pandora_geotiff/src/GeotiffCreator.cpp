/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *   Chamzas Konstantinos <chamzask@gmail.com>
 *********************************************************************/

#include "pandora_geotiff/GeotiffCreator.h"
//~ std::string homeFolderString("/home/konstantinos/Desktop/image.jpg");

GeotiffCreator::GeotiffCreator()
{

  geotiffBackgroundIm_ = NULL
  mapIm_= new QImage(finalSizeX,finalSizeY, QImage::Format_RGB32);
}

void GeotiffCreator::onCreateGeotiffClicked()
{

  ROS_ERROR("onCreateGeotiffClicked()");

  geotiff_= new QImage(finalSizeX,finalSizeY, QImage::Format_RGB32);
  QPainter geotiffPainter;
  geotiffPainter.begin(geotiff_);


  drawCheckers(finalSizeX, finalSizeY, &geotiffPainter);
  drawFileName(finalSizeX, finalSizeY, &geotiffPainter);
  drawMapOrientation(finalSizeX, finalSizeY, &geotiffPainter);
  drawMapScale(finalSizeX, finalSizeY, &geotiffPainter);
  ROS_ERROR("onCreateGeotiffClicked() finished");

  }

void GeotiffCreator::inCreateGeotiffClicked()
{
  ROS_ERROR("inCreateGeotiffClicked()");
  QPainter geotiffPainter;
  geotiffPainter.begin(geotiff_);

  int ioffset = 100;
  int joffset = 100;

  QPoint mapP(ioffset, joffset);
  geotiffPainter.drawImage(mapP, *mapIm_);

  drawExploredArea(finalSizeX, finalSizeY, &geotiffPainter);
  ROS_ERROR("inCreateGeotiffClicked() finished");
}

void GeotiffCreator::saveGeotiff()
{
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


  ROS_ERROR("sizex:%d",xsize);
  ROS_ERROR("Sizey:%d",ysize);

  this->finalSizeY = (xsize/100)*100 +200;
  this->finalSizeX = (ysize/100)*100 +200;


  ROS_ERROR("FinalSizeX:%d",finalSizeX);
  ROS_ERROR("FinalSizeY:%d",finalSizeY);

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










