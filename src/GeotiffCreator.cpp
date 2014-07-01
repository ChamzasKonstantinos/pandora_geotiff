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

GeotiffCreator::GeotiffCreator()
{
   finalSizeX = 10 ;
   finalSizeY = 10;
   onCreateGeotiffClicked();


}

void GeotiffCreator::geotiffTimerCb(const ros::TimerEvent& event){

//~ calling geotiff services and storing data in corresponding arrays
   //~ getGeotiffData();

}


void GeotiffCreator::onCreateGeotiffClicked()
{
  
  
    QImage geotiff(finalSizeX,finalSizeY, QImage::Format_RGB32);
    QPainter geotiffPainter;
    geotiffPainter.begin(&geotiff);

    QColor colorLightGreyGrid;
    QColor colorDarkGreyGrid;
    colorLightGreyGrid.setRgb(226,226,227);
    colorDarkGreyGrid.setRgb(237,237,238);

    QBrush gridBrush(colorLightGreyGrid);

    //draw (checkerboard) grid
    bool dark = true;
        for (int i = 0; i < finalSizeX/50 ; i++) {
          dark = !dark;
          for (int j = 0; j < finalSizeY/50 ; j++) {
              dark = !dark;

              if (dark) {
                  gridBrush.setColor(colorLightGreyGrid);
                   geotiffPainter.setPen(colorLightGreyGrid);
             } 
             else 
             {
                gridBrush.setColor(colorDarkGreyGrid);
                geotiffPainter.setPen(colorDarkGreyGrid);
            }
 
            int x = 50 * i;
            int y = 50 * j;
            int width = 50;
            int height = 50;
            // Draw rectangle to screen.
            geotiffPainter.drawRect(x, y, width, height);
            geotiffPainter.fillRect(x, y, width, height, gridBrush);
          }
    }
    //save geotiff in Desktop
    QString filepath = homeFolderString;
    filepath = filepath.append("/Desktop/");
    //~ filepath.append(filenameString);
    geotiff.save(filepath);
  }
