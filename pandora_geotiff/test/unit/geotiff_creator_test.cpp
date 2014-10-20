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
#include "gtest/gtest.h"
#include "geotiff_creator.h"
#include "map_loader/map_loader.h"
#include <ros/package.h>
namespace pandora_geotiff
  {

     class GeotiffCreatorTest : public ::testing::Test
    {
      protected:
        GeotiffCreatorTest()
        {

          ros::Time::init();
          gc = GeotiffCreator();
          map = map_loader::loadMap(
              ros::package::getPath("pandora_geotiff") +
              "/test/test_maps/map1.yaml");

          points.resize(250);
          
          for (int i = 0; i<251; i ++ )
            {
              points[i] = Eigen::Vector2i(i+20,i+50);
            }
          }
        /*Variables*/
        std::vector<Eigen::Vector2i> points;
        GeotiffCreator gc;
        nav_msgs::OccupancyGrid map;
    };


    
    TEST_F(GeotiffCreatorTest, createBackgroundIm)
    {
      
      gc.drawMap(map,"MAGENTA");
      gc.drawPath(points,"SOLID_ORANGE",3);
      gc.drawObjectOfInterest(Eigen::Vector2i(300,300),"BLACK","HELEANA","WHITE","5",20);
      gc.drawObjectOfInterest(Eigen::Vector2i(250,150),"SOLID_RED","WHITE_MIN","DIAMOND","5wrgergertgertgert",100);
      gc.drawObjectOfInterest(Eigen::Vector2i(450,350),"SOLID_BLUE","WHITE_MAX","CIRCLE","5rstgsregerrgtwrgt",20);
      gc.drawObjectOfInterest(Eigen::Vector2i(150,550),"YELLOW","WHITE","ARROW","5",50);
      gc.createBackgroundIm();
      gc.saveGeotiff();
    }

    
  }// namespace pandora_geotiff
  

