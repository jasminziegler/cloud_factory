/*
 * PubShape.cpp
 *
 *  Created on: Jul 22, 2020
 *      Author: jasmin
 */

#include "PubShape.h"
#include <cmath>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

void PubShape::pubThorus(const int raysIncl, const float inclMin, const float inclRes, const float azimRes, const float azimMin,
                         pcl::PointCloud<pcl::PointXYZ>& thorusCloud)

{
  // Sensor data -- CAREFUL now in DEGREE - maybe change to RAD

  // in DYNRECONF übernommen
  //   unsigned int raysIncl = 50;    // DYNRECONF
  //   double       inclMin  = -15.0; // DYNRECONF
  //   double       inclRes  = 2.0;   // DYNRECONF
  //   double       azimRes  = 0.2;   // DYNRECONF
  //   double       azimMin  = 0.0;   // DYNRECONF

  unsigned int raysAzim = round(static_cast<unsigned>(360.0 / azimRes));
  unsigned int idx      = 0;

  thorusCloud.width    = raysAzim;
  thorusCloud.height   = raysIncl;
  thorusCloud.is_dense = false;
  thorusCloud.points.resize(thorusCloud.width * thorusCloud.height);
  thorusCloud.header.frame_id = "map";

  double inclCorr = 0.0;
  for(unsigned int i = 0; i < raysIncl; i++)
  {
    for(unsigned int j = 0; j < raysAzim; j++)
    {
      idx = i * raysAzim + j;
      std::cout << "idx = " << idx << std::endl;
      double currentInclination = deg2rad(inclMin + i * inclRes);
      double currentAzim        = deg2rad(azimMin + j * azimRes);
      std::cout << "currentInclination = " << rad2deg(currentInclination) << " , currentAzim = " << rad2deg(currentAzim) << std::endl;
      if(currentInclination <= 0) // DAS GLEICH HAB ICH ERGÄNZT für wenn =0
      {
        inclCorr = deg2rad(90.0) + (-1.0 * currentInclination);
      }
      else
      {
        inclCorr = deg2rad(90.0) - currentInclination;
      }

      std::cout << "inclCorr = " << inclCorr << std::endl;
      thorusCloud.points[idx].x = sin(inclCorr) * cos(currentAzim);
      thorusCloud.points[idx].y = cos(inclCorr);
      thorusCloud.points[idx].z = sin(inclCorr) * sin(currentAzim);
    }
  }
}

void PubShape::pubRectang(const int raysIncl, const float azimRes, const float xWidth, const float yWidth, const float zHeight, float xStart, float yStart,
                          float zStart, pcl::PointCloud<pcl::PointXYZ>& rectangleCloud)
{
  // Sensor data -- CAREFUL now in DEGREE - maybe change to RAD
  //   unsigned int raysIncl = 1; //DYN RECONF
  //   double azimRes = 0.2; //DYN RECONF

  // double       inclMin  = -15.0;
  // double       inclRes  = 2.0;
  // double azimRes = 3.6;

  unsigned int raysAzim         = round(static_cast<unsigned>(360.0 / azimRes));
  unsigned int raysAzimQuadrant = raysAzim / 4;
  unsigned int idx              = 0;

  rectangleCloud.width    = raysAzim;
  rectangleCloud.height   = raysIncl;
  rectangleCloud.is_dense = false;
  rectangleCloud.points.resize(rectangleCloud.width * rectangleCloud.height);
  rectangleCloud.header.frame_id = "map";

  double lastX  = 0.0;
  double lastY  = 0.0;
  double lastX2 = 0.0;
  double lastY2 = 0.0;

  // TO DO: dyn reconf with these cuties
  // added to DYNRECONF
  //   double xWidth  = 0.01;
  //   double yWidth  = 0.015;
  //   double zHeight = 0.1;
  //   double xStart  = 0.0;
  //   double yStart  = 2.0;
  //   double zStart  = 0.0;

  for(unsigned int i = 0; i < raysAzim; i++)
  {
    // TO DO: xWIDTH yWIDTH zHEIGHT factors as variables
    for(unsigned int j = 0; j < raysIncl; j++)
    {
      idx = i * raysIncl + j;

      // fill pointcloud points, divided into four quadrants
      // erster Quadrant A: y konstant, x wächst, z wächst mit j
      if(i <= (0.75 * raysAzimQuadrant))
      {
        rectangleCloud.points[idx].x = xStart + i * xWidth;
        rectangleCloud.points[idx].y = yStart;
        rectangleCloud.points[idx].z = zStart + j * zHeight;
        lastX                        = xStart + i * xWidth;
      }
      // erster Quadrant B, x konstant und y wird kleiner, z wächst mit j
      // bis 25% zweiter Quadrant A same conditions
      else if((i > (0.75 * raysAzimQuadrant)) && (i <= (1.25 * raysAzimQuadrant)))
      {
        unsigned int counter1B       = i - 0.75 * raysAzimQuadrant;
        rectangleCloud.points[idx].x = lastX;
        rectangleCloud.points[idx].y = yStart - counter1B * yWidth;
        rectangleCloud.points[idx].z = zStart + j * zHeight;
        lastY                        = yStart - counter1B * yWidth;
      }
      // zweiter Quadrant B: y konstant, x wird kleiner, z same
      // bis 75% dritter Quadrant A same conditions
      else if((i > (1.25 * raysAzimQuadrant)) && (i <= (2.75 * raysAzimQuadrant)))
      {
        unsigned int counter2B       = i - 1.25 * raysAzimQuadrant;
        rectangleCloud.points[idx].x = lastX - counter2B * xWidth;
        rectangleCloud.points[idx].y = lastY;
        rectangleCloud.points[idx].z = zStart + j * zHeight;
        lastX2                       = lastX - counter2B * xWidth;
      }
      // dritter Quadrant B: x konstant, y wird größer, z same
      // bis 25% vierter Quadrant A same conditions
      else if((i > (2.75 * raysAzimQuadrant)) && (i <= (3.25 * raysAzimQuadrant)))
      {
        unsigned int counter3B       = i - 2.75 * raysAzimQuadrant;
        rectangleCloud.points[idx].x = lastX2;
        rectangleCloud.points[idx].y = lastY + counter3B * yWidth;
        rectangleCloud.points[idx].z = zStart + j * zHeight;
        lastY2                       = lastY + counter3B * yWidth;
      }
      // vierter Quadrant B: y konstant, x wird größer, z same
      else
      {
        unsigned int counter4B       = i - 3.25 * raysAzimQuadrant;
        rectangleCloud.points[idx].x = lastX2 + counter4B * xWidth;
        rectangleCloud.points[idx].y = lastY2;
        rectangleCloud.points[idx].z = zStart + j * zHeight;
      }
    }
  }
}