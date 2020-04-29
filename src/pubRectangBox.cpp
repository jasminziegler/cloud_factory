/*
 * pubRectangBox.cpp
 *
 *  Created on: Apr 20, 2020
 *      Author: jasmin
 */

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

ros::Publisher pubArtificialData;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pubRectangBox_node");
  ros::NodeHandle nh;
  ros::Rate       loop_rate(10);
  pubArtificialData = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("puck_rear/velodyne_points", 1);

  while(ros::ok())
  {

    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Sensor data -- CAREFUL now in DEGREE - maybe change to RAD
    unsigned int raysIncl = 16;
    double       inclMin  = -15.0;
    double       inclRes  = 2.0;
    double       azimRes  = 0.2;

    unsigned int raysAzim         = round(static_cast<unsigned>(360.0 / azimRes));
    unsigned int raysAzimQuadrant = raysAzim / 4;
    unsigned int idx              = 0;

    cloud.width    = raysAzim;
    cloud.height   = raysIncl;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    cloud.header.frame_id = "map";

    // TO DO: dyn reconf with these cuties
    double lastX   = 0.0;
    double lastY   = 0.0;
    double lastX2  = 0.0;
    double lastY2  = 0.0;
    double xWidth  = 0.01;
    double yWidth  = 0.015;
    double zHeight = 0.1;
    double xStart  = 0.0;
    double yStart  = 2.0;
    double zStart  = 0.0;

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
          cloud.points[idx].x = xStart + i * xWidth;
          cloud.points[idx].y = yStart;
          cloud.points[idx].z = zStart + j * zHeight;
          lastX               = xStart + i * xWidth;
        }
        // erster Quadrant B, x konstant und y wird kleiner, z wächst mit j
        // bis 25% zweiter Quadrant A same conditions
        else if((i > (0.75 * raysAzimQuadrant)) && (i <= (1.25 * raysAzimQuadrant)))
        {
          unsigned int counter1B = i - 0.75 * raysAzimQuadrant;
          cloud.points[idx].x    = lastX;
          cloud.points[idx].y    = yStart - counter1B * yWidth;
          cloud.points[idx].z    = zStart + j * zHeight;
          lastY                  = yStart - counter1B * yWidth;
        }
        // zweiter Quadrant B: y konstant, x wird kleiner, z same
        // bis 75% dritter Quadrant A same conditions
        else if((i > (1.25 * raysAzimQuadrant)) && (i <= (2.75 * raysAzimQuadrant)))
        {
          unsigned int counter2B = i - 1.25 * raysAzimQuadrant;
          cloud.points[idx].x    = lastX - counter2B * xWidth;
          cloud.points[idx].y    = lastY;
          cloud.points[idx].z    = zStart + j * zHeight;
          lastX2                 = lastX - counter2B * xWidth;
        }
        // dritter Quadrant B: x konstant, y wird größer, z same
        // bis 25% vierter Quadrant A same conditions
        else if((i > (2.75 * raysAzimQuadrant)) && (i <= (3.25 * raysAzimQuadrant)))
        {
          unsigned int counter3B = i - 2.75 * raysAzimQuadrant;
          cloud.points[idx].x    = lastX2;
          cloud.points[idx].y    = lastY + counter3B * yWidth;
          cloud.points[idx].z    = zStart + j * zHeight;
          lastY2                 = lastY + counter3B * yWidth;
        }
        // vierter Quadrant B: y konstant, x wird größer, z same
        else
        {
          unsigned int counter4B = i - 3.25 * raysAzimQuadrant;
          cloud.points[idx].x    = lastX2 + counter4B * xWidth;
          cloud.points[idx].y    = lastY2;
          cloud.points[idx].z    = zStart + j * zHeight;
        }
      }
    }

    pubArtificialData.publish(cloud);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
