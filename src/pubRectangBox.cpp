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

    // std::cout << "raysAzim = " << raysAzim << std::endl;
    // std::cout << "raysAzimQuadrant = " << raysAzimQuadrant << std::endl;

    double       lastX     = 0.0;
    unsigned int counter1B = 0;
    double       lastY     = 0.0;
    double       lastX2    = 0.0;
    double       lastY2    = 0.0;

    for(unsigned int i = 0; i < raysAzim; i++)
    {
      // TO DO: xWIDTH yWIDTH zHEIGHT factors as variables
      for(unsigned int j = 0; j < raysIncl; j++)
      {

        idx = i * raysIncl + j;

        // fill pointcloud points

        // erster Quadrant A: y konstant, x wächst, z wächst mit j
        if(i <= (0.75 * raysAzimQuadrant))
        {
          // cloud.points[idx].x = 0.1 + incrX;
          cloud.points[idx].x = 0.1 + i * 0.015;
          cloud.points[idx].y = 2.0;
          cloud.points[idx].z = 1.0 + j * 0.1;

          lastX = 0.1 + i * 0.015;
        }
        // erster Quadrant B, x konstant und y wird kleiner, z wächst mit j
        // bis 25% zweiter Quadrant A same conditions
        else if((i > (0.75 * raysAzimQuadrant)) && (i <= (1.25 * raysAzimQuadrant)))
        {
          unsigned int counter1B = i - 0.75 * raysAzimQuadrant;
          cloud.points[idx].x    = lastX;
          cloud.points[idx].y    = 2.0 - counter1B * 0.015;
          cloud.points[idx].z    = 1.0 + j * 0.1;
          // counter1B++;
          lastY = 2.0 - counter1B * 0.015;
        }
        // zweiter Quadrant B: y konstant, x wird kleiner, z same
        // bis 75% dritter Quadrant A same conditions
        else if((i > (1.25 * raysAzimQuadrant)) && (i <= (2.75 * raysAzimQuadrant)))
        // else if((i > (1.25 * raysAzimQuadrant)) && (i <= 900))
        {
          unsigned int counter2B = i - 1.25 * raysAzimQuadrant;
          // std::cout << "counter2B = " << counter2B << std::endl;
          cloud.points[idx].x = lastX - counter2B * 0.015;
          cloud.points[idx].y = lastY;
          cloud.points[idx].z = 1.0 + j * 0.1;
          lastX2              = lastX - counter2B * 0.015;
        }
        // dritter Quadrant B: x konstant, y wird größer, z same
        // bis 25% vierter Quadrant A same conditions
        else if((i > (2.75 * raysAzimQuadrant)) && (i <= (3.25 * raysAzimQuadrant)))
        {
          unsigned int counter3B = i - 2.75 * raysAzimQuadrant;

          cloud.points[idx].x = lastX2;
          cloud.points[idx].y = lastY + counter3B * 0.015;
          cloud.points[idx].z = 1.0 + j * 0.1;
          lastY2              = lastY + counter3B * 0.015;
        }
        // vierter Quadrant B: y konstant, x wird größer, z same
        else
        {
          unsigned int counter4B = i - 3.25 * raysAzimQuadrant;

          cloud.points[idx].x = lastX2 + counter4B * 0.015;
          cloud.points[idx].y = lastY2;
          cloud.points[idx].z = 1.0 + j * 0.1;
          // std::cout << "cloud.points xyz = " << cloud.points[idx].x << " " << cloud.points[idx].y << " " << cloud.points[idx].z << std::endl;
        }
      }

      // std::cout << "counter2B: " << counter2B << std::endl;

      // std::cout << "idx = " << idx << std::endl;
    }

    pubArtificialData.publish(cloud);

    // ros::spin();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
