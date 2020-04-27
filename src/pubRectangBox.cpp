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
    unsigned int counterAzim      = 0;
    unsigned int idx              = 0;

    cloud.width    = raysAzim;
    cloud.height   = raysIncl;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    cloud.header.frame_id = "map";

    std::cout << "rayAzim = " << raysAzim << std::endl;

    for(unsigned int i = 0; i < raysAzim; i++)
    {
      std::cout << "a" << std::endl;

      for(unsigned int j = 0; j < raysIncl; j++)
      {
        std::cout << "b" << std::endl;

        idx = i * raysIncl + j;

        // fill pointcloud points

        // AZIM: erster Quadrant: y konstant, x wächst, z kp
        if(i <= 0.75 * raysAzimQuadrant)
        {
          cloud.points[idx].x = 0.1;
          cloud.points[idx].y = 2.0;
          cloud.points[idx].z = 0.0;
          std::cout << "cloud.points xyz = " << cloud.points[idx].x << " " << cloud.points[idx].y << " " << cloud.points[idx].z << std::endl;
          counterAzim++;
        }

        else
        {
          std::cout << "else - counterAzim = " << counterAzim << std::endl;
          cloud.points[idx].x = 3.0;
          cloud.points[idx].y = 3.0;
          cloud.points[idx].z = 3.0;
          // std::cout << "cloud.points xyz = " << cloud.points[idx].x << " " << cloud.points[idx].y << " " << cloud.points[idx].z << std::endl;
        }
      }

      std::cout << "idx = " << idx << std::endl;
    }

    pubArtificialData.publish(cloud);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
