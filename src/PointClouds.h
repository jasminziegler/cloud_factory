#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <pcl_ros/point_cloud.h>

class PointClouds
{
  public:
    static void flatPlane(const float width, const float height, const float resolution, pcl::PointCloud<pcl::PointXYZ>& cloud);
    static void flatRandomPlane(const float threshWidth, const float threshHeight, const unsigned int n, pcl::PointCloud<pcl::PointXYZ>& cloud);
    static void randomRoughPlane(const float width, const float height, const float resolution, const float threshRoughness,pcl::PointCloud<pcl::PointXYZ>& cloud);
};

#endif