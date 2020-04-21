#include "PointClouds.h"
#include <cmath>
#include <random>

void PointClouds::flatPlane(const float width, const float height, const float resolution, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  const unsigned int cellsX = static_cast<unsigned int>(std::round(width / resolution));
  const unsigned int cellsY = static_cast<unsigned int>(std::round(height / resolution));
  for(unsigned int i = 0; i < cellsY; i++)
    for(unsigned int j = 0; j < cellsX; j++)
    {
      pcl::PointXYZ point;
      point.x = static_cast<float>(j) * resolution;
      point.y = static_cast<float>(i) * resolution;
      point.z = 0.0;
      cloud.push_back(point);
    }
}

void PointClouds::flatRandomPlane(const float threshWidth, const float threshHeight, const unsigned int n, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  std::random_device              rd;
  std::default_random_engine      gen(rd());
  std::uniform_real_distribution<> dist_x(-threshWidth, threshWidth);
  std::uniform_real_distribution<> dist_y(-threshHeight, threshHeight);
  
  for(unsigned int i = 0; i < n; i++)
    cloud.push_back(pcl::PointXYZ(dist_x(gen), dist_y(gen), 0.0f));
}

void PointClouds::randomRoughPlane(const float width, const float height, const float resolution, const float threshRoughness, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  std::random_device              rd;
  std::default_random_engine      gen(rd());
  std::uniform_real_distribution<> dis(-threshRoughness, threshRoughness);
 const unsigned int cellsX = static_cast<unsigned int>(std::round(width / resolution));
  const unsigned int cellsY = static_cast<unsigned int>(std::round(height / resolution));
  for(unsigned int i = 0; i < cellsY; i++)
    for(unsigned int j = 0; j < cellsX; j++)
    {
      pcl::PointXYZ point;
      point.x = static_cast<float>(j) * resolution;
      point.y = static_cast<float>(i) * resolution;
      point.z = dis(gen);
      cloud.push_back(point);
    }
}