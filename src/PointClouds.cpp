#include "PointClouds.h"
#include <cmath>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
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
  Eigen::Vector2f center(width / 2.0f, height / 2.0f);
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -center.x(), -center.y(), 0.0f;
  pcl::transformPointCloud(cloud, cloud, transform);
}

void PointClouds::flatRandomPlane(const float threshWidth, const float threshHeight, const unsigned int n, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  std::random_device                rd;
  std::default_random_engine        gen(rd());
  std::uniform_real_distribution<>  dist_x(-threshWidth, threshWidth);
  std::uniform_real_distribution<>  dist_y(-threshHeight, threshHeight);
  pcl::CentroidPoint<pcl::PointXYZ> centroid;

  for(unsigned int i = 0; i < n; i++)
  {
    pcl::PointXYZ point(dist_x(gen), dist_y(gen), 0.0f);
    centroid.add(point);
    cloud.push_back(point);
  }
  pcl::PointXYZ center;
  centroid.get(center);
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -center.x, -center.y, 0.0f;
  pcl::transformPointCloud(cloud, cloud, transform);
}

void PointClouds::randomRoughPlane(const float width, const float height, const float resolution, const float threshRoughness,
                                   pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  std::random_device               rd;
  std::default_random_engine       gen(rd());
  std::uniform_real_distribution<> dis(-threshRoughness, threshRoughness);
  const unsigned int               cellsX = static_cast<unsigned int>(std::round(width / resolution));
  const unsigned int               cellsY = static_cast<unsigned int>(std::round(height / resolution));
  for(unsigned int i = 0; i < cellsY; i++)
    for(unsigned int j = 0; j < cellsX; j++)
    {
      pcl::PointXYZ point;
      point.x = static_cast<float>(j) * resolution;
      point.y = static_cast<float>(i) * resolution;
      point.z = dis(gen);
      cloud.push_back(point);
    }
  Eigen::Vector2f center(width / 2.0f, height / 2.0f);
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -center.x(), -center.y(), 0.0f;
  pcl::transformPointCloud(cloud, cloud, transform);
}

void PointClouds::planeWithGaussian(const float width, const float height, const float resolution, const float A, const Eigen::Vector2f& center,
                                    const float sigmaX, const float sigmaY, const float powerX, const float powerY, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  const unsigned int cellsX = static_cast<unsigned int>(std::round(width / resolution));
  const unsigned int cellsY = static_cast<unsigned int>(std::round(height / resolution));
  for(unsigned int i = 0; i < cellsY; i++)
    for(unsigned int j = 0; j < cellsX; j++)
    {
      pcl::PointXYZ point;

      point.x              = static_cast<float>(j) * resolution;
      point.y              = static_cast<float>(i) * resolution;
      Eigen::Vector2f diff = Eigen::Vector2f(point.x, point.y) - center;
      point.z              = A * std::exp(-(std::pow((diff.x() * diff.x()) / (2.0 * (sigmaX * sigmaX)), powerX)) -
                             (std::pow((diff.y() * diff.y()) / (2.0 * sigmaY * sigmaY), powerY)));
      cloud.push_back(point);
    }
  Eigen::Vector2f centerCloud(width / 2.0f, height / 2.0f);
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -centerCloud.x(), -centerCloud.y(), 0.0f;
  pcl::transformPointCloud(cloud, cloud, transform);
}
