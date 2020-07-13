#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>

class PointClouds
{
public:
  /**
   * @brief flatPlane
   * A method to generate a flat plane point cloud
   * @param width width of the point cloud in m
   * @param height height of the point cloud in m
   * @param resolution resolution of the point cloud in m
   * @param cloud pcl point cloud to store the points in
   */
  static void flatPlane(const float width, const float height, const float resolution, pcl::PointCloud<pcl::PointXYZ>& cloud);

  /**
   * @brief flatRandomPlane
   * A method to generate a flat random plane point cloud
   * @param threshWidth thresh width for random number generation
   * @param threshHeight thresh height for random number generation
   * @param n number of points in the random plane
   * @param cloud pcl point cloud to store the points in
   */
  static void flatRandomPlane(const float threshWidth, const float threshHeight, const unsigned int n, pcl::PointCloud<pcl::PointXYZ>& cloud);

  /**
   * @brief randomRoughPlane
   * A method to generate a plane with a rough surface point cloud
   * @param width width of the point cloud in m
   * @param height height of the point cloud in m
   * @param resolution resolution of the point cloud in m
   * @param threshRoughness thresh for the random roughness
   * @param cloud pcl point cloud to store the points in
   */
  static void randomRoughPlane(const float width, const float height, const float resolution, const float threshRoughness,
                               pcl::PointCloud<pcl::PointXYZ>& cloud);
  /**
   * @brief planeWithGaussian
   * A method to generate a plane with a gaussian obstacle, using super-Gaussian-method from wikipedia (https://en.wikipedia.org/wiki/Gaussian_function)
   * @param width width of the point cloud in m
   * @param height height of the point cloud in m
   * @param resolution resolution of the point cloud in m
   * @param A amplitude of the gaussian
   * @param center Eigen::Vector2f containing the center of the gaussian curve (expectation)
   * @param sigmaX stddeviaton in x
   * @param sigmaY stddeviation in y
   * @param powerX super Gaussian exponent x
   * @param powerY super Gaussian exponent y
   * @param cloud pcl point cloud to store the points in
   */
  static void planeWithGaussian(const float width, const float height, const float resolution, const float A, const Eigen::Vector2f& center,
                                const float sigmaX, const float sigmaY, const float powerX, const float powerY, pcl::PointCloud<pcl::PointXYZ>& cloud);

  /**
   * @brief 
   * 
   * @param cloud 
   * @param threshX 
   * @param threshY 
   * @param nPoints 
   * @param zVariance 
   * @param slopeX 
   * @param slopeY 
   */
  static void randomSlopedPlain(pcl::PointCloud<pcl::PointXYZ>& cloud, const float threshX, const float threshY, const unsigned int nPoints, const float zVariance,
                         const float slopeX, const float slopeY);
  /**
   * @brief 
   * 
   * @param cloud 
   * @param threshX 
   * @param threshY 
   * @param resolution 
   * @param slopeX 
   * @param slopeY 
   * @param zVariance 
   */
  static void regularSlopedPlain(pcl::PointCloud<pcl::PointXYZ>& cloud, const float threshX, const float threshY, const float resolution, const float slopeX,
                               const float slopeY, const float zVariance);
};

#endif