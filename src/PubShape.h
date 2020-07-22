/*
 * PubShape.h
 *
 *  Created on: Jul 22, 2020
 *      Author: jasmin
 */

#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>

class PubShape
{
public:
  static void pubThorus(const int raysIncl, const float inclMin, const float inclRes, const float azimRes, const float azimMin,
                        pcl::PointCloud<pcl::PointXYZ>& thorusCloud);
  static void pubRectang(const int raysIncl, const float azimRes, const float xWidth, const float yWidth, const float zHeight, float xStart, float yStart,
                         float zStart, pcl::PointCloud<pcl::PointXYZ>& rectangleCloud);

private:
  /**
   * @function rad
   * @brief degree to rad conversion
   * @param deg degree value
   * @return rad rad value
   **/
  static inline double deg2rad(const double deg) { return ((M_PI * deg) / 180.0); }

  /**
   * @function deg
   * @brief rad to degree conversion
   * @param rad rad value
   * @return deg degree value
   **/
  static inline double rad2deg(const double rad) { return ((rad * 180.0) / M_PI); }

  // private:
  // void init(const pcl::PointCloud<pcl::PointXYZ>& cloud);
};