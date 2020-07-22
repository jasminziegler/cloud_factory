/*
 * pubRectangBox.cpp
 *
 *  Created on: Apr 20, 2020
 *      Author: jasmin
 */

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>

ros::Publisher pubArtificialData;

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
    // unsigned int raysIncl = 16;
    unsigned int raysIncl = 50;
    double       inclMin  = -15.0;
    double       inclRes  = 2.0;
    // double       azimRes  = 3.6;
    double azimRes = 0.2;

    double azimMin = 0.0;
    // double azimRes = 3.6;

    unsigned int raysAzim = round(static_cast<unsigned>(360.0 / azimRes));
    unsigned int idx      = 0;

    cloud.width    = raysAzim;
    cloud.height   = raysIncl;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    cloud.header.frame_id = "map";

    // // ich muss den inclination angle korrigieren sonst geht dit nit :(((((((((((((((())))))))))))))))
    // double inclCorr = 0.0;

    // if(currentIncl <= 0) // DAS GLEICH HAB ICH ERGÄNZT für wenn =0
    // {
    //   inclCorr = deg2rad(90.0) + (-1.0 * currentIncl);
    // }
    // else
    // {
    //   inclCorr = deg2rad(90.0) - currentIncl;
    // }

    // calcRay(0, 0) = sin(inclCorr) * cos(currentAzim);
    // calcRay(1, 0) = cos(inclCorr);
    // calcRay(2, 0) = sin(inclCorr) * sin(currentAzim);

    double inclCorr = 0.0;
    // test 1: circle on only 1 layer of velodyne
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
        cloud.points[idx].x = sin(inclCorr) * cos(currentAzim);
        cloud.points[idx].y = cos(inclCorr);
        cloud.points[idx].z = sin(inclCorr) * sin(currentAzim);
      }
    }

    // pubArtificialData.publish(cloud);

    // transform cloud: +90 ZAxis
    float           rotationAngle  = M_PI / 2;
    Eigen::Affine3f transformZAxis = Eigen::Affine3f::Identity();
    transformZAxis.translation() << 0.0, 0.0, 0.0; // 0.0 meters translation
    // transformZAxis.rotate(Eigen::AngleAxisf(rotationAngle, Eigen::Vector3f::UnitZ()));
    // transformZAxis.rotate(Eigen::AngleAxisf(rotationAngle, Eigen::Vector3f::UnitY()));
    transformZAxis.rotate(Eigen::AngleAxisf(rotationAngle, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloudZ(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(cloud, *transformedCloudZ, transformZAxis);
    pubArtificialData.publish(transformedCloudZ);
    // std::abort();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
