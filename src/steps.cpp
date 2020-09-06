#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "cloud_factory/StepsConfig.h"
#include <dynamic_reconfigure/server.h>
#include <Eigen/Dense>

static pcl::PointCloud<pcl::PointXYZ> _cloud;
static ros::Publisher                 _pubCloud;

void callbackTimer(const ros::TimerEvent& ev); 
void callBackDynamicReconfigure(cloud_factory::StepsConfig& config, const uint32_t level);
void createCloud(const cloud_factory::StepsConfig& config);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "steps");
  ros::NodeHandle nh;

  // double width       = 10.0;
  // double height      = 10.0;
  // double resolution  = 0.1;
  // double stepHeight  = 1.0;
  // int    nPointsStep = 10;

  // const unsigned int nWidth = static_cast<unsigned int>(std::round(width / resolution));
  // const unsigned int nHeight = static_cast<unsigned int>(std::round(height / resolution));

  // for(unsigned int i = 0; i < nWidth; i++)
  //   for(unsigned int j = 0; j < nHeight; j++)
  //   {
  //     if(i < (nWidth / 2))
  //     {
  //       _cloud.push_back(pcl::PointXYZ(static_cast<float>(j) * resolution, static_cast<float>(i) * resolution, 0.0));
  //     }
  //     else if(i > (nWidth / 2))
  //     {
  //       _cloud.push_back(pcl::PointXYZ(static_cast<float>(j) * resolution, static_cast<float>(i) * resolution, stepHeight));
  //     }
  //     else
  //     { 
  //       continue;
  //     }
  //   }
  // const float heightStep = stepHeight / static_cast<float>(nPointsStep);
  // for(unsigned int i = 0; i < nWidth; i++)
  //   for(unsigned int j = 0; j <= nPointsStep; j++)
  //     _cloud.push_back(pcl::PointXYZ(static_cast<float>(i) * resolution, height / 2.0f, static_cast<float>(j) * heightStep));

  ros::Timer timerMain = nh.createTimer(ros::Duration(1.0 / 20.0), callbackTimer);
  _pubCloud        = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("transformed_cloud", 1);
  _cloud.header.frame_id = "map";
  dynamic_reconfigure::Server<cloud_factory::StepsConfig>               serverReconf;
  dynamic_reconfigure::Server<cloud_factory::StepsConfig>::CallbackType callBackConfig; ///< ROS dynamic reconfigure object
  callBackConfig = boost::bind(callBackDynamicReconfigure, _1, _2);
  serverReconf.setCallback(callBackConfig);
  //createCloud()
  ros::spin();
  return 0;
}

void callbackTimer(const ros::TimerEvent& ev)
{
  pcl_conversions::toPCL(ros::Time::now(), _cloud.header.stamp);
  _pubCloud.publish(_cloud);
} 

void callBackDynamicReconfigure(cloud_factory::StepsConfig& config, const uint32_t level)
{
  //if(_cloud.size())
    createCloud(config);
}

void createCloud(const cloud_factory::StepsConfig& config)
{
  _cloud.clear();
  const double width       = config.width;
  const double height      = config.height;
  const double resolution  = config.resolution;
  const double stepHeight  = config.step_height;
  const int    nPointsStep = config.n_points_step;

  const unsigned int nWidth = static_cast<unsigned int>(std::round(width / resolution));
  const unsigned int nHeight = static_cast<unsigned int>(std::round(height / resolution));

  for(unsigned int i = 0; i < nWidth; i++)
    for(unsigned int j = 0; j < nHeight; j++)
    {
      if(i < (nWidth / 2))
      {
        _cloud.push_back(pcl::PointXYZ(static_cast<float>(j) * resolution, static_cast<float>(i) * resolution, 0.0));
      }
      else if(i > (nWidth / 2))
      {
        _cloud.push_back(pcl::PointXYZ(static_cast<float>(j) * resolution, static_cast<float>(i) * resolution, stepHeight));
      }
      else
      { 
        continue;
      }
    }
  const float heightStep = stepHeight / static_cast<float>(nPointsStep);
  for(unsigned int i = 0; i < nWidth; i++)
    for(unsigned int j = 0; j <= nPointsStep; j++)
      _cloud.push_back(pcl::PointXYZ(static_cast<float>(i) * resolution, height / 2.0f, static_cast<float>(j) * heightStep));
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform.translation() << -width / 2.0, -height / 2.0, 0.0;
  pcl::transformPointCloud(_cloud, _cloud, transform);
}