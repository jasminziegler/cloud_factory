#include "PointClouds.h"
#include "cloud_factory/RandomCloudsConfig.h"
#include <dynamic_reconfigure/server.h>
#include <memory>
#include <ros/ros.h>
#include <string>

static std::unique_ptr<pcl::PointCloud<pcl::PointXYZ> > _cloud;
static cloud_factory::RandomCloudsConfig                _config;
static ros::Publisher                                   _pubCloud;
static std::string                                      _frameCloud;

void generateCloud(const cloud_factory::RandomCloudsConfig& config, pcl::PointCloud<pcl::PointXYZ>& cloud);
void callBackDynamicReconfigure(cloud_factory::RandomCloudsConfig& config, const uint32_t level);
void callbackTimer(const ros::TimerEvent&);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "phils");
  ros::NodeHandle nh;
  ros::NodeHandle prvNh("~");
  std::string     topicCloud;
  double          frameRate = 0.0;
  prvNh.param<std::string>("topic_cloud", topicCloud, "cloud");
  prvNh.param<std::string>("frame_cloud", _frameCloud, "base_link");
  prvNh.param<double>("frame_rate", frameRate, 20.0);
  _pubCloud                                                                          = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >(topicCloud, 1);
  ros::Timer                                                                   timer = nh.createTimer(ros::Duration(1.0 / frameRate), callbackTimer);
  dynamic_reconfigure::Server<cloud_factory::RandomCloudsConfig>               serverReconf;
  dynamic_reconfigure::Server<cloud_factory::RandomCloudsConfig>::CallbackType callBackConfig; ///< ROS dynamic reconfigure object
  callBackConfig = boost::bind(callBackDynamicReconfigure, _1, _2);
  serverReconf.setCallback(callBackConfig);
  ros::spin();
  return 0;
}

void callBackDynamicReconfigure(cloud_factory::RandomCloudsConfig& config, const uint32_t level) { generateCloud(_config, *_cloud.get()); }

void callbackTimer(const ros::TimerEvent&) { _pubCloud.publish(*_cloud); }

void generateCloud(const cloud_factory::RandomCloudsConfig& config, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  switch(config.mode) // toDO: find something more readable...enum class or something similar
  {
  case(0):
  {
    PointClouds::flatPlane(config.width, config.height, config.resolution, cloud);
    break;
  }
  case(1):
  {
    PointClouds::flatRandomPlane(config.width, config.height, config.resolution, cloud);
    break;
  }
  case(2):
  {
    PointClouds::randomRoughPlane(config.width, config.height, config.resolution, config.roughness, cloud);
    break;
  }
  default:
  {
    std::cout << __PRETTY_FUNCTION__ << " error. Mode " << config.mode << " not supported " << std::endl;
    break;
  }
  }
}