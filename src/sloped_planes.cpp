#include "PointClouds.h"
#include "cloud_factory/PlanesConfig.h"
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

static std::unique_ptr<pcl::PointCloud<pcl::PointXYZ> > _cloud;
static cloud_factory::PlanesConfig                      _config;
static ros::Publisher                                   _pubCloud;
static std::string                                      _frameCloud;

void generateCloud(const cloud_factory::PlanesConfig& config, pcl::PointCloud<pcl::PointXYZ>& cloud);
void callBackDynamicReconfigure(cloud_factory::PlanesConfig& config, const uint32_t level);
void callbackTimer(const ros::TimerEvent&);

enum class TypePlane
{
  RANDOM = 0,
  REGULAR
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "random_planes");
  ros::NodeHandle nh;
  ros::NodeHandle prvNh("~");
  std::string     topicCloud;
  double          frameRate = 0.0;
  prvNh.param<std::string>("topic_cloud", topicCloud, "transformed_cloud");
  prvNh.param<std::string>("frame_cloud", _frameCloud, "map");
  prvNh.param<double>("frame_rate", frameRate, 20.0);
  _pubCloud        = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >(topicCloud, 1);
  ros::Timer timer = nh.createTimer(ros::Duration(1.0 / frameRate), callbackTimer);
  _cloud           = std::make_unique<pcl::PointCloud<pcl::PointXYZ> >();
  dynamic_reconfigure::Server<cloud_factory::PlanesConfig>               serverReconf;
  dynamic_reconfigure::Server<cloud_factory::PlanesConfig>::CallbackType callBackConfig; ///< ROS dynamic reconfigure object
  callBackConfig = boost::bind(callBackDynamicReconfigure, _1, _2);
  serverReconf.setCallback(callBackConfig);

  ros::spin();
  return 0;
}

void generateCloud(const cloud_factory::PlanesConfig& config, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  if(!_cloud)
    return;
  std::cout << __PRETTY_FUNCTION__ << " huhu " << std::endl;
  cloud.clear();
  cloud.header.frame_id = _frameCloud;
  cloud.header.stamp    = pcl_conversions::toPCL(ros::Time::now());
  std::cout << __PRETTY_FUNCTION__ << " size in " << cloud.size() << std::endl;
  if(static_cast<TypePlane>(_config.mode) == TypePlane::RANDOM)
    PointClouds::randomSlopedPlain(cloud, config.thresh_x, config.thresh_y, config.n_points, config.variance_z, config.slope_x, config.slope_y);
  else
    PointClouds::regularSlopedPlain(cloud, config.thresh_x, config.thresh_y, config.resolution, config.slope_x, config.slope_y, config.variance_z);
  std::cout << __PRETTY_FUNCTION__ << " size out " << cloud.size() << std::endl;
}

void callBackDynamicReconfigure(cloud_factory::PlanesConfig& config, const uint32_t level)
{
  _config = config;
  generateCloud(config, *_cloud.get());
}

void callbackTimer(const ros::TimerEvent&)
{
  if(!_cloud)
    return;
  _pubCloud.publish(*_cloud);
}