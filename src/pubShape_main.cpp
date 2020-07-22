#include "PubShape.h"
#include "cloud_factory/PubShapeConfig.h"
#include <dynamic_reconfigure/server.h>
#include <memory>
#include <ros/ros.h>
#include <string>

static std::unique_ptr<pcl::PointCloud<pcl::PointXYZ> > _cloud;
static cloud_factory::PubShapeConfig                    _config;
static ros::Publisher                                   _pubCloud;
static std::string                                      _frameCloud;

void generateCloud(const cloud_factory::PubShapeConfig& config, pcl::PointCloud<pcl::PointXYZ>& cloud);
void callBackDynamicReconfigure(cloud_factory::PubShapeConfig& config, const uint32_t level);
void callbackTimer(const ros::TimerEvent&);

int main(int argc, char** argv)
{

  ros::init(argc, argv, "pub_shape");
  ros::NodeHandle nh;
  ros::NodeHandle prvNh("~");
  std::string     topicCloud;
  double          frameRate = 0.0;
  prvNh.param<std::string>("topic_cloud", topicCloud, "cloud");
  //   prvNh.param<std::string>("frame_cloud", _frameCloud, "map");
  prvNh.param<double>("frame_rate", frameRate, 20.0);
  _pubCloud        = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("puck_rear/velodyne_points", 1);
  ros::Timer timer = nh.createTimer(ros::Duration(1.0 / frameRate), callbackTimer);
  _cloud           = std::make_unique<pcl::PointCloud<pcl::PointXYZ> >();
  dynamic_reconfigure::Server<cloud_factory::PubShapeConfig>               serverReconf;
  dynamic_reconfigure::Server<cloud_factory::PubShapeConfig>::CallbackType callBackConfig; ///< ROS dynamic reconfigure object
  callBackConfig = boost::bind(callBackDynamicReconfigure, _1, _2);
  serverReconf.setCallback(callBackConfig);

  ros::spin();
  return 0;
}

void callBackDynamicReconfigure(cloud_factory::PubShapeConfig& config, const uint32_t level)
{
  if(!_cloud)
    return;
  _config = config;
  generateCloud(_config, *_cloud.get());
}

void callbackTimer(const ros::TimerEvent&)
{
  if(!_cloud)
    return;
  _pubCloud.publish(*_cloud); // TODO Maybe add rotations
}

// gen.add("mode", int_t, 0, "Modes available 0 = thorus, 1 = rectangle", 0, 0, 1)
// gen.add("raysIncl", int_t, 0, "vertical inclination rays (layers)", 16, 0, 50)
// gen.add("inclMin", double_t, 0, "inclMin: lowest inclination angle", -15.0, -45.0, 45.0)
// gen.add("inclRes", double_t, 0, "inclRes: resolution of inclination angle", 2.0, 0.1, 4.0)
// gen.add("azimRes", double_t, 0, "azimRes: resolution of azimuth angle", 0.2, 0.1, 3.6)
// gen.add("azimMin", double_t, 0, "azimMin: (smallest) azimuth start angle", 0.2, 0.1, 3.6)
void generateCloud(const cloud_factory::PubShapeConfig& config, pcl::PointCloud<pcl::PointXYZ>& returnCloud)
{
  if(!_cloud)
    return;
  returnCloud.clear();
  returnCloud.header.frame_id = _frameCloud;
  returnCloud.header.stamp    = pcl_conversions::toPCL(ros::Time::now());

  switch(config.mode)
  {
  case(0):
  {
    PubShape::pubThorus(config.raysIncl, config.inclMin, config.inclRes, config.azimRes, config.azimMin, returnCloud);
    break;
  }
  case(1):
  {
    PubShape::pubRectang(config.raysIncl, config.azimRes, config.xWidth, config.yWidth, config.zHeight, config.xStart, config.yStart, config.zStart,
                         returnCloud);
    break;
  }
  default:
  {
    std::cout << __PRETTY_FUNCTION__ << " error. Mode " << config.mode << " not supported " << std::endl;
    break;
  }
  }
}