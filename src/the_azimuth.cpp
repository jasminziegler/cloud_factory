#include "cloud_factory/theAzimuthConfig.h"
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

void timerCallBack(const ros::TimerEvent& ev);
void callBackDynamicReconfigure(cloud_factory::theAzimuthConfig& config, const uint32_t level);

static ros::Publisher                  _pubCloud;
pcl::PointCloud<pcl::PointXYZ>         _cloud;
static cloud_factory::theAzimuthConfig _config;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "the_azimuth");
  ros::NodeHandle                                                            nh;
  dynamic_reconfigure::Server<cloud_factory::theAzimuthConfig>               serverReconf;
  dynamic_reconfigure::Server<cloud_factory::theAzimuthConfig>::CallbackType callBackConfig; ///< ROS dynamic reconfigure object
  callBackConfig = boost::bind(callBackDynamicReconfigure, _1, _2);
  serverReconf.setCallback(callBackConfig);
  ros::Timer timer       = nh.createTimer(ros::Duration(1.0 / 10.0), timerCallBack);
  _pubCloud              = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("puck_rear/velodyne_points", 1);
  _cloud.header.frame_id = "map";

  ros::spin();
  return 0;
}

void timerCallBack(const ros::TimerEvent& ev) { _pubCloud.publish(_cloud); }

void callBackDynamicReconfigure(cloud_factory::theAzimuthConfig& config, const uint32_t level)
{
  _config = config;
  std::cout << __PRETTY_FUNCTION__ << "DynReconf Callback bitchachoes" << std::endl;
  // double inclMin = -0.26180;
  // double inclMax = 0.26180;
  // double inclRes = 0.034907;
  // double azimMin = 0.0;
  // double azimMax = 6.2832;
  // double azimRes = 0.0034907;
  // double absBase = 0.5;
  // double absDiff = 0.01;

  // dyn reconf
  double inclMin = _config.inclMin;
  double inclMax = _config.inclMax;
  double inclRes = _config.inclRes;
  double azimMin = _config.azimMin;
  double azimMax = _config.azimMax;
  double azimRes = _config.azimRes;
  double absBase = _config.absBase;
  double absDiff = _config.absDiff;

  unsigned int ctrAzim = 0;
  unsigned int ctrIncl = 0;

  double r = absBase;
  for(double azim = azimMin; azim < azimMax + 2.0 * azimRes; azim += azimRes, r += absDiff)
  {
    for(double incl = inclMin; incl < inclMax + inclRes; incl += inclRes)
    {
      double theta = 0.0;
      if(incl < 0.0)
        theta = M_PI / 2.0 + std::abs(incl);
      else
      {
        theta = M_PI / 2.0 - incl;
      }

      pcl::PointXYZ point(r * std::sin(theta) * std::cos(azim), r * std::sin(theta) * std::sin(azim), std::cos(theta));
      _cloud.push_back(point);
    }
  }
}