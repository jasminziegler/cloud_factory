#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <cmath>

void timerCallBack(const ros::TimerEvent& ev);

static ros::Publisher _pubCloud;
pcl::PointCloud<pcl::PointXYZ> _cloud;

int main(int argc, char** argv)
{
ros::init(argc, argv, "the_azimuth");
ros::NodeHandle nh;

double       inclMin  = -0.26180;
  double       inclMax  = 0.26180;
  double       inclRes  = 0.034907;
  double       azimMin  = 0.0;
  double       azimMax  = 6.2832;
  double       azimRes  = 0.0034907;
  double absBase = 0.5;
  double absDiff = 0.01;

unsigned int ctrAzim = 0;
unsigned int ctrIncl = 0;

  double r = absBase;
  for(double azim = azimMin; azim < azimMax; azim += azimRes, r += absDiff)
  {
    for(double incl = inclMin; incl < inclMax; incl += inclRes)
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

ros::Timer timer = nh.createTimer(ros::Duration(1.0 / 10.0), timerCallBack);
_pubCloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("puck_rear/velodyne_points", 1);
_cloud.header.frame_id = "map";
ros::spin();
return 0;
}

void timerCallBack(const ros::TimerEvent& ev)
{
  _pubCloud.publish(_cloud);
}