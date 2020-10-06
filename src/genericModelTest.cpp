#include <cmath>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

void timerCallBack(const ros::TimerEvent& ev);

static ros::Publisher          _pubCloud;
pcl::PointCloud<pcl::PointXYZ> _cloud;

int         _raysIncl;
std::string _sensorType;
std::string _laserDataTopic;
std::string _tfLaserDataFrameID;
double      _inclMin;
double      _inclMax;
double      _inclRes;
double      _azimMin;
double      _azimMax;
double      _azimRes;
double      _radiusCloud;
double      _radiusIncrement;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "generic_model_data");
  ros::NodeHandle _prvnh("~");
  ros::NodeHandle _nh;

  ros::Timer timer = _nh.createTimer(ros::Duration(1.0 / 10.0), timerCallBack);

  // launchfile parameters, default vals for VLP16
  _prvnh.param<std::string>("sensorType", _sensorType, "VLP16");
  _prvnh.param<std::string>("laserDataTopic", _laserDataTopic, "puck_rear/velodyne_points");
  _prvnh.param<std::string>("tfLaserDataFrameID", _tfLaserDataFrameID, "map");
  _prvnh.param<int>("raysIncl", _raysIncl, 16);
  _prvnh.param<double>("inclMin", _inclMin, -0.26180); //-15°
  _prvnh.param<double>("inclMax", _inclMax, 0.26180);  //+15°
  _prvnh.param<double>("inclRes", _inclRes, 0.034907); // 2°
  _prvnh.param<double>("azimMin", _azimMin, 0.0);
  _prvnh.param<double>("azimMax", _azimMax, 6.28319);    // 360°
  _prvnh.param<double>("azimRes", _azimRes, 0.00349066); // 0.2°
  _prvnh.param<double>("radiusCloud", _radiusCloud, 0.1);
  _prvnh.param<double>("radiusIncrement", _radiusIncrement, 0.0001);

  std::cout << __PRETTY_FUNCTION__ << " LAUNCH CHECK " << std::endl;
  std::cout << "sensorType = " << _sensorType << std::endl;
  std::cout << "laserDataTopic = " << _laserDataTopic << std::endl;
  std::cout << "tfLaserDataFrameID = " << _tfLaserDataFrameID << std::endl;
  std::cout << "raysIncl = " << _raysIncl << std::endl;
  std::cout << "inclMin = " << _inclMin << " = " << RAD2DEG(_inclMin) << "°" << std::endl;
  std::cout << "inclMax = " << _inclMax << " = " << RAD2DEG(_inclMax) << "°" << std::endl;
  std::cout << "inclRes = " << _inclRes << " = " << RAD2DEG(_inclRes) << "°" << std::endl;
  std::cout << "azimMin = " << _azimMin << " = " << RAD2DEG(_azimMin) << "°" << std::endl;
  std::cout << "azimMax = " << _azimMax << " = " << RAD2DEG(_azimMax) << "°" << std::endl;
  std::cout << "azimRes = " << _azimRes << " = " << RAD2DEG(_azimRes) << "°" << std::endl;
  std::cout << "radiusCloud = " << _radiusCloud << std::endl;
  std::cout << "radiusIncrement = " << _radiusIncrement << std::endl;

  unsigned int cnt       = 0;
  unsigned int countAzim = 0;
  unsigned int countIncl = 0;

  //   double inclSteps = abs(_inclMin - _inclMax) / _raysIncl;

  //   for(double azim = _azimMin; azim < _azimMax - _azimRes; azim += _azimRes, countAzim++)
  //   {
  //     for(double incl = _inclMin; incl < _inclMax + _inclRes; incl += inclSteps, countIncl++)
  //     {
  //       double theta = 0.0;
  //       if(incl < 0.0)
  //         theta = M_PI / 2.0 + std::abs(incl);
  //       else
  //       {
  //         theta = M_PI / 2.0 - incl;
  //       }

  //       pcl::PointXYZ point(_radiusCloud * std::sin(theta) * std::cos(azim), _radiusCloud * std::sin(theta) * std::sin(azim), _radiusCloud *
  //       std::cos(theta));

  //       _cloud.push_back(point);

  //       _radiusCloud += _radiusIncrement;
  //       cnt++;
  //     }
  //     _radiusCloud += _radiusIncrement;
  //   }

  // worked for VLP16, HDL32, OUSTER
  for(double azim = _azimMin; azim < _azimMax - _azimRes; azim += _azimRes, countAzim++)
  {
    for(double incl = _inclMin; incl < _inclMax + _inclRes; incl += _inclRes, countIncl++) //works for VLP16, HDL32, OUSTER
        // for(double incl = _inclMin; incl <= _inclMax-_inclRes; incl += _inclRes, countIncl++) //works for tilt4d

    {
      double theta = 0.0;
      if(incl < 0.0)
        theta = M_PI / 2.0 + std::abs(incl);
      else
      {
        theta = M_PI / 2.0 - incl;
      }

      pcl::PointXYZ point(_radiusCloud * std::sin(theta) * std::cos(azim), _radiusCloud * std::sin(theta) * std::sin(azim), _radiusCloud * std::cos(theta));

      _cloud.push_back(point);

      _radiusCloud += _radiusIncrement;
      cnt++;
    }
    _radiusCloud += _radiusIncrement;
  }
  std::cout << __PRETTY_FUNCTION__ << "cnt = " << cnt << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "countAzim = " << countAzim << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "countIncl = " << countIncl/countAzim << std::endl;

  _pubCloud              = _nh.advertise<pcl::PointCloud<pcl::PointXYZ> >(_laserDataTopic, 1);
  _cloud.header.frame_id = _tfLaserDataFrameID;

  ros::spin();
  return 0;
}

void timerCallBack(const ros::TimerEvent& ev) { _pubCloud.publish(_cloud); }
