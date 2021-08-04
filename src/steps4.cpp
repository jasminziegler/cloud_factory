#include "cloud_factory/Steps4Config.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <random>
#include <ros/ros.h>

static pcl::PointCloud<pcl::PointXYZ> _cloud;
static ros::Publisher _pubCloud;

void callbackTimer(const ros::TimerEvent &ev);
void callBackDynamicReconfigure(cloud_factory::Steps4Config &config,
                                const uint32_t level);
void createCloud(const cloud_factory::Steps4Config &config);
void randomReal(const unsigned int n, std::vector<float> &vals,
                const float threshN, const float threshP);

int main(int argc, char **argv) {
  ros::init(argc, argv, "steps4");
  ros::NodeHandle nh;

  ros::Timer timerMain =
      nh.createTimer(ros::Duration(1.0 / 20.0), callbackTimer);
  _pubCloud =
      nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("transformed_cloud", 1);
  _cloud.header.frame_id = "map";
  dynamic_reconfigure::Server<cloud_factory::Steps4Config> serverReconf;
  dynamic_reconfigure::Server<cloud_factory::Steps4Config>::CallbackType
      callBackConfig; ///< ROS dynamic reconfigure object
  callBackConfig = boost::bind(callBackDynamicReconfigure, _1, _2);
  serverReconf.setCallback(callBackConfig);
  // createCloud()
  ros::spin();
  return 0;
}

void callbackTimer(const ros::TimerEvent &ev) {
  pcl_conversions::toPCL(ros::Time::now(), _cloud.header.stamp);
  _pubCloud.publish(_cloud);
}

void callBackDynamicReconfigure(cloud_factory::Steps4Config &config,
                                const uint32_t level) {
  // if(_cloud.size())
  createCloud(config);
}

void createCloud(const cloud_factory::Steps4Config &config) {
  _cloud.clear();
  const double width = config.width;
  const double height = config.height;
  const double resolution = config.resolution;
  const double stepHeight = config.step_height_0;
  const double stepHeight1 = config.step_height_1;
  const double stepHeight2 = config.step_height_2;
  const double stepHeight3 = config.step_height_3;
  const int nPointsStep = config.n_points_step;

  const unsigned int nWidth =
      static_cast<unsigned int>(std::round(width / resolution));
  const unsigned int nHeight =
      static_cast<unsigned int>(std::round(height / resolution));

  const unsigned int size = nWidth * nHeight;
  std::vector<float> noiseVals;
  randomReal(size, noiseVals, -config.noise_points, config.noise_points);

  for (unsigned int i = 0; i < nHeight; i++)
    for (unsigned int j = 0; j < nWidth; j++) {
      const unsigned int idx = i * nWidth + j;
      if (i <= nWidth / 4) {
        _cloud.push_back(pcl::PointXYZ(static_cast<float>(j) * resolution,
                                       static_cast<float>(i) * resolution,
                                       0.0 + noiseVals[idx]));
      } else if ((i > nWidth / 4) && (i <= 2 * nWidth / 4)) {
        _cloud.push_back(pcl::PointXYZ(static_cast<float>(j) * resolution,
                                       static_cast<float>(i) * resolution,
                                       stepHeight1 + noiseVals[idx]));
      } else if ((i > 2 * nWidth / 4) && (i <= 3 * nWidth / 4)) {
        _cloud.push_back(pcl::PointXYZ(static_cast<float>(j) * resolution,
                                       static_cast<float>(i) * resolution,
                                       stepHeight2 + noiseVals[idx]));
      } else {
        _cloud.push_back(pcl::PointXYZ(static_cast<float>(j) * resolution,
                                       static_cast<float>(i) * resolution,
                                       stepHeight3 + noiseVals[idx]));
      }
    }
  float heightStep = stepHeight1 / static_cast<float>(nPointsStep);

  for (unsigned int i = 0; i < nWidth; i++)
    for (unsigned int j = 0; j <= nPointsStep; j++)
      _cloud.push_back(pcl::PointXYZ(static_cast<float>(i) * resolution,
                                     width / 4.0f,
                                     static_cast<float>(j) * heightStep));
  heightStep = (stepHeight2 - stepHeight1) / static_cast<float>(nPointsStep);
  for (unsigned int i = 0; i < nWidth; i++)
    for (unsigned int j = 0; j <= nPointsStep; j++)
      _cloud.push_back(pcl::PointXYZ(static_cast<float>(i) * resolution,
                                     2.0 * width / 4.0f,
                                     stepHeight1 + static_cast<float>(j) * heightStep));
 heightStep = (stepHeight3 - stepHeight2) / static_cast<float>(nPointsStep);                                    
  for (unsigned int i = 0; i < nWidth; i++)
    for (unsigned int j = 0; j <= nPointsStep; j++)
      _cloud.push_back(pcl::PointXYZ(static_cast<float>(i) * resolution,
                                     3.0 * width / 4.0f,
                                     stepHeight2 + static_cast<float>(j) * heightStep));
  // for (unsigned int i = 0; i < nWidth; i++)
  //   for (unsigned int j = 0; j <= nPointsStep; j++)
  //     _cloud.push_back(pcl::PointXYZ(static_cast<float>(i) * resolution,
  //                                    height / 2.0f,
  //                                    static_cast<float>(j) * heightStep));
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform.translation() << -width / 2.0, -height / 2.0, 0.0;
  pcl::transformPointCloud(_cloud, _cloud, transform);
}

void randomReal(const unsigned int n, std::vector<float> &vals,
                const float threshN, const float threshP) {
  std::random_device rd;
  std::default_random_engine gen(rd());
  std::uniform_real_distribution<> dis(threshN, threshP);
  for (unsigned int i = 0; i < n; i++)
    vals.push_back(dis(gen));
}