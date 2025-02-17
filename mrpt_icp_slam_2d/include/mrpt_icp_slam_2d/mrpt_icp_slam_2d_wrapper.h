/*
 * File: mrpt_icp_slam_2d_wrapper.h
 * Author: Vladislav Tananaev
 *
 */

#ifndef MRPT_ICP_SLAM_2D_WRAPPER_H
#define MRPT_ICP_SLAM_2D_WRAPPER_H

// MRPT libraries
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // This class lives in the lib [mrpt-maps] and must be included by hand
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>

#include <mrpt/gui/CDisplayWindow3D.h>

#include <stdint.h>
#include <iostream>  // std::cout
#include <fstream>   // std::ifstream
#include <string>

// add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// add ros msgs
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/MapMetaData.h"
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// mrpt bridge libs
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/mrpt_log_macros.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/time.h>
#include <mrpt_bridge/point_cloud.h>
#include <mrpt/version.h>
#if MRPT_VERSION >= 0x130
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CRawlog.h>
using namespace mrpt::maps;
using namespace mrpt::obs;
#else
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/slam/CActionRobotMovement3D.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/slam/CRawlog.h>
#endif

#include <queue>
#include <tf_conversions/tf_eigen.h>
#include "mrpt_icp_slam_2d/ObservationWithTransform.h"

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

/**
 * @brief The ICPslamWrapper class provides 2d icp based SLAM from MRPT libraries.
 *
 */
class ICPslamWrapper
{
public:
  /**
   * @brief constructor
   */
  ICPslamWrapper();

  /**
  * @brief destructor
  */
  ~ICPslamWrapper();

  /**
   * @brief read ini file
   *
   * @param ini_filename the name of the ini file to read
   */
  void read_iniFile(std::string ini_filename);
  /**
   * @brief init 3D window from mrpt lib
   */
  void init3Dwindow();
  /**
   * @brief run 3D window update from mrpt lib
   */
  void run3Dwindow();

  /**
   * @brief read the parameters from launch file
   */
  void get_param();
  /**
   * @brief initialize publishers subscribers and icp slam
   */
  void init();
  /**
   * @brief play rawlog file
   *
   * @return true if rawlog file exists and played
   */
  bool rawlogPlay();
  /**
   * @brief check the existance of the file
   *
   * @return true if file exists
   */
  bool is_file_exists(const std::string &name);
  /**
   * @brief callback function for the laser scans
   *
   * Given the laser scans,
   * implement one SLAM update,
   * publish map and pose.
   *
   * @param _msg  the laser scan message
   */
  void laserCallback(const sensor_msgs::LaserScan &_msg);
  
  /**
  * @brief callback function for the observations from other robots
  *
  * Given the laser scans or range only observation with odometry,
  * create the pair of action and observation,
  * implement one SLAM update,
  * publish map and pose.
  *
  * @param _msg ObservationWithTransform message
  */
  void multirobotCallback(const mrpt_icp_slam_2d::ObservationWithTransform& _msg);
  
  /**
  * @brief Convert ObservationWithTransform to CObservation
  *
  * @param _msg ObservationWithTransform message
  * @param _stamp timestamp of odometry
  */
  CObservation::Ptr msgToObservation(const mrpt_icp_slam_2d::ObservationWithTransform& _msg, const ros::Time& _stamp);
  
  /**
  * @brief publish observations to other robots
  *
  * @param scan observation
  * @param beacon observation
  * @param _frame_id frame of sensor
  */
  void publishObservations(const sensor_msgs::LaserScan& scan, const ros::Time& _stamp);
  
  /**
    * @brief  publis tf tree
    *
    */
  void publishTF();
  /**
  * @brief publish point and/or grid map and robot pose
  *
  */
  void publishMapPose();
  /**
    * @brief  update the pose of the sensor with respect to the robot
    *
    *@param frame_id the frame of the sensors
    */
  void updateSensorPose(std::string _frame_id);
  /**
    * @brief  the callback for update trajectory
    *
    *@param event the event for update trajectory
    */
  void updateTrajectoryTimerCallback(const ros::TimerEvent& event);
  /**
    * @brief  the callback for publish trajectory
    *
    *@param event the event for publish trajectory
    */
  void publishTrajectoryTimerCallback(const ros::TimerEvent& event);

protected:
  CMetricMapBuilderICP mapBuilder;  ///< icp slam class
  ros::NodeHandle n_;               ///< Node Handle
  double rawlog_play_delay;         ///< delay of replay from rawlog file
  bool rawlog_play_;                ///< true if rawlog file exists
  bool run_multi_robot;             ///< multi-robot mode
  int robots_count;                 ///< number of robots
  int obs_batch = 1;                ///< number of observation from other robots to add to current action
  int init_x, init_y, init_phi;     ///< init pose

  std::string rawlog_filename;  ///< name of rawlog file
  std::string ini_filename;     ///< name of ini file
  std::string global_frame_id;  ///< /map frame
  std::string odom_frame_id;    ///< /odom frame
  std::string base_frame_id;    ///< robot frame
  std::string base_ns;          ///< constant part of namespace for multi-robot
  geometry_msgs::PoseStamped pose; ///< the robot pose

  ros::Publisher trajectory_pub_;  ///< trajectory publisher
  nav_msgs::Path path;             ///< trajectory path

  ros::Timer update_trajector_timer;   ///< timer for update trajectory
  ros::Timer publish_trajectory_timer; ///< timer for publish trajectory

  double trajectory_update_rate;    ///< trajectory update rate(Hz)
  double trajectory_publish_rate;   ///< trajectory publish rate(Hz)

  // Sensor source
  std::string sensor_source;                                 ///< 2D laser scans
  std::map<std::string, mrpt::poses::CPose3D> laser_poses_;  ///< laser scan poses with respect to the map
  std::string robots_source;  ///< observations and transforms

  // Subscribers
  std::vector<ros::Subscriber> sensorSub_;  ///< list of sensors topics
  std::vector<ros::Subscriber> robotsSubs;  ///< list of robots topics

  const CMultiMetricMap *metric_map_;  ///<receive map after iteration of SLAM to metric map
  // CPose3DPDF::Ptr curPDF;          ///<current robot pose
  ros::Publisher pub_map_, pub_metadata_, pub_pose_, pub_point_cloud_, pub_observations;  ///<publishers for map and pose particles

  tf::TransformListener listenerTF_;         ///<transform listener
  tf::TransformBroadcaster tf_broadcaster_;  ///<transform broadcaster

  CTicTac tictac;  ///<timer for SLAM performance evaluation
  float t_exec;    ///<the time which take one SLAM update execution
  CSensoryFrame::Ptr observations;
  CObservation::Ptr observation;
  mrpt::system::TTimeStamp timeLastUpdate_;  ///< last update of the pose and map
  
  std::queue<mrpt_icp_slam_2d::ObservationWithTransform> obs_queue;  ///< observations of other robots
  boost::mutex obs_mutex;

  ros::Time stamp;  ///< timestamp for observations

  mrpt::gui::CDisplayWindow3D::Ptr win3D_;  ///<MRPT window

  std::vector<CObservation2DRangeScan::Ptr> lst_current_laser_scans;
  bool isObsBasedRawlog;
  bool SHOW_PROGRESS_3D_REAL_TIME;
  int SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS;
  bool SHOW_LASER_SCANS_3D;
  bool CAMERA_3DSCENE_FOLLOWS_ROBOT;
};

#endif /* MRPT_ICP_SLAM_2D_WRAPPER_H */
