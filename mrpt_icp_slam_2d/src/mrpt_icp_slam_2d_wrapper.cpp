/*
 * File: mrpt_icp_slam_2d_wrapper.cpp
 * Author: Vladislav Tananaev
 *
 */

#include "mrpt_icp_slam_2d/mrpt_icp_slam_2d_wrapper.h"
#include <mrpt/version.h>
#if MRPT_VERSION >= 0x150
#include <mrpt_bridge/utils.h>
#endif

ICPslamWrapper::ICPslamWrapper()
{
  rawlog_play_ = false;
  stamp = ros::Time::now();
  // Default parameters for 3D window
  SHOW_PROGRESS_3D_REAL_TIME = false;
  SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = 0;  // this parameter is not used
  SHOW_LASER_SCANS_3D = true;
  CAMERA_3DSCENE_FOLLOWS_ROBOT = true;
  isObsBasedRawlog = true;
}
ICPslamWrapper::~ICPslamWrapper()
{
	try {
		std::string sOutMap = "mrpt_icpslam_";
		mrpt::system::TTimeParts parts;
		mrpt::system::timestampToParts(now(), parts, true);
		sOutMap += format("%04u-%02u-%02u_%02uh%02um%02us",
			(unsigned int)parts.year,
			(unsigned int)parts.month,
			(unsigned int)parts.day,
			(unsigned int)parts.hour,
			(unsigned int)parts.minute,
			(unsigned int)parts.second );
		sOutMap += ".simplemap";

		sOutMap = mrpt::system::fileNameStripInvalidChars( sOutMap );
		ROS_INFO("Saving built map to `%s`", sOutMap.c_str());
		mapBuilder.saveCurrentMapToFile(sOutMap);
	} catch (std::exception &e)
	{
		ROS_ERROR("Exception: %s",e.what());
	}
}
bool ICPslamWrapper::is_file_exists(const std::string &name)
{
  std::ifstream f(name.c_str());
  return f.good();
}

void ICPslamWrapper::read_iniFile(std::string ini_filename)
{
  CConfigFile iniFile(ini_filename);

  mapBuilder.ICP_options.loadFromConfigFile(iniFile, "MappingApplication");
  mapBuilder.ICP_params.loadFromConfigFile(iniFile, "ICP");

#if MRPT_VERSION < 0x150
  mapBuilder.options.verbose = true;
#else
  log4cxx::LoggerPtr ros_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  mapBuilder.setVerbosityLevel(mrpt_bridge::rosLoggerLvlToMRPTLoggerLvl(ros_logger->getLevel()));
  mapBuilder.logging_enable_console_output = false;
#if MRPT_VERSION < 0x199
  mapBuilder.logRegisterCallback(static_cast<output_logger_callback_t>(&mrpt_bridge::mrptToROSLoggerCallback_mrpt_15));
#else
  mapBuilder.logRegisterCallback(static_cast<output_logger_callback_t>(&mrpt_bridge::mrptToROSLoggerCallback));
#endif
#endif
  mapBuilder.options.alwaysInsertByClass.fromString(
      iniFile.read_string("MappingApplication", "alwaysInsertByClass", ""));

  mapBuilder.ICP_params.dumpToConsole();
  mapBuilder.ICP_options.dumpToConsole();

  // parameters for mrpt3D window
  CAMERA_3DSCENE_FOLLOWS_ROBOT =
      iniFile.read_bool("MappingApplication", "CAMERA_3DSCENE_FOLLOWS_ROBOT", true, /*Force existence:*/ true);
  MRPT_LOAD_CONFIG_VAR(SHOW_PROGRESS_3D_REAL_TIME, bool, iniFile, "MappingApplication");
  MRPT_LOAD_CONFIG_VAR(SHOW_LASER_SCANS_3D, bool, iniFile, "MappingApplication");
  MRPT_LOAD_CONFIG_VAR(SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS, int, iniFile, "MappingApplication");
}

void ICPslamWrapper::get_param()
{
  ROS_INFO("READ PARAM FROM LAUNCH FILE");
  n_.param<double>("rawlog_play_delay", rawlog_play_delay, 0.1);
  ROS_INFO("rawlog_play_delay: %f", rawlog_play_delay);

  n_.getParam("rawlog_filename", rawlog_filename);
  ROS_INFO("rawlog_filename: %s", rawlog_filename.c_str());

  n_.getParam("ini_filename", ini_filename);
  ROS_INFO("ini_filename: %s", ini_filename.c_str());

  n_.param<std::string>("global_frame_id", global_frame_id, "map");
  ROS_INFO("global_frame_id: %s", global_frame_id.c_str());

  n_.param<std::string>("odom_frame_id", odom_frame_id, "odom");
  ROS_INFO("odom_frame_id: %s", odom_frame_id.c_str());

  n_.param<std::string>("base_frame_id", base_frame_id, "base_link");
  ROS_INFO("base_frame_id: %s", base_frame_id.c_str());

  n_.param<std::string>("sensor_source", sensor_source, "scan");
  ROS_INFO("sensor_source: %s", sensor_source.c_str());

  n_.param("trajectory_update_rate", trajectory_update_rate, 10.0);
  ROS_INFO("trajectory_update_rate: %f", trajectory_update_rate);

  n_.param("trajectory_publish_rate", trajectory_publish_rate, 5.0);
  ROS_INFO("trajectory_publish_rate: %f", trajectory_publish_rate);
  
  n_.param<bool>("run_multi_robot", run_multi_robot, false);
  ROS_INFO("run_multi_robot: %d", run_multi_robot);
  
  n_.param<std::string>("robots_source", robots_source, "observations");
  ROS_INFO("robots_source: %s", robots_source.c_str());
  
  n_.param<std::string>("base_ns", base_ns, "robot");
  ROS_INFO("base_ns: %s", base_ns.c_str());
  
  n_.param<int>("robots_count", robots_count, 2);
  ROS_INFO("robots_count: %d", robots_count);
  
  n_.param<int>("init_x", init_x, 0);
  ROS_INFO("init_x: %d", init_x);
  
  n_.param<int>("init_y", init_y, 0);
  ROS_INFO("init_y: %d", init_y);
  
  n_.param<int>("init_phi", init_phi, 0);
  ROS_INFO("init_phi: %d", init_phi);
}
void ICPslamWrapper::init3Dwindow()
{
#if MRPT_HAS_WXWIDGETS
  if (SHOW_PROGRESS_3D_REAL_TIME)
  {
    win3D_ = mrpt::gui::CDisplayWindow3D::Create("pf-localization - The MRPT project", 1000, 600);
    win3D_->setCameraZoom(20);
    win3D_->setCameraAzimuthDeg(-45);
  }

#endif
}

void ICPslamWrapper::run3Dwindow()
{
  // Create 3D window if requested (the code is copied from ../mrpt/apps/icp-slam/icp-slam_main.cpp):
  if (SHOW_PROGRESS_3D_REAL_TIME && win3D_)
  {
    // get currently builded map
    metric_map_ = mapBuilder.getCurrentlyBuiltMetricMap();

    lst_current_laser_scans.clear();

    CPose3D robotPose;
    mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);
    COpenGLScene::Ptr scene = COpenGLScene::Create();

    COpenGLViewport::Ptr view = scene->getViewport("main");

    COpenGLViewport::Ptr view_map = scene->createViewport("mini-map");
    view_map->setBorderSize(2);
    view_map->setViewportPosition(0.01, 0.01, 0.35, 0.35);
    view_map->setTransparent(false);

    {
      mrpt::opengl::CCamera &cam = view_map->getCamera();
      cam.setAzimuthDegrees(-90);
      cam.setElevationDegrees(90);
      cam.setPointingAt(robotPose);
      cam.setZoomDistance(20);
      cam.setOrthogonal();
    }

    // The ground:
    mrpt::opengl::CGridPlaneXY::Ptr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200, 200, -200, 200, 0, 5);
    groundPlane->setColor(0.4, 0.4, 0.4);
    view->insert(groundPlane);
    view_map->insert(CRenderizable::Ptr(groundPlane));  // A copy

    // The camera pointing to the current robot pose:
    if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
    {
      scene->enableFollowCamera(true);

      mrpt::opengl::CCamera &cam = view_map->getCamera();
      cam.setAzimuthDegrees(-45);
      cam.setElevationDegrees(45);
      cam.setPointingAt(robotPose);
    }

    // The maps:
    {
      opengl::CSetOfObjects::Ptr obj = opengl::CSetOfObjects::Create();
      metric_map_->getAs3DObject(obj);
      view->insert(obj);

      // Only the point map:
      opengl::CSetOfObjects::Ptr ptsMap = opengl::CSetOfObjects::Create();
      if (metric_map_->m_pointsMaps.size())
      {
        metric_map_->m_pointsMaps[0]->getAs3DObject(ptsMap);
        view_map->insert(ptsMap);
      }
    }

    // Draw the robot path:
    CPose3DPDF::Ptr posePDF = mapBuilder.getCurrentPoseEstimation();
    CPose3D curRobotPose;
    posePDF->getMean(curRobotPose);
    {
      opengl::CSetOfObjects::Ptr obj = opengl::stock_objects::RobotPioneer();
      obj->setPose(curRobotPose);
      view->insert(obj);
    }
    {
      opengl::CSetOfObjects::Ptr obj = opengl::stock_objects::RobotPioneer();
      obj->setPose(curRobotPose);
      view_map->insert(obj);
    }

    opengl::COpenGLScene::Ptr &ptrScene = win3D_->get3DSceneAndLock();
    ptrScene = scene;

    win3D_->unlockAccess3DScene();

    // Move camera:
    win3D_->setCameraPointingToPoint(curRobotPose.x(), curRobotPose.y(), curRobotPose.z());

    // Update:
    win3D_->forceRepaint();

    // Build list of scans:
    if (SHOW_LASER_SCANS_3D)
    {
      // Rawlog in "Observation-only" format:
      if (isObsBasedRawlog)
      {
        if (IS_CLASS(observation, CObservation2DRangeScan))
        {
          lst_current_laser_scans.push_back(mrpt::ptr_cast<CObservation2DRangeScan>::from(observation));
        }
      }
      else
      {
        // Rawlog in the Actions-SF format:
        for (size_t i = 0;; i++)
        {
          CObservation2DRangeScan::Ptr new_obs = observations->getObservationByClass<CObservation2DRangeScan>(i);
          if (!new_obs)
            break;  // There're no more scans
          else
            lst_current_laser_scans.push_back(new_obs);
        }
      }
    }

    // Draw laser scanners in 3D:
    if (SHOW_LASER_SCANS_3D)
    {
      for (size_t i = 0; i < lst_current_laser_scans.size(); i++)
      {
        // Create opengl object and load scan data from the scan observation:
        opengl::CPlanarLaserScan::Ptr obj = opengl::CPlanarLaserScan::Create();
        obj->setScan(*lst_current_laser_scans[i]);
        obj->setPose(curRobotPose);
        obj->setSurfaceColor(1.0f, 0.0f, 0.0f, 0.5f);
        // inser into the scene:
        view->insert(obj);
      }
    }
  }
}
void ICPslamWrapper::init()
{
  // get parameters from ini file
  if (!is_file_exists(ini_filename))
  {
    ROS_ERROR_STREAM("CAN'T READ INI FILE");
    return;
  }
  read_iniFile(ini_filename);
  // read rawlog file if it  exists
  if (is_file_exists(rawlog_filename))
  {
    ROS_WARN_STREAM("PLAY FROM RAWLOG FILE: " << rawlog_filename.c_str());
    rawlog_play_ = true;
  }
  
  //init map builder with init pose
  mrpt::poses::CPosePDFGaussian init_pose(mrpt::poses::CPose2D(init_x, init_y, init_phi));
  mapBuilder.initialize(mrpt::maps::CSimpleMap(), &init_pose);

  /// Create publishers///
  // publish grid map
  pub_map_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  pub_metadata_ = n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  // publish point map
  pub_point_cloud_ = n_.advertise<sensor_msgs::PointCloud>("PointCloudMap", 1, true);

  trajectory_pub_ = n_.advertise<nav_msgs::Path>("trajectory", 1, true);

  // robot pose
  pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);

  update_trajector_timer = n_.createTimer(ros::Duration(1.0 / trajectory_update_rate),
          &ICPslamWrapper::updateTrajectoryTimerCallback, this ,false);

  publish_trajectory_timer = n_.createTimer(ros::Duration(1.0 / trajectory_publish_rate),
          &ICPslamWrapper::publishTrajectoryTimerCallback, this, false);

  // read sensor topics
  std::vector<std::string> lstSources;
  mrpt::system::tokenize(sensor_source, " ,\t\n", lstSources);
  ROS_ASSERT_MSG(!lstSources.empty(), "*Fatal*: At least one sensor source must be provided in ~sensor_sources (e.g. "
                                      "\"scan\" or \"beacon\")");

  /// Create subscribers///
  sensorSub_.resize(lstSources.size());
  for (size_t i = 0; i < lstSources.size(); i++)
  {
    if (lstSources[i].find("scan") != std::string::npos)
    {
      sensorSub_[i] = n_.subscribe(lstSources[i], 1, &ICPslamWrapper::laserCallback, this);
    }
    else
    {
      std::cout << "Sensor topics should be 2d laser scans which inlude in the name the word scan "
                << "\n";
    }
  }
  
  if (run_multi_robot) {
      //advertize own observations
      std::string own_obs = ros::names::resolve(robots_source);
      pub_observations = n_.advertise<mrpt_icp_slam_2d::ObservationWithTransform>(own_obs, 1, true);
      
      //subscribe to others observations
      robotsSubs.resize(robots_count);
      for (int i = 0; i < robots_count; i++)
      {
        std::string other_obs = ros::names::resolve("/" + base_ns + std::to_string(i) + "/" + robots_source);
        if (other_obs != own_obs)
        {
          robotsSubs[i] = n_.subscribe(other_obs, 1000, &ICPslamWrapper::multirobotCallback, this);
        }
      }
  }

  init3Dwindow();
}

void ICPslamWrapper::laserCallback(const sensor_msgs::LaserScan &_msg)
{
#if MRPT_VERSION >= 0x130
  using namespace mrpt::maps;
  using namespace mrpt::obs;
#else
  using namespace mrpt::slam;
#endif
  CObservation2DRangeScan::Ptr laser = CObservation2DRangeScan::Create();
  if (laser_poses_.find(_msg.header.frame_id) == laser_poses_.end())
  {
    updateSensorPose(_msg.header.frame_id);
  }
  else
  {      
    mrpt::poses::CPose3D pose = laser_poses_[_msg.header.frame_id];
    mrpt_bridge::convert(_msg, pose, *laser);

    observation = CObservation::Ptr(laser);
    
    ros::Time prev_stamp = stamp;
    stamp = _msg.header.stamp;
    
    tictac.Tic();
    mapBuilder.processObservation(observation);
    t_exec = tictac.Tac();
    ROS_DEBUG("Map building executed in %.03fms", 1000.0f * t_exec);

    run3Dwindow();
    publishTF();
    publishMapPose();
    
    //add observations from other robots
    if (run_multi_robot) {
        for (int i = 0; i < obs_batch && !obs_queue.empty(); i++) {
            boost::mutex::scoped_lock(obs_mutex);
            CObservation::Ptr mr_obs = msgToObservation(obs_queue.front(), prev_stamp); //timestamp does not matter
            if (mr_obs == nullptr) break;
            mapBuilder.processObservation(mr_obs);
            obs_queue.pop();
        }
        publishObservations(_msg, prev_stamp);
    }
  }
}

void ICPslamWrapper::multirobotCallback(const mrpt_icp_slam_2d::ObservationWithTransform& _msg)
{
  boost::mutex::scoped_lock(obs_mutex);
  obs_queue.push(_msg);

  ROS_DEBUG("Multi-robot observation added");
}

CObservation::Ptr ICPslamWrapper::msgToObservation(const mrpt_icp_slam_2d::ObservationWithTransform& _msg, const ros::Time& _stamp) {
#if MRPT_VERSION >= 0x130
  using namespace mrpt::maps;
  using namespace mrpt::obs;
#else
  using namespace mrpt::slam;
#endif
  //transform to pose
  geometry_msgs::PoseStamped global_pose;
  global_pose.header.stamp = _stamp;
  global_pose.header.frame_id = global_frame_id;
  global_pose.pose.orientation = _msg.transform.transform.rotation;
  global_pose.pose.position.x = _msg.transform.transform.translation.x;
  global_pose.pose.position.y = _msg.transform.transform.translation.y;
  global_pose.pose.position.z = _msg.transform.transform.translation.z;
  //transform to robot base frame
  geometry_msgs::PoseStamped pose;
  try
  {
    listenerTF_.transformPose(base_frame_id, _stamp, global_pose, global_frame_id, pose);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    return nullptr;
  }
  
  //convert tf transform to mrpt pose
  mrpt::poses::CPose3D sensor_pose;
  mrpt_bridge::convert(pose.pose, sensor_pose);
      
  CObservation2DRangeScan::Ptr laser = CObservation2DRangeScan::Create();
  
  mrpt_bridge::convert(_msg.scan, sensor_pose, *laser);
  CObservation::Ptr obs = CObservation::Ptr(laser);
  
  return obs;
}


void ICPslamWrapper::publishObservations(const sensor_msgs::LaserScan& scan, const ros::Time& _stamp) {
  tf::StampedTransform transform;
  try
  {
    listenerTF_.lookupTransform(global_frame_id, scan.header.frame_id, _stamp, transform); //use same timestamp as sensor message
    mrpt_icp_slam_2d::ObservationWithTransform msg;
    tf::transformStampedTFToMsg(transform, msg.transform);
    msg.scan = scan;
    
    pub_observations.publish(msg);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
  }

}

void ICPslamWrapper::publishMapPose()
{
  // get currently builded map
  metric_map_ = mapBuilder.getCurrentlyBuiltMetricMap();

  // publish map
  if (metric_map_->m_gridMaps.size())
  {
    nav_msgs::OccupancyGrid _msg;
    // if we have new map for current sensor update it
    mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg);
    _msg.header.frame_id = global_frame_id;
    pub_map_.publish(_msg);
    pub_metadata_.publish(_msg.info);
  }
  if (metric_map_->m_pointsMaps.size())
  {
    sensor_msgs::PointCloud _msg;
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = global_frame_id;
    // if we have new map for current sensor update it
    mrpt_bridge::point_cloud::mrpt2ros(*metric_map_->m_pointsMaps[0], header, _msg);
    pub_point_cloud_.publish(_msg);
  }

  CPose3D robotPose;
  mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);

  // publish pose
  // geometry_msgs::PoseStamped pose;
  pose.header.frame_id = global_frame_id;

  // the pose
  pose.pose.position.x = robotPose.x();
  pose.pose.position.y = robotPose.y();
  pose.pose.position.z = 0.0;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotPose.yaw());

  pub_pose_.publish(pose);
}
void ICPslamWrapper::updateSensorPose(std::string _frame_id)
{
  CPose3D pose;
  tf::StampedTransform transform;
  try
  {
    listenerTF_.lookupTransform(base_frame_id, _frame_id, ros::Time(0), transform);

    tf::Vector3 translation = transform.getOrigin();
    tf::Quaternion quat = transform.getRotation();
    pose.x() = translation.x();
    pose.y() = translation.y();
    pose.z() = translation.z();
    tf::Matrix3x3 Rsrc(quat);
    CMatrixDouble33 Rdes;
    for (int c = 0; c < 3; c++)
      for (int r = 0; r < 3; r++)
        Rdes(r, c) = Rsrc.getRow(r)[c];
    pose.setRotationMatrix(Rdes);
    laser_poses_[_frame_id] = pose;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}
bool ICPslamWrapper::rawlogPlay()
{
  if (rawlog_play_ == false)
  {
    return false;
  }
  else
  {
    size_t rawlogEntry = 0;
#if MRPT_VERSION>=0x199
#include <mrpt/serialization/CArchive.h>
    CFileGZInputStream rawlog_stream(rawlog_filename);
    auto rawlogFile = mrpt::serialization::archiveFrom(rawlog_stream);
#else
    CFileGZInputStream rawlogFile(rawlog_filename);
#endif
    CActionCollection::Ptr action;

    for (;;)
    {
      if (ros::ok())
      {
        if (!CRawlog::getActionObservationPairOrObservation(rawlogFile, action, observations, observation, rawlogEntry))
        {
          break;  // file EOF
        }
        isObsBasedRawlog = (bool)observation;
        // Execute:
        // ----------------------------------------
        tictac.Tic();
        if (isObsBasedRawlog)
          mapBuilder.processObservation(observation);
        else
          mapBuilder.processActionObservation(*action, *observations);
        t_exec = tictac.Tac();
        ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec);

        ros::Duration(rawlog_play_delay).sleep();

        metric_map_ = mapBuilder.getCurrentlyBuiltMetricMap();

        CPose3D robotPose;
        mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);

        // publish map
        if (metric_map_->m_gridMaps.size())
        {
          nav_msgs::OccupancyGrid _msg;

          // if we have new map for current sensor update it
          mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg);
          _msg.header.frame_id = global_frame_id;
          pub_map_.publish(_msg);
          pub_metadata_.publish(_msg.info);
        }

        if (metric_map_->m_pointsMaps.size())
        {
          sensor_msgs::PointCloud _msg;
          std_msgs::Header header;
          header.stamp = ros::Time::now();
          header.frame_id = global_frame_id;
          // if we have new map for current sensor update it
          mrpt_bridge::point_cloud::mrpt2ros(*metric_map_->m_pointsMaps[0], header, _msg);
          pub_point_cloud_.publish(_msg);
          // pub_metadata_.publish(_msg.info)
        }

        // publish pose
        // geometry_msgs::PoseStamped pose;
        pose.header.frame_id = global_frame_id;

        // the pose
        pose.pose.position.x = robotPose.x();
        pose.pose.position.y = robotPose.y();
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotPose.yaw());

        pub_pose_.publish(pose);
      }

      run3Dwindow();
      ros::spinOnce();
    }

    // if there is mrpt_gui it will wait until push any key in order to close the window
    if (win3D_)
      win3D_->waitForKey();

    return true;
  }
}

void ICPslamWrapper::publishTF()
{
  // Most of this code was copy and pase from ros::amcl
  mrpt::poses::CPose3D robotPoseTF;
  mapBuilder.getCurrentPoseEstimation()->getMean(robotPoseTF);

  tf::Stamped<tf::Pose> odom_to_map;
  tf::Transform tmp_tf;
  mrpt_bridge::convert(robotPoseTF, tmp_tf);

  try
  {
    tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), stamp, base_frame_id);
    listenerTF_.transformPose(odom_frame_id, tmp_tf_stamped, odom_to_map);
  }
  catch (tf::TransformException)
  {
    ROS_INFO("Failed to subtract global_frame (%s) from odom_frame (%s)", global_frame_id.c_str(),
             odom_frame_id.c_str());
    return;
  }

  tf::Transform latest_tf_ =
      tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));

  // We want to send a transform that is good up until a
  // tolerance time so that odom can be used

  ros::Duration transform_tolerance_(0.5);
  ros::Time transform_expiration = (stamp + transform_tolerance_);
  tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration, global_frame_id, odom_frame_id);
  tf_broadcaster_.sendTransform(tmp_tf_stamped);
}

void ICPslamWrapper::updateTrajectoryTimerCallback(const ros::TimerEvent& event)
{
    ROS_DEBUG("update trajectory");
    path.header.frame_id = global_frame_id;
    path.header.stamp = ros::Time::now();
    path.poses.push_back(pose);
}

void ICPslamWrapper::publishTrajectoryTimerCallback(const ros::TimerEvent& event)
{
    ROS_DEBUG("publish trajectory");
    trajectory_pub_.publish(path);
}
