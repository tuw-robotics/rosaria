#include "RosAria.h"

RosAriaNodelet::RosAriaNodelet()
{
}

void RosAriaNodelet::readParameters()
{
  // Robot Parameters. If a parameter was given and is nonzero, set it now.
  // Otherwise, get default value for this robot (from getOrigRobotConfig()).
  // Parameter values are stored in member variables for possible later use by the user with dynamic reconfigure.
  robot->lock();
  if (getPrivateNodeHandle().getParam("TicksMM", TicksMM) && TicksMM > 0)
  {
    ROS_INFO("Setting robot TicksMM from ROS Parameter: %d", TicksMM);
    robot->comInt(93, TicksMM);
  }
  else
  {
    TicksMM = robot->getOrigRobotConfig()->getTicksMM();
    ROS_INFO("This robot's TicksMM parameter: %d", TicksMM);
    //n_.setParam( "TicksMM", TicksMM);
  }
  
  if (getPrivateNodeHandle().getParam("DriftFactor", DriftFactor) && DriftFactor != -99999)
  {
    ROS_INFO("Setting robot DriftFactor from ROS Parameter: %d", DriftFactor);
    robot->comInt(89, DriftFactor);
  }
  else
  {
    DriftFactor = robot->getOrigRobotConfig()->getDriftFactor();
    ROS_INFO("This robot's DriftFactor parameter: %d", DriftFactor);
    //n_.setParam( "DriftFactor", DriftFactor);
  }
  
  if (getPrivateNodeHandle().getParam("RevCount", RevCount) && RevCount > 0)
  {
    ROS_INFO("Setting robot RevCount from ROS Parameter: %d", RevCount);
    robot->comInt(88, RevCount);
  }
  else
  {
    RevCount = robot->getOrigRobotConfig()->getRevCount();
    ROS_INFO("This robot's RevCount parameter: %d", RevCount);
    //n_.setParam( "RevCount", RevCount);
  }
  robot->unlock();
}

void RosAriaNodelet::dynamic_reconfigureCB(rosaria::RosAriaConfig &config, uint32_t level)
{
  //
  // Odometry Settings
  //
  robot->lock();
  if(TicksMM != config.TicksMM && config.TicksMM > 0)
  {
    ROS_INFO("Setting TicksMM from Dynamic Reconfigure: %d -> %d ", TicksMM, config.TicksMM);
    TicksMM = config.TicksMM;
    robot->comInt(93, TicksMM);
  }
  
  if(DriftFactor != config.DriftFactor && config.DriftFactor != -99999) 
  {
    ROS_INFO("Setting DriftFactor from Dynamic Reconfigure: %d -> %d ", DriftFactor, config.DriftFactor);
    DriftFactor = config.DriftFactor;
    robot->comInt(89, DriftFactor);
  }
  
  if(RevCount != config.RevCount && config.RevCount > 0)
  {
    ROS_INFO("Setting RevCount from Dynamic Reconfigure: %d -> %d ", RevCount, config.RevCount);
    RevCount = config.RevCount;
    robot->comInt(88, RevCount);
  }
  
  //
  // Acceleration Parameters
  //
  int value;
  value = config.trans_accel * 1000;
  if(value != robot->getTransAccel() && value > 0)
  {
    ROS_INFO("Setting TransAccel from Dynamic Reconfigure: %d", value);
    robot->setTransAccel(value);
  }
  
  value = config.trans_decel * 1000;
  if(value != robot->getTransDecel() && value > 0)
  {
    ROS_INFO("Setting TransDecel from Dynamic Reconfigure: %d", value);
    robot->setTransDecel(value);
  } 
  
  value = config.lat_accel * 1000;
  if(value != robot->getLatAccel() && value > 0)
  {
    ROS_INFO("Setting LatAccel from Dynamic Reconfigure: %d", value);
    if (robot->getAbsoluteMaxLatAccel() > 0 )
      robot->setLatAccel(value);
  }
  
  value = config.lat_decel * 1000;
  if(value != robot->getLatDecel() && value > 0)
  {
    ROS_INFO("Setting LatDecel from Dynamic Reconfigure: %d", value);
    if (robot->getAbsoluteMaxLatDecel() > 0 )
      robot->setLatDecel(value);
  }
  
  value = config.rot_accel * 180/M_PI;
  if(value != robot->getRotAccel() && value > 0)
  {
    ROS_INFO("Setting RotAccel from Dynamic Reconfigure: %d", value);
    robot->setRotAccel(value);
  }
  
  value = config.rot_decel * 180/M_PI;
  if(value != robot->getRotDecel() && value > 0)
  {
    ROS_INFO("Setting RotDecel from Dynamic Reconfigure: %d", value);
    robot->setRotDecel(value);
  } 

  WheelRadius = (double)config.WheelDiameter / 2.;
 
  robot->unlock();
}

/// Called when another node subscribes or unsubscribes from sonar topic.
void RosAriaNodelet::sonarConnectCb()
{
  publish_sonar = (sonar_pub.getNumSubscribers() > 0);
  publish_sonar_pointcloud2 = (sonar_pointcloud2_pub.getNumSubscribers() > 0);
  robot->lock();
  if (publish_sonar || publish_sonar_pointcloud2)
  {
    robot->enableSonar();
    sonar_enabled = false;
  }
  else if(!publish_sonar && !publish_sonar_pointcloud2)
  {
    robot->disableSonar();
    sonar_enabled = true;
  }
  robot->unlock();
}

RosAriaNodelet::~RosAriaNodelet()
{
  // disable motors and sonar.
  robot->disableMotors();
  robot->disableSonar();

  robot->stopRunning();
  robot->waitForRunExit();
  Aria::shutdown();
}

int RosAriaNodelet::Setup()
{
  // Note, various objects are allocated here which are never deleted (freed), since Setup() is only supposed to be
  // called once per instance, and these objects need to persist until the process terminates.

  robot = new ArRobot();
  ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
  ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
  argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)

  // Now add any parameters given via ros params (see RosAriaNodelet constructor):

  // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
  // for wireless serial connection. Otherwise, interpret it as a serial port name.
  size_t colon_pos = serial_port.find(":");
  if (colon_pos != std::string::npos)
  {
    args->add("-remoteHost"); // pass robot's hostname/IP address to Aria
    args->add(serial_port.substr(0, colon_pos).c_str());
    args->add("-remoteRobotTcpPort"); // pass robot's TCP port to Aria
    args->add(serial_port.substr(colon_pos+1).c_str());
  }
  else
  {
    args->add("-robotPort %s", serial_port.c_str()); // pass robot's serial port to Aria
  }

  // if a baud rate was specified in baud parameter
  if(serial_baud != 0)
  {
    args->add("-robotBaud %d", serial_baud);
  }
  
  if( debug_aria )
  {
    // turn on all ARIA debugging
    args->add("-robotLogPacketsReceived"); // log received packets
    args->add("-robotLogPacketsSent"); // log sent packets
    args->add("-robotLogVelocitiesReceived"); // log received velocities
    args->add("-robotLogMovementSent");
    args->add("-robotLogMovementReceived");
    ArLog::init(ArLog::File, ArLog::Verbose, aria_log_filename.c_str(), true);
  }


  // Connect to the robot
  conn = new ArRobotConnector(argparser, robot); // warning never freed
  if (!conn->connectRobot()) {
    ROS_ERROR("RosAria: ARIA could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
    return 1;
  }

  if(publish_aria_lasers)
    laserConnector = new ArLaserConnector(argparser, robot, conn);

  // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
  if(!Aria::parseArgs())
  {
    ROS_ERROR("RosAria: ARIA error parsing ARIA startup parameters!");
    return 1;
  }

  readParameters();

  // Start dynamic_reconfigure server
  dynamic_reconfigure_server = new dynamic_reconfigure::Server<rosaria::RosAriaConfig>;
  
  // Setup Parameter Minimums and maximums
  rosaria::RosAriaConfig dynConf_min;
  rosaria::RosAriaConfig dynConf_max;
  
  dynConf_max.trans_accel = robot->getAbsoluteMaxTransAccel() / 1000;
  dynConf_max.trans_decel = robot->getAbsoluteMaxTransDecel() / 1000;
  // TODO: Fix rqt dynamic_reconfigure gui to handle empty intervals
  // Until then, set unit length interval.
  dynConf_max.lat_accel = ((robot->getAbsoluteMaxLatAccel() > 0.0) ? robot->getAbsoluteMaxLatAccel() : 0.1) / 1000;
  dynConf_max.lat_decel = ((robot->getAbsoluteMaxLatDecel() > 0.0) ? robot->getAbsoluteMaxLatDecel() : 0.1) / 1000;
  dynConf_max.rot_accel = robot->getAbsoluteMaxRotAccel() * M_PI/180;
  dynConf_max.rot_decel = robot->getAbsoluteMaxRotDecel() * M_PI/180;

  dynConf_min.trans_accel = 0;
  dynConf_min.trans_decel = 0;
  dynConf_min.lat_accel = 0;
  dynConf_min.lat_decel = 0;
  dynConf_min.rot_accel = 0;
  dynConf_min.rot_decel = 0;
  
  dynConf_min.TicksMM     = 0;
  dynConf_max.TicksMM     = 200;
  dynConf_min.DriftFactor = -99999;
  dynConf_max.DriftFactor = 32767;
  dynConf_min.RevCount    = 0;
  dynConf_max.RevCount    = 65535;

  dynamic_reconfigure_server->setConfigMax(dynConf_max);
  dynamic_reconfigure_server->setConfigMin(dynConf_min);
  
  
  rosaria::RosAriaConfig dynConf_default;
  dynConf_default.trans_accel = robot->getTransAccel() / 1000;
  dynConf_default.trans_decel = robot->getTransDecel() / 1000;
  dynConf_default.lat_accel   = robot->getLatAccel() / 1000;
  dynConf_default.lat_decel   = robot->getLatDecel() / 1000;
  dynConf_default.rot_accel   = robot->getRotAccel() * M_PI/180;
  dynConf_default.rot_decel   = robot->getRotDecel() * M_PI/180;

  dynConf_default.TicksMM     = 0;
  dynConf_default.DriftFactor = -99999;
  dynConf_default.RevCount    = 0;
  
  dynamic_reconfigure_server->setConfigDefault(dynConf_default);
  
  dynamic_reconfigure_server->setCallback(boost::bind(&RosAriaNodelet::dynamic_reconfigureCB, this, _1, _2));


  // Enable the motors
  robot->enableMotors();

  // disable sonars on startup
  robot->disableSonar();

  // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
  robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);
//   robot->add

  // Initialize bumpers with robot number of bumpers
  bumpers.front_bumpers.resize(robot->getNumFrontBumpers());
  bumpers.rear_bumpers.resize(robot->getNumRearBumpers());

  // Run ArRobot background processing thread
  robot->runAsync(true);

  // connect to lasers and create publishers
  if(publish_aria_lasers)
  {
    ROS_INFO_NAMED("rosaria", "rosaria: Connecting to laser(s) configured in ARIA parameter file(s)...");
    if (!laserConnector->connectLasers())
    {
      ROS_FATAL_NAMED("rosaria", "rosaria: Error connecting to laser(s)...");
      return 1;
    }

    robot->lock();
    const std::map<int, ArLaser*> *lasers = robot->getLaserMap();
    ROS_INFO_NAMED("rosaria", "rosaria: there are %lu connected lasers", lasers->size());
    for(std::map<int, ArLaser*>::const_iterator i = lasers->begin(); i != lasers->end(); ++i)
    {
      ArLaser *l = i->second;
      int ln = i->first;
      std::string tfname("laser");
      if(lasers->size() > 1 || ln > 1) // no number if only one laser which is also laser 1
        tfname += ln; 
      tfname += "_frame";
      ROS_INFO_NAMED("rosaria", "rosaria: Creating publisher for laser #%d named %s with tf frame name %s", ln, l->getName(), tfname.c_str());
      new LaserPublisher(l, getPrivateNodeHandle(), true, tfname);
    }
    robot->unlock();
    ROS_INFO_NAMED("rosaria", "rosaria: Done creating laser publishers");
  }
    
  // subscribe to command topics
  cmdvel_sub = getPrivateNodeHandle().subscribe( "cmd_vel", 1, (boost::function <void(const geometry_msgs::TwistConstPtr&)>)
      boost::bind(&RosAriaNodelet::cmdvel_cb, this, _1 ));
  cmdwh_sub = getPrivateNodeHandle().subscribe( "joint_cmds", 1, (boost::function <void(const tuw_nav_msgs::JointsIWSConstPtr&)>)
      boost::bind(&RosAriaNodelet::cmdwh_cb, this, _1 ));

  // register a watchdog for cmd_vel timeout
  double cmdvel_timeout_param = 0.6;
  getPrivateNodeHandle().param("cmd_vel_timeout", cmdvel_timeout_param, 0.6);
  cmdvel_timeout = ros::Duration(cmdvel_timeout_param);
  if (cmdvel_timeout_param > 0.0)
    cmdvel_watchdog_timer = getPrivateNodeHandle().createTimer(ros::Duration(0.1), &RosAriaNodelet::cmdvel_watchdog, this);

  ROS_INFO_NAMED("rosaria", "rosaria: Setup complete");
//   robot->run();
  return 0;
}

void RosAriaNodelet::spin()
{
  ros::spin();
}

void RosAriaNodelet::publish()
{
  // Note, this is called via SensorInterpTask callback (myPublishCB, named "ROSPublishingTask"). ArRobot object 'robot' sholud not be locked or unlocked.
  pos = robot->getPose();
  tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(pos.getTh()*M_PI/180), tf::Vector3(pos.getX()/1000,
    pos.getY()/1000, 0)), position.pose.pose); //Aria returns pose in mm.
  position.twist.twist.linear.x = robot->getVel()/1000.0; //Aria returns velocity in mm/s.
  position.twist.twist.linear.y = robot->getLatVel()/1000.0;
  position.twist.twist.angular.z = robot->getRotVel()*M_PI/180;
  
  position.header.frame_id = frame_id_odom;
  position.child_frame_id = frame_id_base_link;
  position.header.stamp = ros::Time::now();
  pose_pub.publish(position);
  
  static tuw_nav_msgs::JointsIWS wheels_measure;
  wheels_measure.header.stamp = ros::Time::now();
  wheels_measure.type_revolute = "measured_velocity";
  wheels_measure.revolute.resize(2);
  wheels_measure.revolute[0] = robot->getRightVel() / WheelRadius;
  wheels_measure.revolute[1] = robot->getLeftVel () / WheelRadius;
  
  wheels_measure_pub.publish(wheels_measure);

  ROS_DEBUG("RosAria: publish: (time %f) pose x: %f, pose y: %f, pose angle: %f; linear vel x: %f, vel y: %f; angular vel z: %f", 
    position.header.stamp.toSec(), 
    (double)position.pose.pose.position.x,
    (double)position.pose.pose.position.y,
    (double)position.pose.pose.orientation.w,
    (double)position.twist.twist.linear.x,
    (double)position.twist.twist.linear.y,
    (double)position.twist.twist.angular.z
  );

  // publishing transform odom->base_link
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = frame_id_odom;
  odom_trans.child_frame_id = frame_id_base_link;
  
  odom_trans.transform.translation.x = pos.getX()/1000;
  odom_trans.transform.translation.y = pos.getY()/1000;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pos.getTh()*M_PI/180);
  
  odom_broadcaster.sendTransform(odom_trans);
  
  // getStallValue returns 2 bytes with stall bit and bumper bits, packed as (00 00 FrontBumpers RearBumpers)
  int stall = robot->getStallValue();
  unsigned char front_bumpers = (unsigned char)(stall >> 8);
  unsigned char rear_bumpers = (unsigned char)(stall);

  bumpers.header.frame_id = frame_id_bumper;
  bumpers.header.stamp = ros::Time::now();

  std::stringstream bumper_info(std::stringstream::out);
  // Bit 0 is for stall, next bits are for bumpers (leftmost is LSB)
  for (unsigned int i=0; i<robot->getNumFrontBumpers(); i++)
  {
    bumpers.front_bumpers[i] = (front_bumpers & (1 << (i+1))) == 0 ? 0 : 1;
    bumper_info << " " << (front_bumpers & (1 << (i+1)));
  }
  ROS_DEBUG("RosAria: Front bumpers:%s", bumper_info.str().c_str());

  bumper_info.str("");
  // Rear bumpers have reverse order (rightmost is LSB)
  unsigned int numRearBumpers = robot->getNumRearBumpers();
  for (unsigned int i=0; i<numRearBumpers; i++)
  {
    bumpers.rear_bumpers[i] = (rear_bumpers & (1 << (numRearBumpers-i))) == 0 ? 0 : 1;
    bumper_info << " " << (rear_bumpers & (1 << (numRearBumpers-i)));
  }
  ROS_DEBUG("RosAria: Rear bumpers:%s", bumper_info.str().c_str());
  
  bumpers_pub.publish(bumpers);

  //Publish battery information
  // TODO: Decide if BatteryVoltageNow (normalized to (0,12)V)  is a better option
  std_msgs::Float64 batteryVoltage;
  batteryVoltage.data = robot->getRealBatteryVoltageNow();
  voltage_pub.publish(batteryVoltage);

  if(robot->haveStateOfCharge())
  {
    std_msgs::Float32 soc;
    soc.data = robot->getStateOfCharge()/100.0;
    state_of_charge_pub.publish(soc);
  }

  // publish recharge state if changed
  char s = robot->getChargeState();
  if(s != recharge_state.data)
  {
    ROS_INFO("RosAria: publishing new recharge state %d.", s);
    recharge_state.data = s;
    recharge_state_pub.publish(recharge_state);
  }

  // publish motors state if changed
  bool e = robot->areMotorsEnabled();
  if(e != motors_state.data || !published_motors_state)
  {
	ROS_INFO("RosAria: publishing new motors state %d.", e);
	motors_state.data = e;
	motors_state_pub.publish(motors_state);
	published_motors_state = true;
  }

  // Publish sonar information, if enabled.
  if (publish_sonar || publish_sonar_pointcloud2)
  {
    sensor_msgs::PointCloud cloud;	//sonar readings.
    cloud.header.stamp = position.header.stamp;	//copy time.
    // sonar sensors relative to base_link
    cloud.header.frame_id = frame_id_sonar;
  

    std::stringstream sonar_debug_info; // Log debugging info
    sonar_debug_info << "Sonar readings: ";

    for (int i = 0; i < robot->getNumSonar(); i++) {
      ArSensorReading* reading = NULL;
      reading = robot->getSonarReading(i);
      if(!reading) {
        ROS_WARN("RosAria: Did not receive a sonar reading.");
        continue;
      }
    
      // getRange() will return an integer between 0 and 5000 (5m)
      sonar_debug_info << reading->getRange() << " ";

      // local (x,y). Appears to be from the centre of the robot, since values may
      // exceed 5000. This is good, since it means we only need 1 transform.
      // x & y seem to be swapped though, i.e. if the robot is driving north
      // x is north/south and y is east/west.
      //
      //ArPose sensor = reading->getSensorPosition();  //position of sensor.
      // sonar_debug_info << "(" << reading->getLocalX() 
      //                  << ", " << reading->getLocalY()
      //                  << ") from (" << sensor.getX() << ", " 
      //                  << sensor.getY() << ") ;; " ;
    
      //add sonar readings (robot-local coordinate frame) to cloud
      geometry_msgs::Point32 p;
      p.x = reading->getLocalX() / 1000.0;
      p.y = reading->getLocalY() / 1000.0;
      p.z = 0.0;
      cloud.points.push_back(p);
    }
    ROS_DEBUG_STREAM(sonar_debug_info.str());
    
    // publish topic(s)

    if(publish_sonar_pointcloud2)
    {
      sensor_msgs::PointCloud2 cloud2;
      if(!sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2))
      {
        ROS_WARN("Error converting sonar point cloud message to point_cloud2 type before publishing! Not publishing this time.");
      }
      else
      {
        sonar_pointcloud2_pub.publish(cloud2);
      }
    }

    if(publish_sonar)
    {
      sonar_pub.publish(cloud);
    }
  } // end if sonar_enabled
}

bool RosAriaNodelet::enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("RosAria: Enable motors request.");
    robot->lock();
    if(robot->isEStopPressed())
        ROS_WARN("RosAria: Warning: Enable motors requested, but robot also has E-Stop button pressed. Motors will not enable.");
    robot->enableMotors();
    robot->unlock();
	// todo could wait and see if motors do become enabled, and send a response with an error flag if not
    return true;
}

bool RosAriaNodelet::disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("RosAria: Disable motors request.");
    robot->lock();
    robot->disableMotors();
    robot->unlock();
	// todo could wait and see if motors do become disabled, and send a response with an error flag if not
    return true;
}

void
RosAriaNodelet::cmdvel_cb( const geometry_msgs::TwistConstPtr &msg)
{
  
  ros::Time timenow = ros::Time::now();
  if ( ros::Duration ( timenow - veltime_cmdwh_cb ).toSec() < 0.5 ) {
    return;
  }
  
  veltime = ros::Time::now();
  ROS_INFO( "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.toSec() );

  robot->lock();
  robot->setVel(msg->linear.x*1e3);
  if(robot->hasLatVel())
    robot->setLatVel(msg->linear.y*1e3);
  robot->setRotVel(msg->angular.z*180/M_PI);
  robot->unlock();
  ROS_DEBUG("RosAria: sent vels to to aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s", veltime.toSec(),
    (double) msg->linear.x * 1e3, (double) msg->linear.y * 1.3, (double) msg->angular.z * 180/M_PI);
}

void
RosAriaNodelet::cmdwh_cb( const tuw_nav_msgs::JointsIWSConstPtr &msg)
{
  if ( msg->revolute.size() != 2 ) {
    ROS_ERROR( "RosAria, in RosAriaNodelet::cmdwh_cb: messsage does not contain two revolute commands. Will not set the command." );
    return;
  }
  std::string revolute_mode = msg->type_revolute;
  if ( revolute_mode.compare("cmd_velocity") ) {
    ROS_ERROR( "RosAria, in RosAriaNodelet::cmdwh_cb: revolute command type not supported. Will not set the command." );
    return;
  }
  
  const double vL = msg->revolute[1] * WheelRadius;
  const double vR = msg->revolute[0] * WheelRadius;
  veltime = ros::Time::now();
  veltime_cmdwh_cb = veltime;
  ROS_INFO( "new wheel speed: [%0.2f,%0.2f](%0.3f)", vL, vR, veltime.toSec() );
  
  robot->lock();
  robot->setVel2(vL, vR);
  robot->unlock();
}

void RosAriaNodelet::cmdvel_watchdog(const ros::TimerEvent& event)
{
  // stop robot if no cmd_vel message was received for 0.6 seconds
  if (ros::Time::now() - veltime > ros::Duration(0.6))
  {
    robot->lock();
    robot->setVel(0.0);
    if(robot->hasLatVel())
      robot->setLatVel(0.0);
    robot->setRotVel(0.0);
    robot->unlock();
  }
}

void RosAriaNodelet::onInit()
{
  Aria::init();

  serial_port = std::string("");
  serial_baud = 0;
  conn = NULL;
  laserConnector = NULL;
  robot = NULL;
  myPublishCB = ArFunctorC<RosAriaNodelet>(this, &RosAriaNodelet::publish);
  sonar_enabled = false;
  publish_sonar = false;
  publish_sonar_pointcloud2 = false;
  debug_aria = false;
  TicksMM = -1;
  DriftFactor = -99999;
  RevCount = -1;
  publish_aria_lasers = false;

  // read in runtime parameters

  // port and baud
  getPrivateNodeHandle().param( "port", serial_port, std::string("/dev/ttyUSB0") );
  ROS_INFO( "RosAria: set port: [%s]", serial_port.c_str() );

  getPrivateNodeHandle().param("baud", serial_baud, 0);
  if(serial_baud != 0)
    ROS_INFO("RosAria: set serial port baud rate %d", serial_baud);

  // handle debugging more elegantly
  getPrivateNodeHandle().param( "debug_aria", debug_aria, false ); // default not to debug
  getPrivateNodeHandle().param( "aria_log_filename", aria_log_filename, std::string("Aria.log") );

  // whether to connect to lasers using aria
  getPrivateNodeHandle().param("publish_aria_lasers", publish_aria_lasers, false);

  // Get frame_ids to use.
  getPrivateNodeHandle().param("odom_frame", frame_id_odom, std::string("odom"));
  getPrivateNodeHandle().param("base_link_frame", frame_id_base_link, std::string("base_link"));
  getPrivateNodeHandle().param("bumpers_frame", frame_id_bumper, std::string("bumpers"));
  getPrivateNodeHandle().param("sonar_frame", frame_id_sonar, std::string("sonar"));

  // advertise services for data topics
  // second argument to advertise() is queue size.
  // other argmuments (optional) are callbacks, or a boolean "latch" flag (whether to send current data to new
  // subscribers when they subscribe).
  // See ros::NodeHandle API docs.
  wheels_measure_pub = getPrivateNodeHandle().advertise<tuw_nav_msgs::JointsIWS>("joint_measures",1000);
  pose_pub = getPrivateNodeHandle().advertise<nav_msgs::Odometry>("pose",1000);
  bumpers_pub = getPrivateNodeHandle().advertise<rosaria::BumperState>("bumper_state",1000);
  sonar_pub = getPrivateNodeHandle().advertise<sensor_msgs::PointCloud>("sonar", 50,
      boost::bind(&RosAriaNodelet::sonarConnectCb, this),
      boost::bind(&RosAriaNodelet::sonarConnectCb, this));
  sonar_pointcloud2_pub = getPrivateNodeHandle().advertise<sensor_msgs::PointCloud2>("sonar_pointcloud2", 50,
      boost::bind(&RosAriaNodelet::sonarConnectCb, this),
      boost::bind(&RosAriaNodelet::sonarConnectCb, this));

  voltage_pub = getPrivateNodeHandle().advertise<std_msgs::Float64>("battery_voltage", 1000);
  recharge_state_pub = getPrivateNodeHandle().advertise<std_msgs::Int8>("battery_recharge_state", 5, true /*latch*/ );
  recharge_state.data = -2;
  state_of_charge_pub = getPrivateNodeHandle().advertise<std_msgs::Float32>("battery_state_of_charge", 100);

  motors_state_pub = getPrivateNodeHandle().advertise<std_msgs::Bool>("motors_state", 5, true /*latch*/ );
  motors_state.data = false;
  published_motors_state = false;

  // advertise enable/disable services
  enable_srv = getPrivateNodeHandle().advertiseService("enable_motors", &RosAriaNodelet::enable_motors_cb, this);
  disable_srv = getPrivateNodeHandle().advertiseService("disable_motors", &RosAriaNodelet::disable_motors_cb, this);

  veltime = ros::Time::now();
  veltime_cmdwh_cb = ros::Time::now();

  if( Setup() != 0 )
  {
    ROS_FATAL( "RosAria: ROS node setup failed... \n" );
    ros::shutdown();
    std::exit(-1);
  }
}

PLUGINLIB_EXPORT_CLASS(RosAriaNodelet, nodelet::Nodelet)
