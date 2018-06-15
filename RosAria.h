#ifndef ROSARIA_H
#define ROSARIA_H

#include <stdio.h>
#include <math.h>
#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/PointCloud.h>  //for sonar data
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> // can optionally publish sonar as new type pointcloud2
#include "nav_msgs/Odometry.h"
#include "rosaria/BumperState.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"  //for tf::getPrefixParam
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <dynamic_reconfigure/server.h>
#include <rosaria/RosAriaConfig.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"

#include "LaserPublisher.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <sstream>


/** @brief Node that interfaces between ROS and mobile robot base features via ARIA library.

    RosAriaNodelet will use ARIA to connect to a robot controller (configure via
    ~port parameter), either direct serial connection or over the network.  It
    runs ARIA's robot communications cycle in a background thread, and
    as part of that cycle (a sensor interpretation task which calls RosAriaNodelet::publish()),
    it  publishes various topics with newly received robot
    data.  It also sends velocity commands to the robot when received in the
    cmd_vel topic, and handles dynamic_reconfigure and Service requests.

    For more information about ARIA see
    http://robots.mobilerobots.com/wiki/Aria.

    RosAria uses the roscpp client library, see http://www.ros.org/wiki/roscpp for
    information, tutorials and documentation.
*/
class RosAriaNodelet : public nodelet::Nodelet
{
  public:
    RosAriaNodelet();
    virtual ~RosAriaNodelet();

  public:
    int Setup();
    void cmdvel_cb( const geometry_msgs::TwistConstPtr &);
    void cmdvel_watchdog(const ros::TimerEvent& event);
    //void cmd_enable_motors_cb();
    //void cmd_disable_motors_cb();
    void spin();
    void publish();
    void sonarConnectCb();
    void dynamic_reconfigureCB(rosaria::RosAriaConfig &config, uint32_t level);
    void readParameters();
    virtual void onInit();

  protected:
    ros::Publisher pose_pub;
    ros::Publisher bumpers_pub;
    ros::Publisher sonar_pub;
    ros::Publisher sonar_pointcloud2_pub;
    ros::Publisher voltage_pub;

    ros::Publisher recharge_state_pub;
    std_msgs::Int8 recharge_state;

    ros::Publisher state_of_charge_pub;

    ros::Publisher motors_state_pub;
    std_msgs::Bool motors_state;
    bool published_motors_state;

    ros::Subscriber cmdvel_sub;

    ros::ServiceServer enable_srv;
    ros::ServiceServer disable_srv;
    bool enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    ros::Time veltime;
    ros::Time veltime_cmdwh_cb;
    ros::Timer cmdvel_watchdog_timer;
    ros::Duration cmdvel_timeout;

    std::string serial_port;
    int serial_baud;

    ArRobotConnector *conn;
    ArLaserConnector *laserConnector;
    ArRobot *robot;
    nav_msgs::Odometry position;
    rosaria::BumperState bumpers;
    ArPose pos;
    ArFunctorC<RosAriaNodelet> myPublishCB;
    //ArRobot::ChargeState batteryCharge;

    //for odom->base_link transform
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;

    std::string frame_id_odom;
    std::string frame_id_base_link;
    std::string frame_id_bumper;
    std::string frame_id_sonar;

    // flag indicating whether sonar was enabled or disabled on the robot
    bool sonar_enabled;

    // enable and publish sonar topics. set to true when first subscriber connects, set to false when last subscriber disconnects.
    bool publish_sonar;
    bool publish_sonar_pointcloud2;

    // Debug Aria
    bool debug_aria;
    std::string aria_log_filename;

    // Robot Calibration Parameters (see readParameters() function)
    int TicksMM, DriftFactor, RevCount;  //If TicksMM or RevCount are <0, don't use. If DriftFactor is -99999, don't use (DriftFactor could be 0 or negative).
    double WheelRadius;

    // dynamic_reconfigure
    dynamic_reconfigure::Server<rosaria::RosAriaConfig> *dynamic_reconfigure_server;

    // whether to publish aria lasers
    bool publish_aria_lasers;
};

#endif
