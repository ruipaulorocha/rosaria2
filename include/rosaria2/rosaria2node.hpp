#ifndef ROSARIA2_INCLUDE_ROSARIA2_ROSARIA2NODE_HPP_
#define ROSARIA2_INCLUDE_ROSARIA2_ROSARIA2NODE_HPP_

#define ADEPT_PKG

#include <stdio.h>
#include <math.h>
#ifdef ADEPT_PKG
  #include <Aria.h>
  #include <ArRobotConfigPacketReader.h>        // todo remove after ArRobotConfig implemented in AriaCoda
#else
  #include <Aria/Aria.h>
  #include <Aria/ArRobotConfigPacketReader.h>  // todo remove after ArRobotConfig implemented in AriaCoda
#endif
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include <sensor_msgs/PointCloud.h>             // for sonar data
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> // can optionally publish sonar as new type pointcloud2
#include "nav_msgs/Odometry.h"
//#include "tf2/tf.h"
#include "tf2_ros/transform_listener.h"              // for tf::getPrefixParam
#include <tf2_ros/transform_broadcaster.h>
//#include "tf2/transform_datatypes.h"
// ---> #include <dynamic_reconfigure/server.h>

// ---> #include <rosaria/RosAriaConfig.h>
// ---> #include "rosaria/BumperState.h"

//#include "LaserPublisher.h"
#include <sstream>

class RosAria2Node : public rclcpp::Node {
 public:
    // RosAriaNode(rclcpp::NodeHandle n);
    RosAria2Node(const std::string& name = "rosaria2");
    virtual ~RosAria2Node() = default;
  
  public:
    int Setup();
    void cmdvel_cb( const geometry_msgs::TwistConstPtr &);
    void cmdvel_watchdog(const rclcpp::TimerEvent& event);
    //void cmd_enable_motors_cb();
    //void cmd_disable_motors_cb();
    void spin();
    void publish();
    void sonarConnectCb();
    // ---> void dynamic_reconfigureCB(rosaria::RosAriaConfig &config, uint32_t level);
    void readParameters();

  protected:
    // rclcpp::NodeHandle n;
    rclcpp::Publisher pose_pub;
    rclcpp::Publisher bumpers_pub;
    rclcpp::Publisher sonar_pub;
    rclcpp::Publisher sonar_pointcloud2_pub;
    rclcpp::Publisher voltage_pub;

    rclcpp::Publisher recharge_state_pub;
    std_msgs::Int8 recharge_state;

    rclcpp::Publisher state_of_charge_pub;

    rclcpp::Publisher motors_state_pub;
    std_msgs::Bool motors_state;
    bool published_motors_state;

    rclcpp::Subscriber cmdvel_sub;

    rclcpp::ServiceServer enable_srv;
    rclcpp::ServiceServer disable_srv;
    bool enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    rclcpp::Time veltime;
    rclcpp::Timer cmdvel_watchdog_timer;
    rclcpp::Duration cmdvel_timeout;

    std::string serial_port;
    int serial_baud;

    ArRobotConnector *conn;
    ArLaserConnector *laserConnector;
    ArRobot *robot;
    nav_msgs::Odometry position;
    // ---> rosaria::BumperState bumpers;
    ArPose pos;
    ArFunctorC< RosAria2Node > myPublishCB;
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
    
    // dynamic_reconfigure
    // ---> dynamic_reconfigure::Server< rosaria::RosAriaConfig > *dynamic_reconfigure_server;

    // whether to publish aria lasers
    bool publish_aria_lasers;
};


#endif  // ROSARIA2_INCLUDE_ROSARIA2_ROSARIA2NODE_HPP_
