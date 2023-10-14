#ifndef ROSARIA2_INCLUDE_ROSARIA2_ROSARIA2NODE_HPP_
#define ROSARIA2_INCLUDE_ROSARIA2_ROSARIA2NODE_HPP_

// #include <Aria/ArRobotConnector.h>
#include <memory>
#include <stdio.h>
#include <math.h>
#ifdef ADEPT_PKG
  #include <Aria.h>
  #include <ArRobotConfigPacketReader.h>        // @todo remove after ArRobotConfig implemented in AriaCoda
#else
  #include <Aria/Aria.h>
  #include <Aria/ArRobotConfigPacketReader.h>   // @todo remove after ArRobotConfig implemented in AriaCoda
#endif
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>            // for sonar data
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>     // can optionally publish sonar as new type pointcloud2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "rosaria2/msg/bumper_state.hpp"
#include "laser_publisher.hpp"

// @todo
//      consistent naming for members
//      add Doxygen docstrings
//      remove unnecessary/commented calls
//      publish *only* in PointCloud2 messages
//      use smart pointers where possible



//------------------------------------------------------------------------------
/// @brief      Class that implements a ROS2 node wrapping around the ARIA library for use with Pioneer-compatible mobile robots.
///
/// @note       Based upon rosaria package (cf. https://github.com/amor-ros-pkg/rosaria) [GPLv2]
///
/// @todo       DynamicReconfigure is not available in ROS2, instead implementations should monitor parameter for changes:
///                 https://docs.ros.org/en/iron/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html
///             Store parameters in a std::map<> instance and define a callback that can be applied to all parameters
///
/// @todo       No sense to use sensor_msgs::PointCloud in 2023, should move to sensor_msgs::PointCloud2 
///
class RosAria2Node : public rclcpp::Node {
  public:
    //--------------------------------------------------------------------------
    /// @brief      Empty service request/response type aliases, for convenience and verbosity.
    ///
    /// @todo       Should be declared outside RosAria2Node.
    ///
    /// {
    using SrvRequest = std_srvs::srv::Empty::Request::SharedPtr;
    using SrvResponse = std_srvs::srv::Empty::Response::SharedPtr;
    /// }





    //--------------------------------------------------------------------------
    /// @brief      ROSAria2 parameter type.
    ///
    /// @note       Centralizes configuration on a single ROSAria2Node member 
    ///
    /// @todo       Implement callbacks to monitor parameter for changes
    ///
    class Parameters {
     public:
        //----------------------------------------------------------------------
        /// @brief      Pointer type alias.
        ///
        using Ptr = std::shared_ptr< Parameters >;

        //----------------------------------------------------------------------
        /// @brief      Constructs a new instance.
        ///
        Parameters() = default;

        //----------------------------------------------------------------------
        /// @brief      Destroys the object.
        ///
        ~Parameters() = default;

        //----------------------------------------------------------------------
        /// @brief      Serial port to use in robot connection.
        ///
        std::string serial_port = "/dev/ttyUSB0";

        //----------------------------------------------------------------------
        /// @brief      Baud rate to use in robot connection.
        ///
        int serial_baud = 9600;

        //----------------------------------------------------------------------
        /// @brief      flag indicating whether sonar was enabled or disabled on the robot.
        ///
        bool sonar_enabled;

        // enable and publish sonar topics. set to true when first subscriber connects, set to false when last subscriber disconnects.
        bool publish_sonar;
        bool publish_sonar_pointcloud2;

        // whether to publish aria lasers
        bool publish_aria_lasers;

        // whether to publish motor state (ON/OFF)
        bool publish_motors_state;

        // debug Aria
        bool debug_aria;
        std::string aria_log_filename;

        // robot calibration Parameters (see read_parameters() function)
        int TicksMM, DriftFactor, RevCount;  //If TicksMM or RevCount are <0, don't use. If DriftFactor is -99999, don't use (DriftFactor could be 0 or negative).


        void update();

        // update values from ROS Parameter server
        static Ptr fetch(const rclcpp::Node& node);

        static Ptr load(const std::string& yaml);

        static Ptr create();
    };


    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @param[in]  name  Name to attribute the node. Defaults to 'rosaria2'
    ///
    RosAria2Node(const std::string& name = "rosaria2");

    //--------------------------------------------------------------------------
    /// @brief      Destroys the object.
    ///
    virtual ~RosAria2Node();

    //--------------------------------------------------------------------------
    /// @brief      Performs an initial setup of the robot.
    ///
    /// @return     0 on sucessfull setup, 1 if an error occurs.
    ///
    /// @note       Should be called only once.
    ///
    /// @todo       Add "_setup" flag to avoid multiple calls.
    ///
    /// @todo       Review implementation, use smart pointers where possible (vs raw pointers).
    ///
    int setup();

    //--------------------------------------------------------------------------
    /// @brief      Callback for the 'cmd_vel' topic
    ///
    /// @param[in]  twist  Target twist to apply to the robot
    ///
    /// @note       Should be protected (?)
    ///
    void cmdvel_cb(const geometry_msgs::msg::Twist& twist);

    //--------------------------------------------------------------------------
    /// @brief      Callback for the watchdog timer (ensures operation only while vel commands are being published)
    ///
    /// @note       Should be protected (?)
    ///
    void cmdvel_watchdog();

    //--------------------------------------------------------------------------
    /// @brief      Callback for the 'connec_sonar' (????)
    ///
    void sonar_connect_cb();

    //--------------------------------------------------------------------------
    /// @brief      Publish robot state (batter/motor state + sensor data).
    ///
    void publish();

    //--------------------------------------------------------------------------
    /// @brief      Reads parameters.
    ///
    void read_parameters();

    //--------------------------------------------------------------------------
    /// @brief      Configuration.
    ///
    Parameters::Ptr config;

  protected:

    enum PublisherType {
        SONARS,
        LASERS,
        MOTOR_STATE,
        BATTERY_INFO
        // ...
    };
    std::array< bool, 4 /* N_PUBLISHERS*/ > _publish;

    struct AriaHandle {
        std::shared_ptr< ArRobot > robot;
        std::shared_ptr< ArRobotConnector > connector;
        std::shared_ptr< ArLaserConnector > laser_connector;
    };



    //--------------------------------------------------------------------------

    std::string serial_port;
    int serial_baud;

    // Aria-specific types
    ArRobotConnector *conn;
    ArLaserConnector *laserConnector;
    ArRobot *robot;

    // Aria wrapper implementation (for verbosity)
    // aria::RobotHandle::Ptr _robot;

    // flag indicating whether sonar was enabled or disabled on the robot
    bool sonar_enabled;

    // enable and publish sonar topics. set to true when first subscriber connects, set to false when last subscriber disconnects.
    bool publish_sonar;
    bool publish_sonar_pointcloud2;

    // whether to publish aria lasers
    bool publish_aria_lasers;

    // whether to publish motor state (ON/OFF)
    bool publish_motors_state;

    // debug Aria
    bool debug_aria;
    std::string aria_log_filename;

    // robot calibration Parameters (see read_parameters() function)
    int TicksMM, DriftFactor, RevCount;  //If TicksMM or RevCount are <0, don't use. If DriftFactor is -99999, don't use (DriftFactor could be 0 or negative).

    // rclcpp::NodeHandle n;
    std_msgs::msg::Int8 recharge_state;
    std_msgs::msg::Bool motors_state;

    rclcpp::Publisher< nav_msgs::msg::Odometry >::SharedPtr pose_pub;
    rclcpp::Publisher< rosaria2::msg::BumperState >::SharedPtr bumpers_pub;
    rclcpp::Publisher< sensor_msgs::msg::PointCloud >::SharedPtr sonar_pub;  // old type -> remove
    rclcpp::Publisher< sensor_msgs::msg::PointCloud2 >::SharedPtr sonar_pointcloud2_pub;
    rclcpp::Publisher< std_msgs::msg::Float64 >::SharedPtr voltage_pub;
    rclcpp::Publisher< std_msgs::msg::Int8 >::SharedPtr recharge_state_pub;
    rclcpp::Publisher< std_msgs::msg::Float32 >::SharedPtr state_of_charge_pub;
    rclcpp::Publisher< std_msgs::msg::Bool >::SharedPtr motors_state_pub;

    rclcpp::Subscription< geometry_msgs::msg::Twist >::SharedPtr cmdvel_sub;

    rclcpp::Service< std_srvs::srv::Empty >::SharedPtr enable_srv;
    rclcpp::Service< std_srvs::srv::Empty >::SharedPtr disable_srv;
    bool enable_motors_cb(const SrvRequest request, SrvResponse response);  // ptr types can be used instead
    bool disable_motors_cb(const SrvRequest request, SrvResponse response);  // ptr types can be used instead

    rclcpp::Time veltime;
    // @todo change type to WallTimer: can a wall timer be used or only a simple timer
    // rclcpp::WallTimer< > cmdvel_watchdog_timer;
    rclcpp::TimerBase::SharedPtr cmdvel_watchdog_timer;
    rclcpp::Duration cmdvel_timeout;

    ArPose pose;
    ArFunctorC< RosAria2Node > publish_cb;  // callback
    //ArRobot::ChargeState batteryCharge;

    nav_msgs::msg::Odometry position;

    rosaria2::msg::BumperState bumpers;

    //for odom->base_link transform
    std::unique_ptr< tf2_ros::TransformBroadcaster > odom_broadcaster;
    geometry_msgs::msg::TransformStamped odom_trans;

    std::string frame_id_odom;
    std::string frame_id_base_link;
    std::string frame_id_bumper;
    std::string frame_id_sonar;

    // dynamic_reconfigure
    // ---> dynamic_reconfigure::Server< rosaria::RosAriaConfig > *dynamic_reconfigure_server;

};


#endif  // ROSARIA2_INCLUDE_ROSARIA2_ROSARIA2NODE_HPP_
