#ifndef ROSARIA2_INCLUDE_ROSARIA2_ARIAUTILS_HPP_
#define ROSARIA2_INCLUDE_ROSARIA2_ARIAUTILS_HPP_


#include <memory>
namespace aria {


// should be a singleton class?

#include <rclcpp/rclcpp.hpp>
#ifdef ADEPT_PKG
#include "ariaUtil.h"
#else
#include "Aria/ariaUtil.h"
#endif


rclcpp::Time toROSTime(const ArTime& t, const rclcpp::Node& node) {
    // ARIA/ARNL times are in reference to an arbitrary starting time, not OS clock, so find the time elapsed between now and t
    // to adjust the time stamp in ROS time vs. now accordingly.
    ArTime arianow;
    const double dtsec = (double) t.mSecSince(arianow) / 1000.0;
    //printf("was %f seconds ago\n", dtsec);
    return rclcpp::Time(node.now().seconds() - dtsec);
}



class RobotHandle {
 public:
    //--------------------------------------------------------------------------
    /// @brief      Pointer type alias
    ///
    using Ptr = std::shared_ptr< RobotHandle >;

    //--------------------------------------------------------------------------
    /// @brief      Class instance (as pointer type)
    ///
    static Ptr instance;

    //--------------------------------------------------------------------------
    /// @brief      Static named constructor
    ///
    /// @return     { description_of_the_return_value }
    ///
    /// @todo       Call init()?
    ///
    static Ptr get() {
        if (!instance) {
            // if no instance has been created, create one
            instance = Ptr(new RobotHandle());
        }
        return instance;
    }

    //--------------------------------------------------------------------------
    /// @brief      Destroys the object.
    ///
    virtual ~RobotHandle() = default;

    //--------------------------------------------------------------------------
    /// @brief      *no copy constructor declared*
    ///
    /// @note       Singleton types can't be copied, thus default copy constructor must be deleted.
    ///
    RobotHandle(const RobotHandle&) = delete;

    //--------------------------------------------------------------------------
    /// @brief      Initializes the object.
    ///
    /// @return     { description_of_the_return_value }
    ///
    int init(bool laser);

    const std::shared_ptr< ArRobot > robot;
    const std::shared_ptr< ArRobotConnector > connector;
    const std::shared_ptr< ArLaserConnector > laser_connector;

    //--------------------------------------------------------------------------
    /// @brief      Checks if robot handle has been initialized.
    ///
    /// @return     True if init() has been called, false otherwise.
    ///
    bool valid() const;

    //--------------------------------------------------------------------------
    /// @brief      Checks if initialized.
    ///
    /// @note       Wraps around valid() member, provided for convenience.
    ///
    operator bool() const;

protected:
    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @note       Singleton types can only be instantiated once, thus default constructor is private/protected.
    ///
    RobotHandle() = default;
};


RobotHandle::RobotHandle() :
    robot(new ArRobot()) {
        // only if embedded laser connection is required

}


int RobotHandle::init(bool laser) {

    robot = std::make_shared< ArRobot >(new ArRobot());

    // only needed within init()
    ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
    ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
    argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)


    // connect to the robot
    connector = std::make_shared< ArRobotConnector >(new ArRobotConnector(argparser, robot));  // warning: should never be freed
    if (!conn->connectRobot()) {
        RCLCPP_ERROR(this->get_logger(), "Aria could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
        return 1;
    }


    // create laser connection (when configured)
    if(laser) {
        laser_connection = new ArLaserConnector(argparser, robot, conn);
    }

    // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
    if (!Aria::parseArgs()) {
        RCLCPP_ERROR(this->get_logger(), "Aria error parsing startup parameters!");
        return 1;
    }

    return 0;
}


bool RobotHandle::valid() const {
    return robot;  // cast of pointer to bool is implicit
}


RobotHandle::operator bool() const {
    return this->valid();
}

}  // namespace aria

#endif  // ROSARIA2_INCLUDE_ROSARIA2_ARIAUTILS_HPP_
