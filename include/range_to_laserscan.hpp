#ifndef RANGE_TO_LASERSCAN
#define RANGE_TO_LASERSCAN

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

namespace range_to_laserscan {

class RangeToLaserScan {
   private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    sensor_msgs::LaserScan scan_sonar_;

    float buf0, buf1, buf2, max_range;  // Buffer for rejecting noises

   public:
     RangeToLaserScan();
    RangeToLaserScan(ros::NodeHandle &nh, ros::Publisher &pub,
                     const std::string &range_topic);
    ~RangeToLaserScan();

    void callback(const sensor_msgs::Range::ConstPtr &msg);

    void initSonarLaserScan();
};
}

#endif
