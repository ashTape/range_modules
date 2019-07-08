#ifndef RANGE_TO_LASERSCAN
#define RANGE_TO_LASERSCAN

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

class RangeToLaserScan {
private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  sensor_msgs::LaserScan scan_sonar_;

  float buf0, buf1, buf2, max_range; // Buffer for rejecting noises

public:
  RangeToLaserScan() {}
  RangeToLaserScan(ros::NodeHandle &nh, ros::Publisher &pub,
                   const std::string &range_topic) {
    sub_ = nh.subscribe(range_topic, 10, &RangeToLaserScan::callback, this);
    pub_ = pub;
    initSonarLaserScan();
  }
  RangeToLaserScan(const RangeToLaserScan &RTL)
      : buf0(RTL.buf0), buf1(RTL.buf1), buf2(RTL.buf2) {
    sub_ = RTL.sub_;
    pub_ = RTL.pub_;
    scan_sonar_ = RTL.scan_sonar_;
  }
  RangeToLaserScan &operator=(const RangeToLaserScan &RTL) {
    this->buf0 = RTL.buf0;
    this->buf1 = RTL.buf1;
    this->buf2 = RTL.buf2;
    this->sub_ = RTL.sub_;
    this->pub_ = RTL.pub_;
    this->scan_sonar_ = RTL.scan_sonar_;
  }
  ~RangeToLaserScan() {}

  void callback(const sensor_msgs::Range::ConstPtr &msg) {
    buf0 = buf1;       // Buffer for rejecting noises
    buf1 = buf2;       // Buffer for rejecting noises
    buf2 = msg->range; // Buffer for rejecting noises

    if (std::max({buf0, buf1, buf2}) < msg->max_range) { // Rejecting noises
      scan_sonar_.header = msg->header;
      scan_sonar_.range_min = msg->min_range;
      scan_sonar_.range_max = msg->max_range;

      std::vector<float> vec(3, msg->range); // Fake
      scan_sonar_.ranges = vec;
      pub_.publish(scan_sonar_);
    } else {
      return;
    }
  }

  void initSonarLaserScan() {
    scan_sonar_.angle_min = -0.0005;
    scan_sonar_.angle_max = 0.0005;
    scan_sonar_.angle_increment = 0.00025;
    scan_sonar_.time_increment = 0.01;
    scan_sonar_.scan_time = 0.01;
  }
}; // End of class

#endif
