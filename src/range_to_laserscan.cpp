#include "range_to_laserscan.hpp"

namespace range_to_laserscan {

RangeToLaserScan::RangeToLaserScan(){
    ROS_ERROR("RangeToLaserScan Class needs arguments!");
}

RangeToLaserScan::RangeToLaserScan(ros::NodeHandle &nh, ros::Publisher &pub,
                   const std::string &range_topic) {
    sub_ = nh.subscribe(range_topic, 10, &RangeToLaserScan::callback, this);
    pub_ = pub;
    initSonarLaserScan();
}

RangeToLaserScan::~RangeToLaserScan() {}

void RangeToLaserScan::callback(const sensor_msgs::Range::ConstPtr &msg) {
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

void RangeToLaserScan::initSonarLaserScan() {
    scan_sonar_.angle_min = -0.0005;
    scan_sonar_.angle_max = 0.0005;
    scan_sonar_.angle_increment = 0.00025;
    scan_sonar_.time_increment = 0.01;
    scan_sonar_.scan_time = 0.01;
}

} //End of namespace
