#include "range_to_laserscan.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "range_to_laserscan");
  ROS_INFO("[%s] Node Init", ros::this_node::getName().c_str());

  ros::NodeHandle nh;
  ros::Publisher pub;

  pub = nh.advertise<sensor_msgs::LaserScan>("/scan_sonar", 10);

  std::vector<std::string> sub_topic = {"/range"};
  std::vector<std::string> range_topics = nh.param<std::vector<std::string>>(
      ros::this_node::getName() + "/range_topics", {"/range_topic"});

  std::vector<RangeToLaserScan *> RTL(range_topics.size());

  for (int i = 0; i < range_topics.size(); i++) {
    RTL[i] = new RangeToLaserScan(nh, pub, range_topics[i]);
    ROS_INFO("[%s] Subscribe for %s", ros::this_node::getName().c_str(),
             range_topics[i].c_str());
  }

  ros::spin();
};
