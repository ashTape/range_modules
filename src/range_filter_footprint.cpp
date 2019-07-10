#include "range_filter_footprint.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "range_filter_footprint");
  ros::NodeHandle nh;
  ros::Rate rate(40);

  std::string footprint_topic = nh.param<std::string>(
      ros::this_node::getName() + "/footprint_topic", "/footprint");
  std::vector<std::string> range_topics = nh.param<std::vector<std::string>>(
      ros::this_node::getName() + "/range_topics", {"/range_topics"});
  std::vector<range_filter_footprint::RangeFilterFootprint *> RFF(range_topics.size());
  range_filter_footprint::SubFootprint SF(nh, footprint_topic);

  for (int i = 0; i < range_topics.size(); i++)
    RFF[i] = new range_filter_footprint::RangeFilterFootprint(nh, range_topics[i], SF);

  ros::spin();
};
