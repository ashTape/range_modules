#include "range_filter_footprint.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "range_filter_footprint");
    ros::NodeHandle nh;
    ros::Rate rate(40);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    std::string footprint_topic = nh.param<std::string>(ros::this_node::getName() + "/footprint_topic", "/footprint");
    std::vector<std::string> range_topics = nh.param<std::vector<std::string>>(ros::this_node::getName() + "/range_topics", {"/range_topics"});
    std::vector<RangeFilterFootprint*> RFF(range_topics.size());

    for(int i=0;i<range_topics.size();i++)
        RFF[i] = new RangeFilterFootprint(nh, range_topics[i], footprint_topic);

    ros::spin();
};
