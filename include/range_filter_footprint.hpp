#ifndef RANGE_FILTER_FOOTPRINT
#define RANGE_FILTER_FOOTPRINT

#include "ros/ros.h"
#include "cmath"
#include "sensor_msgs/Range.h"
#include "tf/tf.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PolygonStamped.h"

namespace range_filter_footprint {

class SubFootprint {
   private:
    ros::Subscriber sub_;
    geometry_msgs::PolygonStamped footprint_;
    bool flag_footprint_;

   public:
    SubFootprint(){};
    ~SubFootprint() { ROS_INFO("Finigh Getting Footprint"); };
    SubFootprint(ros::NodeHandle &, const std::string &);
    void callback(const geometry_msgs::PolygonStamped::ConstPtr &);
    bool getFlag() const;
    geometry_msgs::PolygonStamped getFootprint() const;
};  // End of class

class RangeFilterFootprint {
   private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    SubFootprint *pSF_;
    sensor_msgs::Range range_msg_;
    geometry_msgs::PolygonStamped footprint_;
    geometry_msgs::TransformStamped tf_sonar_;
    tf2_ros::Buffer *ptfBuffer_;
    std::string frame_;
    std::float_t threshold_, offset_;
    bool flag_;
    RangeFilterFootprint(const RangeFilterFootprint &);  // Prohibit to copy
    RangeFilterFootprint &
    operator=(const RangeFilterFootprint &);  // Prohibit to substitute

   public:
    RangeFilterFootprint() {}
    ~RangeFilterFootprint() { delete pSF_; }
    RangeFilterFootprint(ros::NodeHandle &, const std::string &, SubFootprint &,
                         tf2_ros::Buffer &, const std::float_t &);
    void publish();
    void callback(const sensor_msgs::Range::ConstPtr &);
    sensor_msgs::Range filterFootprint(const sensor_msgs::Range &);
    void setSonarTF();
    void setThreshold();
};  // End of class

}  // End of namespace

#endif
