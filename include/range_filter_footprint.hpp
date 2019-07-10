#ifndef RANGE_FILTER_FOOTPRINT
#define RANGE_FILTER_FOOTPRINT

#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/Range.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PolygonStamped.h>

namespace range_filter_footprint {

class SubFootprint {
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  geometry_msgs::PolygonStamped footprint_;
  int counter_footprint_;

public:
  SubFootprint(){};
  ~SubFootprint() { ROS_INFO("Finigh Getting Footprint"); };
  SubFootprint(const ros::NodeHandle &, const std::string &);
  void callback(const geometry_msgs::PolygonStamped::ConstPtr &);
  int getCounter();
  geometry_msgs::PolygonStamped getFootprint();
}; // End of class

class RangeFilterFootprint {
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  SubFootprint *pSF_;
  sensor_msgs::Range range_msg_;
  geometry_msgs::PolygonStamped footprint_;
  geometry_msgs::TransformStamped tf_sonar_;
  std::string frame_;
  float threshold_;
  int counter_;

public:
  RangeFilterFootprint() {}
  ~RangeFilterFootprint() { delete pSF_; }
  RangeFilterFootprint(const ros::NodeHandle &, const std::string &,
                       SubFootprint &);
  RangeFilterFootprint(const RangeFilterFootprint &);
  void publish();
  void callback(const sensor_msgs::Range::ConstPtr &);
  sensor_msgs::Range filterFootprint(const sensor_msgs::Range &);
  void setSonarTF();
  void setThreshold();
}; // End of class

SubFootprint::SubFootprint(const ros::NodeHandle &nh,
                           const std::string &footprint_topic) {
  nh_ = nh;
  sub_ = nh_.subscribe(footprint_topic, 10, &SubFootprint::callback, this);
  counter_footprint_ = 0;
};

void SubFootprint::callback(
    const geometry_msgs::PolygonStamped::ConstPtr &msg) {
  if (msg != nullptr) {
    if (counter_footprint_ == 0) {
      footprint_ = *msg;
      counter_footprint_++;
    }
  }
}

int SubFootprint::getCounter() { return counter_footprint_; }

geometry_msgs::PolygonStamped SubFootprint::getFootprint() {
  return footprint_;
}

RangeFilterFootprint::RangeFilterFootprint(const ros::NodeHandle &nh,
                                           const std::string &range_topic,
                                           SubFootprint &SF) { // Constructor
  nh_ = nh;
  sub_ = nh_.subscribe(range_topic, 10, &RangeFilterFootprint::callback, this);
  pub_ = nh_.advertise<sensor_msgs::Range>(range_topic + "_filtered", 10);
  pSF_ = &SF;
  counter_ = 0;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_(tfBuffer_);

} // End of Constructor

RangeFilterFootprint::RangeFilterFootprint(
    const RangeFilterFootprint &RF) { // Copy Constructor
  nh_ = RF.nh_;
  sub_ = RF.sub_;
  pub_ = RF.pub_;
  pSF_ = RF.pSF_;
  range_msg_ = RF.range_msg_;
  footprint_ = RF.footprint_;
  tf_sonar_ = RF.tf_sonar_;
  frame_ = RF.frame_;
  threshold_ = RF.threshold_;
  counter_ = RF.counter_;
} // End of Copy Constructor

void RangeFilterFootprint::publish() {
  pub_.publish(filterFootprint(range_msg_));
}
void RangeFilterFootprint::callback(const sensor_msgs::Range::ConstPtr &msg) {
  if (msg != nullptr) {
    if (pSF_->getCounter() != 0) {
      if (counter_ == 0) {
        frame_ = msg->header.frame_id.substr(1); // exclude "/"
        footprint_ = pSF_->getFootprint();
        setSonarTF();
        setThreshold();
        counter_++;
        ROS_INFO("[%s] (%s) Threshold : %f", ros::this_node::getName().c_str(),
                 frame_.c_str(), threshold_);
      }
      range_msg_ = *msg;
      publish();
    }
  }
}

sensor_msgs::Range
RangeFilterFootprint::filterFootprint(const sensor_msgs::Range &msg) {
  sensor_msgs::Range range_filtered_ = msg;
  if (range_filtered_.range <= threshold_) {
    range_filtered_.range = threshold_;
  }
  return range_filtered_;
}

void RangeFilterFootprint::setSonarTF() {
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_(tfBuffer_);
  while (ros::ok()) {
    try {
      tf_sonar_ = tfBuffer_.lookupTransform(footprint_.header.frame_id, frame_,
                                            ros::Time(0), ros::Duration(3.0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1).sleep();
      continue;
    }
    break;
  }
}

void RangeFilterFootprint::setThreshold() {
  double roll, pitch, yaw, a1, a2, b1, b2;
  geometry_msgs::Point32 sonar_initial_point, intersection;
  int size = sizeof(footprint_.polygon.points) / 6;
  std::vector<double> threshold(size);

  tf::Quaternion quaternion(
      tf_sonar_.transform.rotation.x, tf_sonar_.transform.rotation.y,
      tf_sonar_.transform.rotation.z, tf_sonar_.transform.rotation.w);
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

  sonar_initial_point.x = tf_sonar_.transform.translation.x;
  sonar_initial_point.y = tf_sonar_.transform.translation.y;
  // ROS_INFO("sonar initial point (%lf %lf), yaw %lf", sonar_initial_point.x,
  // sonar_initial_point.y, yaw);
  a1 = tan(yaw);
  b1 = sonar_initial_point.y - a1 * sonar_initial_point.x;

  // ROS_INFO("[%s] y = %lf x + %lf", frame_.c_str(), a1,b1);

  for (int i = 0; i < size; i++) {
    if (i != size - 1) {
      a2 =
          (footprint_.polygon.points[i].y -
           footprint_.polygon.points[i + 1].y) /
          (footprint_.polygon.points[i].x - footprint_.polygon.points[i + 1].x);
      b2 = footprint_.polygon.points[i].y - a2 * footprint_.polygon.points[i].x;
    } else {
      a2 = (footprint_.polygon.points[i].y - footprint_.polygon.points[0].y) /
           (footprint_.polygon.points[i].x - footprint_.polygon.points[0].x);
      b2 = footprint_.polygon.points[i].y - a2 * footprint_.polygon.points[i].x;
    }
    intersection.x = (-b1 + b2) / (a1 - a2);
    intersection.y = (a1 * b2 - a2 * b1) / (a1 - a2);
    // ROS_INFO("[%s](%d) y = %lf x + %lf", frame_.c_str(), i,a2,b2);
    // ROS_INFO("[%s] Intersection (%lf, %lf)", frame_.c_str(),
    // intersection.x, intersection.y);
    threshold[i] = (sqrt(pow((intersection.x - sonar_initial_point.x), 2) +
                         pow((intersection.y - sonar_initial_point.y), 2)));
    // ROS_INFO("[%s] threshold (%d) : %lf", frame_.c_str(), i, threshold[i]);
  }

  threshold_ =
      static_cast<float>(*min_element(threshold.begin(), threshold.end()));
}

} // End of namespace

#endif
