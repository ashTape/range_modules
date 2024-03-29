#include "range_filter_footprint.hpp"

namespace range_filter_footprint {

SubFootprint::SubFootprint(ros::NodeHandle &nh,
                           const std::string &footprint_topic) {
  sub_ = nh.subscribe(footprint_topic, 10, &SubFootprint::callback, this);
  flag_footprint_ = false;
}

void SubFootprint::callback(
    const geometry_msgs::PolygonStamped::ConstPtr &msg) {
  if (flag_footprint_ == false) {
    footprint_ = *msg;
    flag_footprint_ = true;
  }
}

bool SubFootprint::getFlag() const { return flag_footprint_; }

geometry_msgs::PolygonStamped SubFootprint::getFootprint() const {
  return footprint_;
}

RangeFilterFootprint::RangeFilterFootprint(
    ros::NodeHandle &nh, const std::string &range_topic, SubFootprint &SF,
    tf2_ros::Buffer &tfBuffer_, const std::float_t &offset) { // Constructor
  sub_ = nh.subscribe(range_topic, 10, &RangeFilterFootprint::callback, this);
  pub_ = nh.advertise<sensor_msgs::Range>(range_topic + "_filtered", 10);
  pSF_ = &SF;
  ptfBuffer_ = &tfBuffer_;
  offset_ = offset;
  flag_ = false;

} // End of Copy Constructor

void RangeFilterFootprint::publish() {
  pub_.publish(filterFootprint(range_msg_));
}

void RangeFilterFootprint::callback(const sensor_msgs::Range::ConstPtr &msg) {
  if (pSF_->getFlag() == true) {
    if (flag_ == false) {
      frame_ = msg->header.frame_id.substr(1); // exclude "/"
      footprint_ = pSF_->getFootprint();
      setSonarTF();
      setThreshold();
      flag_ = true;
      ROS_INFO("[%s] (%s) Threshold : %f", ros::this_node::getName().c_str(),
               frame_.c_str(), threshold_);
    }
    range_msg_ = *msg;
    publish();
  }
}

sensor_msgs::Range
RangeFilterFootprint::filterFootprint(const sensor_msgs::Range &msg) {
  sensor_msgs::Range range_filtered_ = msg;
  if (range_filtered_.range <= threshold_ + offset_) {
    range_filtered_.range = threshold_ + offset_;
  }
  return range_filtered_;
}

void RangeFilterFootprint::setSonarTF() {
  while (ros::ok()) {
    try {
      tf_sonar_ = ptfBuffer_->lookupTransform(
          footprint_.header.frame_id, frame_, ros::Time(0), ros::Duration(3.0));
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
      static_cast<std::float_t>(*min_element(threshold.begin(), threshold.end()));
}

} // End of namespace
