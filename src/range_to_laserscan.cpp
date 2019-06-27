#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>


class RangeToLaserScan{
private:
  ros::NodeHandle nh_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  
  // float buf0,buf1,buf2,max_range; // Buffer for rejecting noises

  void callback(const  sensor_msgs::Range::ConstPtr &msg){
    // buf0 = buf1;  // Buffer for rejecting noises
    // buf1 = buf2; // Buffer for rejecting noises
    // buf2 = msg->max_range; // Buffer for rejecting noises
    // max_range = msg->max_range; 

    // if(buf0 < max_range && buf1 < max_range && buf2 < max_range){ //Rejecting noises
    sensor_msgs::LaserScan scan_sonar;

    scan_sonar.header= msg->header;
    scan_sonar.angle_min = -0.0005;
    scan_sonar.angle_max = 0.0005;
    scan_sonar.angle_increment = 0.00025;
    scan_sonar.time_increment = 0.01;
    scan_sonar.range_min = msg->min_range;
    scan_sonar.range_max = msg->max_range;
    scan_sonar.scan_time = 0.01;
    std::vector<float> vec(3,msg->range); // Fake
    scan_sonar.ranges = vec;

    pub_.publish(scan_sonar);

  // }else{
    // ROS_INFO("[%s] callback Buffers %f %f %f",msg->header.frame_id.c_str(),buf0,buf1,buf2);
    // return;
    // }
  }
public:
  RangeToLaserScan(std::string name){
    pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_sonar", 10);
    sub_ = nh_.subscribe(name.c_str(), 10, &RangeToLaserScan::callback, this);
  }

};//End of class

int main(int argc, char **argv){
  ros::init(argc, argv, "range_to_laserscan");
  ros::NodeHandle nh;
  
  std::vector<std::string> default_topic = {"/range"};
  std::vector<std::string> sonar_topics = nh.param<std::vector<std::string>>("/range_to_laserscan/range_topics" , default_topic);

  std::vector<RangeToLaserScan> RTL;

  for(int i=0;i<sonar_topics.size();i++){
    ROS_INFO("Initialize functions for %s", sonar_topics[i].c_str());
    RangeToLaserScan obj(sonar_topics[i].c_str());
    RTL.push_back(obj);
  }

  ros::spin();
};
