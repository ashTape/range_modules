#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>


class RangeToLaserScan{
private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  
  float buf0,buf1,buf2,max_range; // Buffer for rejecting noises

public:
  RangeToLaserScan(){ROS_INFO("Default Constructor");}
  RangeToLaserScan(ros::NodeHandle nh, ros::Publisher pub, const std::string range_topic){
    sub_ = nh.subscribe(range_topic, 10, &RangeToLaserScan::callback, this);
    pub_ = pub;
  }
  RangeToLaserScan(const RangeToLaserScan& RTL):buf0(RTL.buf0),buf1(RTL.buf1),buf2(RTL.buf2){
    sub_ = RTL.sub_;
    ROS_INFO("[%s] Copy Constructor %f %f %f %d %d",this,buf0,buf1,buf2);
  }
  RangeToLaserScan& operator=(const RangeToLaserScan& obj){
    this->buf0=obj.buf0;
    this->buf1=obj.buf1;
    this->buf2=obj.buf2;
    this->sub_=obj.sub_;
    ROS_INFO("[%s] Copy Operator %f %f %f %d %d",this,buf0,buf1,buf2);
  }
  ~RangeToLaserScan(){ROS_INFO("[%s] Destructor", this);}

  void callback(const  sensor_msgs::Range::ConstPtr &msg){
    buf0 = buf1;  // Buffer for rejecting noises
    buf1 = buf2; // Buffer for rejecting noises
    buf2 = msg->range; // Buffer for rejecting noises
    max_range = msg->max_range; 

    if(buf0 < max_range && buf1 < max_range && buf2 < max_range){ //Rejecting noises
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
    }else{
    }
  }
};//End of class

int main(int argc, char **argv){
  ros::init(argc, argv, "range_to_laserscan");
  ROS_INFO("[%s] Node Init",ros::this_node::getName().c_str());

  ros::NodeHandle nh;
  ros::Publisher pub;

  pub = nh.advertise<sensor_msgs::LaserScan>("/scan_sonar",10);
  
  std::vector<std::string> sub_topic = {"/range"};
  std::vector<std::string> range_topics = nh.param<std::vector<std::string>>("/range_to_laserscan/range_topics" , sub_topic);

  std::vector<RangeToLaserScan*> RTL(sonar_topics.size());

  for(int i=0;i<sonar_topics.size();i++){
    RTL[i]= new RangeToLaserScan(nh, pub, sonar_topics[i]);
    ROS_INFO("[%s] Subscribe for %s", ros::this_node::getName().c_str(), sonar_topics[i].c_str());
  }

  ros::spin();
};
