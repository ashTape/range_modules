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

class RangeFilterFootprint{ 
    ros::NodeHandle nh_; 
    ros::Subscriber sub_, sub_footprint_; 
    ros::Publisher pub_; 
    sensor_msgs::Range range_msg_; 
    geometry_msgs::PolygonStamped footprint_; 
    geometry_msgs::TransformStamped tf_sonar_; 
    std::string frame_; 
    float offset_; 
    int counter_, counter_footprint_; 
 
public: 
    RangeFilterFootprint(){} 
    ~RangeFilterFootprint(){} 
    RangeFilterFootprint(const RangeFilterFootprint& RF){ // Copy Constructor 
        nh_ = RF.nh_; 
        sub_ = RF.sub_; 
        pub_ = RF.pub_; 
        range_msg_ = RF.range_msg_; 
        footprint_ = RF.footprint_; 
        tf_sonar_ = RF.tf_sonar_; 
        frame_ = RF.frame_; 
        offset_ = RF.offset_; 
        counter_ = RF.counter_; 
        counter_footprint_ = RF.counter_footprint_; 
    } 
 
    RangeFilterFootprint(const ros::NodeHandle &nh, const std::string &range_topic, const std::string &footprint_topic){ //Constructor 
        nh_ = nh; 
        sub_ = nh_.subscribe(range_topic, 10, &RangeFilterFootprint::callback, this); 
        pub_ = nh_.advertise<sensor_msgs::Range>(range_topic + "_filtered", 10); 
        sub_footprint_ = nh_.subscribe(footprint_topic, 10, &RangeFilterFootprint::callbackFootprint, this); 
         
        counter_ = 0; 
        counter_footprint_ = 0; 
     
        tf2_ros::Buffer tfBuffer_; 
        tf2_ros::TransformListener tfListener_(tfBuffer_); 
 
    }// End of Constructor, will begin subscribe 
 
    void publish(){ 
        pub_.publish(filterFootprint(range_msg_)); 
        ROS_INFO("Publish filteredRange"); 
    } 
 
    void callback(const sensor_msgs::Range::ConstPtr &msg){ 
        if(msg != NULL){ 
            if(counter_footprint_ != 0){ 
                if(counter_ == 0){ 
                    frame_ = msg -> header.frame_id.substr(1); //exclude "/" 
                    setSonarTF(); 
                    setOffset(); 
                    counter_ ++; 
                    ROS_INFO("%s offset : %f", frame_.c_str(), offset_); 
                } 
            range_msg_ = *msg; 
            publish(); 
            } 
        } 
    } 
 
    void callbackFootprint(const geometry_msgs::PolygonStamped::ConstPtr &msg){ 
        if (msg != NULL){ 
            if (counter_footprint_ == 0){ 
                footprint_ = *msg; 
                counter_footprint_ ++; 
            } 
        } 
    } 
 
    sensor_msgs::Range filterFootprint(const sensor_msgs::Range &msg){ 
        sensor_msgs::Range range_filtered_ = msg; 
        range_filtered_.range -= offset_; 
        return range_filtered_; 
    } 
 
    void setSonarTF(){ 
        tf2_ros::Buffer tfBuffer_; 
        tf2_ros::TransformListener tfListener_(tfBuffer_); 
        while(ros::ok()){ 
            try{ 
                tf_sonar_ = tfBuffer_.lookupTransform(frame_, footprint_.header.frame_id, ros::Time::now(), ros::Duration(3.0)); 
            } 
            catch(tf2::TransformException &ex){ 
                ROS_WARN("%s", ex.what()); 
                ros::Duration(1).sleep(); 
                continue; 
            }       
            break; 
        }         
    } 
 
    void setOffset(){ 
        double roll, pitch, yaw, a1, a2, b1, b2; 
        geometry_msgs::Point32 sonar_initial_point, sonar_virtual_point, intersection; 
        int size = sizeof(footprint_.polygon.points)/6; 
        std::vector<double> offset(size); 
 
        tf::Quaternion quaternion(tf_sonar_.transform.rotation.x, tf_sonar_.transform.rotation.y, tf_sonar_.transform.rotation.z, tf_sonar_.transform.rotation.w); 
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); 
         
        sonar_initial_point.x = tf_sonar_.transform.translation.x; 
        sonar_initial_point.y = tf_sonar_.transform.translation.y; 
        sonar_virtual_point.x = sonar_initial_point.x + cos(yaw); 
        sonar_virtual_point.y = sonar_initial_point.y + sin(yaw); 
        ROS_INFO("sonar initial point (%lf %lf), yaw %lf", sonar_initial_point.x, sonar_initial_point.y, yaw); 
 
        a1 = (sonar_virtual_point.y - sonar_initial_point.y) / (sonar_virtual_point.y - sonar_initial_point.y); 
        b1 = sonar_initial_point.y - a1 * sonar_initial_point.x;  
 
        for (int i=0 ;i < size ; i++){ 
            ROS_INFO("Polygon points[%d].(%lf %lf)", i,footprint_.polygon.points[i].x, footprint_.polygon.points[i].y); 
            if(i != sizeof(footprint_.polygon.points) - 1) { 
                a2 = (footprint_.polygon.points[i].y - footprint_.polygon.points[i+1].y) / (footprint_.polygon.points[i].x - footprint_.polygon.points[i+1].x); 
                b2 = footprint_.polygon.points[i].y - a2 * footprint_.polygon.points[i].x; 
            } 
            else{ 
                a2 = (footprint_.polygon.points[i].y - footprint_.polygon.points[0].y) / (footprint_.polygon.points[i].x - footprint_.polygon.points[0].x); 
                b2 = footprint_.polygon.points[i].y - a2 * footprint_.polygon.points[i].x; 
            } 
            intersection.x = (-b1 + b2) / (a1 - a2); 
            intersection.y = (a1 * b2 - a2 * b1) / (a1 - a2); 
            ROS_INFO("[%s] # %d ", frame_.c_str(), i); 
            ROS_INFO("[%s] y = %lf x + %lf and y = %lf x + %lf", frame_.c_str(), a1,b1,a2,b2); 
            ROS_INFO("[%s] Intersection (%lf, %lf)", frame_.c_str(), intersection.x, intersection.y); 
            offset[i]=(sqrt(pow((intersection.x - sonar_initial_point.x), 2) + pow((intersection.y - sonar_initial_point.y), 2)));  
        } 
 
        offset_ = (float)*min_element(offset.begin(), offset.end()); 
    } 
 
    std::string getFrame(){return frame_;} 
 
    void setFootprint(geometry_msgs::PolygonStamped &msg){ 
        footprint_ = msg; 
    } 
};

#endif
