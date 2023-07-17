
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


#include "piot_can_msgs/msg/bms_flag_fb.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <chrono>
#include <iostream>



typedef std::pair<float, float> Point;
typedef std::vector<Point> pointList;


class clustering : public rclcpp::Node{

    public:
        explicit clustering ( const rclcpp::NodeOptions &);
        
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    


    rclcpp::Subscription<piot_can_msgs::msg::BmsFlagFb>::SharedPtr bms_flag_fb_sub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_array; 
     
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_array2;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan2; 
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry::SharedPtr current_odom;
    Point charge_object;
    float opponent_deg;
    // std::vector<Cluster> clusters;

    float get_yaw(float x, float y, float z, float w );

    void bmscallback(const piot_can_msgs::msg::BmsFlagFb::SharedPtr msg);
    void timerCallback();
    void timerCallback2();
    void docking_y();
    void OdomsubReceived(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callback(const sensor_msgs::msg::LaserScan::ConstPtr &);
    void Clustering(const sensor_msgs::msg::LaserScan::ConstPtr& scan_in , std::vector<pointList> &clusters , std::vector<float> &object_deg_);
    void transformPointList(const pointList& , pointList& );
    float docking_yoff = 0.01;
    float docking_xoff = 0.5;
    float target_rad = 0.0;
    bool target_rad_flag = false;
    float dth =0.05;
    float charge_object_distance = 1.0;
    bool find_object = false;
    bool keep_charge_location = false;
    int reverse =0;
    int docking_y_axis = 0;
    int rotation_aline =0;
    int rotation_opp_aline =0;
    int process_number =0;
    float reverse_sample;
    bool timer_flag =false;
    bool process_flag = false;
    bool bms_charge_flag =false;
    bool timer2_start =false;
    private:

};


