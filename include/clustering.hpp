
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_array; 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan2; 
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry::SharedPtr current_odom;
    Point charge_object;
    float opponent_deg;
    // std::vector<Cluster> clusters;

    void docking_y();
    void OdomsubReceived(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callback(const sensor_msgs::msg::LaserScan::ConstPtr &);
    void Clustering(const sensor_msgs::msg::LaserScan::ConstPtr& scan_in , std::vector<pointList> &clusters , std::vector<float> &object_deg_);
    void transformPointList(const pointList& , pointList& );
    float dth =0.1;
    float charge_object_distance = 2.6;
    bool find_object = false;
    
    private:

};


