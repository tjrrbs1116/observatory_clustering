
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
#include <vector>
#include <chrono>
#include <iostream>



typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;


class clustering : public rclcpp::Node{

    public:
        explicit clustering ( const rclcpp::NodeOptions &);
        
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_array; 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan2; 
    sensor_msgs::msg::LaserScan scan;
    // std::vector<Cluster> clusters;

    void callback(const sensor_msgs::msg::LaserScan::ConstPtr &);
    void Clustering(const sensor_msgs::msg::LaserScan::ConstPtr& scan_in , std::vector<pointList> &clusters);
    void transformPointList(const pointList& , pointList& );
    float dth =0.1;
    private:

};


