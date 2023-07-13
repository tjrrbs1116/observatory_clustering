#include "clustering.hpp"



clustering::clustering(const rclcpp::NodeOptions & options)
: Node("clustering" , options)

{
    RCLCPP_INFO(get_logger(), "clustering start");

    auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));   
    auto subscriber_options = rclcpp::SubscriptionOptions();  
    sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",custom_qos, std::bind(&clustering::callback,this,std::placeholders::_1),subscriber_options);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    pub_marker_array =  this->create_publisher<visualization_msgs::msg::Marker>("clustering/marker", rclcpp::QoS(10));

}



void clustering::callback(const sensor_msgs::msg::LaserScan::ConstPtr& scan_in){



  // delete all Markers 
  std::vector<pointList> point_clusters_not_transformed;
  clustering::Clustering(scan_in, point_clusters_not_transformed); // 극좌표계 형식으로 clustering저장

  std::vector<pointList> point_clusters; // 좌표 변환 해서 보내줌
  for (unsigned int i = 0; i < point_clusters_not_transformed.size(); ++i) {
    pointList point_cluster;
    clustering::transformPointList(point_clusters_not_transformed[i], point_cluster);
    point_clusters.push_back(point_cluster);

  }
    // RCLCPP_INFO(get_logger(),"point cluster size is %d", point_clusters_not_transformed.size());

    // Cluster Association based on the Euclidean distance
    // I should check first all the distances and then associate based on the closest distance

    std::vector<bool> g_matched(point_clusters.size(),false);   // The Group has been matched with a Cluster
    // std::vector<bool> c_matched(clusters.size(),false); // The Cluster object has been matched with a group

    // double euclidean[point_clusters.size()][2];// Matrix object to save the euclidean distances
    std::vector< std::vector<float> > euclidean(point_clusters.size() ,std::vector<float>(2));
    //Finding mean coordinates of group and associating with cluster Objects
    double mean_x = 0, mean_y = 0;

    for(unsigned int g = 0; g<point_clusters.size();++g){
      double sum_x = 0, sum_y = 0;
        
      for(unsigned int l =0; l<point_clusters[g].size(); l++){
        sum_x = sum_x + point_clusters[g][l].first;
        sum_y = sum_y + point_clusters[g][l].second;
      }
      mean_x = sum_x / point_clusters[g].size();
      mean_y = sum_y / point_clusters[g].size();

        euclidean[g][0] = abs( mean_x); 
        euclidean[g][1] = abs (mean_y);
      }

      // visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = scan_in->header.stamp;
        marker.ns = "";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      for (int i=0; i<euclidean.size();i++)
      {
        RCLCPP_INFO(get_logger(),"cluster[%d] mean x %4f",i,euclidean[i][0]);
        RCLCPP_INFO(get_logger(),"cluster[%d] mean y %4f",i,euclidean[i][1]);
        geometry_msgs::msg::Point temp;
        temp.x = euclidean[i][0];
        temp.y = euclidean[i][1];
        marker.points.push_back(temp);
        
      }
      pub_marker_array->publish(marker);


    }







void clustering::Clustering(const sensor_msgs::msg::LaserScan::ConstPtr& scan_in , std::vector<pointList> &clusters)
{

    scan = *scan_in;

  int cpoints = 0;
  
    //Find the number of non inf laser scan values and save them in c_points
    for (unsigned int i = 0; i < scan.ranges.size(); ++i){
      if( (i*scan.angle_increment >0.78 && i*scan.angle_increment < 5.49 )  ){continue;}
      if(isinf(scan.ranges[i])){continue;}
      cpoints++;
    }
    const int c_points = cpoints;
    
      // RCLCPP_INFO (get_logger(),"c_points is %d",c_points);

    int j = 0;
    std::vector< std::vector<float> > polar(c_points +1 ,std::vector<float>(2)); //c_points+1 for wrapping
    for(unsigned int i = 0; i<scan.ranges.size(); ++i){
      if(!isinf(scan.ranges[i])){

        if( (i*scan.angle_increment >0.78 && i*scan.angle_increment < 5.49 )  ){continue;}

        // RCLCPP_INFO (get_logger(), "now rad  %4f", i*scan.angle_increment);
        polar[j][0] = scan.ranges[i]; //first column is the range 
        polar[j][1] = scan.angle_min + i*scan.angle_increment; //second angle in rad
        j++;
      }
    }
    polar[c_points]=polar[0];

      // c_points = 점군개수 91개 
      // for (int i=0; i<polar.size(); i++)
      // {
      //       RCLCPP_INFO (get_logger(),"polar[%d]is ranges is  %4f", i,polar[i][0]);
      //       RCLCPP_INFO (get_logger(),"polar[%d]is rad is  %4f", i,polar[i][1]);
      // }

      //complete the circle


        //Find clusters based on adaptive threshold distance
      float d;

    //There are two flags, since two consecutive points can belong to two independent clusters
      std::vector<bool> clustered1(c_points+1 ,false); //change to true when it is the first of the cluster
      std::vector<bool> clustered2(c_points+1 ,false); // change to true when it is clustered by another one


      float l = 10; // λ is an acceptable angle for determining the points to be of the same cluster
      l = l * 0.0174532;   // degree to radian conversion;
      const float s = 0;   // σr is the standard deviation of the noise of the distance measure
      for (unsigned int i=0; i < c_points ; ++i){
        double dtheta = polar[i+1][1]- polar[i][1];
        double adaptive = std::min(polar[i][0],polar[i+1][0]) * (sin(dth)) / (sin(l - (dth))) + s; //Dthreshold
        d = sqrt( pow(polar[i][0],2) + pow(polar[i+1][0],2)-2 * polar[i][0]*polar[i+1][0]*cos(polar[i+1][1] - polar[i][1])); //제2코사인법칙
        //ROS_INFO_STREAM("distance: "<<dth<<", adapt: "<<adaptive<<", dtheta: "<<dtheta);
        //if(polar[i+1][1]- polar[i][1]<0){
          //ROS_INFO_STREAM("problem");
        //}


      // RCLCPP_INFO (get_logger(),"dtheta %4f", dtheta);
      // RCLCPP_INFO (get_logger(),"adaptive %4f",adaptive);
      // RCLCPP_INFO (get_logger(),"d %4f",d);

      if(d<dth) {
        // RCLCPP_INFO (get_logger(),"cluster now");
        clustered1[i] = true; //both points belong to clusters
        clustered2[i+1] = true;}
      }

      clustered2[0] = clustered2[c_points];

      std::vector<int> begin; //saving the first index of a cluster
      std::vector<int> nclus; //number of clustered points
      int i =0;
      bool flag = true; // flag for not going back through the stack 

      while(i<c_points && flag==true){

        if (clustered1[i] == true && clustered2[i] == false && flag == true){
          begin.push_back(i);
          nclus.push_back(1);
          while(clustered2[i+1] == true && clustered1[i+1] == true ){
      i++;
      ++nclus.back();
      if(i==c_points-1 && flag == true){ // lidar 마지막 점군까지 간 경우 
        i = -1;
        flag = false;
      }
          }
          ++nclus.back();//take care of 0 1 flags - last of the cluster
        }
      i++;
      } // while



        // take care of last point being beginning of cluster
  if(clustered1[cpoints-1]== true and clustered2[c_points-1] == false){
      begin.push_back(cpoints-1);
      nclus.push_back(1);
      i = 0;
      while(clustered2[i] == true && clustered1[i] == true ){
	i++;
	++nclus.back();
      }

  }
  polar.pop_back(); //remove the wrapping element
  int len = polar.size();

  for(unsigned int i=0; i<begin.size(); ++i){

    pointList cluster;

    double x,y;
    int j =begin[i];
    bool fl = true; // flag for not going back through the stack 

    while (j<nclus[i]+begin[i]){
      if(j== len && fl == true) fl = false;
      if (fl == true)
      {
        x = polar[j][0] * cos(polar[j][1]);       //x = r × cos( θ )
        y = polar[j][0] * sin(polar[j][1]);       //y = r × sin( θ )
      }
      else{
       x = polar[j-len][0] *cos(polar[j-len][1]); //x = r × cos( θ )
       y = polar[j-len][0] *sin(polar[j-len][1]); //y = r × sin( θ ) 
      }
      cluster.push_back(Point(x, y));
      ++j;
    }
    clusters.push_back(cluster); // 극좌표계 형식으로 포인트 저장 
  }
}


void clustering::transformPointList(const pointList& in, pointList& out){
  //This funcion transforms pointlist between coordinate frames and it is a wrapper for the
  //transformPoint function
  //There is not try catch block because it is supposed to be already encompassed into one
  
  geometry_msgs::msg::PointStamped point_in, point_out;

  geometry_msgs::msg::TransformStamped latest_tf_;
  try{
  latest_tf_ = tf_buffer_->lookupTransform("base_link","base_scan",tf2::TimePointZero);
  }
  catch (tf2::TransformException & e){return;}

  Point point; 
  point_in.header.frame_id = "base_scan";
  rclcpp::Time rclcpp_time = now();
  point_in.header.stamp = rclcpp_time;
  for (unsigned int i = 0; i < in.size(); ++i) {
    point_in.point.x = in[i].first;
    point_in.point.y = in[i].second;
    // RCLCPP_INFO (get_logger(),"point in x %4f", point_in.point.x);
    // RCLCPP_INFO (get_logger(),"point in y %4f", point_in.point.y);
    tf2::doTransform( point_in , point_out,latest_tf_);
    point.first = point_out.point.x;
    point.second= point_out.point.y;
    // RCLCPP_INFO (get_logger(),"point out x %4f", point.first);
    // RCLCPP_INFO (get_logger(),"point out y %4f", point.second);
    out.push_back(point);
  }
}
