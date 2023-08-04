#include "clustering.hpp"

#define PI  3.141592
#define rad2deg  57.2958
#define Deg2rad 0.0174533
#define debug 

using namespace std::chrono_literals;

namespace observatory{
clustering::clustering(const rclcpp::NodeOptions & options)
: Node("observatoryclustering" , options)

{
    RCLCPP_INFO(get_logger(), "observatory clustering start");

    auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));   
    auto subscriber_options = rclcpp::SubscriptionOptions();  
    sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",custom_qos, std::bind(&clustering::callback,this,std::placeholders::_1),subscriber_options);
  


  
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    pub_tracks_box_kf     =this->create_publisher<clustering_msgs::msg::TrackArray>("datmo/box_kf", 10);
    pub_marker_array =  this->create_publisher<visualization_msgs::msg::MarkerArray>("observatory_clustering/marker", rclcpp::QoS(10));
    // pub_marker_array2 =  this->create_publisher<visualization_msgs::msg::Marker>("clustering/marker2", rclcpp::QoS(10));
    pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",rclcpp::QoS(10));
    // odom_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    // "odom", rclcpp::SystemDefaultsQoS(),
    // std::bind(&clustering::OdomsubReceived, this, std::placeholders::_1));s
    // timer_ = this->create_wall_timer(10ms,std::bind(&clustering::timerCallback,this));


    dt = 0.08;
    euclidean_distance = 0.25;
    max_cluster_size = 360;
    p_marker_pub = false;
    // timer_->cancel();

 

}


void clustering::timerCallback(){
  }




float clustering::get_yaw(float x, float y ,float z, float w){
      float roll , pitch ,yaw;
                float t0 = 2.0 * (w *x + y*z);
                float t1 = 1.0 - 2.0 * (x*x + y*y);
                roll = atan2(t0,t1);

                float t2 = 2.0 * (w*z + x*y);
                if(t2> 1.0){ t2 = 1.0; }
                if(t2<-1.0){ t2 = -1.0;}
                pitch = asin(t2);

                float t3 = 2.0 * (w*z + x*y);
                float t4 = 1.0 - 2.0 * (y*y + z*z);
                yaw = atan2(t3,t4);

return yaw;
}




void clustering::bmscallback(const piot_can_msgs::msg::BmsFlagFb::SharedPtr msg){


     bms_charge_flag =msg->bms_flag_fb_charge_flag ;

  
     if (bms_charge_flag){RCLCPP_INFO(get_logger(),"this is chargeflag true");}

}
void clustering::callback(const sensor_msgs::msg::LaserScan::ConstPtr& scan_in){

  visualization_msgs::msg::Marker marker;
  visualization_msgs::msg::MarkerArray markera;
  marker.action =3;
  markera.markers.push_back(marker);
  pub_marker_array->publish(markera);



  RCLCPP_INFO(get_logger(),"deubg1");
  try{
  geometry_msgs::msg::TransformStamped stamped = tf_buffer_->lookupTransform("base_link","base_scan",tf2::TimePointZero);
  tf2::fromMsg(stamped.transform, ego_pose);
  }
  
  catch (tf2::TransformException & e){return;}


  // delete all Markers 
  std::vector<pointList> point_clusters_not_transformed;
  std::vector<float> object_degs;
  clustering::Clustering(scan_in, point_clusters_not_transformed,object_degs); // 극좌표계 형식으로 clustering저장
  std::vector<pointList> point_clusters; // 좌표 변환 해서 보내줌

  RCLCPP_INFO(get_logger(),"point cluster size is %d", point_clusters_not_transformed.size());


  for (unsigned int i = 0; i < point_clusters_not_transformed.size(); ++i) {
    pointList point_cluster;
    clustering::transformPointList(point_clusters_not_transformed[i], point_cluster);
    point_clusters.push_back(point_cluster);

  }

    std::vector<bool> g_matched(point_clusters.size(),false);   // The Group has been matched with a Cluster
    std::vector<bool> c_matched(clusters.size(),false); // The Cluster object has been matched with a group


    std::vector< std::vector<float> > euclidean(point_clusters.size() ,std::vector<float>(2));
    //Finding mean coordinates of group and associating with cluster Objects
    float mean_x = 0, mean_y = 0;
    RCLCPP_INFO(get_logger(),"point clusters size is %d", point_clusters.size());
    for(unsigned int g = 0; g<point_clusters.size();++g){
      float sum_x = 0, sum_y = 0;
        
      RCLCPP_INFO(get_logger(),"point clusters[%d] size is %d",g,point_clusters[g].size());
      int temp =0;
      for(unsigned int l =0; l<point_clusters[g].size(); l++){

        sum_x = sum_x + point_clusters[g][l].first;
        sum_y = sum_y + point_clusters[g][l].second;
        RCLCPP_INFO(get_logger(),"temp is %d", temp);
        temp ++;
      }
      mean_x = sum_x / point_clusters[g].size();
      mean_y = sum_y / point_clusters[g].size();

      //---------------------------------------------------------------------------------//
      // RCLCPP_INFO (get_logger(),"pointcluster[%d]  is x y {%4f ,%4f} ",g,mean_x , mean_y);

      for(unsigned int c=0;c<clusters.size();++c)
      {
        euclidean[g][c] = abs( mean_x - clusters[c].meanX()) + abs(mean_y - clusters[c].meanY()); 
      }
    }
      

       //Matrix object to save the euclidean distances 
      std::vector<std::pair <int,int> > pairs;
      for(unsigned int c=0; c<clusters.size();++c){
        unsigned int position;
        double min_distance = euclidean_distance;
        for(unsigned int g=0; g<point_clusters.size();++g){
          if(euclidean[g][c] < min_distance){
              min_distance = euclidean[g][c];
              position = g;
          }
        }
      if(min_distance < euclidean_distance){
        g_matched[position] = true, c_matched[c] = true;
        pairs.push_back(std::pair<int,int>(c,position));
        }
    }




    //Update Tracked Clusters
    // #pragma omp parallel for
    // for(unsigned int p=0; p<pairs.size();++p){
    //   clusters[pairs[p].first].update(point_clusters[pairs[p].second], dt, ego_pose);
    // }
    RCLCPP_INFO(get_logger(),"deubg2");

    //Delete Not Associated Clusters
    unsigned int o=0;
    unsigned int p = clusters.size();
    while(o<p){
      if(c_matched[o] == false){

        std::swap(clusters[o], clusters.back());
        clusters.pop_back();

        std::swap(c_matched[o], c_matched.back());
        c_matched.pop_back();

        o--;
        p--;
      }
    o++;
    }
    
    // Initialisation of new Cluster Objects
    for(unsigned int i=0; i<point_clusters.size();++i){
      if(g_matched[i] == false && point_clusters[i].size()< max_cluster_size){
	  Cluster cl(cclusters, point_clusters[i], dt, "base_link", ego_pose);
	  cclusters++;
	clusters.push_back(cl);
      } 
    }


    //Visualizations and msg publications
    visualization_msgs::msg::MarkerArray marker_array;
    clustering_msgs::msg::TrackArray track_array_box_kf; 
    for (unsigned int i =0; i<clusters.size();i++){

      track_array_box_kf.tracks.push_back(clusters[i].msg_track_box_kf);
     
      if (p_marker_pub){
        marker_array.markers.push_back(clusters[i].getClosestCornerPointVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getBoundingBoxCenterVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getArrowVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getThetaL1VisualisationMessage());
        marker_array.markers.push_back(clusters[i].getThetaL2VisualisationMessage());
        marker_array.markers.push_back(clusters[i].getThetaBoxVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getClusterVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getBoundingBoxVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getBoxModelKFVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getLShapeVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getLineVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getBoxSolidVisualisationMessage());
      }; 
    }
RCLCPP_INFO(get_logger(),"deubg3");
    pub_marker_array->publish(marker_array);
    pub_tracks_box_kf->publish(track_array_box_kf);
    visualiseGroupedPoints(point_clusters);

  RCLCPP_INFO(get_logger(),"deubg4");

  }
    


    



void clustering::visualiseGroupedPoints(const std::vector<pointList>& point_clusters){
  //Publishing the clusters with different colors
  visualization_msgs::msg::MarkerArray marker_array;
  //Populate grouped points message
  visualization_msgs::msg::Marker gpoints;
  gpoints.header.frame_id = "base_link";
  gpoints.header.stamp = rclcpp::Clock().now();
  gpoints.ns = "clustered_points";
  gpoints.action = visualization_msgs::msg::Marker::ADD;
  gpoints.pose.orientation.w = 1.0;
  gpoints.type = visualization_msgs::msg::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  gpoints.scale.x = 0.04;
  gpoints.scale.y = 0.04;
  for(unsigned int i=0; i<point_clusters.size(); ++i){

    gpoints.id = cg;
    cg++;
    gpoints.color.g = rand() / double(RAND_MAX);
    gpoints.color.b = rand() / double(RAND_MAX);
    gpoints.color.r = rand() / double(RAND_MAX);
    gpoints.color.a = 1.0;
    //gpoints.lifetime = ros::Duration(0.08);
    for(unsigned int j=0; j<point_clusters[i].size(); ++j){
      geometry_msgs::msg::Point p;
      p.x = point_clusters[i][j].first;
      p.y = point_clusters[i][j].second;
      p.z = 0;
      gpoints.points.push_back(p);
    }
    marker_array.markers.push_back(gpoints);
    gpoints.points.clear();
  }
  pub_marker_array->publish(marker_array);

}


void clustering::Clustering(const sensor_msgs::msg::LaserScan::ConstPtr& scan_in , std::vector<pointList> &clusters , std::vector<float > &object_deg_)
{

      scan = *scan_in;
      int cpoints = 0;

      //Find the number of non inf laser scan values and save them in c_points
      for (unsigned int i = 0; i < scan.ranges.size(); ++i){
        if(isinf(scan.ranges[i]) == 0){
          cpoints++;
        }
      }
      const int c_points = cpoints;

      int j = 0;
      std::vector< std::vector<float> > polar(c_points +1 ,std::vector<float>(2)); //c_points+1 for wrapping
      for(unsigned int i = 0; i<scan.ranges.size(); ++i){
        if(!isinf(scan.ranges[i])){
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


      float l = 45; // λ is an acceptable angle for determining the points to be of the same cluster
      l = l * 0.0174532;   // degree to radian conversion;
      const float s = 0;   // σr is the standard deviation of the noise of the distance measure
      for (unsigned int i=0; i < c_points ; ++i){
        double dtheta = polar[i+1][1]- polar[i][1];
        double adaptive = std::min(polar[i][0],polar[i+1][0]) * (sin(dth)) / (sin(l - (dth))) + s; //Dthreshold
        d = sqrt( pow(polar[i][0],2) + pow(polar[i+1][0],2)-2 * polar[i][0]*polar[i+1][0]*cos(polar[i+1][1] - polar[i][1])); //제2코사인법칙


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

  // for(int i=0; i<begin.size(); i++)
  // {RCLCPP_INFO (get_logger(),"begin point is %d", begin[i]);
  // RCLCPP_INFO (get_logger()," nclus is %d", nclus[i]);
  // }

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
  point_in.header.stamp = rclcpp::Clock().now();
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


void
clustering::OdomsubReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // RCLCPP_INFO (get_logger(),"now  odom received");
    this->current_odom = msg ;

  

}

}


