#include "clustering.hpp"

#define PI  3.141592
#define rad2deg  57.2958
#define Deg2rad 0.0174533
#define debug 

using namespace std::chrono_literals;
clustering::clustering(const rclcpp::NodeOptions & options)
: Node("clustering" , options)

{
    RCLCPP_INFO(get_logger(), "clustering start");

    auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));   
    auto subscriber_options = rclcpp::SubscriptionOptions();  
    sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",custom_qos, std::bind(&clustering::callback,this,std::placeholders::_1),subscriber_options);
  


  
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    pub_marker_array =  this->create_publisher<visualization_msgs::msg::MarkerArray>("clustering/marker", rclcpp::QoS(10));
    pub_marker_array2 =  this->create_publisher<visualization_msgs::msg::Marker>("clustering/marker2", rclcpp::QoS(10));
    pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",rclcpp::QoS(10));
    // odom_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    // "odom", rclcpp::SystemDefaultsQoS(),
    // std::bind(&clustering::OdomsubReceived, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(10ms,std::bind(&clustering::timerCallback,this));

    first_aline_degree_threshold = 0.01; // rad
    docking_y_axis_x_offset = 0.55;
    docking_y_axis_tolerance =0.03;
    charge_object_distance = 1.0;
    find_rad = 1.0472;
    timer_->cancel();

 

}


void clustering::timerCallback(){
  timer_flag = true ;
  geometry_msgs::msg::Twist cmd ;
  process_number =  first_aline +docking_y_axis + rotation_aline +rotation_opp_aline+reverse;
  // RCLCPP_INFO (get_logger(),"process_number is %d ", process_number);




  switch(process_number){

    case 0 :
    {
        // RCLCPP_INFO (get_logger(),"first aline process");

          Eigen::VectorXf v (2);
          Eigen::VectorXf v1 (2);
          v(0)=  charge_object_f.first- charge_object_l.first ; v(1) = charge_object_l.first -charge_object_l.second;
          v1(0) = 1. ; v1(1) =0.;
          float dot = v.dot(v1);
          float v_norm = v.norm();
          float v1_norm = v1.norm();

          float dist_rad = acos(dot/(v_norm * v1_norm));
      #ifdef debug
        RCLCPP_INFO (get_logger(),"deg distance is %4f", abs(dist_rad - PI/2) *rad2deg);
        RCLCPP_INFO (get_logger(),"deg is %4f", dist_rad *rad2deg);
      #endif

          if (abs(dist_rad - PI/2) >= first_aline_degree_threshold ){
                  if(dist_rad >= PI/2){cmd.angular.z = -0.05;}
                  else{cmd.angular.z = 0.05;}
          }

          else{  first_aline=1; process_flag =true ;timer_->cancel(); }



      break; 
    }



    case 1 :
    {
      RCLCPP_INFO (get_logger(),"this is docking_y_axis process");
      if(    sqrt(pow(charge_object.first -docking_y_axis_x_offset ,2)+pow(charge_object.second,2))>= docking_y_axis_tolerance ){
        {
            float cmd_angle = atan2(charge_object.second,charge_object.first-docking_y_axis_x_offset);
            cmd.linear.x = 0.1*cos(cmd_angle);
            cmd.linear.y = 0.1*sin(cmd_angle);
        }
      }

      else{docking_y_axis = 1;          
          odom_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "odom", rclcpp::SystemDefaultsQoS(),
          std::bind(&clustering::OdomsubReceived, this, std::placeholders::_1));
          timer_->cancel();process_flag =true;}

      break;
    }

    case 2 :
    {
      RCLCPP_INFO (get_logger(),"this is rotation_aline process");
      float deg = atan2(charge_object.second,charge_object.first);
        
        if(abs(deg)>= 0.005){
            if(deg>0){
          cmd.angular.z = 0.05;}
          if(deg<=0) {cmd.angular.z = -0.05;}


          // pub_cmd_vel->publish(cmd);
        }

        else {
          keep_charge_location =true;
          rotation_aline = 1;
          process_flag=true;
          timer_->cancel();

       
        }


      break;}


      case 3 :
      {

        RCLCPP_INFO (get_logger(),"this is oppo rotation process");
        float yaw = clustering::get_yaw(current_odom->pose.pose.orientation.x,current_odom->pose.pose.orientation.y,current_odom->pose.pose.orientation.z,current_odom->pose.pose.orientation.w);

        if(!target_rad_flag){
              target_rad = yaw+ 3.14159;
              if(target_rad>=3.14159){target_rad = -6.28319+target_rad;}
              
              target_rad_flag = true;
        }
        RCLCPP_INFO (get_logger(),"now target yaw is %4f", target_rad*57.2958);
        RCLCPP_INFO (get_logger(),"now odom yaw is %4f", yaw*57.2958);
        if(abs(target_rad - yaw) >=0.01  ){


          cmd.angular.z = 0.3;
        }

        // pub_cmd_vel->publish(cmd);
        if(abs(target_rad - yaw) <0.01){   rotation_opp_aline =1; timer_->cancel(); process_flag =true; 
            auto custom_qos2 = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));   
        auto subscriber_options2 = rclcpp::SubscriptionOptions();  
         bms_flag_fb_sub = this->create_subscription<piot_can_msgs::msg::BmsFlagFb>("/bms_flag_fb",custom_qos2, std::bind(&clustering::bmscallback,this,std::placeholders::_1),subscriber_options2);
        

        
        }

      break;
      }


      case 4 :


      {

        RCLCPP_INFO (get_logger(),"this is reverse process");
          if(abs(reverse_sample) < charge_object.first - 0.35){
              if(!bms_charge_flag){

                cmd.linear.x = -0.05;

                reverse_sample += cmd.linear.x * 0.01;
                 RCLCPP_INFO (get_logger(),"reverse_sample is %4f",reverse_sample);

                // pub_cmd_vel->publish(cmd);
              }


              else{RCLCPP_INFO (get_logger(),"charge process success");
              timer_->cancel();}


          }

        
          else{ RCLCPP_INFO (get_logger(),"charge process failed");
          timer_->cancel();
          }





        break;
      }


  }

pub_cmd_vel->publish(cmd);


  

//timer_->cancel();
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



  // delete all Markers 
  std::vector<pointList> point_clusters_not_transformed;
  std::vector<float> object_degs;
  clustering::Clustering(scan_in, point_clusters_not_transformed,object_degs); // 극좌표계 형식으로 clustering저장
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
    float mean_x = 0, mean_y = 0;

    for(unsigned int g = 0; g<point_clusters.size();++g){
      float sum_x = 0, sum_y = 0;
        
      for(unsigned int l =0; l<point_clusters[g].size(); l++){
        sum_x = sum_x + point_clusters[g][l].first;
        sum_y = sum_y + point_clusters[g][l].second;
      }
      mean_x = sum_x / point_clusters[g].size();
      mean_y = sum_y / point_clusters[g].size();

        euclidean[g][0] =mean_x; 
        euclidean[g][1] =mean_y;

        //find charge
        
        if(sqrt(pow(euclidean[g][0],2) + pow(euclidean[g][1],2)) <= charge_object_distance){
              find_object=true;
              if(!keep_charge_location){
              charge_object= {euclidean[g][0],euclidean[g][1]}; 
              opponent_deg  = atan2(euclidean[g][1],euclidean[g][0]);
              int last_index = point_clusters[g].size() -1;
              int first_index = last_index /4 ;
              last_index =  3*last_index/4;
              charge_object_f = {point_clusters[g][first_index].first,point_clusters[g][first_index].second};
              charge_object_l = {point_clusters[g][last_index].first,point_clusters[g][last_index].second};

              }
                          visualization_msgs::msg::Marker marker2;
                          marker2.header.frame_id = "base_link";
                          marker2.header.stamp = scan_in->header.stamp;
                          marker2.ns = "";
                          marker2.id = 0;
                          marker2.type = visualization_msgs::msg::Marker::POINTS;
                          marker2.action = visualization_msgs::msg::Marker::ADD;
                          marker2.scale.x = 0.1;
                          marker2.scale.y = 0.1;
                          marker2.scale.z = 0.1;
                          marker2.color.a = 1.0;
                          marker2.color.r = 1.0;
                          marker2.color.g = 0.0;
                          marker2.color.b = 1.0;
             for(unsigned int l =0; l<point_clusters[g].size(); l++){  
                          geometry_msgs::msg::Point temp2;
                          temp2.x = point_clusters[g][l].first;
                          temp2.y = point_clusters[g][l].second;
                          marker2.points.push_back(temp2);}

                      pub_marker_array2->publish(marker2);
              // RCLCPP_INFO (get_logger(),"charge clusters is x y {%4f ,%4f} ",point_clusters[g][l].first , point_clusters[g][l].second);

      }
        }
      

      // RCLCPP_INFO (get_logger(),"charge is x y {%4f ,%4f}  deg is %4f",charge_object.first,charge_object.second,opponent_deg*57.2958);

      if(find_object){
        if(!timer_flag){
        timer_->reset();timer_flag =true;}


  
        if( process_flag){
           
          for(int i=0; i<50000; i++){
            geometry_msgs::msg::Twist cmd ;
           RCLCPP_INFO (get_logger(),"ig is %d",i);
           pub_cmd_vel->publish(cmd);
          }
             process_flag=false;
            timer_->reset() ; 
            
        } 



      }



        visualization_msgs::msg::MarkerArray marker_array;
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
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      for (int i=0; i<euclidean.size();i++)
      {
        
        geometry_msgs::msg::Point temp;
        temp.x = euclidean[i][0];
        temp.y = euclidean[i][1];
        marker.points.push_back(temp);
        
      }


        visualization_msgs::msg::Marker marker3;
        marker3.header.frame_id = "base_scan";
        marker3.header.stamp = scan_in->header.stamp;
        marker3.ns = "";
        marker3.id = 1;
        marker3.type = visualization_msgs::msg::Marker::ARROW;
        marker3.action = visualization_msgs::msg::Marker::ADD;
        marker3.scale.x = 0.05;
        marker3.scale.y = 0.05;
        marker3.scale.z = 0.05;
        marker3.color.a = 1.0;
        marker3.color.r = 0.0;
        marker3.color.g = 0.0;
        marker3.color.b = 1.0;
        marker3.pose.orientation.w = 1.0;
        geometry_msgs::msg::Point start_p,end_p;
        // geometry_msgs::msg::Point start_p2,end_p2;
        start_p.x = 0.0;
        start_p.y = 0.0;
        end_p.x = 2*cos(find_rad);
        end_p.y = 2*sin(find_rad);
        // start_p2.x =0.0;
        // start_p2.y =0.0;
        // end_p2.x = 2*cos(-1*0.87);
        // end_p2.y = 2*sin(-1*0.87);

        marker3.points.push_back(start_p);
        marker3.points.push_back(end_p);

        visualization_msgs::msg::Marker marker4;
        marker4.header.frame_id = "base_scan";
        marker4.header.stamp = scan_in->header.stamp;
        marker4.ns = "";
        marker4.id = 2;
        marker4.type = visualization_msgs::msg::Marker::ARROW;
        marker4.action = visualization_msgs::msg::Marker::ADD;
        marker4.scale.x = 0.05;
        marker4.scale.y = 0.05;
        marker4.scale.z = 0.05;
        marker4.color.a = 1.0;
        marker4.color.r = 0.0;
        marker4.color.g = 0.0;
        marker4.color.b = 1.0;
        marker4.pose.orientation.w = 1.0;

        geometry_msgs::msg::Point start_p2,end_p2;
        // start_p.x = 0.0;
        // start_p.y = 0.0;
        // end_p.x = 2*cos(0.87);
        // end_p.y = 2*sin(0.87);
        start_p2.x =0.0;
        start_p2.y =0.0;
        end_p2.x = 2*cos(-1*find_rad);
        end_p2.y = 2*sin(-1*find_rad);

        marker4.points.push_back(start_p2);
        marker4.points.push_back(end_p2);
      
        marker_array.markers.push_back(marker3);
        marker_array.markers.push_back(marker4);
        marker_array.markers.push_back(marker);
        pub_marker_array->publish(marker_array);






    }


    






void clustering::Clustering(const sensor_msgs::msg::LaserScan::ConstPtr& scan_in , std::vector<pointList> &clusters , std::vector<float > &object_deg_)
{

    scan = *scan_in;

  int cpoints = 0;
  
    //Find the number of non inf laser scan values and save them in c_points
    for (unsigned int i = 0; i < scan.ranges.size(); ++i){
      if( (i*scan.angle_increment >find_rad && i*scan.angle_increment < 2*PI -find_rad )  ){continue;}
      if(isinf(scan.ranges[i])){continue;}
      cpoints++;
    }
    const int c_points = cpoints;
    
  

    int j = 0;
    std::vector< std::vector<float> > polar(c_points +1 ,std::vector<float>(2)); //c_points+1 for wrapping
    for(unsigned int i = 0; i<scan.ranges.size(); ++i){
      if(!isinf(scan.ranges[i])){

        if( (i*scan.angle_increment >find_rad && i*scan.angle_increment < 2*PI -find_rad )  ){continue;}

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


      float l = 45; // λ is an acceptable angle for determining the points to be of the same cluster
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

    // for(int i=0; i< clustered1.size();i++)
    // {
    //     RCLCPP_INFO (get_logger(),"clustered1[%d] is %s",i ,clustered1[i] ? "true": "false");
    //     RCLCPP_INFO (get_logger(),"clustered2[%d] is %s",i ,clustered2[i] ? "true" :"false");
    // }


  
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
    float temp;
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


void
clustering::OdomsubReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // RCLCPP_INFO (get_logger(),"now  odom received");
    this->current_odom = msg ;

  

}