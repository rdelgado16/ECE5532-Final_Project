#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <ugv_course_libs/gps_conv.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <math.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

const double RATIO = 17.3;
const double LENGHT = 2.65;

ros::Publisher steering_pub;
ros::Publisher speed_pub;

UTMCoords ref_point;

UTMCoords ref_coords;
tf::Vector3 ref_relative_position;
nav_msgs::Path gps_path;
ros::Publisher controller_pub;
geometry_msgs::Twist audibot_params; // Twist message for the audibot parameters
UTMCoords UTM_local_intersections[18];

std_msgs::Float64 path_steer;
std_msgs::Float64 path_speed;

std::vector<int> int_actions;  // Legend: -1 = Stop, 0 = Straight, 1 = Left, 2 = Right
std::vector<int> int_order;
int int_count = 0;

// Starting coordinates of the audiobot
double ref_lat;
double ref_lon;

// Vehicle speed and yaw rate
double veh_spd;
double veh_yaw;

double conv_angle; // Convergence angle
double waypoint_theta; // Angle between waypoint and vehicle
double dist; // Direct distance between vehicle and waypoint
double central_meridian; // Central meridian of the robots reference position

double heading_ROS;
double desired_heading = 0; // Heading of the vehicle but in ROS format (due East)
bool int_flag = false;
int heading_count = 0;

double heading_error;


// List of UTM converted waypoint coordinates
UTMCoords waypoint_UTM[8];

void initGraph(){
  int_actions.resize(6);
  int_actions = {1, 0, 1, 0, 0, -1};
  int_order.resize(6);
  int_order = {7, 8 , 14, 13, 12, 18};
}

void initIntersections(){
  // List of GPS waypoint coordinates
  const double intersections[18][2] = {
    {42.0024367640, -83.0026815700},
    {42.0024491386, -83.0018985140},
    {42.0052849918,	-82.9982609600},
    {42.0025810159,	-82.9944807940},
    {42.0016393444,	-82.9931539150},
    {42.0011509630,	-82.9951296391},
    {41.9997857060,	-82.9956885461},
    {42.0003776418,	-82.9957040005},
    {42.0003525220,	-82.9970950120},
    {42.0009389400,	-82.9971147990},
    {42.0003403860,	-82.9978830675},
    {42.0013712950,	-82.9979085647},
    {42.0013999875,	-82.9965243760},
    {42.0014086810,	-82.9957412321},
    {42.0032462520,	-82.9976927114},
    {42.0027408455,	-82.9969826401},
    {42.0026591745,	-82.9976741668},
    {42.0026749210,	-83.0005488420},
    };
  for (int i = 0; i < 18; i++) {
    UTMCoords waypoint(LatLon(intersections[i][0], intersections[i][1], 0.0));
    UTM_local_intersections[i] = waypoint;
  }
}

// Grabbing vehicle's speed and yaw rate
void recvVehState(const geometry_msgs::TwistStampedConstPtr& msg){
  veh_spd = msg->twist.linear.x;
  veh_yaw = msg->twist.angular.z;
}

void recvPathFollowing(const geometry_msgs::TwistConstPtr& msg){
  double v = msg->linear.x;
  double psi_dot = msg->angular.z;

  // path_speed.linear.x = v;
  // path_speed.angular.z = psi_dot;
  path_speed.data = 0.1;
  path_steer.data = RATIO*atan(LENGHT*psi_dot/v);
}

void recvFix(const sensor_msgs::NavSatFixConstPtr& msg){
  UTMCoords current_coords(*msg);
  double veh_lat = msg->latitude;
  double veh_lon = msg->longitude;
  double veh_x = current_coords.getX();
  double veh_y = current_coords.getY();
  // int traveled_int = 1; // If the intersection has already been traversed, don't count it again

  tf::Vector3 waypoint_veh_pos;

  // Calculate convergence angle for ENU heading calc and convert to rads
  conv_angle = (atan(tan(veh_lon-central_meridian)*sin(veh_lat)))* (M_PI/180);

  double waypoint_x = UTM_local_intersections[int_order[int_count]-1].getX();
  double waypoint_y = UTM_local_intersections[int_order[int_count]-1].getY();
  // // Calculate heading offset between vehicle and waypoint
  waypoint_theta = atan2((waypoint_y - veh_y), (waypoint_x - veh_x)) - conv_angle; // Subtract convergence angle
  if(waypoint_theta <= 0){
    waypoint_theta += 2*M_PI; // Keeps the waypoint angle between 0 and 2pi
  }
  ROS_INFO("Waypoint_theta: {%f}, waypoint_y: {%f}, waypoint_x: {%f}, veh_x: {%f}, veh_y: {%f}, conv_angle: {%f}", waypoint_theta - heading_ROS, waypoint_y, waypoint_x, veh_x, veh_y, conv_angle);
  // Vector between vehicle and waypoints
  waypoint_veh_pos = current_coords - UTM_local_intersections[int_order[int_count]-1];

  // Finding direct distance between vehicle and waypoint
  dist = sqrt(pow(waypoint_veh_pos.getX(),2) + pow(waypoint_veh_pos.getY(),2));
  double time_to_int = dist/veh_spd;
  ROS_INFO("Vehicle speed: {%f}", veh_spd);
  ROS_INFO("Distance to intersection %d: (%f)", int_order[int_count], dist);    
  ROS_INFO("Time to Intersection %f", time_to_int);
  ROS_INFO("FLAG: {%d}", int_flag);
  ROS_INFO("Current Heading: %f, Heading Error: %f, Desired Heading: %f, Heading Count: %d", heading_ROS, heading_error, desired_heading, heading_count);

  if (time_to_int <= 3 && time_to_int > 0 && int_flag == false){
    int_flag = true;
  }

  if(abs(veh_yaw) >= .5 && veh_spd > 2){
    audibot_params.linear.x = audibot_params.linear.x;
    audibot_params.angular.z = 0;
    controller_pub.publish(audibot_params);
   ROS_INFO("Saving car from oversteer");
  }

  if (heading_count == 1){
    if(int_actions[int_count] == 1){
      desired_heading = heading_ROS + M_PI/2;
    }
    else if (int_actions[int_count] == 2){
      desired_heading = heading_ROS - M_PI/2;
    }
    else if (int_actions[int_count] < 0){
      desired_heading = heading_ROS;
    }
  } 
  heading_error = (desired_heading - heading_ROS);
  if(heading_error <= -M_PI){
    heading_error = 2 * M_PI - heading_error;
  }
  else if(heading_error > M_PI){
    heading_error = -(2 * M_PI - heading_error);
  }
  
  if(int_flag == true){
    ROS_INFO("TURNING AT INTERSECTION {%d}", int_order[int_count]);
    // LEFT OR RIGHT TURN
    if(int_actions[int_count] > 0 ){
      if(time_to_int < 3){
        heading_count++;
      }
      if(int_actions[int_count] == 1){
        if(dist > 7){
          // set_speed = 10;
          audibot_params.linear.x = 5;
          audibot_params.angular.z = 0;
          controller_pub.publish(audibot_params);
        }
        else{
          if (abs(heading_error) < 0.5 && heading_error != 0){
            int_flag = false;
            heading_count = 0;
            if(int_count<int_actions.size()){
              int_count++;
            }
          }
          else{
                  audibot_params.linear.x = 3;
                  audibot_params.angular.z = 4.5 * heading_error;
                  controller_pub.publish(audibot_params);
              }
        }
      }
      else if (int_actions[int_count] == 2){
        if(dist > 11){
          // set_speed = 10;
          audibot_params.linear.x = 5;
          audibot_params.angular.z = 0;
          controller_pub.publish(audibot_params);
        }
        else{
          if (abs(heading_error) < 0.5 && heading_error != 0){
            int_flag = false;
            heading_count = 0;
            if(int_count<int_actions.size()){
              int_count++;
            }
          }
          else{
                  audibot_params.linear.x = 3;
                  audibot_params.angular.z = 5 * heading_error;
                  controller_pub.publish(audibot_params);
              }
        }
      }
    }
    // Going straight at the intersection
    else if (int_actions[int_count] == 0){
         if (veh_spd<20){
      path_speed.data = 1;
    }  
    else{
      path_speed.data = 0;
    }
    speed_pub.publish(path_speed);  
    steering_pub.publish(path_steer);
   
      if (dist<8){
        int_flag = false;
        if(int_count<int_actions.size()){
          int_count++;
        }
        }
      }
      // Stopping at intersection
      else if (int_actions[int_count] == -1){
        if(dist<40){
          audibot_params.linear.x = 0;
          audibot_params.angular.z = waypoint_theta - heading_ROS;
          controller_pub.publish(audibot_params);        
        }
        else if (dist<5){
          ROS_INFO("WE MADE IT");
          audibot_params.linear.x = 0;
          audibot_params.angular.z = 0;
          controller_pub.publish(audibot_params);    
        }
      }
      
  }else{
    ROS_INFO("NEXT INTERSECTION {%d}", int_order[int_count]);
    double max_spd;
    if(path_steer.data < 0.5){
      max_spd = 25;
    }else{
      max_spd = 20;
    }
    if (veh_spd<max_spd){
      path_speed.data = 1;
    }  
    else{
      path_speed.data = 0;
    }
    speed_pub.publish(path_speed);  
    steering_pub.publish(path_steer);
  }
}

void recvHeading(const std_msgs::Float64ConstPtr& msg){
  std_msgs::Float64 robot_heading; // Audibot's current heading
  robot_heading.data = msg->data; // Heading is reported from robot in degrees relative due North
  double heading_rad = 2*M_PI - (robot_heading.data * (M_PI/180)); // Convert to Radians and make counter-clockwise;
  // Convert from GPS heading to ROS heading
  if ((3*M_PI/2) <= heading_rad && heading_rad <= 2*M_PI){
    heading_ROS = abs(heading_rad - (3*M_PI/2)); // If GPS heading is between 3pi/2 and 2pi, subtract 3pi/2
  }
  else{
    heading_ROS = heading_rad + (M_PI/2); // If GPS heading is anywhere else, just add pi/2
  }
}

int main(int argc, char** argv){
  ros::init(argc,argv,"final_project");
  ros::NodeHandle nh;
  initGraph();
  initIntersections();
  ros::Subscriber gps_sub = nh.subscribe("/audibot/gps/fix",1,recvFix); // Grab current gps position
  ros::Subscriber heading_sub = nh.subscribe("/audibot/gps/heading",1,recvHeading); // Grab current heading
  ros::Subscriber veh_spd_yaw_sub = nh.subscribe("audibot/twist",1,recvVehState); // Grab vehicle status

  ros::Subscriber path_following = nh.subscribe("audibot/cmd_vel", 1, recvPathFollowing);

  speed_pub = nh.advertise<std_msgs::Float64>("audibot/throttle_cmd", 1);
  steering_pub = nh.advertise<std_msgs::Float64>("audibot/steering_cmd", 1);

  // Vehicle parameter publisher
  controller_pub = nh.advertise<geometry_msgs::Twist>("/audibot/cmd_vel", 1); // Publishing audibot params

  // Getting reference lat and lon
  nh.getParam("/audibot/gps/ref_lat",ref_lat);
  nh.getParam("/audibot/gps/ref_lon",ref_lon);

  ref_point = UTMCoords(LatLon(ref_lat, ref_lon, 0.0));
  
  // Getting UTM of the reference (starting) coordinates
  LatLon ref_coords_lat_lon(ref_lat, ref_lon, 0);
  ref_coords = UTMCoords(ref_coords_lat_lon);

  // Finding central meridian, since robot won't be moving long distances, central meridian won't change
  central_meridian = ref_coords.getCentralMeridian();

  ros::spin();
}