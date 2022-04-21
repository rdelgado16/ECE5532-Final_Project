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
#include <bits/stdc++.h>

UTMCoords ref_point;

UTMCoords ref_coords;
tf::Vector3 ref_relative_position;
nav_msgs::Path gps_path;
ros::Publisher path_pub;
ros::Publisher markers_pub;
// ros::Publisher throttle_pub;
// ros::Publisher brake_pub;
// ros::Publisher steering_pub;
ros::Publisher controller_pub;
geometry_msgs::Twist audibot_params; // Twist message for the audibot parameters
UTMCoords UTM_local_intersections[18];
// ros::NodeHandle gh;
double set_speed = 20;

std::vector<int> int_actions;  // Legend: -2 = U-turn, -1 = Stop, 0 = Straight, 1 = Left, 2 = Right
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

double dist_traveled = 0; // Distance traveled by robot
  
// size of dist and action matrix
const int S = 18;
// adjacency distance matrix
const int dist_matrix[S][S] = {
  {0, 65, 775, 567, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 375, 0, 0, 65, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 167, 196, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 115, 0, 0, 0, 115, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 65, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 115, 0, 0, 0, 538, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 65, 0, 65, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 88, 65, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

// Destination intersection number
int dest = 0;

// Matrix for actions to perform at an intersection
const int actions_matrix[S][S] = {
  {0, 2, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

// List of UTM converted waypoint coordinates
UTMCoords waypoint_UTM[8];

// A utility function to find the vertex with minimum distance value, from the set of vertices not yet included in shortest path tree
int minDistance(int dist[], bool sptSet[])
{
    // Initialize min value
    int min = INT_MAX, min_index;
    for (int i = 0; i < S; i++)
        if (sptSet[i] == false && dist[i] <= min)
            min = dist[i], min_index = i;
    return min_index;
}

// Function to print shortest path from source to j using parent array
void printPath(int parent[], int j)
{
    // Base Case : If j is source
    if (parent[j] == -1)
        return;
    printPath(parent, parent[j]);
    // printf("%d ", j);
    int_order.insert(int_order.end(), j); // Appending intersection number to int_order
}

void dijkstra(const int graph[S][S], int src, int dest)
{
    // The output array. dist[i] will hold the shortest distance from src to i
    int dist[S];
 
    // sptSet[i] will true if vertex i is included / in shortest path tree or shortest distance from src to i is finalized
    bool sptSet[S] = { false };
 
    // Parent array to store shortest path tree
    int parent[S] = {-1};
 
    // Initialize all distances as INFINITE
    for (int i = 0; i < S; i++)
        dist[i] = INT_MAX;
 
    // Distance of source vertex from itself is always 0
    dist[src] = 0;
 
    // Find shortest path for all vertices
    for (int count = 0; count < S - 1; count++) {
        // Pick the minimum distance vertex from the set of vertices not yet processed. u is always equal to src in first iteration.
        int u = minDistance(dist, sptSet);
        // Mark the picked vertex as processed
        sptSet[u] = true;
        // Update dist value of the adjacent vertices of the picked vertex.
        for (int v = 0; v < S; v++)
            // Update dist[v] only if is not in sptSet, there is an edge from u to v, and total weight of path from src to v through u is smaller than current value of dist[v]
            if (!sptSet[v] && graph[u][v]
                && dist[u] + graph[u][v] < dist[v]) {
                parent[v] = u;
                dist[v] = dist[u] + graph[u][v];
            }
    } 
    // print the constructed distance array
    printPath(parent, dest);
   
}

void initGraph(int dest){
  if(dest>17 || dest<0){dest=0;} // If the dest value is not within 0 and 17, set it to default of 0

  int_order.insert(int_order.begin(), 0); // Adding starting intersection to int_order list
  dijkstra(dist_matrix, 0, dest); // Calling dijkstra's algorithm to find optimal path to destination

  printf("\nDestination: %d", dest);

  printf("\nIntersection Numbers: ");
  for(int i = 0; i < int_order.size(); i++){
    printf("%d ", int_order[i]);
  }
  int_actions.resize(int_order.size());
  printf("\nIntersection Actions: ");

  // Creating list of actions to perform at each intersection based on actions matrix
  for(int j = 0; j < int_order.size(); j++){
    if(j < int_order.size()-1){
      int_actions[j] = actions_matrix[int_order[j]][int_order[j+1]];
    }
    else{
      int_actions[j] = -1; // At the final intersection, stop the vehicle
    }
    printf("%d ", int_actions[j]);
  }
  printf("\n");
}

void initIntersections(){
  // List of GPS waypoint coordinates
  const double intersections[18][2] = {
  {42.0024491386,	-83.0018985140},
  {42.0024367640,	-83.0026815700},
  {41.9997857060,	-82.9956885461},
  {42.0052849918,	-82.9982609600},
  {42.0025810159,	-82.9944807940},
  {42.0016393444,	-82.9931539150},
  {42.0011509630,	-82.9951296391},
  {42.0003776418,	-82.9957040005},
  {42.0014086810,	-82.9957412321},
  {42.0013999875,	-82.9965243760},
  {42.0013712950,	-82.9979085647},
  {42.0003403860,	-82.9978830675},
  {42.0003525220,	-82.9970950120},
  {42.0009389400,	-82.9971147990},
  {42.0032462520,	-82.9976927114},
  {42.0027408455,	-82.9969826401},
  {42.0026591745,	-82.9976741668},
  {42.0026749210,	-83.0005488420}
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

void recvFix(const sensor_msgs::NavSatFixConstPtr& msg){
  UTMCoords current_coords(*msg);
  double veh_lat = msg->latitude;
  double veh_lon = msg->longitude;
  double veh_x = current_coords.getX();
  double veh_y = current_coords.getY();
  double time_delta = 0.02;
  // int traveled_int = 1; // If the intersection has already been traversed, don't count it again
  
  tf::Vector3 waypoint_veh_pos;

  // Calculate convergence angle for ENU heading calc and convert to rads
  conv_angle = (atan(tan(veh_lon-central_meridian)*sin(veh_lat)))* (M_PI/180);

  // for(int i = 0; i < 18; i++){

  // double waypoint_x = UTM_local_intersections[i].getX();
  // double waypoint_y = UTM_local_intersections[i].getY();
  // // Calculate heading offset between vehicle and waypoint
  // waypoint_theta = atan2((waypoint_y - veh_y), (waypoint_x - veh_x)) - conv_angle; // Subtract convergence angle
  // if(waypoint_theta <= 0){
  //   waypoint_theta += 2*M_PI; // Keeps the waypoint angle between 0 and 2pi
  // }
  // ROS_INFO("INTERSECTION: {%d}", int_order[int_count]);
  // Vector between vehicle and waypoints
  waypoint_veh_pos = current_coords - UTM_local_intersections[int_order[int_count]];

  dist_traveled = dist_traveled + (veh_spd*time_delta);
  ROS_INFO("Veh Speed, Distance Traveled: (%f, %f)", veh_spd, dist_traveled);

  // Finding direct distance between vehicle and waypoint
  dist = sqrt(pow(waypoint_veh_pos.getX(),2) + pow(waypoint_veh_pos.getY(),2));
  double time_to_int = dist/veh_spd;
  ROS_INFO("Vehicle speed: {%f}", veh_spd);
  // if(dist<100){
  // ROS_INFO("Veh Pos (%f, %f)", veh_x, veh_y);
  ROS_INFO("Distance to intersection %d: (%f)", int_order[int_count], dist);    
  ROS_INFO("Time to Intersection %f", time_to_int);
  ROS_INFO("FLAG: {%d}", int_flag);
  ROS_INFO("Current Heading: %f, Heading Error: %f, Desired Heading: %f, Heading Count: %d", heading_ROS, heading_error, desired_heading, heading_count);
  // }

  if (time_to_int <= 3 && int_flag == false){
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
    ROS_INFO("Desired Heading has been calculated and is %f", desired_heading);
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
          ROS_INFO("SLOWING DOWN FOR TURN OR STOP");
          // set_speed = 10;
          audibot_params.linear.x = 5;
          audibot_params.angular.z = 0;
          controller_pub.publish(audibot_params);
        }
        else{
          if (abs(heading_error) < 0.5 && heading_error != 0){
            ROS_INFO("WE MADE IT TO INTERSECTION");
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
      else if (int_actions[int_count] == 2){
        if(dist > 12){
          ROS_INFO("SLOWING DOWN FOR TURN OR STOP");
          // set_speed = 10;
          audibot_params.linear.x = 5;
          audibot_params.angular.z = 0;
          controller_pub.publish(audibot_params);
        }
        else{
          if (abs(heading_error) < 0.5 && heading_error != 0){
            ROS_INFO("WE MADE IT TO INTERSECTION");
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
        if (dist<8){
          ROS_INFO("WE MADE IT TO INTERSECTION");
          int_flag = false;
          if(int_count<int_actions.size()){
            int_count++;
          }
        }
      }
      // Stopping at intersection
      else if (int_actions[int_count] == -1){
        if(dist<30){
          audibot_params.linear.x = 0;
          audibot_params.angular.z = 0;
          controller_pub.publish(audibot_params);        
        }
        else if(dist<8){
          ROS_INFO("WE FREAKING MADE IT!!!!");
        }
      }
      
  }else{
    ROS_INFO("NEXT INTERSECTION {%d}", int_order[int_count]);
    ROS_INFO("Setting heading count back to 0");
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
  // ROS_INFO("Current Heading %f", heading_ROS);


  // Calculate heading error'

  // heading_error = waypoint_theta - heading_ROS;

  // Find time to the waypoint, used when determining when to brake, based on speed and distance from waypoint

  // // Addressing issues at discontinuities
  // if(heading_error > M_PI){
  //   heading_error = -(2*M_PI - heading_error);
  // }
  // else if(heading_error < -M_PI){
  //   heading_error = 2*M_PI + heading_error;
  // }
}
//   // Setting thottle, brake, and steering parameters
//   if(dist <= 1 && waypoint_index <= 8){
//     waypoint_index += 1; // Move onto the next waypoint once one is reached until the final one
//   }
//   else if(time_to_waypoint>1 && veh_spd <= 40 && abs(heading_error) < 0.5){ // Speed up the vehicle if far from waypoint and error is small
//     throttle_pos.data = 100;
//     brake_force.data = 0;
//   }
//   else if(waypoint_index > 8){ // Once you reach the end, stop
//     throttle_pos.data = 0;
//     brake_force.data = 7500; 
//   }
//   else if(time_to_waypoint<=1 && veh_spd>20){ // Slow down for waypoint
//     throttle_pos.data = 0;
//     brake_force.data = 7500;
//   }
//   else{ // Don't brake or accelerate the car while its turning, tends to spin out
//     throttle_pos.data = 0;
//     brake_force.data = 0;
//   }

//   // Using proportional gain to set steering angle
//   steering_angle.data = 5*heading_error;

//   // All the damn prints...
//   // ROS_INFO("Waypoint index: (%d)", waypoint_index);
//   // ROS_INFO("Waypoint theta: (%f)", waypoint_theta);
//   // ROS_INFO("Heading ROS: (%f)", heading_ROS);
//   // ROS_INFO("Heading Error: (%f)", heading_error);
//   // ROS_INFO("Waypoint distance: (%f)", dist);
//   // ROS_INFO("Speed/Yaw: (%f,%f)", veh_spd, veh_yaw);
//   // ROS_INFO("Throttle/Brake/Steering: (%f,%f,%f)", throttle_pos.data, brake_force.data, steering_angle.data);
//   // ROS_INFO("Vehicle UTM: (%f, %f)", current_coords.getX(), current_coords.getY());
//   // ROS_INFO("Waypoint UTM: (%f, %f)", waypoint_UTM[waypoint_index].getX(), waypoint_UTM[waypoint_index].getY());
//   // ROS_INFO("Relative Position: (%f, %f)", relative_position.x(), relative_position.y());

//   throttle_pub.publish(throttle_pos);
//   brake_pub.publish(brake_force);
//   steering_pub.publish(steering_angle);
// }

int main(int argc, char** argv){
  ros::init(argc,argv,"final_project");
  ros::NodeHandle nh;
  ros::param::param <int> ("~dest", dest, 0);
  // dest = argc;
  initGraph(dest);
  // initGraph();
  initIntersections();
  ros::Subscriber gps_sub = nh.subscribe("/audibot/gps/fix",1,recvFix); // Grab current gps position
  ros::Subscriber heading_sub = nh.subscribe("/audibot/gps/heading",1,recvHeading); // Grab current heading
  ros::Subscriber veh_spd_yaw_sub = nh.subscribe("audibot/twist",1,recvVehState); // Grab vehicle status


  // Vehicle parameter publisher
  // throttle_pub = nh.advertise<std_msgs::Float64>("/audibot/throttle_cmd", 1); // Publishing throttle param
  // brake_pub = nh.advertise<std_msgs::Float64>("/audibot/brake_cmd", 1); // Publishing brake param
  // steering_pub = nh.advertise<std_msgs::Float64>("/audibot/steering_cmd", 1); // Publishing throttle param
  controller_pub = nh.advertise<geometry_msgs::Twist>("/audibot/cmd_vel", 1); // Publishing audibot params

  // Getting reference lat and lon
  nh.getParam("/audibot/gps/ref_lat",ref_lat);
  nh.getParam("/audibot/gps/ref_lon",ref_lon);
  
  // nh.setParam("/audibot/path_following/speed", set_speed);
  
  ref_point = UTMCoords(LatLon(ref_lat, ref_lon, 0.0));
  
  // Getting UTM of the reference (starting) coordinates
  LatLon ref_coords_lat_lon(ref_lat, ref_lon, 0);
  ref_coords = UTMCoords(ref_coords_lat_lon);

  // Finding central meridian, since robot won't be moving long distances, central meridian won't change
  central_meridian = ref_coords.getCentralMeridian();
  
  ros::spin();
}