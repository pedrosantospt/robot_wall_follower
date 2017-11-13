/*
* robo_control.cpp
*
* Name  : Pedro Santos e Teresa Chaves
*/
//Common ROS headers.
#include "ros/ros.h"
//This is needed for the data structure containing the motor command.
#include "geometry_msgs/Twist.h"
//This is needed for the data structure containing the laser scan.
#include "sensor_msgs/LaserScan.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>

#define LEFT_SIDE 1
#define RIGHT_SIDE -1

#define COEF_P 45        //P -> PD controller coeff
#define COEF_I 0         //I -> PD controller coeff
#define COEF_D 1       //D -> PD controller coeff
#define PID_MAX 1
#define PID_MIN -1
#define PID_DT 0.001
#define WALL_DIST 2.1  //Wall distance
#define MAX_VEL 0.7     //Maximum speed (not change this, otherwise needs to update PD co)
#define PI 3.14         //PI value
#define DIR LEFT_SIDE //choose between RIGHT_SIDE or LEFT_SIDE

double e_t1 = 0; //Last error (for PD controller)


ros::Publisher motor_command_publisher;
ros::Subscriber laser_subscriber;
ros::Subscriber sub_odom;
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist motor_command;

double odo_current_angle = 0;
double odo_past_angle = 0;

int state = 0;
double error = 0;

double v_integral = 0;
double v_pre_error = 0;

double normalize(double value, double min, double max){
  return (double) ((value - min)/(max - min));
}

double PID_controller(double value, double set_point){
  // Calculate error
  double error = set_point - value;
  // Proportional term
  double Pout = COEF_P * error;
  // Integral term
  v_integral += error * PID_DT;
  double Iout = COEF_I * v_integral;
  // Derivative term
  double derivative = (error - v_pre_error) / PID_DT;
  double Dout = COEF_D * derivative;
  // Calculate total output
  double output = Pout + Iout + Dout;
  // Restrict to max/min
  if( output > PID_MAX )
      output = PID_MAX;
  else if( output < PID_MIN )
      output = PID_MIN;
  // Save error to previous error
  v_pre_error = error;

  return output;
}

void laser_callback ( const sensor_msgs::LaserScan::ConstPtr &scan_msg ) {

    laser_msg = *scan_msg;
    std::vector<float> laser_ranges;

    laser_ranges = laser_msg.ranges;
    double is_nan = 0;

    int size_t1 = laser_ranges.size();

    //Min and maximum ranges
    float range_min_t1 = laser_msg.range_max;
    float range_max_t1 = laser_msg.range_min;

    //Limit the laser range by direction (ex. if DIR= LEFT_SIDE robot should
    //follow the wall by his left side )
    int minIndex_t1 = size_t1*(DIR+1)/4;
    int maxIndex_t1 = size_t1*(DIR+3)/4;

    ///////////////////////////
    //find minimum value
    for(int i_t1 = 0; i_t1 < size_t1; i_t1++){
      if(std::isnan(laser_ranges[i_t1])){
        laser_ranges[i_t1] = 11;
        is_nan++;
      }
        if (laser_ranges[i_t1] < range_min_t1){
            range_min_t1 = laser_ranges[i_t1];
            minIndex_t1 = i_t1;
        }
    }

    int middle_Index = laser_ranges.size()/2;
    double range_in_the_middle = laser_ranges[(int)middle_Index];

    //FIND WALL AND get close to the wall
    if(state == 0){
      if(is_nan/laser_ranges.size() == 1){
        motor_command.angular.z = rand()%2 - 0.5;
        motor_command.linear.x = rand()%2 - 0.5;
      } else if(minIndex_t1 < (middle_Index)){
        motor_command.angular.z = - normalize(minIndex_t1, middle_Index, 0) * MAX_VEL;
        motor_command.linear.x = normalize(range_in_the_middle, 1.6, 4) * MAX_VEL;
      } else if (minIndex_t1 > (middle_Index)){
        motor_command.angular.z = - normalize(minIndex_t1,laser_ranges.size(), middle_Index) * MAX_VEL;
        motor_command.linear.x = normalize(range_in_the_middle, 1.6, 4) * MAX_VEL;
      } else {
        motor_command.linear.x = normalize(range_in_the_middle, 1.6, 4) * MAX_VEL;
        if (fabs(motor_command.linear.x)<0.1){
          state++;
        }
      }
    } else
    // Follow wall with PID control
    if(state == 1) {

      // Calculate error
      double error = WALL_DIST - laser_ranges[0];

      //Control the angular velocity with a PD controller
      motor_command.angular.z = PID_controller(laser_ranges[0], WALL_DIST);

      //if error too high will decrease the velocity
      motor_command.linear.x = normalize(fabs(error), 1.5, 0) * MAX_VEL > 0 ? normalize(fabs(error), 1.5, 0) * MAX_VEL : 0;

      //too close to the wall
      if(laser_ranges[middle_Index] < 1.2){
        motor_command.linear.x = - normalize(fabs(error), 1.2, 0) * 0.1;
      }

      //almost dont see walls... maybe a 90 degree curve?
      if(is_nan/laser_ranges.size() > 0.85){ //HACK to the curves....
        motor_command.linear.x +=0.8 * (is_nan/laser_ranges.size());
      }

    }

    if(std::isnan(motor_command.linear.x)){
      motor_command.linear.x = 0;
    }
    if(std::isnan(motor_command.angular.z)){
      motor_command.angular.z = 0;
    }

    motor_command_publisher.publish ( motor_command );

/*
    //Laser scan is an array (vector) of distances.
    std::cout<<"No of points in laser scan: "<<laser_ranges.size()<<std::endl;
    std::cout<<"Distance to the rightmost scanned point is "<<laser_ranges[0]<<std::endl;
    std::cout<<"Distance to the leftmost scanned point is "<<laser_ranges[laser_ranges.size()-1]<<std::endl;
    std::cout<<"Distance to the middle scanned point is "<<laser_ranges[middle_Index]<<std::endl;

    //Basic trignometry with the above scan array and the following information to find out exactly where the laser scan found something:
    std::cout<<"The minimum angle scanned by the laser is "<<laser_msg.angle_min<<std::endl;
    std::cout<<"The maximum angle scanned by the laser is "<<laser_msg.angle_max<<std::endl;
    std::cout<<"The increment in angles scanned by the laser is "<<laser_msg.angle_increment<<std::endl; //angle_min+angle_increment*laser_ranges.size() = angle_max
    std::cout<<"The minimum range (distance) the laser can perceive is "<<laser_msg.range_min<<std::endl;
    std::cout<<"The maximum range (distance) the laser can perceive is "<<laser_msg.range_max<<std::endl;
*/
}


int main ( int argc, char **argv ) {
    // must always do this when starting a ROS node - and it should be the first thing to happen
    ros::init ( argc, argv, "amble" );
    // the NodeHandle object is our access point to ROS
    ros::NodeHandle n;
    //"Twist" messages to the topic /cmd_vel_mux/navi
    motor_command_publisher = n.advertise<geometry_msgs::Twist> ( "/cmd_vel_mux/input/navi", 100 );
    //New laser messages when they arrive
    laser_subscriber = n.subscribe ( "/scan", 1000, laser_callback );
    //now enter an infinite loop - the callback functions above will be called when new laser or map messages arrive.
    ros::Duration time_between_ros_wakeups ( 0.001 );
    std::cout<<"Running..."<<laser_msg.range_max<<std::endl;
    while ( ros::ok() ) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }
    return 0;
}
