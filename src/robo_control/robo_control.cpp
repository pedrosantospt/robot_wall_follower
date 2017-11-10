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

#include "nav_msgs/Odometry.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>

#define LEFT_SIDE 1
#define RIGHT_SIDE -1

#define COEF_P 5        //P -> PD controller coeff
#define COEF_D 2.5      //D -> PD controller coeff
#define WALL_DIST 0.73  //Wall distance
#define MAX_VEL 0.25     //Maximum speed (not change this, otherwise needs to update PD co)
#define PI 3.14         //PI value
#define DIR LEFT_SIDE //choose between RIGHT_SIDE or LEFT_SIDE

double e_t1 = 0; //Last error (for PD controller)

// For information on what publishing and subscribing is in ROS, look up the tutorials.
ros::Publisher motor_command_publisher;
ros::Subscriber laser_subscriber;
ros::Subscriber sub_odom;

// For information on what a "message" is in ROS, look up the tutorials.
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist motor_command;

double odo_current_angle = 0;
double odo_past_angle = 0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
/*
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
*/
  odo_current_angle = msg->pose.pose.orientation.z;
}

int state = 0;
double error = 0;
double c_Kp = 45;
double c_Ki = 9.7; //0.25;
double c_Kd = 1;
double v_max = 1;
double v_min = -1;
double v_integral = 0;
double v_pre_error = 0;


void laser_callback ( const sensor_msgs::LaserScan::ConstPtr &scan_msg ) {

    laser_msg = *scan_msg;
    std::vector<float> laser_ranges;

    laser_ranges = laser_msg.ranges;

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
      }
        if (laser_ranges[i_t1] < range_min_t1){
            range_min_t1 = laser_ranges[i_t1];
            minIndex_t1 = i_t1;
        }
    }

    std::cout<<">>>>> minIndex_t1: "<< minIndex_t1 <<std::endl;
    std::cout<<">>>>> range_min_t1: "<< range_min_t1 <<std::endl;

    std::cout<<">>>>> laser_ranges.size()/2: "<< laser_ranges.size()/2 <<std::endl;

    //FIND WALL
    if(state == 0){
      if(minIndex_t1 < (laser_ranges.size()/2) - 5 ){
        motor_command.angular.z = - (float) (minIndex_t1 - (laser_ranges.size()/2)) / ( - (laser_ranges.size()/2));
        motor_command.linear.x = (float) ((laser_ranges[(int)laser_ranges.size()/2] - 1)/(4 - 1));

      } else if (minIndex_t1 > (laser_ranges.size()/2) + 5){
        motor_command.angular.z = (float) (minIndex_t1 - (laser_ranges.size()/2)) / ( laser_ranges.size() - (laser_ranges.size()/2));// + 0.75;
        motor_command.linear.x = (float) ((laser_ranges[(int)laser_ranges.size()/2] - 1)/(4 - 1));

      } else {
        motor_command.linear.x = (float) ((laser_ranges[(int)laser_ranges.size()/2] - 1)/(4 - 1));
        if (fabs(motor_command.linear.x)<0.1){
          odo_past_angle = odo_current_angle;
          state++;
        }
      }
    } else
    // Follow wall with PID control
    if(state == 1) {

      // Calculate error
      double error = 1.2 - laser_ranges[0];
      // Proportional term
      double Pout = c_Kp * error;
      // Integral term
      v_integral += error * 0.001;
      double Iout = c_Ki * v_integral;
      // Derivative term
      double derivative = (error - v_pre_error) / 0.001;
      double Dout = c_Kd * derivative;
      // Calculate total output
      double output = Pout + Iout + Dout;
      // Restrict to max/min
      if( output > v_max )
          output = v_max;
      else if( output < v_min )
          output = v_min;

      // Save error to previous error
      v_pre_error = error;

      motor_command.linear.x = ((fabs(error) - 1.5) / (0 - 1.5 )) * 0.7 > 0 ? ((fabs(error) - 1.5) / (0 - 1.5 )) * 0.7 : 0;//0.6*MAX_VEL; //0.25 = Maxspeed;//0.4;

      if(laser_ranges[laser_ranges.size()/2] < 1.2){
          motor_command.linear.x = 0;
      }

      motor_command.angular.z = output;

    }

    if(std::isnan(motor_command.linear.x)){
      motor_command.linear.x = 0;
    }
    if(std::isnan(motor_command.angular.z)){
      motor_command.angular.z = 0;
    }





    ////////////////////////////
    /*
    //find minimum value
    for(int i_t1 = minIndex_t1; i_t1 < maxIndex_t1; i_t1++){
        if (laser_ranges[i_t1] < range_min_t1){
            range_min_t1 = laser_ranges[i_t1];
            minIndex_t1 = i_t1;
        }
    }
    //Calculation of angles from indexes and storing data to class variables.
    double angleMin_t1 = (minIndex_t1-size_t1/2)*laser_msg.angle_increment;
    double distMin_t1 = laser_ranges[minIndex_t1];
    double distFront_t1 = laser_ranges[size_t1/2];

    double diffE_t1 = (distMin_t1 - WALL_DIST) - e_t1;
    e_t1 = distMin_t1 - WALL_DIST;

    //PD CONTROLLER (a fast-made PD controller, not very scientific)
    motor_command.angular.z = DIR *(COEF_P * e_t1 + COEF_D * diffE_t1) + (angleMin_t1 - PI*DIR/2);

    //prevent PD controller bad-behavior
    if(std::isnan(motor_command.angular.z)){
      motor_command.angular.z = 0;
    }

    if (distFront_t1 < WALL_DIST * 2){
      //reduce velocity by multiple the normalize distance between WALL_DIST and WALL_DIST*2 by MAX_VEL (aka smooth stop)
      motor_command.linear.x  = ((distFront_t1 - WALL_DIST) / (WALL_DIST * 2 - WALL_DIST )) * MAX_VEL;//0.6*MAX_VEL; //0.25 = Maxspeed
    }
    else {
      motor_command.linear.x  = MAX_VEL;
      motor_command.angular.z = ( fabs(motor_command.angular.z) > PI/2 ? 0 : motor_command.angular.z ); // limit angular velocity
    }
    */

    motor_command_publisher.publish ( motor_command );


    //Laser scan is an array (vector) of distances.
    std::cout<<"No of points in laser scan: "<<laser_ranges.size()<<std::endl;
    std::cout<<"Distance to the rightmost scanned point is "<<laser_ranges[0]<<std::endl;
    std::cout<<"Distance to the leftmost scanned point is "<<laser_ranges[laser_ranges.size()-1]<<std::endl;
    std::cout<<"Distance to the middle scanned point is "<<laser_ranges[laser_ranges.size()/2]<<std::endl;

    //Basic trignometry with the above scan array and the following information to find out exactly where the laser scan found something:
    std::cout<<"The minimum angle scanned by the laser is "<<laser_msg.angle_min<<std::endl;
    std::cout<<"The maximum angle scanned by the laser is "<<laser_msg.angle_max<<std::endl;
    std::cout<<"The increment in angles scanned by the laser is "<<laser_msg.angle_increment<<std::endl; //angle_min+angle_increment*laser_ranges.size() = angle_max
    std::cout<<"The minimum range (distance) the laser can perceive is "<<laser_msg.range_min<<std::endl;
    std::cout<<"The maximum range (distance) the laser can perceive is "<<laser_msg.range_max<<std::endl;

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

    sub_odom = n.subscribe("odom", 1000, odomCallback);

    //now enter an infinite loop - the callback functions above will be called when new laser or map messages arrive.
    ros::Duration time_between_ros_wakeups ( 0.001 );
    while ( ros::ok() ) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }
    return 0;
}
