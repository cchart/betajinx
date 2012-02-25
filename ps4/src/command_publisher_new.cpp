#include<ros/ros.h>
#include<cwru_base/cRIOSensors.h>
#include<std_msgs/Bool.h>
#include<math.h>
#include <cwru_base/Pose.h>
#include<geometry_msgs/Twist.h> //data type for velocities
#include<geometry_msgs/PoseWithCovarianceStamped.h>

#define HALF_PI 1.67079633
#define CW -1
#define nap 2
#define v_max 1.0
#define a_max 0.25
#define omega_max 1.0
#define alpha_max 0.5

bool estop;
bool stopped;
double dt = 0.02;

using namespace std;

void estopCallback(const std_msgs::Bool::ConstPtr& est) 
{
	if (est->data == true){
		estop=true;  // means motors are ENABLED
		ROS_INFO("Not e-stopped");
	}
	else if (est->data == false){
		estop=false;  // means motors are DISABLED
		ROS_INFO("e-stopped");
	}
}

double min(double a, double b){
	if(a<b)
		return a;
	return b;
}

double max(double a, double b){
	if(a>b)
		return a;
	return b;
}

void holdingPattern(double time, ros::Publisher pub){
	double t=0;
	ros::Rate naptime(nap);
	geometry_msgs::Twist vel_object;
	while(ros::ok() && t<time){
		t=t+dt;
		vel_object.linear.x = 0.0;
		vel_object.angular.z = 0.0;
		ROS_INFO("standing %f %f", t, dt);
		pub.publish(vel_object);  // this action causes the commands in vel_object to be published 
		
		naptime.sleep();
	}
	return;
}
