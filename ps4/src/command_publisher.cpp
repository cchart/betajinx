#include<ros/ros.h>
#include<cwru_base/cRIOSensors.h>
#include<std_msgs/Bool.h>
#include<math.h>
#include <cwru_base/Pose.h>
#include<geometry_msgs/Twist.h> //data type for velocities

#define HALF_PI 1.67079633
#define CW -1
#define nap 50
#define v_max 1.0
#define a_max 0.25
#define omega_max 1.0
#define alpha_max 0.5

bool estop;
bool stopped;
double dt = 0.02;

using namespace std;

void poseCallback(const cwru_base::Pose::ConstPtr& pose) {
	cout << pose->x << " | " << pose->y << endl;
}

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

void runLinear(double segLength, ros::Publisher pub){

	geometry_msgs::Twist vel_object;
	ros::Rate naptime(nap);
	double segDistDone = 0;
	double v_cmd = 0;
	double v_past=0;
	double t=0;
	double T_accel = v_max/a_max;
	double dist_accel= 0.5*a_max*T_accel*T_accel;
	double dist_deccel = dist_accel;
	double dist_const_v = segLength - dist_accel -dist_deccel;
	double v_scheduled, v_test,temp;
	if(dist_const_v<0){
		dist_accel = segLength/2;
		dist_deccel = dist_accel;
	}

	while (ros::ok() && segDistDone<segLength) // do work here
	{
		ros::spinOnce();
		while(estop == false){
			ros::spinOnce();
			naptime.sleep();
			stopped=true;
			ROS_INFO("crazy dance");
		}
		if(stopped){
			holdingPattern(1,pub);
			stopped=false;
			t=0;
			segLength = segLength - segDistDone;
			segDistDone=0;
			v_cmd = 0;
			v_past=0;
			T_accel = v_max/a_max;
			dist_accel= 0.5*a_max*T_accel*T_accel;
			dist_deccel = dist_accel;
			dist_const_v = segLength - dist_accel -dist_deccel;
			v_scheduled=0;
			v_test=0;
			temp=0;
		}

		t=t+dt;
		segDistDone += ((v_past+v_cmd)/2)*dt;
		v_past = v_cmd;

		if (segDistDone<dist_accel)
		{
			v_scheduled = sqrt(2*segDistDone*a_max);
			if (v_scheduled < a_max*dt){
				v_scheduled = a_max*dt;
			}
		}
		else if (segDistDone<dist_accel+dist_const_v){
			v_scheduled = v_max;
		}
		else{
			temp = 2*(segLength-segDistDone)*a_max;
			if(temp<0){
				v_scheduled = 0.0;
			}
			else{
				v_scheduled = sqrt(temp);
			}
		}

		if (v_cmd < v_scheduled){
			v_test=v_cmd+a_max*dt;
			v_cmd = min(v_test,v_scheduled);
		}
		else if (v_cmd > v_scheduled){
			v_test = v_cmd-1.2*a_max*dt;
			v_cmd = max(v_test,v_scheduled);
		}
		else{
			v_cmd=v_scheduled;
		}
		if(v_cmd>v_max){
			v_cmd = v_max;
		}

		vel_object.linear.x = v_cmd;
		vel_object.angular.z = 0.0;
		ROS_INFO("linear %f %f %f %f",vel_object.linear.x,vel_object.linear.y,vel_object.linear.z,segLength);
		pub.publish(vel_object);  // this action causes the commands in vel_object to be published 
		
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (50ms) period
		//thus enforcing that we achieve the desired update rate (20Hz)
	}
	return;
}

void runInPlaceTurn(double segLength, double direction, ros::Publisher pub){

	geometry_msgs::Twist vel_object;
	ros::Rate naptime(nap);
	double segDistDone = 0;
	double omega_cmd = 0;
	double omega_past = 0;
	double t=0;
	double T_accel = omega_max/alpha_max;
	double dist_accel= 0.5*alpha_max*T_accel*T_accel;
	double dist_deccel = dist_accel;
	double dist_const_v = segLength - dist_accel -dist_deccel;
	double omega_scheduled, omega_test,temp;
	if(dist_const_v<0){
		dist_accel = segLength/2;
		dist_deccel = dist_accel;
	}

	while (ros::ok() && segDistDone<segLength) // do work here
	{
		ros::spinOnce();
		while(estop == false){
			ros::spinOnce();
			naptime.sleep();
			stopped=true;
		}
		if(stopped){
			holdingPattern(1,pub);
			stopped=false;
			t=0;
			segLength = segLength - segDistDone +0.15;
			segDistDone=0;
			omega_cmd = 0;
			omega_past=0;
			T_accel = omega_max/alpha_max;
			dist_accel= 0.5*alpha_max*T_accel*T_accel;
			dist_deccel = dist_accel;
			dist_const_v = segLength - dist_accel -dist_deccel;
			omega_scheduled=0;
			omega_test=0;
			temp=0;
		}
		
		t=t+dt;
		segDistDone += ((omega_past+omega_cmd)/2)*dt;
		omega_past = omega_cmd;

		if (segDistDone<dist_accel)
		{
			omega_scheduled = sqrt(2*segDistDone*alpha_max);
			if (omega_scheduled < alpha_max*dt){
				omega_scheduled = alpha_max*dt;
			}
		}
		else if (segDistDone<dist_accel+dist_const_v){
			omega_scheduled = omega_max;
		}
		else{
			temp = 2*(segLength-segDistDone)*alpha_max;
			if(temp<0){
				omega_scheduled = 0.0;
			}
			else{
				omega_scheduled = sqrt(temp);
			}
		}

		if (omega_cmd < omega_scheduled){
			omega_test=omega_cmd+alpha_max*dt;
			omega_cmd = min(omega_test,omega_scheduled);
		}
		else if (omega_cmd > omega_scheduled){
			omega_test = omega_cmd-1.2*alpha_max*dt;
			omega_cmd = max(omega_test,omega_scheduled);
		}
		else{
			omega_cmd=omega_scheduled;
		}
		if(omega_cmd>omega_max){
			omega_cmd = omega_max;
		}
		
		
		vel_object.linear.x = 0.0;
		vel_object.angular.z = omega_cmd*direction;
		ROS_INFO("angular %f %f %f",vel_object.angular.x,vel_object.angular.y,vel_object.angular.z);
		pub.publish(vel_object);  // this action causes the commands in vel_object to be published 
		
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (50ms) period
		//thus enforcing that we achieve the desired update rate (20Hz)
	}
	return;
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"command_publisher");//name of this node
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Subscriber sub = n.subscribe("motors_enabled",1,estopCallback);
	ros::Subscriber sub1 = n.subscribe("flipped_pose",1,poseCallback);

	//"cmd_vel" is the topic name to publish velocity commands
	//"1" is the buffer size (could use buffer>1 in case network bogs down)

	while (!ros::Time::isValid()) {} // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid
	ros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
	ROS_INFO("birthday started as %f", birthday.toSec());

	holdingPattern(0.5,pub);

	runLinear(3,pub);
	holdingPattern(0.2,pub);

	runInPlaceTurn(HALF_PI,CW,pub);
	holdingPattern(0.2,pub);

	runLinear(12.5,pub);
	holdingPattern(0.2,pub);

	runInPlaceTurn(HALF_PI,CW,pub);
	holdingPattern(0.2,pub);

	runLinear(4,pub);
	holdingPattern(0.05,pub);
	
	return 0;
}
