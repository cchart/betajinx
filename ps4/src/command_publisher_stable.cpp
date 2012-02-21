#include<ros/ros.h>
#include <math.h>
#include<geometry_msgs/Twist.h> //data type for velocities

#define HALF_PI 1.57079633
#define CW -1

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

int main(int argc,char **argv)
{
	ros::init(argc,argv,"command_publisher");//name of this node
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	//"cmd_vel" is the topic name to publish velocity commands
	//"1" is the buffer size (could use buffer>1 in case network bogs down)
	
	geometry_msgs::Twist vel_object;
	ros::Duration elapsed_time; // define a variable to hold elapsed time
	ros::Rate naptime(20); //will perform sleeps to enforce loop rate of "10" Hz

	double dt = 0.1; //for 10Hz update; (can go faster on Jinx)
	double v_max = 1.0; //1m/sec is a slow walk
	double a_max = 2.0; //1m/sec^2 is 0.1 g's
	double omega_max = 1.0; //1 rad/sec-> about 6 seconds to rotate 1 full rev
	double alpha_max = 0.5; //0.5 rad/sec^2-> takes 2 sec to get from rest to full omega
	double delta_v = alpha_max*dt; //this is incremental vel update each cycle during accel
	double segLength;
	double segDistDone = 0;
	double v_cmd,v_test,v_scheduled,v_past, temp;
	double omega_cmd=0,omega_past;
	double t = 0;
	double T_accel,T_deccel,T_const_v,T_segTot;
	double dist_accel, dist_deccel,dist_const_v;

	

	while (!ros::Time::isValid()) {} // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid
	ros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
	ROS_INFO("birthday started as %f", birthday.toSec());

	//Now Set Per Segment
	segLength = 3;
	segDistDone = 0;
	v_cmd = 0;
	v_past=0;
	omega_cmd = 0;
	t=0;
	T_accel = v_max/a_max;
	T_deccel = T_accel;
	dist_accel= 0.5*a_max*T_accel*T_accel;
	dist_deccel = dist_accel;
	dist_const_v = segLength - dist_accel -dist_deccel;
	T_const_v = dist_const_v/v_max;
	T_segTot = T_accel + T_deccel + T_const_v;

	while (ros::ok() && segDistDone<segLength) // do work here
	{
		t=t+dt;
		elapsed_time= ros::Time::now()-birthday;
		ROS_INFO("elapsed time is %f", elapsed_time.toSec());
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
		ROS_INFO("linear %f %f %f",vel_object.linear.x,vel_object.linear.y,vel_object.linear.z);
		pub.publish(vel_object);  // this action causes the commands in vel_object to be published 
		
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}

	vel_object.linear.x = 0.0;
	vel_object.angular.z = 0.0;
	pub.publish(vel_object);

	//Now Set Per Segment
	segLength = HALF_PI;
	segDistDone = 0;
	v_cmd = 0;
	v_past=0;
	omega_cmd = 0;
	omega_past = 0;
	t=0;
	T_accel = omega_max/alpha_max;
	T_deccel = T_accel;
	dist_accel= 0.5*alpha_max*T_accel*T_accel;
	dist_deccel = dist_accel;
	dist_const_v = segLength - dist_accel -dist_deccel;
	T_const_v = dist_const_v/omega_max;
	T_segTot = T_accel + T_deccel + T_const_v;

	if(dist_const_v<0){
		dist_accel = segLength/2;
		dist_deccel = dist_accel;
	}

	while (ros::ok() && segDistDone<segLength) // do work here
	{
		t=t+dt;
		elapsed_time= ros::Time::now()-birthday;
		ROS_INFO("elapsed time is %f", elapsed_time.toSec());
		segDistDone += ((omega_past+omega_cmd)/2)*dt;
		omega_past = omega_cmd;

		if (segDistDone<dist_accel)
		{
			v_scheduled = sqrt(2*segDistDone*alpha_max);
			if (v_scheduled < alpha_max*dt){
				v_scheduled = alpha_max*dt;
			}
		}
		else if (segDistDone<dist_accel+dist_const_v){
			v_scheduled = omega_max;
		}
		else{
			temp = 2*(segLength-segDistDone)*alpha_max;
			if(temp<0){
				v_scheduled = 0.0;
			}
			else{
				v_scheduled = sqrt(temp);
			}
			ROS_INFO("derp %f %f",v_scheduled, temp);
		}

		if (omega_cmd < v_scheduled){
			v_test=omega_cmd+alpha_max*dt;
			omega_cmd = min(v_test,v_scheduled);
		}
		else if (omega_cmd > v_scheduled){
			v_test = omega_cmd-1.2*alpha_max*dt;
			omega_cmd = max(v_test,v_scheduled);
		}
		else{
			omega_cmd=v_scheduled;
		}
		if(v_cmd>v_max){
			omega_cmd = omega_max;
		}
		
		
		vel_object.linear.x = 0.0;
		vel_object.angular.z = omega_cmd*CW;	//CW for clockwise direction
		ROS_INFO("angular %f %f %f",vel_object.angular.x,vel_object.angular.y,vel_object.angular.z);
		pub.publish(vel_object);  // this action causes the commands in vel_object to be published 
		
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}

	vel_object.linear.x = 0.0;
	vel_object.angular.z = 0.0;
	pub.publish(vel_object);

	//Now Set Per Segment
	segLength = 13;
	segDistDone = 0;
	v_cmd = 0;
	v_past=0;
	omega_cmd = 0;
	t=0;
	T_accel = v_max/a_max;
	T_deccel = T_accel;
	dist_accel= 0.5*a_max*T_accel*T_accel;
	dist_deccel = dist_accel;
	dist_const_v = segLength - dist_accel -dist_deccel;
	T_const_v = dist_const_v/v_max;
	T_segTot = T_accel + T_deccel + T_const_v;

	while (ros::ok() && segDistDone<segLength) // do work here
	{
		t=t+dt;
		elapsed_time= ros::Time::now()-birthday;
		ROS_INFO("elapsed time is %f", elapsed_time.toSec());
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
		ROS_INFO("linear %f %f %f",vel_object.linear.x,vel_object.linear.y,vel_object.linear.z);
		pub.publish(vel_object);  // this action causes the commands in vel_object to be published 
		
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}

	vel_object.linear.x = 0.0;
	vel_object.angular.z = 0.0;
	pub.publish(vel_object);

	//Now Set Per Segment
	segLength = HALF_PI;
	segDistDone = 0;
	v_cmd = 0;
	v_past=0;
	omega_cmd = 0;
	omega_past = 0;
	t=0;
	T_accel = omega_max/alpha_max;
	T_deccel = T_accel;
	dist_accel= 0.5*alpha_max*T_accel*T_accel;
	dist_deccel = dist_accel;
	dist_const_v = segLength - dist_accel -dist_deccel;
	T_const_v = dist_const_v/omega_max;
	T_segTot = T_accel + T_deccel + T_const_v;
	if(dist_const_v<0){
		dist_accel = segLength/2;
		dist_deccel = dist_accel;
	}

	while (ros::ok() && segDistDone<segLength) // do work here
	{
		t=t+dt;
		elapsed_time= ros::Time::now()-birthday;
		ROS_INFO("elapsed time is %f", elapsed_time.toSec());
		segDistDone += ((omega_past+omega_cmd)/2)*dt;
		omega_past = omega_cmd;

		if (segDistDone<dist_accel)
		{
			v_scheduled = sqrt(2*segDistDone*alpha_max);
			if (v_scheduled < alpha_max*dt){
				v_scheduled = alpha_max*dt;
			}
		}
		else if (segDistDone<dist_accel+dist_const_v){
			v_scheduled = omega_max;
		}
		else{
			temp = 2*(segLength-segDistDone)*alpha_max;
			if(temp<0){
				v_scheduled = 0.0;
			}
			else{
				v_scheduled = sqrt(temp);
			}
		}

		if (omega_cmd < v_scheduled){
			v_test=omega_cmd+alpha_max*dt;
			omega_cmd = min(v_test,v_scheduled);
		}
		else if (omega_cmd > v_scheduled){
			v_test = omega_cmd-1.2*alpha_max*dt;
			omega_cmd = max(v_test,v_scheduled);
		}
		else{
			omega_cmd=v_scheduled;
		}
		if(v_cmd>v_max){
			omega_cmd = omega_max;
		}
		
		
		vel_object.linear.x = 0.0;
		vel_object.angular.z = omega_cmd*CW;	//CW for clockwise direction
		ROS_INFO("angular %f %f %f",vel_object.angular.x,vel_object.angular.y,vel_object.angular.z);
		pub.publish(vel_object);  // this action causes the commands in vel_object to be published 
		
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}

	vel_object.linear.x = 0.0;
	vel_object.angular.z = 0.0;
	pub.publish(vel_object);

	//Now Set Per Segment
	segLength = 4;
	segDistDone = 0;
	v_cmd = 0;
	v_past=0;
	omega_cmd = 0;
	t=0;
	T_accel = v_max/a_max;
	T_deccel = T_accel;
	dist_accel= 0.5*a_max*T_accel*T_accel;
	dist_deccel = dist_accel;
	dist_const_v = segLength - dist_accel -dist_deccel;
	T_const_v = dist_const_v/v_max;
	T_segTot = T_accel + T_deccel + T_const_v;

	while (ros::ok() && segDistDone<segLength) // do work here
	{
		t=t+dt;
		elapsed_time= ros::Time::now()-birthday;
		ROS_INFO("elapsed time is %f", elapsed_time.toSec());
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
		ROS_INFO("linear %f %f %f",vel_object.linear.x,vel_object.linear.y,vel_object.linear.z);
		pub.publish(vel_object);  // this action causes the commands in vel_object to be published 
		
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}

	vel_object.linear.x = 0.0;
	vel_object.angular.z = 0.0;
	pub.publish(vel_object);

	return 0; // this code will only get here if this node was told to shut down, which is
	// reflected in ros::ok() is false 
}
