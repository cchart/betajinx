#include<ros/ros.h>
#include<cwru_base/cRIOSensors.h>
#include<std_msgs/Bool.h>
#include<math.h>
#include <cwru_base/Pose.h>
#include <laser_listener/obstacle.h>
#include<geometry_msgs/Twist.h> //data type for velocities

#define HALF_PI 1.57079633
#define CW -1.0
#define nap 10
#define v_max 1.0
#define a_max 1
#define omega_max 1.0
#define alpha_max 0.5

bool estop;
bool stopped;
double dt = 0.1;
bool obstacle;
double segDistLeft;
/* Obstacle Avoidance Mode, E-Stop Mode */
void OAM(double segLength, double segDistDone, double v_cmd, double v_past, ros::Publisher pub);
void ESM(double segLength, double segDistDone, ros::Publisher pub);
using namespace std;

void poseCallback(const cwru_base::Pose::ConstPtr& pose) {
	cout << pose->x << " | " << pose->y << endl;
}

void obstructionsCallback(const laser_listener::obstacle::ConstPtr& obs) {
/*	if(obs->nearestObstacle < segDistLeft){
		obstacle = obs->obstacle;
	}
	else{obstacle=0;}
*/
	if (obs->obstacle == 1) { ROS_INFO("OBSTACLE CALLBACK: DETECTED "); obstacle = true; }
	else { obstacle = false;}
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


/* Obstacle detection = segType 4 */
geometry_msgs::Twist getVelocity(double time, double v_past, double v_cmd, double segDistDone, double segLength, double segType){
	geometry_msgs::Twist velocity_cmd;
	double T_accel, T_decel;
	double dist_accel;
	double dist_deccel;
	double dist_const_v;
	double v_scheduled, v_test,temp, accel_max, velocity_max;

	if(segType==1){
		accel_max = a_max;
		velocity_max = v_max;
		T_accel = v_max/a_max;
		//T_decel = v_max/a_max;
		dist_accel= 0.5*accel_max*T_accel*T_accel;
		dist_deccel = 0.5*accel_max*T_accel*T_accel;
		dist_const_v = segLength - dist_accel -dist_deccel;
	}
	else if(segType==2){
		accel_max = alpha_max;
		velocity_max = omega_max;
		T_accel = omega_max/alpha_max;
		dist_accel= 0.5*accel_max*T_accel*T_accel;
		dist_deccel = 0.5*accel_max*T_accel*T_accel;
		dist_const_v = segLength - dist_accel -dist_deccel;
	}
	/* Obstacle detected, Ramp down V to 0 */
	else if (segType == 4) {
		dist_accel = 0;
		dist_deccel = 0.5*accel_max*T_accel*T_accel;
		dist_const_v = 0;
	}
	else{ 
		velocity_cmd.linear.y = -1;
		return velocity_cmd;
	}

	if(dist_const_v<0){
		dist_accel = segLength/2;
		dist_deccel = dist_accel;
	}

	time=time+dt;
	segDistDone += ((v_past+v_cmd)/2)*dt;
	/*/ Accel Phase */	
	if (segDistDone<dist_accel)
	{
		v_scheduled = sqrt(2*segDistDone*accel_max);
		if (v_scheduled < accel_max*dt){
			v_scheduled = accel_max*dt;
		}
	}
	/* Constant V Phase */
	else if (segDistDone<dist_accel+dist_const_v){
		v_scheduled = velocity_max;
	}
	/* Decel Phase */
	else
	{
		ROS_INFO("DECEL PHASE HANDLING");
		ROS_INFO("DECEL (SEGLENGTH - DISTDONE) = %f", (segLength - segDistDone));
		temp = 2*(segLength-segDistDone)*accel_max;
		ROS_INFO("DECEL TEMP: %f", temp);
		if(temp<0){
			v_scheduled = 0.0;
		}
		else{
			v_scheduled = sqrt(temp);
		}

		ROS_INFO("V_SCHEDULED = %f", v_scheduled);
	}

	if (v_cmd < v_scheduled){
		ROS_INFO("V_CMD < V_SCHEDULED");
		v_test=v_cmd+accel_max*dt;
		v_cmd = min(v_test,v_scheduled);
	}
	else if (v_cmd > v_scheduled){
		ROS_INFO("V_CMD > V_SCHEDULED");
		v_test = v_cmd-1.2*accel_max*dt;
		v_cmd = max(v_test,v_scheduled);
	}
	else{
		ROS_INFO("ELSE ");
		v_cmd=v_scheduled;
	}
	if(v_cmd>velocity_max){
		ROS_INFO ("V_CMD > VELOCITY MAX");
		v_cmd = velocity_max;
	}

	ROS_INFO("V-CMD == %f", v_cmd);

	if(segType==1){	
		velocity_cmd.linear.x = v_cmd;
		velocity_cmd.angular.z = 0.0;
		return velocity_cmd;
	}
	else if(segType==2){
		velocity_cmd.linear.x = 0.0;
		velocity_cmd.angular.z = v_cmd;
		return velocity_cmd;
	}
	else if (segType == 4) {
		velocity_cmd.linear.x = v_cmd;
		velocity_cmd.angular.z = 0.0;
		return velocity_cmd;
	}
	velocity_cmd.linear.y = -1;
	return velocity_cmd;
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
	estop = true;	
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
	double goalSegLength = segLength;
	
	if(dist_const_v<0){
		dist_accel = segLength/2;
		dist_deccel = dist_accel;
	}

	while (ros::ok() && segDistDone<segLength) // do work here
	{
		ros::spinOnce();

		if (estop == false) {
			ESM(segLength, segDistDone, pub);
			return;
		}

		if (obstacle) {
			// Grab current segDistDone and segLength
			OAM(segLength, segDistDone, v_cmd, v_past, pub);
			return;
		}

		t=t+dt;
		segDistDone += ((v_past+v_cmd)/2)*dt;
		v_past = v_cmd;

		vel_object = getVelocity(t,v_past,v_cmd,segDistDone,segLength,1);
		if(vel_object.linear.y==-1){
			ROS_INFO("We had a bad segType");
			return;
		}
		v_cmd = vel_object.linear.x;		
		ROS_INFO("NORMAL VELOCITY COMMAND");
		ROS_INFO("linear %f %f %f %f %f %f",vel_object.linear.x,vel_object.linear.y,vel_object.linear.z,segLength,segDistDone,t);
		pub.publish(vel_object);  // this action causes the commands in vel_object to be published 
	
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (50ms) period
		//thus enforcing that we achieve the desired update rate (20Hz)
		
	}
	return;
}

void ESM(double segLength, double segDistDone, ros::Publisher pub) {
	geometry_msgs::Twist vel_object;
	ros::Rate naptime(nap);
	vel_object.linear.x = 0.0;
	vel_object.linear.y = 0.0;
	vel_object.linear.z = 0.0;
	vel_object.angular.z = 0.0;
	while (!estop) {
		ros::spinOnce();
		// Publish 0.
		pub.publish(vel_object); 
		naptime.sleep();
		ROS_INFO("ESTOP ENFORCED");
	}

	holdingPattern(1, pub);
	segLength = segLength - segDistDone;
	runLinear(segLength, pub);
}	

void OAM(double segLength, double segDistDone, double v_cmd, double v_past, ros::Publisher pub) {
	geometry_msgs::Twist vel_object;
	ros::Rate naptime(nap);
	double t=0;
	double T_accel = v_max/a_max;
	double dist_accel= 0.5*a_max*T_accel*T_accel;
	double dist_deccel = dist_accel;
	double dist_const_v = segLength - dist_accel -dist_deccel;
	double v_scheduled, v_test,temp;
	double goalSegLength = segLength;
	segLength = segDistDone + 1;
	
	if(dist_const_v<0){
		dist_accel = segLength/2;
		dist_deccel = dist_accel;
	}
	
	while (obstacle) {
		ros::spinOnce();
		if (!estop) {
			// completed so far
			ESM(goalSegLength, segDistDone, pub);
			return;
		}
		t = t+dt;
		segDistDone += ((v_past+v_cmd)/2)*dt;
		v_past = v_cmd;
		vel_object = getVelocity(t,v_past,v_cmd,segDistDone,segLength,1);
		if(vel_object.linear.y==-1){
			ROS_INFO("We had a bad segType");
			return;
		}
		v_cmd = vel_object.linear.x;	
		ROS_INFO("OBSTACLE V: %f DIST DONE: %f DIST LENGTH %f", v_cmd, segDistDone, segLength);	
		pub.publish(vel_object);
		naptime.sleep();
	}

	double restOfSeg = goalSegLength - segLength;
	ROS_INFO("EXITING OAM.  REST OF SEG = %f", restOfSeg);
	runLinear(restOfSeg, pub);

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

		vel_object = getVelocity(t,omega_past,omega_cmd,segDistDone,segLength,2);
		if(vel_object.linear.y==-1){
			ROS_INFO("We had a bad segType");
			return;
		}
		omega_cmd = vel_object.angular.z;
		vel_object.angular.z*=direction;

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
	ros::Subscriber sub2 = n.subscribe("obstructions", 1,obstructionsCallback);

	//"cmd_vel" is the topic name to publish velocity commands
	//"1" is the buffer size (could use buffer>1 in case network bogs down)

	while (!ros::Time::isValid()) {} // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid
	ros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
	ROS_INFO("birthday started as %f", birthday.toSec());

	holdingPattern(0.5,pub);

	runLinear(5,pub);
	holdingPattern(0.2,pub);
/*
	runInPlaceTurn(HALF_PI,CW,pub);
	holdingPattern(0.2,pub);

	runLinear(12.5,pub);
	holdingPattern(0.2,pub);

	runInPlaceTurn(HALF_PI,CW,pub);
	holdingPattern(0.2,pub);

	runLinear(4,pub);
	holdingPattern(0.05,pub); 
*/
	return 0;
}
