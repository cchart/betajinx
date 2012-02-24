#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> //data type for velocities

#define HZ 10
#define ADJ 0
#define D1 4
#define D2 12
#define D3 3

#define PI 3.1415926535897
#define SEGMENTS 4
#define LINE 1
#define ARC  2
#define SPIN 3
#define CW -1
#define CCW 1

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

int main(int argc,char **argv)
{
	ros::init(argc,argv,"command_publisher_pt2");//name of this node
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	//"cmd_vel" is the topic name to publish velocity commands
	//"1" is the buffer size (could use buffer>1 in case network bogs down)

	geometry_msgs::Twist vel_object; // declare once, set internal data many times

	ros::Duration pause_duration(1.0);
	ros::Duration elapsed_time; // define a variable to hold elapsed time

	//send robot forward for 3 seconds, reiterated at 10Hz.  Need a ROS "rate" object to enforce a rate
	ros::Rate naptime(HZ); //will perform sleeps to enforce loop rate of "10" Hz
	double t = 0.0;
	double dt  = 1.0/HZ;
	double v_max = 1.0;
	double a_max = 0.5;
	double omega_max = 1.0;
	double alpha_max = 0.3;
	//double delta_v = alpha_max*dt;
	
	double T_accel = v_max/a_max; //...assumes start from rest
	double T_decel = T_accel; //(for same decel as accel); assumes brake to full halt
	double dist_accel= 0.5*a_max*T_accel*T_accel; //distance rqd to ramp up to full speed
	double dist_decel = dist_accel; //same as ramp-up distance
	double rot_accel= 0.5*alpha_max*T_accel*T_accel; //distance rqd to ramp up to full speed
	double rot_decel = rot_accel; //same as ramp-up distance

	//double T_const_v = 0.0;
	//double T_segment_tot = 0.0;
	double dist_const_v = 0.0;
	double rot_const_omega = 0.0;
	double segDist = 0.0;
	double segDistDone = 0.0;
	double v_cmd = 0.0;
	double v_scheduled = 0.0;
	double lastv = 0.0;

	double omega_cmd = 0.0;
	double omega_scheduled = 0.0;
	double lastomega = 0.0;

	int segment_number = 0;
	int total_segments = SEGMENTS;
	int segType = LINE;

	while (!ros::Time::isValid()) {} // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid

	ros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
	ROS_INFO("birthday started as %f", birthday.toSec());

	while (ros::ok()) // do work here
	{
		t = t + dt; //not sure if this t is ever used
		elapsed_time= ros::Time::now()-birthday;
		//ROS_INFO("birthday is %f", birthday.toSec());
		ROS_INFO("elapsed time is %f", elapsed_time.toSec());
		
		if (elapsed_time < pause_duration)
		{
			vel_object.linear.x = 0.0;
			vel_object.angular.z = 0.0;
			pub.publish(vel_object);
		}
		else if(segDistDone <= segDist)
		{
		//	ROS_INFO("Segment %d...", segment_number);
			switch(segment_number)
			{
				case 0:
					segDist = D1;
					segType = LINE;
					break;
				case 1:
					segDist = floor(( PI/2 )/dt)*dt+ADJ;
					segType = SPIN;
					break;
				case 2: 
					segDist = D2;
					segType = LINE;
					break;
				case 3: 
					segDist = floor(( PI/2 )/dt)*dt+ADJ;
					segType = SPIN;
					break;
				case 4:
					segDist = D3;
					segType = LINE;
					break;

				default://should never get here
					segDist = 1.0;
					v_max = 0.0;
					a_max = 0.0;
					break;
			}
			if (segType == LINE)
			{
				segDistDone = segDistDone + ((lastv+v_cmd)/2)*dt;
				//T_const_v = dist_const_v/v_max;
				//T_segment_tot = T_accel + T_decel + T_const_v;
				T_accel = v_max/a_max; //...assumes start from rest
				T_decel = T_accel; //(for same decel as accel); assumes brake to full halt
				dist_accel= 0.5*a_max*T_accel*T_accel; //dist rqd to get full speed
				dist_decel = dist_accel; //same as ramp-up distance
				dist_const_v = segDist - dist_accel - dist_decel; //will be >0 @ speed

				if(dist_const_v < 0)
				{
					dist_accel = segDist / 2;
					dist_decel = dist_accel;
				}

				if (segDistDone<dist_accel)  //ramp-up phase
				{	//in ramp-up phase; what is appropriate velocity for this phase?
					v_scheduled = sqrt(2*segDistDone*a_max); // v' = a
					if (v_scheduled < a_max*dt)
						v_scheduled = a_max*dt;//add some extra, avoid sticking in place
				}
				else if (segDistDone<dist_accel+dist_const_v) //const vel phase
				{
					v_scheduled = v_max;
				}
				else //ramp-down phase
				{
					if(segDist > segDistDone)
					{
						v_scheduled = sqrt(2*(segDist-segDistDone)*a_max);
					}
					else
					{
						v_scheduled = 0;   //beware the NaN!!
					}
				}

				if (v_cmd < v_scheduled)
				{//min
				//	v_cmd = (v_scheduled < v_cmd+a_max*dt ? v_scheduled : v_cmd+a_max*dt );
					v_cmd = min(v_scheduled, v_cmd+a_max*dt);
				}
				else if (v_cmd > v_scheduled)
				{//max
				//	v_cmd = (v_scheduled > v_cmd-1.2*a_max*dt ? v_scheduled : v_cmd-1.2*a_max*dt );
					v_cmd = max(v_scheduled, v_cmd-1.2*a_max*dt);
				}
				else
				{//meh
					v_cmd = v_scheduled;
				}
				lastv = v_cmd;
				omega_cmd = 0.0;

			}
			else if (segType == SPIN)
			{
				segDistDone = segDistDone + (omega_cmd+lastomega)/2 * dt;
				//T_const_v = dist_const_v/v_max;
				//T_segment_tot = T_accel + T_decel + T_const_v;
				T_accel = omega_max/alpha_max; //...assumes start from rest
				T_decel = T_accel; //(for same decel as accel); assumes brake to full halt
				rot_accel= 0.5*alpha_max*T_accel*T_accel; //rqd to ramp to full speed
				rot_decel = rot_accel; //same as ramp-up distance
				rot_const_omega = segDist - rot_accel - rot_decel; // >0 @full speed
				
				if(rot_const_omega < 0)
				{
					rot_accel = segDist / 2;
					rot_decel = rot_accel;
				}

				if(segDistDone < rot_accel)//ramp-up
				{
					omega_scheduled = sqrt(2*segDistDone*alpha_max); //omega' = alpha
					if (omega_scheduled < alpha_max*dt) //dont get stuck at 0
						omega_scheduled = alpha_max*dt;
				}
				else if (segDistDone < rot_accel+rot_const_omega)//constant-omega
				{
					omega_scheduled = omega_max;
				}
				else//ramp-down
				{
					if(segDist > segDistDone)
					{
						omega_scheduled = sqrt(2*(segDist-segDistDone)*alpha_max);
					}
					else //rather than NaN
					{
						omega_scheduled = 0;
					}
				}

				if (omega_cmd < omega_scheduled)
				{
					omega_cmd = min(omega_cmd+alpha_max*dt, omega_scheduled);
				}
				else if (omega_cmd > v_scheduled)
				{
					omega_cmd = max(omega_cmd-1.2*alpha_max*dt, omega_scheduled);
				}
				else
				{
					omega_cmd = omega_scheduled;
				}

				if (omega_cmd > omega_max)
				{
					omega_cmd = omega_max;
				}
				lastomega = omega_cmd;
				v_cmd = 0.0;


			}

	 		vel_object.linear.x = v_cmd;
			vel_object.angular.z = omega_cmd*CW;
	 		pub.publish(vel_object);
		}
		else
		{
			if ( segment_number < total_segments )
			{
				segment_number++;
				segDistDone = 0;
				ROS_INFO("Starting Segment %d", segment_number);
			}
			else
			{
				return 0; //we're done here
			}
			vel_object.linear.x = 0.0;
			vel_object.angular.z = 0.0;
			pub.publish(vel_object);
		}
		naptime.sleep(); // this will cause the loop to sleep for balance of time of desired (100ms) period
		//thus enforcing that we achieve the desired update rate (10Hz)
	}
	return 0; // this code will only get here if this node was told to shut down, which is
	// reflected in ros::ok() is false 
}
