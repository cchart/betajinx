#include<ros/ros.h>
#include<cwru_base/cRIOSensors.h>
#include<std_msgs/Bool.h>
#include<math.h>
#include <cwru_base/Pose.h>
#include<geometry_msgs/Twist.h>

#define HALF_PI 1.67079633
#define CW -1
#define nap 2
#define v_max 1.0
#define a_max 0.25
#define omega_max 1.0
#define alpha_max 0.5

struct Segment{
double xStart,yStart,xEnd,yEnd,heading,segLength;
};

struct Path{
Segment *segments;
};

class StateGen{
	public:
    		StateGen();
	private:
		bool computeState(precision_navigation_msgs::DesiredState& new_des_state);
		void SetPath();
};

bool StateGen::computeState(){

}

void SetPath(){

}


