#include <math.h>
#include <ros/ros.h>
#include <cwru_base/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_listener/obstacle.h>
#include <iostream>

#define HZ 100
#define PI 3.141562653589
#define D2R 0.0174532925 //pi/180

using namespace std;
float nearestObstacle;
float nearestTheta;
bool objectInRange = false;
bool holla = false;

// For using the LaserScan, see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html

//inDeBox tells if a point at a given theta and radial distance from the laser 
//lies in a rectrangular area infront of the robot, whose width and length are 
//defined for fun in the 'obstacle' message 
bool inDeBox(int theta, float r)
{

    laser_listener::obstacle referenceMsg; //this is used to get the constants from the message
    bool result = false;

    if(abs(r*sin(theta*D2R))<referenceMsg.boxLength) //abs() hack for negative values ...
    {
        if(abs(r*cos(theta*D2R))<(referenceMsg.boxWidth))
        {
            cout << "front: " << r*sin(theta*D2R) << "side: " << r*cos(theta*D2R) << endl;            
            result = true;
        }
    }

    return result;
}

bool diff(float a, float b){
	if(a<b-0.5 || a>b+0.05){
		return false;
	}
	return true;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{
    objectInRange = false;
    nearestTheta = 0;
    float shortestRange = 999.999;

    for(uint i = 1; i < 179; i++)
    {
      if (laserScan->ranges[i]<shortestRange)
      {
          shortestRange = laserScan->ranges[i];
          nearestTheta = i;
      }

      //cout << inDeBox(i, laserScan->ranges[i]);
      //cout<<(laserScan->ranges[i]<0.75); //primitive visualization of lazerz

      if (inDeBox(i, laserScan->ranges[i]) && diff(laserScan->ranges[i],laserScan->ranges[i-1]) && diff(laserScan->ranges[i],laserScan->ranges[i+1]))
      {
          objectInRange = true;
      }
      /*
      if (laserScan->ranges[i]<0.75)
      {
          objectInRange = true;
      }
      */
    }
    nearestObstacle = shortestRange*sin((((180-nearestTheta)*PI)/180.0)); 
    //This is the braking distance for an obstacle in front of our track. 
    //for large boxes and obstacles just barely in the box, but whose theta
    //is large or small (0-10,170-180), stopping distance may be quite short.  
    holla = true;
    cout<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_listener");

    ros::NodeHandle n;

    ros::Rate r(HZ);
    
    ros::Subscriber sub = n.subscribe("base_scan",1,laserCallback);

    ros::Publisher pub = n.advertise<laser_listener::obstacle>("obstructions", 10);
    
    laser_listener::obstacle obstacleMsg;

    //ros::spin();
    while(!ros::Time::isValid())
    {
        ros::spinOnce();
    }

    while(ros::ok())
    {
        //objectInRange = false;
        ros::spinOnce(); //triggers callbacks 
        if (holla)
        {
            holla = false;

            obstacleMsg.nearestObstacle = nearestObstacle;
            obstacleMsg.nearestTheta = nearestTheta;

            if (objectInRange)
            {
                cout <<"ZONK!!"<<endl;
                obstacleMsg.obstacle = true;
            }
            else
            {
                obstacleMsg.obstacle = false;
            }
            
            pub.publish(obstacleMsg);
        }
        else 
        {
        //    cout << "RATE IS MORE THAN FAST ENOUGH" << endl;
        }

        r.sleep();

    }
    return 0;
}
