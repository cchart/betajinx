#include <math.h>
#include <ros/ros.h>
#include <cwru_base/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_listener/obstacle.h>
#include <iostream>

#define HZ 100
#define PI 3.141562653589

using namespace std;
float nearestObstacle;
float nearestTheta;
bool objectInRange = false;
bool holla = false;

// For using the LaserScan, see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html


bool inDeBox(int theta, float r)
{

    laser_listener::obstacle referenceMsg;

    //cout << "using box width: " << referenceMsg.boxWidth << endl;
    //cout << "using box length: " << referenceMsg.boxLength << endl;

    //do some math here to find if a point lies in a box. uff.

    return false;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{
    objectInRange = false;
    nearestTheta = 0;
    float shortestRange = 999.999;

    for(uint i = 0; i < laserScan->ranges.size(); i++)
    {
      if (laserScan->ranges[i]<shortestRange)
      {
          shortestRange = laserScan->ranges[i];
          nearestTheta = i;
      }

      //cout << inDeBox(i, laserScan->ranges[i]);
      cout<<(laserScan->ranges[i]<0.75);

      if (laserScan->ranges[i]<0.75)
      {
          objectInRange = true;
      }
    }
    nearestObstacle = shortestRange*sin((((180-nearestTheta)*PI)/180));
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
