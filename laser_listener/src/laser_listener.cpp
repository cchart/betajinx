#include <ros/ros.h>
#include <cwru_base/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_listener/obstacle.h>
#include <iostream>

#define HZ 20

using namespace std;

bool objectInRange = false;

// For using the LaserScan, see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{
    objectInRange = false;
    for(uint i = 0; i < laserScan->ranges.size(); i++)
    {
      if (laserScan->ranges[i]<0.75)
      {
	      cout<<"ouch, ";
          objectInRange = true;
      }
    }
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
        
        if (objectInRange == true)
        {
            cout <<"ZONK!!";
            obstacleMsg.obstacle = true;
        }
        else
        {
            obstacleMsg.obstacle = false;
        }
        
        pub.publish(obstacleMsg);

        r.sleep();

    }
    return 0;
}

