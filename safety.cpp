#include "ros/ros.h"
#include <std_msgs/Bool.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "safety");
  ros::NodeHandle n;
  std_msgs::Bool safe;
  ros::Publisher safety_pub = n.advertise<std_msgs::Bool>("elfin/velocity_controller/allclear", 1000);
  ros::Rate loop_rate(1000);
  safe.data = true;
  int count = 0;
  bool loop; 
  
  
  while (ros::ok())
  {
    
    loop = true;
    while (loop == true){
     safety_pub.publish(safe);
     std::cout << "Press 'Enter' for emergency stop: ";
     std::cin.get();
     loop = false;
     }
	
    safe.data = !safe.data;
    //safety_pub.publish(safe);
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

