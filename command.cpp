#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "command");
  ros::NodeHandle n;
  
  //change topic to the controller you are using e.g. computed_torque_clik_controller/command"
  //ros::Publisher pose_pub = n.advertise<std_msgs::Float64MultiArray>("elfin/computed_torque_clik_controller/command", 1000);
  ros::Publisher pose_pub = n.advertise<std_msgs::Float64MultiArray>("elfin/velocity_controller/command", 1000);
  ros::Rate loop_rate(1000);
  int count = 0;
  bool loop; 
  float a, b, c, d, e, f;
  int m, i; 
  double comp[6];
  
  while (ros::ok())
  {
    //std_msgs::Float64MultiArray msg;
    std_msgs::Float64MultiArray com;
    loop = true;
    while (loop == true){
     std::cout << "Enter '3' for coordinates only, or '6' for pose \n";
     std::cin >> m;
     if (m == 3){
        std::cout << "Enter the desired coordinates x,y,z \n";
        std::cin >> a >> b >> c; //i is index 
        d = 0, e=0, f=0;
        com.data = {a, b, c, d, e, f};
        loop = false;    
     }

     else if (m == 6){
     std::cout << "Enter the desired coordinates and pose \n";
     std::cin >> a >> b >> c >> d >> e >> f; //i is index
     //com.data = {1.00, 1.00, 1.00, 0.00, 0.00, 0.00};
     com.data = {a, b, c, d, e, f};
     loop = false;
     }
     }
    //msg.data = [com];	
    pose_pub.publish(com);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

