/*
 * comms_sim_main.cpp
 *
 *  Created on: 28 Jan 2015
 *      Author: nick
 */

#include <comms_sim/comms_sim.h>

int int main(int argc, char **argv)
{
  cout << "Initialising ROS with args: " << endl;
  for (int i = 0; i < argc; ++i)
  {
    cout << "  " << i << " : " << argv[i] << endl;
  }

  ros::init(argc, argv, "comms_sim");

  ros::NodeHandlePtr rosNodeHandle(new ros::NodeHandle());

  CommsSim simulator(rosNodeHandle);

  if (!simulator.init())
  {
    ROS_ERROR_STREAM("simulator.init returned false.");
    exit(-1);
  }

  ROS_INFO_STREAM("CommsSim initialisation successful; running now.");

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    simulator.doWork();
    loop_rate.sleep();
  }

  return 0;
}
