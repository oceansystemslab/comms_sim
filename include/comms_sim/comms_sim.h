/*
 * comms_sim.h
 *
 *  Created on: 19 Jan 2015
 *      Author: nick
 */

#ifndef COMMS_SIM_H_
#define COMMS_SIM_H_

#include <comms_sim/comms_node.h>
#include <ros/ros.h>

class CommsSim
{
  std::vector<CommsNode> node_list_;
  std::vector<ros::Subscriber> sub_v_;
  std::map<std::string, ros::Publisher> pub_m_;
  ros::NodeHandlePtr nhp_;
  void createPubSub(std::string node_name);
public:
  CommsSim(ros::NodeHandlePtr nhp);
  void init();
  void doWork();
};

#endif /* COMMS_SIM_H_ */
