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
#include <auv_msgs/NavSts.h>
#include <boost/algorithm/string.hpp>

class CommsSim
{
  std::vector<CommsNode> node_list_;
  std::vector<ros::Subscriber> modem_sub_v_;
  std::vector<ros::Subscriber> nav_sub_v_;
  std::map<std::string, ros::Publisher> pub_m_;
  ros::NodeHandlePtr nhp_;

  int per_type_;
  double per_;
  double collision_window_;
  std::string platform_names_;

  void addCommsNode(std::string node_name);
  void modemOutCB(vehicle_interface::AcousticModemPayload::ConstPtr &msg, std::string node_name);
  void navStsOutCB(auv_msgs::NavSts::ConstPtr &msg, std::string node_name);
public:
  CommsSim(ros::NodeHandlePtr nhp);
  bool init();
  void doWork();
};

#endif /* COMMS_SIM_H_ */
