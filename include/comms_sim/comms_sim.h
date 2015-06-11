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
#include <vehicle_interface/AcousticModemAck.h>
#include <sstream>

class CommsSim
{
  boost::mt19937 rng;//TODO: Should add seed.
  boost::uniform_01<boost::mt19937> zeroone;
  //boost::variate_generator<boost::mt19937, boost::uniform_01<> > gen(rng, zeroone);

  std::vector<CommsNode> node_list_;
  std::vector<ros::Subscriber> modem_burst_sub_v_;
  std::vector<ros::Subscriber> modem_im_sub_v_;
  std::vector<ros::Subscriber> nav_sub_v_;
  std::map<std::string, ros::Publisher> burst_ack_pub_m_;
  std::map<std::string, ros::Publisher> im_ack_pub_m_;
  ros::NodeHandlePtr nhp_;

  int per_type_;
  double per_;
  bool use_fixed_flight_time_;
  double flight_time_;
  double medium_speed_;
  double collision_window_;
  std::vector<std::string> platform_names_;
  std::vector<int> platform_ids_;

  void addCommsNode(std::string node_name, int id);
  void modemOutBurstCB(const vehicle_interface::AcousticModemPayload::ConstPtr &msg, std::string node_name);
  void modemOutIMCB(const vehicle_interface::AcousticModemPayloadConstPtr &msg, std::string node_name);
  void navStsOutCB(const auv_msgs::NavSts::ConstPtr &msg, std::string node_name);
  bool isAckReceived();
  void publishAckMsg(CommsMsg msg, bool ackReceived);
  ros::Time calculateDeliveryTime(std::string n1, std::string n2, ros::Time transmission_time);
  vehicle_interface::AcousticModemAckPtr generateAckMsg(unsigned int msg_id, bool ack);
public:
  CommsSim(ros::NodeHandlePtr nhp);
  bool init();
  void doWork();
};

#endif /* COMMS_SIM_H_ */
