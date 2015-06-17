/*
 * comms_node.h
 *
 *  Created on: 19 Jan 2015
 *      Author: nick
 */

#ifndef COMMS_NODE_H_
#define COMMS_NODE_H_

#include <vector>
#include <deque>
#include <map>
#include <osl_core/GlobalCoordConverter.h>
#include <comms_sim/comms_msg.h>
#include <ros/ros.h>

#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

/*
 * Class that represents a communications node in the network.
 */

class CommsNode
{
  std::deque<CommsMsg> msg_queue_;
  std::string name_;
  static std::map<std::string, osl_core::LLD> node_position_map_;
  ros::Time last_transmission_time_;
  int per_type_;
  int id_;
  double per_;

  double collision_window_; //How much time does the modem take to listen and process the message in seconds. This is transmission length plus processing time.

  static boost::mt19937 rng;//TODO: Should add seed.
  static boost::uniform_01<boost::mt19937> generator;// TODO: check if numbers are generated properly
//  static boost::uniform_real<> dist;
//  static boost::variate_generator<boost::mt19937&, boost::uniform_real< > > generate;

  ros::NodeHandlePtr nhp_;
  ros::Publisher in_burst_pub_;
  ros::Publisher in_im_pub_;
  ros::Publisher ack_burst_pub_;
  ros::Publisher ack_im_pub_;

  void checkForMessageCollisions();
  void checkForPER(CommsMsg &msg);
  vehicle_interface::AcousticModemPayload generatePayloadMsg(vehicle_interface::AcousticModemPayloadPtr msg);
  vehicle_interface::AcousticModemAck generateAckMsg(vehicle_interface::AcousticModemAckPtr msg);
public:
  CommsNode(std::string name, int id, int per_type, double per, double collision_window, ros::NodeHandlePtr nhp);
  static void updatePositionMap(std::string node_name, osl_core::LLD pos);
  static osl_core::LLD getPosition(std::string node_name);
  bool isMessageTime(ros::Time now);
  bool isMessageReceived();
  void pushMessage(CommsMsg msg);
  void publishMessage(CommsMsg msg);
  bool handleMsg(CommsMsg &msg);
  CommsMsg popMsg();
  std::string getName();
  int getID();
};

#endif /* COMMS_NODE_H_ */
