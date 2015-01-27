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
  double per_ = 0;
  int per_type_ = 0;

  double collision_window_ = 0.5; //How much time does the modem take to listen and process the message in seconds. This is transmission length plus processing time.

  static boost::mt19937 rng;//TODO: Should add seed.
  static boost::uniform_01<boost::mt19937> generator;// TODO: check if numbers are generated properly
//  static boost::uniform_real<> dist;
//  static boost::variate_generator<boost::mt19937&, boost::uniform_real< > > generate;

  ros::NodeHandlePtr nhp_;
  ros::Publisher in_pub_;

  void checkForMessageCollisions();
  void checkForPER(CommsMsg &msg);
public:
  CommsNode(std::string name, int per_type, double per, double collision_window, ros::NodeHandlePtr nhp);
  static void updatePositionMap(std::string node_name, osl_core::LLD pos);
  bool isMessageTime(ros::Time now);
  bool isMessageReceived();
  void pushMessage(CommsMsg msg);
  void publishMessage(CommsMsg msg);
  CommsMsg popMsg();
};

#endif /* COMMS_NODE_H_ */
