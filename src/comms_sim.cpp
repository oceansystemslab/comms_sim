/*
 * comms_sim.cpp
 *
 *  Created on: 22 Jan 2015
 *      Author: nick
 */

#include <comms_sim/comms_sim.h>

void CommsSim::addCommsNode(std::string node_name)
{
  // Create all the infrastructure for the comms node (Publishers, subscribers, class etc.)
  node_list_.push_back(CommsNode(node_name, per_type_, per_, collision_window_, nhp_));
  std::stringstream topic;
  topic << "/" << node_name << "/modem/out";
  modem_sub_v_.push_back(
      nhp_->subscribe<vehicle_interface::AcousticModemPayload>(topic.str(), 1, boost::bind(modemOutCB, _1, node_name)));
  topic.clear();
  topic << "/" << node_name << "/nav/nav_sts";
  nav_sub_v_.push_back(nhp_->subscribe<auv_msgs::NavSts>(topic.str(), 1, boost::bind(navStsOutCB, _1, node_name)));
}

void CommsSim::modemOutCB(vehicle_interface::AcousticModemPayload::ConstPtr &msg, std::string node_name)
{
  //Here we receive a message from the node with node_name. We should put the message to all the other nodes' queues.

}

void CommsSim::navStsOutCB(auv_msgs::NavSts::ConstPtr &msg, std::string node_name)
{
  //Here we receive position update from the node with node_name. We should update it in the appropriate map.
  osl_core::LLD pos;
  pos.lat = msg->global_position.latitude;
  pos.lon = msg->global_position.longitude;
  pos.depth = msg->altitude;
  CommsNode::updatePositionMap(node_name, pos);
}

CommsSim::CommsSim(ros::NodeHandlePtr nhp)
{
  nhp_ = nhp;
}

bool CommsSim::init()
{
  //Do initialisation of the node. Read all the ros params. If something goes wrong crash the node.
  if (!nhp_->getParam("sim/platform_names", platform_names_))
  {
    ROS_ERROR_STREAM("Didn't find platform names. Exiting...");
    return false;
  }

  if (!nhp_->getParam("sim/per_type", per_type_))
  {
    ROS_ERROR_STREAM("Didn't find per_type. Exiting...");
    return false;
  }

  if (!nhp_->getParam("sim/per", per_))
  {
    ROS_ERROR_STREAM("Didn't find per. Exiting...");
    return false;
  }

  if (!nhp_->getParam("sim/collision_window", collision_window_))
  {
    ROS_ERROR_STREAM("Didn't find collision_window. Exiting...");
    return false;
  }

  //Here we create each of the comms_node instances
  std::vector<std::string> nodes;
  boost::split(nodes, platform_names_, boost::is_any_of("\t "));
}

void CommsSim::doWork()
{
  /*
   * Main loop of the node.
   * Check for each node if any message is received and process it accordingly.
   */

}
