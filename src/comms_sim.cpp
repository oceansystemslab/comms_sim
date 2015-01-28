/*
 * comms_sim.cpp
 *
 *  Created on: 22 Jan 2015
 *      Author: nick
 */

#include <comms_sim/comms_sim.h>

double findDistance(osl_core::T_LLD a, osl_core::T_LLD b)
{
  int Rk = 6373000;  // mean radius of the earth (m) at 39 degrees from the equator
  //Convert to radians
  double alat = a.lat*(M_PI/180);
  double alon = a.lon*(M_PI/180);
  double blat = b.lat*(M_PI/180);
  double blon = b.lon*(M_PI/180);
  //Calculate distance between points in meters.
  double dlon = blon - alon;
  double dlat = blat - alat;
  double a1 = pow((sin(dlat/2)),2) + cos(alat) * cos(blat) * pow((sin(dlon/2)),2);
  double c = 2 * atan2(sqrt(a1), sqrt(1-a1));
  double distance =  Rk * c;

  double height = a.depth - b.depth;
  distance = pow(distance,2) + pow(height,2);
  return sqrt(distance);
}

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
  std::vector<CommsNode>::iterator it;
  for (it = node_list_.begin(); it != node_list_.end(); it++)
  {
    //Handle incoming message.
    if (it->getName() != node_name) //We don't want to hear ourselves.
    {
      ros::Time transmission_time = ros::Time::now();
      ros::Time delivery_time;
      if (use_fixed_flight_time_)
      {
        delivery_time = transmission_time + ros::Duration(flight_time_);
      }
      else
      {
        double distance = findDistance(CommsNode::getPosition(node_name), CommsNode::getPosition(it->getName()));
        delivery_time = transmission_time + ros::Duration(distance/medium_speed_);
      }
      it->pushMessage(CommsMsg(msg, transmission_time, delivery_time, node_name, it->getName(), false));
    }
  }
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
  std::string nodes;
  if (!nhp_->getParam("sim/platform_names", nodes))
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

  if (!nhp_->getParam("sim/use_fixed_flight_time", use_fixed_flight_time_))
  {
    ROS_ERROR_STREAM("Didn't find use_fixed_flight_time. Exiting...");
    return false;
  }

  if (!nhp_->getParam("sim/flight_time", flight_time_))
  {
    ROS_ERROR_STREAM("Didn't find flight_time. Exiting...");
    return false;
  }

  if (!nhp_->getParam("sim/meduim_speed", medium_speed_))
  {
    ROS_ERROR_STREAM("Didn't find medium_speed. Exiting...");
    return false;
  }

  //Here we create each of the comms_node instances
  boost::split(platform_names_, nodes, boost::is_any_of("\t "));

  std::vector<std::string>::iterator it;
  for (it = platform_names_.begin(); it != platform_names_.end(); it++)
  {
    //Create all the classes here.
    addCommsNode(*it);
  }


}

void CommsSim::doWork()
{
  /*
   * Main loop of the node.
   * Check for each node if any message is received and process it accordingly.
   * Loop around the comms nodes, check if it is time to receive a message, and decide if you receive it.
   */
  std::vector<CommsNode>::iterator it;
  for(it = node_list_.begin(); it != node_list_.end(); it++)
  {
    if(it->isMessageTime(ros::Time::now()))
    {
      it->handleMsg();
    }
  }
}
