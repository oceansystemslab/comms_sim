/*
 * comms_node.cpp
 *
 *  Created on: 20 Jan 2015
 *      Author: nick
 */


#include <comms_sim/comms_node.h>

std::map<std::string, osl_core::LLD> CommsNode::node_position_map_;
boost::mt19937 CommsNode::rng;
boost::uniform_01<boost::mt19937> CommsNode::generator(rng);

//boost::uniform_real<> CommsNode::dist(0.0,1.0);
//boost::variate_generator<boost::mt19937&,boost::uniform_real< > > CommsNode::generate(rng,dist);

void CommsNode::checkForMessageCollisions()
{
  //Check if a message collides with any other messages in the queue and if you transmitted at the same time as the other part.
  std::deque<CommsMsg>::iterator it1;
  std::deque<CommsMsg>::iterator it2;

  for(it1 = msg_queue_.begin(); it1 != msg_queue_.end(); it1++)
  {
    for(it2 = it1+1; it2 != msg_queue_.end(); it2++)
    {
      if((it1->getDeliveryTime() - it2->getDeliveryTime()) < ros::Duration(collision_window_))
      {
        it1->setErrorStatus(true);
        it2->setErrorStatus(true);
      }
      //Check for the case the transmission of my last message was concurrent with any message received.
      if((it1->getTransmissionTime() <= last_transmission_time_)&&(last_transmission_time_ <= it1->getDeliveryTime()))
      {
        it1->setErrorStatus(true);
      }
    }
  }
}

void CommsNode::checkForPER(CommsMsg &msg)
{
  //Check if the packet arrives correctly.
  double rndNum = generator();
  if(per_type_ == 0)
  {
    if(rndNum < per_)
    {
      msg.setErrorStatus(true);
    }
  }
  // TODO: Implement other per types (i.e. distance based).

}

CommsNode::CommsNode(std::string name, int per_type, double per,
												double collision_window, ros::NodeHandlePtr nhp)
	: name_(name), per_type_(per_type), per_(per),
		collision_window_(collision_window), nhp_(nhp)
{
	//TODO: Add specific per for this node in launch file.

  std::stringstream topic;
  topic << "/" << name << "/modem/in";
  in_pub_ = nhp_->advertise<vehicle_interface::AcousticModemPayload>(topic.str(), 10);
}

void CommsNode::updatePositionMap(std::string node_name, osl_core::LLD pos)
{
  //Static method that updates the position of a vehicle in the node_position_map_
  std::map<std::string, osl_core::LLD>::iterator it;
  it = node_position_map_.find(node_name);
  if(it == node_position_map_.end())
  {
    //Node doesn't exist, so insert it.
    node_position_map_.insert(std::pair<std::string, osl_core::LLD>(node_name,pos));
  }
  else
  {
    it->second = pos;
  }
}

osl_core::LLD CommsNode::getPosition(std::string node_name)
{
  return node_position_map_[node_name];
}

bool CommsNode::isMessageTime(ros::Time now)
{
	if (msg_queue_.empty())
	{
		return false;
	}
  //Compare the current time with the first message time of arrival and return if it is time to receive the message.
  if(abs((msg_queue_[0].getDeliveryTime() - now).toSec()) < 0.005) //Todo: Check this number.
  {
    return true;
  }
  return false;
}

bool CommsNode::isMessageReceived()
{
  //Check if message is to be received. if not just discard the message.
  checkForMessageCollisions();
  checkForPER(msg_queue_[0]);
  return !msg_queue_[0].getErrorStatus(); //If error status is true then the message is not received.
}

void CommsNode::pushMessage(CommsMsg msg)
{
  msg_queue_.push_back(msg);
}

CommsMsg CommsNode::popMsg()
{
  //Remove the first message from the queue and return it.
  CommsMsg ret = msg_queue_.front();
  msg_queue_.pop_front();
  return ret;
}

void CommsNode::publishMessage(CommsMsg msg)
{
  in_pub_.publish(*msg.getMessage());
}

std::string CommsNode::getName()
{
  return name_;
}


void CommsNode::handleMsg()
{
  if(isMessageReceived())
  {
    publishMessage(popMsg());
  }
}
