/*
 * comms_sim.cpp
 *
 *  Created on: 22 Jan 2015
 *      Author: nick
 * TEST COMMENT
 */

#include <comms_sim/comms_sim.h>
#include <boost/bind.hpp>

double findDistance(osl_core::T_LLD a, osl_core::T_LLD b)
{
  int Rk = 6373000;  // mean radius of the earth (m) at 39 degrees from the equator
  //Convert to radians
  double alat = a.lat * (M_PI / 180);
  double alon = a.lon * (M_PI / 180);
  double blat = b.lat * (M_PI / 180);
  double blon = b.lon * (M_PI / 180);
  //Calculate distance between points in meters.
  double dlon = blon - alon;
  double dlat = blat - alat;
  double a1 = pow((sin(dlat / 2)), 2) + cos(alat) * cos(blat) * pow((sin(dlon / 2)), 2);
  double c = 2 * atan2(sqrt(a1), sqrt(1 - a1));
  double distance = Rk * c;

  double height = a.depth - b.depth;
  distance = pow(distance, 2) + pow(height, 2);
  return sqrt(distance);
}

void CommsSim::addCommsNode(std::string node_name, int id)
{
  // Create all the infrastructure for the comms node (Publishers, subscribers, class etc.)
  node_list_.push_back(CommsNode(node_name, id, per_type_, per_, collision_window_, nhp_));
  std::stringstream topic;
  topic << "/" << node_name << "/modem/burst/out";
  const boost::function<void(const boost::shared_ptr<vehicle_interface::AcousticModemPayload const> &)> cb =
      boost::bind(&CommsSim::modemOutBurstCB, this, _1, node_name);
  modem_burst_sub_v_.push_back(nhp_->subscribe<vehicle_interface::AcousticModemPayload>(topic.str(), 1, cb));
  topic.str("");

  topic << "/" << node_name << "/modem/im/out";
  const boost::function<void(const boost::shared_ptr<vehicle_interface::AcousticModemPayload const> &)> cb2 =
      boost::bind(&CommsSim::modemOutIMCB, this, _1, node_name);
  modem_im_sub_v_.push_back(nhp_->subscribe<vehicle_interface::AcousticModemPayload>(topic.str(), 1, cb2));
  topic.str("");

  topic << "/" << node_name << "/nav/nav_sts";
  const boost::function<void(const boost::shared_ptr<auv_msgs::NavSts const> &)> cb3 = boost::bind(
      &CommsSim::navStsOutCB, this, _1, node_name);
  nav_sub_v_.push_back(nhp_->subscribe<auv_msgs::NavSts>(topic.str(), 1, cb3));
  topic.str("");

  topic << "/" << node_name << "/modem/burst/ack";
  burst_ack_pub_m_.insert(
      std::pair<std::string, ros::Publisher>(node_name,
                                             nhp_->advertise<vehicle_interface::AcousticModemAck>(topic.str(), 10)));
  topic.str("");

  topic << "/" << node_name << "/modem/im/ack";
  im_ack_pub_m_.insert(
      std::pair<std::string, ros::Publisher>(node_name,
                                             nhp_->advertise<vehicle_interface::AcousticModemAck>(topic.str(), 10)));
  topic.str("");

  std::cout << "added node " << node_name << std::endl;
}

void CommsSim::modemOutBurstCB(const vehicle_interface::AcousticModemPayload::ConstPtr &msg, std::string node_name)
{
  //Here we receive a message from the node with node_name. We should put the message to all the other nodes' queues.
  ROS_INFO_STREAM("Ack: " << msg->ack << " address: " << msg->address << " id: " << msg->msg_id);
  std::vector<CommsNode>::iterator it;
  vehicle_interface::AcousticModemPayloadPtr payload_msg(new vehicle_interface::AcousticModemPayload);
  payload_msg->header = msg->header;
  payload_msg->ack = msg->ack;
  payload_msg->address = msg->address;
  payload_msg->msg_id = msg->msg_id;
  payload_msg->payload = msg->payload;
  for (it = node_list_.begin(); it != node_list_.end(); it++)
  {
    //Handle incoming message.
    if (it->getName() != node_name) //We don't want to hear ourselves.
    {
      ros::Time transmission_time = ros::Time::now();
      ros::Time delivery_time = calculateDeliveryTime(node_name, it->getName(), transmission_time);
      it->pushMessage(
          CommsMsg(payload_msg, transmission_time, delivery_time, node_name, it->getName(), false, "Burst"));
    }
  }
}

void CommsSim::modemOutIMCB(const vehicle_interface::AcousticModemPayloadConstPtr& msg, std::string node_name)
{
  //Here we receive a message from the node with node_name. We should put the message to all the other nodes' queues.
  ROS_ERROR_STREAM("Node: " << node_name << " published a message at: " << ros::Time::now());
  ROS_INFO_STREAM("Ack: " << msg->ack << " address: " << msg->address << " id: " << msg->msg_id);
  std::vector<CommsNode>::iterator it;
  vehicle_interface::AcousticModemPayloadPtr payload_msg(new vehicle_interface::AcousticModemPayload());
  payload_msg->header = msg->header;
  payload_msg->ack = msg->ack;
  payload_msg->address = msg->address;
  payload_msg->msg_id = msg->msg_id;
  payload_msg->payload = msg->payload;
  ROS_INFO_STREAM("Ack: " << payload_msg->ack << " address: " << payload_msg->address << " id: " << payload_msg->msg_id);
  for (it = node_list_.begin(); it != node_list_.end(); it++)
  {
    //Handle incoming message.
    if (it->getName() != node_name) //We don't want to hear ourselves.
    {
      ros::Time transmission_time = ros::Time::now();
      ros::Time delivery_time = calculateDeliveryTime(node_name, it->getName(), transmission_time);
      ROS_ERROR_STREAM("Node: " << it->getName() << " will get the message at: " << delivery_time);
      it->pushMessage(CommsMsg(payload_msg, transmission_time, delivery_time, node_name, it->getName(), false, "IM"));
    }
  }
}

void CommsSim::navStsOutCB(const auv_msgs::NavSts::ConstPtr &msg, std::string node_name)
{
  //Here we receive position update from the node with node_name. We should update it in the appropriate map.
  osl_core::LLD pos;
  pos.lat = msg->global_position.latitude;
  pos.lon = msg->global_position.longitude;
  pos.depth = msg->altitude;
  CommsNode::updatePositionMap(node_name, pos);
}

bool CommsSim::isAckReceived()
{
  //Check if the ack arrives correctly.
  double rndNum = zeroone();
  if (per_type_ == 0)
  {
    if (rndNum < per_)
    {
      return false;
    }
    return true;
  }
  // TODO: Implement other per types (i.e. distance based).
}

void CommsSim::publishAckMsg(CommsMsg msg, bool ackReceived)
{
  vehicle_interface::AcousticModemAck ack;
  ack.ack = ackReceived;
  ack.msg_id = msg.getMessage()->msg_id;
  if (msg.getType() == "Burst")
    burst_ack_pub_m_[msg.getSender()].publish(ack);
  else if (msg.getType() == "IM")
    im_ack_pub_m_[msg.getSender()].publish(ack);
  else
    ROS_ERROR_STREAM("Could not recognise a valid message type");
}

ros::Time CommsSim::calculateDeliveryTime(std::string n1, std::string n2, ros::Time transmission_time)
{
  if (use_fixed_flight_time_)
  {
    return transmission_time + ros::Duration(flight_time_/1000);
  }
  else
  {
    double distance = findDistance(CommsNode::getPosition(n1), CommsNode::getPosition(n2));
    return transmission_time + ros::Duration(distance / medium_speed_);
  }
}

vehicle_interface::AcousticModemAckPtr CommsSim::generateAckMsg(unsigned int msg_id, bool ack)
{
  vehicle_interface::AcousticModemAckPtr msg(new vehicle_interface::AcousticModemAck());
  msg->ack = ack;
  msg->msg_id = msg_id;
  ROS_INFO_STREAM("ACK_MSG " << ack << " : " << msg->ack);
  return msg;
}

CommsSim::CommsSim(ros::NodeHandlePtr nhp) :
    zeroone(rng)
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

  std::string ids;

  if (!nhp_->getParam("sim/platform_modem_ids", ids))
  {
    ROS_ERROR_STREAM("Didn't find platform modem ids. Exiting...");
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

  std::stringstream ss(ids);
  int n;
  while (ss >> n)
  {
    platform_ids_.push_back(n);
  }

  //Here we create each of the comms_node instances
  boost::split(platform_names_, nodes, boost::is_any_of("\t "));

  std::vector<std::string>::iterator it;
  int index = 0;
  int id;
  for (it = platform_names_.begin(); it != platform_names_.end(); it++)
  {
    //Create all the classes here.
    try
    {
      id = platform_ids_.at(index);
    }
    catch (const std::out_of_range& oor)
    {
      std::cerr << "Out of Range error: " << oor.what() << '\n';
      return false;
    }
    addCommsNode(*it, id);
    index++;
  }

  return true;
}

void CommsSim::doWork()
{
  /*
   * Main loop of the node.
   * Check for each node if any message is received and process it accordingly.
   * Loop around the comms nodes, check if it is time to receive a message, and decide if you receive it.
   */
  //ROS_ERROR_STREAM_THROTTLE(0.1,"Doing work...");
  //std::cout << "doWork(), list size = " << node_list_.size() << std::endl;
  ros::spinOnce();
  std::vector<CommsNode>::iterator it;
  for (it = node_list_.begin(); it != node_list_.end(); it++)
  {
    if (it->isMessageTime(ros::Time::now()))
    {
      CommsMsg msg;
      bool received;
      received = it->handleMsg(msg);
      ROS_INFO_STREAM("Message: " << msg.getMessage()->msg_id << " received: " << received << " from node: " << msg.getMessage()->address);
      if (!(msg.getType() == "BurstAck" || msg.getType() == "IMAck")) //We don't ack other acks
      {
        if (msg.getMessage()->ack == true)
        {
          if (msg.getMessage()->address != 255) //Broadcast messages won't be acked even if they request an ack.
          {
            vehicle_interface::AcousticModemAckPtr ack_msg;// = generateAckMsg(msg.getMessage()->msg_id, received);
            ack_msg->ack = received;
            ack_msg->msg_id = msg.getMessage()->msg_id;
            ros::Time transmission_time = ros::Time::now();
            ros::Time delivery_time = calculateDeliveryTime(msg.getSender(), msg.getReceiver(), transmission_time);
            ROS_INFO_STREAM("Generating ack for: " << msg.getSender() << " with transmission time: " << transmission_time);
            ROS_INFO_STREAM(ack_msg->ack);
            std::vector<CommsNode>::iterator it_ack;
            for (it_ack = node_list_.begin(); it_ack != node_list_.end(); it_ack++)
            {
              if (it_ack->getName() == msg.getSender()) //We publish the ack to the sender of the message
              {
                if (msg.getType() == "Burst") {
                  CommsMsg ack(ack_msg, transmission_time, delivery_time, msg.getReceiver(), msg.getSender(), false,
                               "BurstAck");
                  it_ack->pushMessage(ack);
                }
                  /*it_ack->pushMessage(
                      CommsMsg(ack_msg, transmission_time, delivery_time, msg.getReceiver(), msg.getSender(), false,
                               "BurstAck"));*/
                else if (msg.getType() == "IM") {
                  CommsMsg ack(ack_msg, transmission_time, delivery_time, msg.getReceiver(), msg.getSender(), false,
                               "IMAck");
                  it_ack->pushMessage(ack);
                }
                  /*it_ack->pushMessage(
                      CommsMsg(ack_msg, transmission_time, delivery_time, msg.getReceiver(), msg.getSender(), false,
                               "IMAck"));*/
                break;
              }
            }
          }
        }
      }
    }
  }
}
