/*
 * comms_msg.cpp
 *
 *  Created on: 20 Jan 2015
 *      Author: nick
 */

#include <comms_sim/comms_msg.h>

CommsMsg::CommsMsg(vehicle_interface::AcousticModemPayloadConstPtr msg_ptr, ros::Time transmission_time,
                   ros::Time delivery_time, std::string sender, std::string receiver, bool status, std::string type)
	: msg_ptr_(msg_ptr), transmission_time_(transmission_time), delivery_time_(delivery_time),
    sender_(sender), receiver_(receiver), error_sts_(status), type_(type)
{
}


bool CommsMsg::getErrorStatus()
{
  return error_sts_;
}

void CommsMsg::setErrorStatus(bool status)
{
  error_sts_ = status;
}

std::string CommsMsg::getSender()
{
  return sender_;
}

std::string CommsMsg::getReceiver()
{
  return receiver_;
}

std::string CommsMsg::getType()
{
  return type_;
}

ros::Time CommsMsg::getDeliveryTime()
{
  return delivery_time_;
}

ros::Time CommsMsg::getTransmissionTime()
{
  return transmission_time_;
}

vehicle_interface::AcousticModemPayloadConstPtr CommsMsg::getMessage()
{
  return msg_ptr_;
}
