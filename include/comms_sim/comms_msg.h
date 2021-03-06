/*
 * comms_msg.h
 *
 *  Created on: 19 Jan 2015
 *      Author: nick
 */

#ifndef COMMS_MSG_H_
#define COMMS_MSG_H_

#include <vehicle_interface/AcousticModemPayload.h>
#include <vehicle_interface/AcousticModemAck.h>
#include <ros/time.h>

/*
 * Here we define a wrapper class to the acoustic modem message.
 */

class CommsMsg
{
  ros::Time transmission_time_;
  ros::Time delivery_time_;
  vehicle_interface::AcousticModemPayloadPtr msg_ptr_;
  vehicle_interface::AcousticModemAckPtr ack_msg_ptr_;
  std::string sender_;
  std::string receiver_;
  std::string type_;

  bool error_sts_;

public:
  CommsMsg(vehicle_interface::AcousticModemPayloadPtr msg_ptr, ros::Time transmission_time,
           ros::Time delivery_time, std::string sender, std::string receiver, bool status, std::string type);

  CommsMsg(vehicle_interface::AcousticModemAckPtr msg_ptr, ros::Time transmission_time,
             ros::Time delivery_time, std::string sender, std::string receiver, bool status, std::string type);

  CommsMsg()
  {
  }

  bool getErrorStatus();
  void setErrorStatus(bool status);
  std::string getSender();
  std::string getReceiver();
  std::string getType();
  ros::Time getDeliveryTime();
  ros::Time getTransmissionTime();
  vehicle_interface::AcousticModemPayloadPtr getMessage();
  vehicle_interface::AcousticModemAckPtr getAck();
};

#endif /* COMMS_MSG_H_ */
