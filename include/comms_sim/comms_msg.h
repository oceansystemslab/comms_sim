/*
 * comms_msg.h
 *
 *  Created on: 19 Jan 2015
 *      Author: nick
 */

#ifndef COMMS_MSG_H_
#define COMMS_MSG_H_

#include <vehicle_interface/AcousticModemPayload.h>
#include <ros/time.h>

/*
 * Here we define a wrapper class to the acoustic modem message.
 */

class CommsMsg
{
  ros::Time transmission_time_;
  ros::Time delivery_time_;
  vehicle_interface::AcousticModemPayloadConstPtr msg_ptr_;
  std::string sender_;
  std::string receiver_;

  bool error_sts_;

public:
  CommsMsg(vehicle_interface::AcousticModemPayloadConstPtr msg_ptr, ros::Time transmission_time, ros::Time delivery_time,
           std::string sender, std::string receiver, bool status);

  bool getErrorStatus();
  void setErrorStatus(bool status);
  std::string getSender();
  std::string getReceiver();
  ros::Time getDeliveryTime();
  ros::Time getTransmissionTime();
  vehicle_interface::AcousticModemPayloadConstPtr getMessage();
};

#endif /* COMMS_MSG_H_ */
