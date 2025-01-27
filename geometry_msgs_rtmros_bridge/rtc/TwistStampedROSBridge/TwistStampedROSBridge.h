#ifndef TwistStampedROSBridge_H
#define TwistStampedROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

class TwistStampedROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh; // これがないとうまく通信できなくなったり、CPU使用率100%になったりする

  RTC::TimedVelocity3D m_twistRTM_;
  RTC::InPort<RTC::TimedVelocity3D> m_twistIn_;
  ros::Publisher pub_;

  RTC::TimedVelocity3D m_twistROS_;
  RTC::OutPort<RTC::TimedVelocity3D> m_twistOut_;
  ros::Subscriber sub_;

  std::string frame_id_;

  void topicCb(geometry_msgs::TwistStamped::ConstPtr msg);

public:
  TwistStampedROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

};


extern "C"
{
  void TwistStampedROSBridgeInit(RTC::Manager* manager);
};

#endif // TwistStampedROSBridge_H
