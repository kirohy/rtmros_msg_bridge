#include "TwistStampedROSBridge.h"

TwistStampedROSBridge::TwistStampedROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_twistIn_("twistIn", m_twistRTM_),
  m_twistOut_("twistOut", m_twistROS_)
{
}

RTC::ReturnCode_t TwistStampedROSBridge::onInitialize(){
  addInPort("twistIn", this->m_twistIn_);
  addOutPort("twistOut", this->m_twistOut_);

  ros::NodeHandle pnh("~");
  pnh.param("frame_id", frame_id_, std::string(""));
  sub_ = pnh.subscribe("input", 1, &TwistStampedROSBridge::topicCb, this);
  pub_ = pnh.advertise<geometry_msgs::TwistStamped>("output", 1);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t TwistStampedROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_twistIn_.isNew()){
    this->m_twistIn_.read();
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = this->frame_id_;
    msg.twist.linear.x = this->m_twistRTM_.data.vx;
    msg.twist.linear.y = this->m_twistRTM_.data.vy;
    msg.twist.linear.z = this->m_twistRTM_.data.vz;
    msg.twist.angular.x = this->m_twistRTM_.data.vr;
    msg.twist.angular.y = this->m_twistRTM_.data.vp;
    msg.twist.angular.z = this->m_twistRTM_.data.va;
    this->pub_.publish(msg);
  }
  return RTC::RTC_OK;
}

void TwistStampedROSBridge::topicCb(geometry_msgs::TwistStamped::ConstPtr msg){
  m_twistROS_.tm.sec = msg->header.stamp.sec;
  m_twistROS_.tm.nsec = msg->header.stamp.nsec;
  m_twistROS_.data.vx = msg->twist.linear.x;
  m_twistROS_.data.vy = msg->twist.linear.y;
  m_twistROS_.data.vz = msg->twist.linear.z;
  m_twistROS_.data.vr = msg->twist.angular.x;
  m_twistROS_.data.vp = msg->twist.angular.y;
  m_twistROS_.data.va = msg->twist.angular.z;
  m_twistOut_.write();
}

static const char* TwistStampedROSBridge_spec[] = {
  "implementation_id", "TwistStampedROSBridge",
  "type_name",         "TwistStampedROSBridge",
  "description",       "TwistStampedROSBridge component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void TwistStampedROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(TwistStampedROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<TwistStampedROSBridge>, RTC::Delete<TwistStampedROSBridge>);
    }
};
