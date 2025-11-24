// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from whistle_safety_msgs:msg/WhistleAlert.idl
// generated code does not contain a copyright notice

#ifndef WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__BUILDER_HPP_
#define WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "whistle_safety_msgs/msg/detail/whistle_alert__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace whistle_safety_msgs
{

namespace msg
{

namespace builder
{

class Init_WhistleAlert_alert_id
{
public:
  explicit Init_WhistleAlert_alert_id(::whistle_safety_msgs::msg::WhistleAlert & msg)
  : msg_(msg)
  {}
  ::whistle_safety_msgs::msg::WhistleAlert alert_id(::whistle_safety_msgs::msg::WhistleAlert::_alert_id_type arg)
  {
    msg_.alert_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::whistle_safety_msgs::msg::WhistleAlert msg_;
};

class Init_WhistleAlert_severity
{
public:
  explicit Init_WhistleAlert_severity(::whistle_safety_msgs::msg::WhistleAlert & msg)
  : msg_(msg)
  {}
  Init_WhistleAlert_alert_id severity(::whistle_safety_msgs::msg::WhistleAlert::_severity_type arg)
  {
    msg_.severity = std::move(arg);
    return Init_WhistleAlert_alert_id(msg_);
  }

private:
  ::whistle_safety_msgs::msg::WhistleAlert msg_;
};

class Init_WhistleAlert_position
{
public:
  explicit Init_WhistleAlert_position(::whistle_safety_msgs::msg::WhistleAlert & msg)
  : msg_(msg)
  {}
  Init_WhistleAlert_severity position(::whistle_safety_msgs::msg::WhistleAlert::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_WhistleAlert_severity(msg_);
  }

private:
  ::whistle_safety_msgs::msg::WhistleAlert msg_;
};

class Init_WhistleAlert_intensity
{
public:
  explicit Init_WhistleAlert_intensity(::whistle_safety_msgs::msg::WhistleAlert & msg)
  : msg_(msg)
  {}
  Init_WhistleAlert_position intensity(::whistle_safety_msgs::msg::WhistleAlert::_intensity_type arg)
  {
    msg_.intensity = std::move(arg);
    return Init_WhistleAlert_position(msg_);
  }

private:
  ::whistle_safety_msgs::msg::WhistleAlert msg_;
};

class Init_WhistleAlert_header
{
public:
  Init_WhistleAlert_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WhistleAlert_intensity header(::whistle_safety_msgs::msg::WhistleAlert::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WhistleAlert_intensity(msg_);
  }

private:
  ::whistle_safety_msgs::msg::WhistleAlert msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::whistle_safety_msgs::msg::WhistleAlert>()
{
  return whistle_safety_msgs::msg::builder::Init_WhistleAlert_header();
}

}  // namespace whistle_safety_msgs

#endif  // WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__BUILDER_HPP_
