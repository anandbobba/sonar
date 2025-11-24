// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from whistle_safety_msgs:msg/AudioLevel.idl
// generated code does not contain a copyright notice

#ifndef WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__BUILDER_HPP_
#define WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "whistle_safety_msgs/msg/detail/audio_level__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace whistle_safety_msgs
{

namespace msg
{

namespace builder
{

class Init_AudioLevel_sensor_position
{
public:
  explicit Init_AudioLevel_sensor_position(::whistle_safety_msgs::msg::AudioLevel & msg)
  : msg_(msg)
  {}
  ::whistle_safety_msgs::msg::AudioLevel sensor_position(::whistle_safety_msgs::msg::AudioLevel::_sensor_position_type arg)
  {
    msg_.sensor_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::whistle_safety_msgs::msg::AudioLevel msg_;
};

class Init_AudioLevel_frequency
{
public:
  explicit Init_AudioLevel_frequency(::whistle_safety_msgs::msg::AudioLevel & msg)
  : msg_(msg)
  {}
  Init_AudioLevel_sensor_position frequency(::whistle_safety_msgs::msg::AudioLevel::_frequency_type arg)
  {
    msg_.frequency = std::move(arg);
    return Init_AudioLevel_sensor_position(msg_);
  }

private:
  ::whistle_safety_msgs::msg::AudioLevel msg_;
};

class Init_AudioLevel_level
{
public:
  explicit Init_AudioLevel_level(::whistle_safety_msgs::msg::AudioLevel & msg)
  : msg_(msg)
  {}
  Init_AudioLevel_frequency level(::whistle_safety_msgs::msg::AudioLevel::_level_type arg)
  {
    msg_.level = std::move(arg);
    return Init_AudioLevel_frequency(msg_);
  }

private:
  ::whistle_safety_msgs::msg::AudioLevel msg_;
};

class Init_AudioLevel_header
{
public:
  Init_AudioLevel_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AudioLevel_level header(::whistle_safety_msgs::msg::AudioLevel::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AudioLevel_level(msg_);
  }

private:
  ::whistle_safety_msgs::msg::AudioLevel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::whistle_safety_msgs::msg::AudioLevel>()
{
  return whistle_safety_msgs::msg::builder::Init_AudioLevel_header();
}

}  // namespace whistle_safety_msgs

#endif  // WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__BUILDER_HPP_
