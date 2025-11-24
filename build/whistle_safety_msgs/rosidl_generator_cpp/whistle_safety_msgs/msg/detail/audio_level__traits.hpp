// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from whistle_safety_msgs:msg/AudioLevel.idl
// generated code does not contain a copyright notice

#ifndef WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__TRAITS_HPP_
#define WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "whistle_safety_msgs/msg/detail/audio_level__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'sensor_position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace whistle_safety_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AudioLevel & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: level
  {
    out << "level: ";
    rosidl_generator_traits::value_to_yaml(msg.level, out);
    out << ", ";
  }

  // member: frequency
  {
    out << "frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.frequency, out);
    out << ", ";
  }

  // member: sensor_position
  {
    out << "sensor_position: ";
    to_flow_style_yaml(msg.sensor_position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AudioLevel & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "level: ";
    rosidl_generator_traits::value_to_yaml(msg.level, out);
    out << "\n";
  }

  // member: frequency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.frequency, out);
    out << "\n";
  }

  // member: sensor_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor_position:\n";
    to_block_style_yaml(msg.sensor_position, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AudioLevel & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace whistle_safety_msgs

namespace rosidl_generator_traits
{

[[deprecated("use whistle_safety_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const whistle_safety_msgs::msg::AudioLevel & msg,
  std::ostream & out, size_t indentation = 0)
{
  whistle_safety_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use whistle_safety_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const whistle_safety_msgs::msg::AudioLevel & msg)
{
  return whistle_safety_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<whistle_safety_msgs::msg::AudioLevel>()
{
  return "whistle_safety_msgs::msg::AudioLevel";
}

template<>
inline const char * name<whistle_safety_msgs::msg::AudioLevel>()
{
  return "whistle_safety_msgs/msg/AudioLevel";
}

template<>
struct has_fixed_size<whistle_safety_msgs::msg::AudioLevel>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<whistle_safety_msgs::msg::AudioLevel>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<whistle_safety_msgs::msg::AudioLevel>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__TRAITS_HPP_
