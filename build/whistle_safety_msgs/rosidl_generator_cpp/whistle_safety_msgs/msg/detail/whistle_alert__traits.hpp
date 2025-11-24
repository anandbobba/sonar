// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from whistle_safety_msgs:msg/WhistleAlert.idl
// generated code does not contain a copyright notice

#ifndef WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__TRAITS_HPP_
#define WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "whistle_safety_msgs/msg/detail/whistle_alert__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace whistle_safety_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const WhistleAlert & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: intensity
  {
    out << "intensity: ";
    rosidl_generator_traits::value_to_yaml(msg.intensity, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: severity
  {
    out << "severity: ";
    rosidl_generator_traits::value_to_yaml(msg.severity, out);
    out << ", ";
  }

  // member: alert_id
  {
    out << "alert_id: ";
    rosidl_generator_traits::value_to_yaml(msg.alert_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WhistleAlert & msg,
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

  // member: intensity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "intensity: ";
    rosidl_generator_traits::value_to_yaml(msg.intensity, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: severity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "severity: ";
    rosidl_generator_traits::value_to_yaml(msg.severity, out);
    out << "\n";
  }

  // member: alert_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alert_id: ";
    rosidl_generator_traits::value_to_yaml(msg.alert_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WhistleAlert & msg, bool use_flow_style = false)
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
  const whistle_safety_msgs::msg::WhistleAlert & msg,
  std::ostream & out, size_t indentation = 0)
{
  whistle_safety_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use whistle_safety_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const whistle_safety_msgs::msg::WhistleAlert & msg)
{
  return whistle_safety_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<whistle_safety_msgs::msg::WhistleAlert>()
{
  return "whistle_safety_msgs::msg::WhistleAlert";
}

template<>
inline const char * name<whistle_safety_msgs::msg::WhistleAlert>()
{
  return "whistle_safety_msgs/msg/WhistleAlert";
}

template<>
struct has_fixed_size<whistle_safety_msgs::msg::WhistleAlert>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<whistle_safety_msgs::msg::WhistleAlert>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<whistle_safety_msgs::msg::WhistleAlert>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__TRAITS_HPP_
