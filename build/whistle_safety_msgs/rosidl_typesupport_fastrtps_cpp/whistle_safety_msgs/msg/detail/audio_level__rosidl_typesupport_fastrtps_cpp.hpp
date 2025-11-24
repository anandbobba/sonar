// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from whistle_safety_msgs:msg/AudioLevel.idl
// generated code does not contain a copyright notice

#ifndef WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "whistle_safety_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "whistle_safety_msgs/msg/detail/audio_level__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace whistle_safety_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_whistle_safety_msgs
cdr_serialize(
  const whistle_safety_msgs::msg::AudioLevel & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_whistle_safety_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  whistle_safety_msgs::msg::AudioLevel & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_whistle_safety_msgs
get_serialized_size(
  const whistle_safety_msgs::msg::AudioLevel & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_whistle_safety_msgs
max_serialized_size_AudioLevel(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace whistle_safety_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_whistle_safety_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, whistle_safety_msgs, msg, AudioLevel)();

#ifdef __cplusplus
}
#endif

#endif  // WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
