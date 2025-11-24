// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from whistle_safety_msgs:msg/WhistleAlert.idl
// generated code does not contain a copyright notice

#ifndef WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__STRUCT_H_
#define WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'severity'
// Member 'alert_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/WhistleAlert in the package whistle_safety_msgs.
/**
  * WhistleAlert.msg
  * Message type for whistle detection events
 */
typedef struct whistle_safety_msgs__msg__WhistleAlert
{
  /// Header with timestamp
  std_msgs__msg__Header header;
  /// Detection confidence (0.0 to 1.0)
  float intensity;
  /// Robot position when whistle detected
  geometry_msgs__msg__Point position;
  /// Alert severity level
  /// "low", "medium", "high", "critical"
  rosidl_runtime_c__String severity;
  /// Unique alert ID
  rosidl_runtime_c__String alert_id;
} whistle_safety_msgs__msg__WhistleAlert;

// Struct for a sequence of whistle_safety_msgs__msg__WhistleAlert.
typedef struct whistle_safety_msgs__msg__WhistleAlert__Sequence
{
  whistle_safety_msgs__msg__WhistleAlert * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} whistle_safety_msgs__msg__WhistleAlert__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__STRUCT_H_
