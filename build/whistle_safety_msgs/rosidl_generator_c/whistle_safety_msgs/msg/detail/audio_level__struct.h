// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from whistle_safety_msgs:msg/AudioLevel.idl
// generated code does not contain a copyright notice

#ifndef WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__STRUCT_H_
#define WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__STRUCT_H_

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
// Member 'sensor_position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/AudioLevel in the package whistle_safety_msgs.
/**
  * AudioLevel.msg
  * Simulated microphone sensor reading
 */
typedef struct whistle_safety_msgs__msg__AudioLevel
{
  /// Header with timestamp
  std_msgs__msg__Header header;
  /// Audio intensity level (0.0 = silence, 1.0 = max)
  float level;
  /// Frequency band (Hz) - for future use
  float frequency;
  /// Sensor location on robot
  geometry_msgs__msg__Point sensor_position;
} whistle_safety_msgs__msg__AudioLevel;

// Struct for a sequence of whistle_safety_msgs__msg__AudioLevel.
typedef struct whistle_safety_msgs__msg__AudioLevel__Sequence
{
  whistle_safety_msgs__msg__AudioLevel * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} whistle_safety_msgs__msg__AudioLevel__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__STRUCT_H_
