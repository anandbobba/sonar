// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from whistle_safety_msgs:msg/AudioLevel.idl
// generated code does not contain a copyright notice

#ifndef WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__STRUCT_HPP_
#define WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'sensor_position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__whistle_safety_msgs__msg__AudioLevel __attribute__((deprecated))
#else
# define DEPRECATED__whistle_safety_msgs__msg__AudioLevel __declspec(deprecated)
#endif

namespace whistle_safety_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AudioLevel_
{
  using Type = AudioLevel_<ContainerAllocator>;

  explicit AudioLevel_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    sensor_position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->level = 0.0f;
      this->frequency = 0.0f;
    }
  }

  explicit AudioLevel_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    sensor_position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->level = 0.0f;
      this->frequency = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _level_type =
    float;
  _level_type level;
  using _frequency_type =
    float;
  _frequency_type frequency;
  using _sensor_position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _sensor_position_type sensor_position;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__level(
    const float & _arg)
  {
    this->level = _arg;
    return *this;
  }
  Type & set__frequency(
    const float & _arg)
  {
    this->frequency = _arg;
    return *this;
  }
  Type & set__sensor_position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->sensor_position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    whistle_safety_msgs::msg::AudioLevel_<ContainerAllocator> *;
  using ConstRawPtr =
    const whistle_safety_msgs::msg::AudioLevel_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<whistle_safety_msgs::msg::AudioLevel_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<whistle_safety_msgs::msg::AudioLevel_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      whistle_safety_msgs::msg::AudioLevel_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<whistle_safety_msgs::msg::AudioLevel_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      whistle_safety_msgs::msg::AudioLevel_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<whistle_safety_msgs::msg::AudioLevel_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<whistle_safety_msgs::msg::AudioLevel_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<whistle_safety_msgs::msg::AudioLevel_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__whistle_safety_msgs__msg__AudioLevel
    std::shared_ptr<whistle_safety_msgs::msg::AudioLevel_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__whistle_safety_msgs__msg__AudioLevel
    std::shared_ptr<whistle_safety_msgs::msg::AudioLevel_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AudioLevel_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->level != other.level) {
      return false;
    }
    if (this->frequency != other.frequency) {
      return false;
    }
    if (this->sensor_position != other.sensor_position) {
      return false;
    }
    return true;
  }
  bool operator!=(const AudioLevel_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AudioLevel_

// alias to use template instance with default allocator
using AudioLevel =
  whistle_safety_msgs::msg::AudioLevel_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace whistle_safety_msgs

#endif  // WHISTLE_SAFETY_MSGS__MSG__DETAIL__AUDIO_LEVEL__STRUCT_HPP_
