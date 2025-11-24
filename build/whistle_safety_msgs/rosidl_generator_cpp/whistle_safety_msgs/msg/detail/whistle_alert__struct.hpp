// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from whistle_safety_msgs:msg/WhistleAlert.idl
// generated code does not contain a copyright notice

#ifndef WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__STRUCT_HPP_
#define WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__STRUCT_HPP_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__whistle_safety_msgs__msg__WhistleAlert __attribute__((deprecated))
#else
# define DEPRECATED__whistle_safety_msgs__msg__WhistleAlert __declspec(deprecated)
#endif

namespace whistle_safety_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WhistleAlert_
{
  using Type = WhistleAlert_<ContainerAllocator>;

  explicit WhistleAlert_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->intensity = 0.0f;
      this->severity = "";
      this->alert_id = "";
    }
  }

  explicit WhistleAlert_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    severity(_alloc),
    alert_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->intensity = 0.0f;
      this->severity = "";
      this->alert_id = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _intensity_type =
    float;
  _intensity_type intensity;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _severity_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _severity_type severity;
  using _alert_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _alert_id_type alert_id;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__intensity(
    const float & _arg)
  {
    this->intensity = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__severity(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->severity = _arg;
    return *this;
  }
  Type & set__alert_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->alert_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    whistle_safety_msgs::msg::WhistleAlert_<ContainerAllocator> *;
  using ConstRawPtr =
    const whistle_safety_msgs::msg::WhistleAlert_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<whistle_safety_msgs::msg::WhistleAlert_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<whistle_safety_msgs::msg::WhistleAlert_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      whistle_safety_msgs::msg::WhistleAlert_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<whistle_safety_msgs::msg::WhistleAlert_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      whistle_safety_msgs::msg::WhistleAlert_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<whistle_safety_msgs::msg::WhistleAlert_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<whistle_safety_msgs::msg::WhistleAlert_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<whistle_safety_msgs::msg::WhistleAlert_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__whistle_safety_msgs__msg__WhistleAlert
    std::shared_ptr<whistle_safety_msgs::msg::WhistleAlert_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__whistle_safety_msgs__msg__WhistleAlert
    std::shared_ptr<whistle_safety_msgs::msg::WhistleAlert_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WhistleAlert_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->intensity != other.intensity) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->severity != other.severity) {
      return false;
    }
    if (this->alert_id != other.alert_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const WhistleAlert_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WhistleAlert_

// alias to use template instance with default allocator
using WhistleAlert =
  whistle_safety_msgs::msg::WhistleAlert_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace whistle_safety_msgs

#endif  // WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__STRUCT_HPP_
