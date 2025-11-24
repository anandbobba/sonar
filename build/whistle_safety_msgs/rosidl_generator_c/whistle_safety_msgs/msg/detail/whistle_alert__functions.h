// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from whistle_safety_msgs:msg/WhistleAlert.idl
// generated code does not contain a copyright notice

#ifndef WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__FUNCTIONS_H_
#define WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "whistle_safety_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "whistle_safety_msgs/msg/detail/whistle_alert__struct.h"

/// Initialize msg/WhistleAlert message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * whistle_safety_msgs__msg__WhistleAlert
 * )) before or use
 * whistle_safety_msgs__msg__WhistleAlert__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_whistle_safety_msgs
bool
whistle_safety_msgs__msg__WhistleAlert__init(whistle_safety_msgs__msg__WhistleAlert * msg);

/// Finalize msg/WhistleAlert message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_whistle_safety_msgs
void
whistle_safety_msgs__msg__WhistleAlert__fini(whistle_safety_msgs__msg__WhistleAlert * msg);

/// Create msg/WhistleAlert message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * whistle_safety_msgs__msg__WhistleAlert__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_whistle_safety_msgs
whistle_safety_msgs__msg__WhistleAlert *
whistle_safety_msgs__msg__WhistleAlert__create();

/// Destroy msg/WhistleAlert message.
/**
 * It calls
 * whistle_safety_msgs__msg__WhistleAlert__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_whistle_safety_msgs
void
whistle_safety_msgs__msg__WhistleAlert__destroy(whistle_safety_msgs__msg__WhistleAlert * msg);

/// Check for msg/WhistleAlert message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_whistle_safety_msgs
bool
whistle_safety_msgs__msg__WhistleAlert__are_equal(const whistle_safety_msgs__msg__WhistleAlert * lhs, const whistle_safety_msgs__msg__WhistleAlert * rhs);

/// Copy a msg/WhistleAlert message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_whistle_safety_msgs
bool
whistle_safety_msgs__msg__WhistleAlert__copy(
  const whistle_safety_msgs__msg__WhistleAlert * input,
  whistle_safety_msgs__msg__WhistleAlert * output);

/// Initialize array of msg/WhistleAlert messages.
/**
 * It allocates the memory for the number of elements and calls
 * whistle_safety_msgs__msg__WhistleAlert__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_whistle_safety_msgs
bool
whistle_safety_msgs__msg__WhistleAlert__Sequence__init(whistle_safety_msgs__msg__WhistleAlert__Sequence * array, size_t size);

/// Finalize array of msg/WhistleAlert messages.
/**
 * It calls
 * whistle_safety_msgs__msg__WhistleAlert__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_whistle_safety_msgs
void
whistle_safety_msgs__msg__WhistleAlert__Sequence__fini(whistle_safety_msgs__msg__WhistleAlert__Sequence * array);

/// Create array of msg/WhistleAlert messages.
/**
 * It allocates the memory for the array and calls
 * whistle_safety_msgs__msg__WhistleAlert__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_whistle_safety_msgs
whistle_safety_msgs__msg__WhistleAlert__Sequence *
whistle_safety_msgs__msg__WhistleAlert__Sequence__create(size_t size);

/// Destroy array of msg/WhistleAlert messages.
/**
 * It calls
 * whistle_safety_msgs__msg__WhistleAlert__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_whistle_safety_msgs
void
whistle_safety_msgs__msg__WhistleAlert__Sequence__destroy(whistle_safety_msgs__msg__WhistleAlert__Sequence * array);

/// Check for msg/WhistleAlert message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_whistle_safety_msgs
bool
whistle_safety_msgs__msg__WhistleAlert__Sequence__are_equal(const whistle_safety_msgs__msg__WhistleAlert__Sequence * lhs, const whistle_safety_msgs__msg__WhistleAlert__Sequence * rhs);

/// Copy an array of msg/WhistleAlert messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_whistle_safety_msgs
bool
whistle_safety_msgs__msg__WhistleAlert__Sequence__copy(
  const whistle_safety_msgs__msg__WhistleAlert__Sequence * input,
  whistle_safety_msgs__msg__WhistleAlert__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // WHISTLE_SAFETY_MSGS__MSG__DETAIL__WHISTLE_ALERT__FUNCTIONS_H_
