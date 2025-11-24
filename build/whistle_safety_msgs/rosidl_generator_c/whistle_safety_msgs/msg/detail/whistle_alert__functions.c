// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from whistle_safety_msgs:msg/WhistleAlert.idl
// generated code does not contain a copyright notice
#include "whistle_safety_msgs/msg/detail/whistle_alert__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `severity`
// Member `alert_id`
#include "rosidl_runtime_c/string_functions.h"

bool
whistle_safety_msgs__msg__WhistleAlert__init(whistle_safety_msgs__msg__WhistleAlert * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    whistle_safety_msgs__msg__WhistleAlert__fini(msg);
    return false;
  }
  // intensity
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    whistle_safety_msgs__msg__WhistleAlert__fini(msg);
    return false;
  }
  // severity
  if (!rosidl_runtime_c__String__init(&msg->severity)) {
    whistle_safety_msgs__msg__WhistleAlert__fini(msg);
    return false;
  }
  // alert_id
  if (!rosidl_runtime_c__String__init(&msg->alert_id)) {
    whistle_safety_msgs__msg__WhistleAlert__fini(msg);
    return false;
  }
  return true;
}

void
whistle_safety_msgs__msg__WhistleAlert__fini(whistle_safety_msgs__msg__WhistleAlert * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // intensity
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // severity
  rosidl_runtime_c__String__fini(&msg->severity);
  // alert_id
  rosidl_runtime_c__String__fini(&msg->alert_id);
}

bool
whistle_safety_msgs__msg__WhistleAlert__are_equal(const whistle_safety_msgs__msg__WhistleAlert * lhs, const whistle_safety_msgs__msg__WhistleAlert * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // intensity
  if (lhs->intensity != rhs->intensity) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // severity
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->severity), &(rhs->severity)))
  {
    return false;
  }
  // alert_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->alert_id), &(rhs->alert_id)))
  {
    return false;
  }
  return true;
}

bool
whistle_safety_msgs__msg__WhistleAlert__copy(
  const whistle_safety_msgs__msg__WhistleAlert * input,
  whistle_safety_msgs__msg__WhistleAlert * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // intensity
  output->intensity = input->intensity;
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // severity
  if (!rosidl_runtime_c__String__copy(
      &(input->severity), &(output->severity)))
  {
    return false;
  }
  // alert_id
  if (!rosidl_runtime_c__String__copy(
      &(input->alert_id), &(output->alert_id)))
  {
    return false;
  }
  return true;
}

whistle_safety_msgs__msg__WhistleAlert *
whistle_safety_msgs__msg__WhistleAlert__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  whistle_safety_msgs__msg__WhistleAlert * msg = (whistle_safety_msgs__msg__WhistleAlert *)allocator.allocate(sizeof(whistle_safety_msgs__msg__WhistleAlert), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(whistle_safety_msgs__msg__WhistleAlert));
  bool success = whistle_safety_msgs__msg__WhistleAlert__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
whistle_safety_msgs__msg__WhistleAlert__destroy(whistle_safety_msgs__msg__WhistleAlert * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    whistle_safety_msgs__msg__WhistleAlert__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
whistle_safety_msgs__msg__WhistleAlert__Sequence__init(whistle_safety_msgs__msg__WhistleAlert__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  whistle_safety_msgs__msg__WhistleAlert * data = NULL;

  if (size) {
    data = (whistle_safety_msgs__msg__WhistleAlert *)allocator.zero_allocate(size, sizeof(whistle_safety_msgs__msg__WhistleAlert), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = whistle_safety_msgs__msg__WhistleAlert__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        whistle_safety_msgs__msg__WhistleAlert__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
whistle_safety_msgs__msg__WhistleAlert__Sequence__fini(whistle_safety_msgs__msg__WhistleAlert__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      whistle_safety_msgs__msg__WhistleAlert__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

whistle_safety_msgs__msg__WhistleAlert__Sequence *
whistle_safety_msgs__msg__WhistleAlert__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  whistle_safety_msgs__msg__WhistleAlert__Sequence * array = (whistle_safety_msgs__msg__WhistleAlert__Sequence *)allocator.allocate(sizeof(whistle_safety_msgs__msg__WhistleAlert__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = whistle_safety_msgs__msg__WhistleAlert__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
whistle_safety_msgs__msg__WhistleAlert__Sequence__destroy(whistle_safety_msgs__msg__WhistleAlert__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    whistle_safety_msgs__msg__WhistleAlert__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
whistle_safety_msgs__msg__WhistleAlert__Sequence__are_equal(const whistle_safety_msgs__msg__WhistleAlert__Sequence * lhs, const whistle_safety_msgs__msg__WhistleAlert__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!whistle_safety_msgs__msg__WhistleAlert__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
whistle_safety_msgs__msg__WhistleAlert__Sequence__copy(
  const whistle_safety_msgs__msg__WhistleAlert__Sequence * input,
  whistle_safety_msgs__msg__WhistleAlert__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(whistle_safety_msgs__msg__WhistleAlert);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    whistle_safety_msgs__msg__WhistleAlert * data =
      (whistle_safety_msgs__msg__WhistleAlert *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!whistle_safety_msgs__msg__WhistleAlert__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          whistle_safety_msgs__msg__WhistleAlert__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!whistle_safety_msgs__msg__WhistleAlert__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
