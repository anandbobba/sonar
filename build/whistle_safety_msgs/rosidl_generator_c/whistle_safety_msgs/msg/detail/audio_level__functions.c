// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from whistle_safety_msgs:msg/AudioLevel.idl
// generated code does not contain a copyright notice
#include "whistle_safety_msgs/msg/detail/audio_level__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `sensor_position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
whistle_safety_msgs__msg__AudioLevel__init(whistle_safety_msgs__msg__AudioLevel * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    whistle_safety_msgs__msg__AudioLevel__fini(msg);
    return false;
  }
  // level
  // frequency
  // sensor_position
  if (!geometry_msgs__msg__Point__init(&msg->sensor_position)) {
    whistle_safety_msgs__msg__AudioLevel__fini(msg);
    return false;
  }
  return true;
}

void
whistle_safety_msgs__msg__AudioLevel__fini(whistle_safety_msgs__msg__AudioLevel * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // level
  // frequency
  // sensor_position
  geometry_msgs__msg__Point__fini(&msg->sensor_position);
}

bool
whistle_safety_msgs__msg__AudioLevel__are_equal(const whistle_safety_msgs__msg__AudioLevel * lhs, const whistle_safety_msgs__msg__AudioLevel * rhs)
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
  // level
  if (lhs->level != rhs->level) {
    return false;
  }
  // frequency
  if (lhs->frequency != rhs->frequency) {
    return false;
  }
  // sensor_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->sensor_position), &(rhs->sensor_position)))
  {
    return false;
  }
  return true;
}

bool
whistle_safety_msgs__msg__AudioLevel__copy(
  const whistle_safety_msgs__msg__AudioLevel * input,
  whistle_safety_msgs__msg__AudioLevel * output)
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
  // level
  output->level = input->level;
  // frequency
  output->frequency = input->frequency;
  // sensor_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->sensor_position), &(output->sensor_position)))
  {
    return false;
  }
  return true;
}

whistle_safety_msgs__msg__AudioLevel *
whistle_safety_msgs__msg__AudioLevel__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  whistle_safety_msgs__msg__AudioLevel * msg = (whistle_safety_msgs__msg__AudioLevel *)allocator.allocate(sizeof(whistle_safety_msgs__msg__AudioLevel), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(whistle_safety_msgs__msg__AudioLevel));
  bool success = whistle_safety_msgs__msg__AudioLevel__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
whistle_safety_msgs__msg__AudioLevel__destroy(whistle_safety_msgs__msg__AudioLevel * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    whistle_safety_msgs__msg__AudioLevel__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
whistle_safety_msgs__msg__AudioLevel__Sequence__init(whistle_safety_msgs__msg__AudioLevel__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  whistle_safety_msgs__msg__AudioLevel * data = NULL;

  if (size) {
    data = (whistle_safety_msgs__msg__AudioLevel *)allocator.zero_allocate(size, sizeof(whistle_safety_msgs__msg__AudioLevel), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = whistle_safety_msgs__msg__AudioLevel__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        whistle_safety_msgs__msg__AudioLevel__fini(&data[i - 1]);
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
whistle_safety_msgs__msg__AudioLevel__Sequence__fini(whistle_safety_msgs__msg__AudioLevel__Sequence * array)
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
      whistle_safety_msgs__msg__AudioLevel__fini(&array->data[i]);
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

whistle_safety_msgs__msg__AudioLevel__Sequence *
whistle_safety_msgs__msg__AudioLevel__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  whistle_safety_msgs__msg__AudioLevel__Sequence * array = (whistle_safety_msgs__msg__AudioLevel__Sequence *)allocator.allocate(sizeof(whistle_safety_msgs__msg__AudioLevel__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = whistle_safety_msgs__msg__AudioLevel__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
whistle_safety_msgs__msg__AudioLevel__Sequence__destroy(whistle_safety_msgs__msg__AudioLevel__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    whistle_safety_msgs__msg__AudioLevel__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
whistle_safety_msgs__msg__AudioLevel__Sequence__are_equal(const whistle_safety_msgs__msg__AudioLevel__Sequence * lhs, const whistle_safety_msgs__msg__AudioLevel__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!whistle_safety_msgs__msg__AudioLevel__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
whistle_safety_msgs__msg__AudioLevel__Sequence__copy(
  const whistle_safety_msgs__msg__AudioLevel__Sequence * input,
  whistle_safety_msgs__msg__AudioLevel__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(whistle_safety_msgs__msg__AudioLevel);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    whistle_safety_msgs__msg__AudioLevel * data =
      (whistle_safety_msgs__msg__AudioLevel *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!whistle_safety_msgs__msg__AudioLevel__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          whistle_safety_msgs__msg__AudioLevel__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!whistle_safety_msgs__msg__AudioLevel__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
