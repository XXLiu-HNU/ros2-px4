// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uav_interfaces:srv/UavControl.idl
// generated code does not contain a copyright notice
#include "uav_interfaces/srv/detail/uav_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
uav_interfaces__srv__UavControl_Request__init(uav_interfaces__srv__UavControl_Request * msg)
{
  if (!msg) {
    return false;
  }
  // takeoff_land
  // x_deta
  // y_deta
  // z_deta
  return true;
}

void
uav_interfaces__srv__UavControl_Request__fini(uav_interfaces__srv__UavControl_Request * msg)
{
  if (!msg) {
    return;
  }
  // takeoff_land
  // x_deta
  // y_deta
  // z_deta
}

bool
uav_interfaces__srv__UavControl_Request__are_equal(const uav_interfaces__srv__UavControl_Request * lhs, const uav_interfaces__srv__UavControl_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // takeoff_land
  if (lhs->takeoff_land != rhs->takeoff_land) {
    return false;
  }
  // x_deta
  if (lhs->x_deta != rhs->x_deta) {
    return false;
  }
  // y_deta
  if (lhs->y_deta != rhs->y_deta) {
    return false;
  }
  // z_deta
  if (lhs->z_deta != rhs->z_deta) {
    return false;
  }
  return true;
}

bool
uav_interfaces__srv__UavControl_Request__copy(
  const uav_interfaces__srv__UavControl_Request * input,
  uav_interfaces__srv__UavControl_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // takeoff_land
  output->takeoff_land = input->takeoff_land;
  // x_deta
  output->x_deta = input->x_deta;
  // y_deta
  output->y_deta = input->y_deta;
  // z_deta
  output->z_deta = input->z_deta;
  return true;
}

uav_interfaces__srv__UavControl_Request *
uav_interfaces__srv__UavControl_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_interfaces__srv__UavControl_Request * msg = (uav_interfaces__srv__UavControl_Request *)allocator.allocate(sizeof(uav_interfaces__srv__UavControl_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uav_interfaces__srv__UavControl_Request));
  bool success = uav_interfaces__srv__UavControl_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uav_interfaces__srv__UavControl_Request__destroy(uav_interfaces__srv__UavControl_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uav_interfaces__srv__UavControl_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uav_interfaces__srv__UavControl_Request__Sequence__init(uav_interfaces__srv__UavControl_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_interfaces__srv__UavControl_Request * data = NULL;

  if (size) {
    data = (uav_interfaces__srv__UavControl_Request *)allocator.zero_allocate(size, sizeof(uav_interfaces__srv__UavControl_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uav_interfaces__srv__UavControl_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uav_interfaces__srv__UavControl_Request__fini(&data[i - 1]);
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
uav_interfaces__srv__UavControl_Request__Sequence__fini(uav_interfaces__srv__UavControl_Request__Sequence * array)
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
      uav_interfaces__srv__UavControl_Request__fini(&array->data[i]);
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

uav_interfaces__srv__UavControl_Request__Sequence *
uav_interfaces__srv__UavControl_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_interfaces__srv__UavControl_Request__Sequence * array = (uav_interfaces__srv__UavControl_Request__Sequence *)allocator.allocate(sizeof(uav_interfaces__srv__UavControl_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uav_interfaces__srv__UavControl_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uav_interfaces__srv__UavControl_Request__Sequence__destroy(uav_interfaces__srv__UavControl_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uav_interfaces__srv__UavControl_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uav_interfaces__srv__UavControl_Request__Sequence__are_equal(const uav_interfaces__srv__UavControl_Request__Sequence * lhs, const uav_interfaces__srv__UavControl_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uav_interfaces__srv__UavControl_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uav_interfaces__srv__UavControl_Request__Sequence__copy(
  const uav_interfaces__srv__UavControl_Request__Sequence * input,
  uav_interfaces__srv__UavControl_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uav_interfaces__srv__UavControl_Request);
    uav_interfaces__srv__UavControl_Request * data =
      (uav_interfaces__srv__UavControl_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uav_interfaces__srv__UavControl_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          uav_interfaces__srv__UavControl_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uav_interfaces__srv__UavControl_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
uav_interfaces__srv__UavControl_Response__init(uav_interfaces__srv__UavControl_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
uav_interfaces__srv__UavControl_Response__fini(uav_interfaces__srv__UavControl_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
uav_interfaces__srv__UavControl_Response__are_equal(const uav_interfaces__srv__UavControl_Response * lhs, const uav_interfaces__srv__UavControl_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
uav_interfaces__srv__UavControl_Response__copy(
  const uav_interfaces__srv__UavControl_Response * input,
  uav_interfaces__srv__UavControl_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

uav_interfaces__srv__UavControl_Response *
uav_interfaces__srv__UavControl_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_interfaces__srv__UavControl_Response * msg = (uav_interfaces__srv__UavControl_Response *)allocator.allocate(sizeof(uav_interfaces__srv__UavControl_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uav_interfaces__srv__UavControl_Response));
  bool success = uav_interfaces__srv__UavControl_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uav_interfaces__srv__UavControl_Response__destroy(uav_interfaces__srv__UavControl_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uav_interfaces__srv__UavControl_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uav_interfaces__srv__UavControl_Response__Sequence__init(uav_interfaces__srv__UavControl_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_interfaces__srv__UavControl_Response * data = NULL;

  if (size) {
    data = (uav_interfaces__srv__UavControl_Response *)allocator.zero_allocate(size, sizeof(uav_interfaces__srv__UavControl_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uav_interfaces__srv__UavControl_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uav_interfaces__srv__UavControl_Response__fini(&data[i - 1]);
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
uav_interfaces__srv__UavControl_Response__Sequence__fini(uav_interfaces__srv__UavControl_Response__Sequence * array)
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
      uav_interfaces__srv__UavControl_Response__fini(&array->data[i]);
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

uav_interfaces__srv__UavControl_Response__Sequence *
uav_interfaces__srv__UavControl_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_interfaces__srv__UavControl_Response__Sequence * array = (uav_interfaces__srv__UavControl_Response__Sequence *)allocator.allocate(sizeof(uav_interfaces__srv__UavControl_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uav_interfaces__srv__UavControl_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uav_interfaces__srv__UavControl_Response__Sequence__destroy(uav_interfaces__srv__UavControl_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uav_interfaces__srv__UavControl_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uav_interfaces__srv__UavControl_Response__Sequence__are_equal(const uav_interfaces__srv__UavControl_Response__Sequence * lhs, const uav_interfaces__srv__UavControl_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uav_interfaces__srv__UavControl_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uav_interfaces__srv__UavControl_Response__Sequence__copy(
  const uav_interfaces__srv__UavControl_Response__Sequence * input,
  uav_interfaces__srv__UavControl_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uav_interfaces__srv__UavControl_Response);
    uav_interfaces__srv__UavControl_Response * data =
      (uav_interfaces__srv__UavControl_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uav_interfaces__srv__UavControl_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          uav_interfaces__srv__UavControl_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uav_interfaces__srv__UavControl_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
