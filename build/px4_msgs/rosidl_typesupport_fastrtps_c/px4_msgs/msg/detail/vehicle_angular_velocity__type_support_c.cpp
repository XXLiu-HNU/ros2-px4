// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/VehicleAngularVelocity.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/vehicle_angular_velocity__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/vehicle_angular_velocity__struct.h"
#include "px4_msgs/msg/detail/vehicle_angular_velocity__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _VehicleAngularVelocity__ros_msg_type = px4_msgs__msg__VehicleAngularVelocity;

static bool _VehicleAngularVelocity__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _VehicleAngularVelocity__ros_msg_type * ros_message = static_cast<const _VehicleAngularVelocity__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: timestamp_sample
  {
    cdr << ros_message->timestamp_sample;
  }

  // Field name: xyz
  {
    size_t size = 3;
    auto array_ptr = ros_message->xyz;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: xyz_derivative
  {
    size_t size = 3;
    auto array_ptr = ros_message->xyz_derivative;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _VehicleAngularVelocity__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _VehicleAngularVelocity__ros_msg_type * ros_message = static_cast<_VehicleAngularVelocity__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: timestamp_sample
  {
    cdr >> ros_message->timestamp_sample;
  }

  // Field name: xyz
  {
    size_t size = 3;
    auto array_ptr = ros_message->xyz;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: xyz_derivative
  {
    size_t size = 3;
    auto array_ptr = ros_message->xyz_derivative;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__VehicleAngularVelocity(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _VehicleAngularVelocity__ros_msg_type * ros_message = static_cast<const _VehicleAngularVelocity__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name timestamp_sample
  {
    size_t item_size = sizeof(ros_message->timestamp_sample);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name xyz
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->xyz;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name xyz_derivative
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->xyz_derivative;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _VehicleAngularVelocity__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__VehicleAngularVelocity(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__VehicleAngularVelocity(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: timestamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: timestamp_sample
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: xyz
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: xyz_derivative
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _VehicleAngularVelocity__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_px4_msgs__msg__VehicleAngularVelocity(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_VehicleAngularVelocity = {
  "px4_msgs::msg",
  "VehicleAngularVelocity",
  _VehicleAngularVelocity__cdr_serialize,
  _VehicleAngularVelocity__cdr_deserialize,
  _VehicleAngularVelocity__get_serialized_size,
  _VehicleAngularVelocity__max_serialized_size
};

static rosidl_message_type_support_t _VehicleAngularVelocity__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_VehicleAngularVelocity,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, VehicleAngularVelocity)() {
  return &_VehicleAngularVelocity__type_support;
}

#if defined(__cplusplus)
}
#endif
