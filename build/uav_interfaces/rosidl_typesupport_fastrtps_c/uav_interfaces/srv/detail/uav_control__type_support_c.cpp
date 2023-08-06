// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from uav_interfaces:srv/UavControl.idl
// generated code does not contain a copyright notice
#include "uav_interfaces/srv/detail/uav_control__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "uav_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "uav_interfaces/srv/detail/uav_control__struct.h"
#include "uav_interfaces/srv/detail/uav_control__functions.h"
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


using _UavControl_Request__ros_msg_type = uav_interfaces__srv__UavControl_Request;

static bool _UavControl_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _UavControl_Request__ros_msg_type * ros_message = static_cast<const _UavControl_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: takeoff_land
  {
    cdr << ros_message->takeoff_land;
  }

  // Field name: x_deta
  {
    cdr << ros_message->x_deta;
  }

  // Field name: y_deta
  {
    cdr << ros_message->y_deta;
  }

  // Field name: z_deta
  {
    cdr << ros_message->z_deta;
  }

  return true;
}

static bool _UavControl_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _UavControl_Request__ros_msg_type * ros_message = static_cast<_UavControl_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: takeoff_land
  {
    cdr >> ros_message->takeoff_land;
  }

  // Field name: x_deta
  {
    cdr >> ros_message->x_deta;
  }

  // Field name: y_deta
  {
    cdr >> ros_message->y_deta;
  }

  // Field name: z_deta
  {
    cdr >> ros_message->z_deta;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uav_interfaces
size_t get_serialized_size_uav_interfaces__srv__UavControl_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UavControl_Request__ros_msg_type * ros_message = static_cast<const _UavControl_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name takeoff_land
  {
    size_t item_size = sizeof(ros_message->takeoff_land);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name x_deta
  {
    size_t item_size = sizeof(ros_message->x_deta);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name y_deta
  {
    size_t item_size = sizeof(ros_message->y_deta);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name z_deta
  {
    size_t item_size = sizeof(ros_message->z_deta);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _UavControl_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_uav_interfaces__srv__UavControl_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uav_interfaces
size_t max_serialized_size_uav_interfaces__srv__UavControl_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: takeoff_land
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: x_deta
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: y_deta
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: z_deta
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _UavControl_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_uav_interfaces__srv__UavControl_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_UavControl_Request = {
  "uav_interfaces::srv",
  "UavControl_Request",
  _UavControl_Request__cdr_serialize,
  _UavControl_Request__cdr_deserialize,
  _UavControl_Request__get_serialized_size,
  _UavControl_Request__max_serialized_size
};

static rosidl_message_type_support_t _UavControl_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UavControl_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, uav_interfaces, srv, UavControl_Request)() {
  return &_UavControl_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "uav_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "uav_interfaces/srv/detail/uav_control__struct.h"
// already included above
// #include "uav_interfaces/srv/detail/uav_control__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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


using _UavControl_Response__ros_msg_type = uav_interfaces__srv__UavControl_Response;

static bool _UavControl_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _UavControl_Response__ros_msg_type * ros_message = static_cast<const _UavControl_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << ros_message->success;
  }

  return true;
}

static bool _UavControl_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _UavControl_Response__ros_msg_type * ros_message = static_cast<_UavControl_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr >> ros_message->success;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uav_interfaces
size_t get_serialized_size_uav_interfaces__srv__UavControl_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UavControl_Response__ros_msg_type * ros_message = static_cast<const _UavControl_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _UavControl_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_uav_interfaces__srv__UavControl_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uav_interfaces
size_t max_serialized_size_uav_interfaces__srv__UavControl_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: success
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _UavControl_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_uav_interfaces__srv__UavControl_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_UavControl_Response = {
  "uav_interfaces::srv",
  "UavControl_Response",
  _UavControl_Response__cdr_serialize,
  _UavControl_Response__cdr_deserialize,
  _UavControl_Response__get_serialized_size,
  _UavControl_Response__max_serialized_size
};

static rosidl_message_type_support_t _UavControl_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UavControl_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, uav_interfaces, srv, UavControl_Response)() {
  return &_UavControl_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "uav_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "uav_interfaces/srv/uav_control.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t UavControl__callbacks = {
  "uav_interfaces::srv",
  "UavControl",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, uav_interfaces, srv, UavControl_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, uav_interfaces, srv, UavControl_Response)(),
};

static rosidl_service_type_support_t UavControl__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &UavControl__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, uav_interfaces, srv, UavControl)() {
  return &UavControl__handle;
}

#if defined(__cplusplus)
}
#endif
