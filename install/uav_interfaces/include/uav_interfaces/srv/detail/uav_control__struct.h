// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uav_interfaces:srv/UavControl.idl
// generated code does not contain a copyright notice

#ifndef UAV_INTERFACES__SRV__DETAIL__UAV_CONTROL__STRUCT_H_
#define UAV_INTERFACES__SRV__DETAIL__UAV_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/UavControl in the package uav_interfaces.
typedef struct uav_interfaces__srv__UavControl_Request
{
  int32_t takeoff_land;
  int32_t x_deta;
  int32_t y_deta;
  int32_t z_deta;
} uav_interfaces__srv__UavControl_Request;

// Struct for a sequence of uav_interfaces__srv__UavControl_Request.
typedef struct uav_interfaces__srv__UavControl_Request__Sequence
{
  uav_interfaces__srv__UavControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uav_interfaces__srv__UavControl_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/UavControl in the package uav_interfaces.
typedef struct uav_interfaces__srv__UavControl_Response
{
  int32_t success;
} uav_interfaces__srv__UavControl_Response;

// Struct for a sequence of uav_interfaces__srv__UavControl_Response.
typedef struct uav_interfaces__srv__UavControl_Response__Sequence
{
  uav_interfaces__srv__UavControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uav_interfaces__srv__UavControl_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UAV_INTERFACES__SRV__DETAIL__UAV_CONTROL__STRUCT_H_
