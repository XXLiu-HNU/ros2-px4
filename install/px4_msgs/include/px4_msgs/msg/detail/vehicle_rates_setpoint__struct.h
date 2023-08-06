// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/VehicleRatesSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__VEHICLE_RATES_SETPOINT__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__VEHICLE_RATES_SETPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/VehicleRatesSetpoint in the package px4_msgs.
typedef struct px4_msgs__msg__VehicleRatesSetpoint
{
  uint64_t timestamp;
  float roll;
  float pitch;
  float yaw;
  float thrust_body[3];
  bool reset_integral;
} px4_msgs__msg__VehicleRatesSetpoint;

// Struct for a sequence of px4_msgs__msg__VehicleRatesSetpoint.
typedef struct px4_msgs__msg__VehicleRatesSetpoint__Sequence
{
  px4_msgs__msg__VehicleRatesSetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__VehicleRatesSetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__VEHICLE_RATES_SETPOINT__STRUCT_H_
