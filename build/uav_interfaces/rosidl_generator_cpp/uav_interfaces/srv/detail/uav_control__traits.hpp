// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uav_interfaces:srv/UavControl.idl
// generated code does not contain a copyright notice

#ifndef UAV_INTERFACES__SRV__DETAIL__UAV_CONTROL__TRAITS_HPP_
#define UAV_INTERFACES__SRV__DETAIL__UAV_CONTROL__TRAITS_HPP_

#include "uav_interfaces/srv/detail/uav_control__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<uav_interfaces::srv::UavControl_Request>()
{
  return "uav_interfaces::srv::UavControl_Request";
}

template<>
inline const char * name<uav_interfaces::srv::UavControl_Request>()
{
  return "uav_interfaces/srv/UavControl_Request";
}

template<>
struct has_fixed_size<uav_interfaces::srv::UavControl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<uav_interfaces::srv::UavControl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<uav_interfaces::srv::UavControl_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<uav_interfaces::srv::UavControl_Response>()
{
  return "uav_interfaces::srv::UavControl_Response";
}

template<>
inline const char * name<uav_interfaces::srv::UavControl_Response>()
{
  return "uav_interfaces/srv/UavControl_Response";
}

template<>
struct has_fixed_size<uav_interfaces::srv::UavControl_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<uav_interfaces::srv::UavControl_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<uav_interfaces::srv::UavControl_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<uav_interfaces::srv::UavControl>()
{
  return "uav_interfaces::srv::UavControl";
}

template<>
inline const char * name<uav_interfaces::srv::UavControl>()
{
  return "uav_interfaces/srv/UavControl";
}

template<>
struct has_fixed_size<uav_interfaces::srv::UavControl>
  : std::integral_constant<
    bool,
    has_fixed_size<uav_interfaces::srv::UavControl_Request>::value &&
    has_fixed_size<uav_interfaces::srv::UavControl_Response>::value
  >
{
};

template<>
struct has_bounded_size<uav_interfaces::srv::UavControl>
  : std::integral_constant<
    bool,
    has_bounded_size<uav_interfaces::srv::UavControl_Request>::value &&
    has_bounded_size<uav_interfaces::srv::UavControl_Response>::value
  >
{
};

template<>
struct is_service<uav_interfaces::srv::UavControl>
  : std::true_type
{
};

template<>
struct is_service_request<uav_interfaces::srv::UavControl_Request>
  : std::true_type
{
};

template<>
struct is_service_response<uav_interfaces::srv::UavControl_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // UAV_INTERFACES__SRV__DETAIL__UAV_CONTROL__TRAITS_HPP_
