// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uav_interfaces:srv/UavControl.idl
// generated code does not contain a copyright notice

#ifndef UAV_INTERFACES__SRV__DETAIL__UAV_CONTROL__BUILDER_HPP_
#define UAV_INTERFACES__SRV__DETAIL__UAV_CONTROL__BUILDER_HPP_

#include "uav_interfaces/srv/detail/uav_control__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace uav_interfaces
{

namespace srv
{

namespace builder
{

class Init_UavControl_Request_z_deta
{
public:
  explicit Init_UavControl_Request_z_deta(::uav_interfaces::srv::UavControl_Request & msg)
  : msg_(msg)
  {}
  ::uav_interfaces::srv::UavControl_Request z_deta(::uav_interfaces::srv::UavControl_Request::_z_deta_type arg)
  {
    msg_.z_deta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uav_interfaces::srv::UavControl_Request msg_;
};

class Init_UavControl_Request_y_deta
{
public:
  explicit Init_UavControl_Request_y_deta(::uav_interfaces::srv::UavControl_Request & msg)
  : msg_(msg)
  {}
  Init_UavControl_Request_z_deta y_deta(::uav_interfaces::srv::UavControl_Request::_y_deta_type arg)
  {
    msg_.y_deta = std::move(arg);
    return Init_UavControl_Request_z_deta(msg_);
  }

private:
  ::uav_interfaces::srv::UavControl_Request msg_;
};

class Init_UavControl_Request_x_deta
{
public:
  explicit Init_UavControl_Request_x_deta(::uav_interfaces::srv::UavControl_Request & msg)
  : msg_(msg)
  {}
  Init_UavControl_Request_y_deta x_deta(::uav_interfaces::srv::UavControl_Request::_x_deta_type arg)
  {
    msg_.x_deta = std::move(arg);
    return Init_UavControl_Request_y_deta(msg_);
  }

private:
  ::uav_interfaces::srv::UavControl_Request msg_;
};

class Init_UavControl_Request_takeoff_land
{
public:
  Init_UavControl_Request_takeoff_land()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UavControl_Request_x_deta takeoff_land(::uav_interfaces::srv::UavControl_Request::_takeoff_land_type arg)
  {
    msg_.takeoff_land = std::move(arg);
    return Init_UavControl_Request_x_deta(msg_);
  }

private:
  ::uav_interfaces::srv::UavControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::uav_interfaces::srv::UavControl_Request>()
{
  return uav_interfaces::srv::builder::Init_UavControl_Request_takeoff_land();
}

}  // namespace uav_interfaces


namespace uav_interfaces
{

namespace srv
{

namespace builder
{

class Init_UavControl_Response_success
{
public:
  Init_UavControl_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::uav_interfaces::srv::UavControl_Response success(::uav_interfaces::srv::UavControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uav_interfaces::srv::UavControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::uav_interfaces::srv::UavControl_Response>()
{
  return uav_interfaces::srv::builder::Init_UavControl_Response_success();
}

}  // namespace uav_interfaces

#endif  // UAV_INTERFACES__SRV__DETAIL__UAV_CONTROL__BUILDER_HPP_
