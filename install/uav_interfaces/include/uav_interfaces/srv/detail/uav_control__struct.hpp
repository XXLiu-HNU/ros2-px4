// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uav_interfaces:srv/UavControl.idl
// generated code does not contain a copyright notice

#ifndef UAV_INTERFACES__SRV__DETAIL__UAV_CONTROL__STRUCT_HPP_
#define UAV_INTERFACES__SRV__DETAIL__UAV_CONTROL__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__uav_interfaces__srv__UavControl_Request __attribute__((deprecated))
#else
# define DEPRECATED__uav_interfaces__srv__UavControl_Request __declspec(deprecated)
#endif

namespace uav_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UavControl_Request_
{
  using Type = UavControl_Request_<ContainerAllocator>;

  explicit UavControl_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->takeoff_land = 0l;
      this->x_deta = 0l;
      this->y_deta = 0l;
      this->z_deta = 0l;
    }
  }

  explicit UavControl_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->takeoff_land = 0l;
      this->x_deta = 0l;
      this->y_deta = 0l;
      this->z_deta = 0l;
    }
  }

  // field types and members
  using _takeoff_land_type =
    int32_t;
  _takeoff_land_type takeoff_land;
  using _x_deta_type =
    int32_t;
  _x_deta_type x_deta;
  using _y_deta_type =
    int32_t;
  _y_deta_type y_deta;
  using _z_deta_type =
    int32_t;
  _z_deta_type z_deta;

  // setters for named parameter idiom
  Type & set__takeoff_land(
    const int32_t & _arg)
  {
    this->takeoff_land = _arg;
    return *this;
  }
  Type & set__x_deta(
    const int32_t & _arg)
  {
    this->x_deta = _arg;
    return *this;
  }
  Type & set__y_deta(
    const int32_t & _arg)
  {
    this->y_deta = _arg;
    return *this;
  }
  Type & set__z_deta(
    const int32_t & _arg)
  {
    this->z_deta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uav_interfaces::srv::UavControl_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const uav_interfaces::srv::UavControl_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uav_interfaces::srv::UavControl_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uav_interfaces::srv::UavControl_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uav_interfaces::srv::UavControl_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uav_interfaces::srv::UavControl_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uav_interfaces::srv::UavControl_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uav_interfaces::srv::UavControl_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uav_interfaces::srv::UavControl_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uav_interfaces::srv::UavControl_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uav_interfaces__srv__UavControl_Request
    std::shared_ptr<uav_interfaces::srv::UavControl_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uav_interfaces__srv__UavControl_Request
    std::shared_ptr<uav_interfaces::srv::UavControl_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UavControl_Request_ & other) const
  {
    if (this->takeoff_land != other.takeoff_land) {
      return false;
    }
    if (this->x_deta != other.x_deta) {
      return false;
    }
    if (this->y_deta != other.y_deta) {
      return false;
    }
    if (this->z_deta != other.z_deta) {
      return false;
    }
    return true;
  }
  bool operator!=(const UavControl_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UavControl_Request_

// alias to use template instance with default allocator
using UavControl_Request =
  uav_interfaces::srv::UavControl_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace uav_interfaces


#ifndef _WIN32
# define DEPRECATED__uav_interfaces__srv__UavControl_Response __attribute__((deprecated))
#else
# define DEPRECATED__uav_interfaces__srv__UavControl_Response __declspec(deprecated)
#endif

namespace uav_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UavControl_Response_
{
  using Type = UavControl_Response_<ContainerAllocator>;

  explicit UavControl_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = 0l;
    }
  }

  explicit UavControl_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = 0l;
    }
  }

  // field types and members
  using _success_type =
    int32_t;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const int32_t & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uav_interfaces::srv::UavControl_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const uav_interfaces::srv::UavControl_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uav_interfaces::srv::UavControl_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uav_interfaces::srv::UavControl_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uav_interfaces::srv::UavControl_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uav_interfaces::srv::UavControl_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uav_interfaces::srv::UavControl_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uav_interfaces::srv::UavControl_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uav_interfaces::srv::UavControl_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uav_interfaces::srv::UavControl_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uav_interfaces__srv__UavControl_Response
    std::shared_ptr<uav_interfaces::srv::UavControl_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uav_interfaces__srv__UavControl_Response
    std::shared_ptr<uav_interfaces::srv::UavControl_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UavControl_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const UavControl_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UavControl_Response_

// alias to use template instance with default allocator
using UavControl_Response =
  uav_interfaces::srv::UavControl_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace uav_interfaces

namespace uav_interfaces
{

namespace srv
{

struct UavControl
{
  using Request = uav_interfaces::srv::UavControl_Request;
  using Response = uav_interfaces::srv::UavControl_Response;
};

}  // namespace srv

}  // namespace uav_interfaces

#endif  // UAV_INTERFACES__SRV__DETAIL__UAV_CONTROL__STRUCT_HPP_
