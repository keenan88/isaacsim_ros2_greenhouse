// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from moveit_msgs:srv/ServoCommandType.idl
// generated code does not contain a copyright notice

#ifndef MOVEIT_MSGS__SRV__DETAIL__SERVO_COMMAND_TYPE__STRUCT_HPP_
#define MOVEIT_MSGS__SRV__DETAIL__SERVO_COMMAND_TYPE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__moveit_msgs__srv__ServoCommandType_Request __attribute__((deprecated))
#else
# define DEPRECATED__moveit_msgs__srv__ServoCommandType_Request __declspec(deprecated)
#endif

namespace moveit_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ServoCommandType_Request_
{
  using Type = ServoCommandType_Request_<ContainerAllocator>;

  explicit ServoCommandType_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command_type = 0;
    }
  }

  explicit ServoCommandType_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command_type = 0;
    }
  }

  // field types and members
  using _command_type_type =
    int8_t;
  _command_type_type command_type;

  // setters for named parameter idiom
  Type & set__command_type(
    const int8_t & _arg)
  {
    this->command_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t JOINT_JOG =
    0;
  static constexpr int8_t TWIST =
    1;
  static constexpr int8_t POSE =
    2;

  // pointer types
  using RawPtr =
    moveit_msgs::srv::ServoCommandType_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const moveit_msgs::srv::ServoCommandType_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<moveit_msgs::srv::ServoCommandType_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<moveit_msgs::srv::ServoCommandType_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      moveit_msgs::srv::ServoCommandType_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<moveit_msgs::srv::ServoCommandType_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      moveit_msgs::srv::ServoCommandType_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<moveit_msgs::srv::ServoCommandType_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<moveit_msgs::srv::ServoCommandType_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<moveit_msgs::srv::ServoCommandType_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__moveit_msgs__srv__ServoCommandType_Request
    std::shared_ptr<moveit_msgs::srv::ServoCommandType_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__moveit_msgs__srv__ServoCommandType_Request
    std::shared_ptr<moveit_msgs::srv::ServoCommandType_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ServoCommandType_Request_ & other) const
  {
    if (this->command_type != other.command_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const ServoCommandType_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ServoCommandType_Request_

// alias to use template instance with default allocator
using ServoCommandType_Request =
  moveit_msgs::srv::ServoCommandType_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t ServoCommandType_Request_<ContainerAllocator>::JOINT_JOG;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t ServoCommandType_Request_<ContainerAllocator>::TWIST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t ServoCommandType_Request_<ContainerAllocator>::POSE;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace moveit_msgs


#ifndef _WIN32
# define DEPRECATED__moveit_msgs__srv__ServoCommandType_Response __attribute__((deprecated))
#else
# define DEPRECATED__moveit_msgs__srv__ServoCommandType_Response __declspec(deprecated)
#endif

namespace moveit_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ServoCommandType_Response_
{
  using Type = ServoCommandType_Response_<ContainerAllocator>;

  explicit ServoCommandType_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit ServoCommandType_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    moveit_msgs::srv::ServoCommandType_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const moveit_msgs::srv::ServoCommandType_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<moveit_msgs::srv::ServoCommandType_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<moveit_msgs::srv::ServoCommandType_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      moveit_msgs::srv::ServoCommandType_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<moveit_msgs::srv::ServoCommandType_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      moveit_msgs::srv::ServoCommandType_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<moveit_msgs::srv::ServoCommandType_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<moveit_msgs::srv::ServoCommandType_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<moveit_msgs::srv::ServoCommandType_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__moveit_msgs__srv__ServoCommandType_Response
    std::shared_ptr<moveit_msgs::srv::ServoCommandType_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__moveit_msgs__srv__ServoCommandType_Response
    std::shared_ptr<moveit_msgs::srv::ServoCommandType_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ServoCommandType_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const ServoCommandType_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ServoCommandType_Response_

// alias to use template instance with default allocator
using ServoCommandType_Response =
  moveit_msgs::srv::ServoCommandType_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace moveit_msgs

namespace moveit_msgs
{

namespace srv
{

struct ServoCommandType
{
  using Request = moveit_msgs::srv::ServoCommandType_Request;
  using Response = moveit_msgs::srv::ServoCommandType_Response;
};

}  // namespace srv

}  // namespace moveit_msgs

#endif  // MOVEIT_MSGS__SRV__DETAIL__SERVO_COMMAND_TYPE__STRUCT_HPP_
