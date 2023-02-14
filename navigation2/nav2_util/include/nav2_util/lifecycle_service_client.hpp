// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_util
{

/// Helper functions to interact with a lifecycle node.
/// 是LifecycleNode 状态改变的操作者
 /// 再底层类是 ServiceClient 
class LifecycleServiceClient
{
public:
  // parent_node是改变LifecycleNode的上一级Node
  explicit LifecycleServiceClient(const std::string & lifecycle_node_name);
  LifecycleServiceClient(
    const std::string & lifecycle_node_name,
    rclcpp::Node::SharedPtr parent_node);

  /// Trigger a state change
  /**
   * Throws std::runtime_error on failure
   */
  void change_state(
    const uint8_t transition,  // takes a lifecycle_msgs::msg::Transition id
    const std::chrono::seconds timeout);

  /// Trigger a state change, returning result
  bool change_state(std::uint8_t transition);

  /// Get the current state as a lifecycle_msgs::msg::State id value
  /**
   * Throws std::runtime_error on failure
   */
  uint8_t get_state(const std::chrono::seconds timeout = std::chrono::seconds::max());

protected:
  rclcpp::Node::SharedPtr node_;
  // 接口（协议）lifecycle_msgs::srv::ChangeState
   // request: Transition transition
    // uint8 TRANSITION_CREATE = 0
    // uint8 TRANSITION_CONFIGURE = 1
    // uint8 TRANSITION_CLEANUP = 2
    // uint8 TRANSITION_ACTIVATE = 3 
    // uint8 TRANSITION_DEACTIVATE = 4
    // uint8 TRANSITION_UNCONFIGURED_SHUTDOWN = 5
    // uint8 TRANSITION_INACTIVE_SHUTDOWN = 6
    // uint8 TRANSITION_ACTIVE_SHUTDOWN = 7
    // uint8 TRANSITION_DESTROY = 8
    // ...
   // response: bool success
  
  // 模板类对象
  ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;

  // State current_state
   // uint8 PRIMARY_STATE_UNKNOWN = 0
   // uint8 PRIMARY_STATE_UNCONFIGURED = 1
   // uint8 PRIMARY_STATE_INACTIVE = 2
   // uint8 PRIMARY_STATE_ACTIVE = 3
   // uint8 PRIMARY_STATE_FINALIZED = 4
   // ...
  
  // 模板类对象
  ServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
