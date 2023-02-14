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

#ifndef NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "nav2_util/lifecycle_service_client.hpp"
#include "nav2_util/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "bondcpp/bond.hpp"

namespace nav2_lifecycle_manager
{

using nav2_msgs::srv::ManageLifecycleNodes;
/**
 * @class nav2_lifecycle_manager::LifecycleManager
 * @brief Implements service interface to transition the lifecycle nodes of
 * Nav2 stack. It receives transition request and then uses lifecycle
 * interface to change lifecycle node's state.
 */

/**
 * 管理多个生命周期节点的类,通过client-server实现
 * 在Nav2中管理以下LifecycleNodes节点：
 * navigator_server
 * controller_server 
 * planner_server
 * recovery_server 
 * LifecycleManager该类中 创建了 service 
 * 并在该包下的 lifecycle_manager_client.hpp 创建了对应的 client
 * 这个 client本身也是一个包装在 lifecycleNode 中
 */
class LifecycleManager : public rclcpp::Node
{
public:
  /**
   * @brief A constructor for nav2_lifecycle_manager::LifecycleManager
   */
  LifecycleManager();
  /**
   * @brief A destructor for nav2_lifecycle_manager::LifecycleManager
   */
  ~LifecycleManager();

protected:
  // Callback group used by services and timers
  // 回调组，此node中包含两个service回调函数：
   // managerCallback(...)
   // isActiveCallback(...)
  // 及执行rclcpp::TimerBase的timer 触发的函数（topic的消息发布通常以timer定时触发publish的回调实现）
  // 这些回调函数用于响应 该包下的 lifecycle_manager_client.hpp的LifecycleManagerClient
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  
  // 用于在另一个线程 spin 此node
  std::unique_ptr<nav2_util::NodeThread> service_thread_;

  // The services provided by this node
   // 用于切换 lifecycleNodes 状态
  rclcpp::Service<ManageLifecycleNodes>::SharedPtr manager_srv_;
   // 用于判断此 node 管理的所有lifecycleNode 是否已经 active 的service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr is_active_srv_;

  /**
   * @brief Lifecycle node manager callback function
   * @param request_header Header of the service request
   * @param request Service request
   * @param reponse Service response
   */
  // 切换所管理的Lifecycle节点 状态的回调函数
  void managerCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManageLifecycleNodes::Request> request,
    std::shared_ptr<ManageLifecycleNodes::Response> response);

  /**
   * @brief Trigger callback function checks if the managed nodes are in active
   * state.
   * @param request_header Header of the request
   * @param request Service request
   * @param reponse Service response
   */
  // 检查所管理的Lifecycle节点 是否已处于active状态的回调函数
  void isActiveCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Support functions for the service calls
  /**
   * @brief Start up managed nodes.
   * @return true or false
   */
  // 所管理的LifecycleNodes 实现configure及activate 
  bool startup();
  /**
   * @brief Deactivate, clean up and shut down all the managed nodes.
   * @return true or false
   */
  bool shutdown();
  /**
   * @brief Reset all the managed nodes.
   * @return true or false
   */
  bool reset();
  /**
   * @brief Pause all the managed nodes.
   * @return true or false
   */
  bool pause();
  /**
   * @brief Resume all the managed nodes.
   * @return true or false
   */
  bool resume();

  // Support function for creating service clients
  /**
   * @brief Support function for creating service clients
   */
  // 为被管理的LifecycleNodes 创建service的客户端
  // 有多少个 被管理的LifecycleNodes 就有多少个 客户端，成员变量 node_map_中
  // 每一个LifecycleNode的 客户端(client) 与 对应LifecycleNodes的 服务端(server) 之间的存在一个bond
  // 以检查每一对 client-server 是否还存在 链接
  void createLifecycleServiceClients();

  // Support functions for shutdown
  /**
   * @brief Support function for shutdown
   */
  void shutdownAllNodes();
  /**
   * @brief Destroy all the lifecycle service clients.
   */
  void destroyLifecycleServiceClients();

  // Support function for creating bond timer
  /**
   * @brief Support function for creating bond timer
   */
  void createBondTimer();

  // Support function for creating bond connection
  /**
   * @brief Support function for creating bond connections
   */
  bool createBondConnection(const std::string & node_name);

  // Support function for killing bond connections
  /**
   * @brief Support function for killing bond connections
   */
  void destroyBondTimer();

  // Support function for checking on bond connections
  /**
   * @ brief Support function for checking on bond connections
   * will take down system if there's something non-responsive
   */
  void checkBondConnections();

  /**
   * @brief For a node, transition to the new target state
   */
  // 改变LifecycleNode状态的函数,若此LifecycleNode的状态要改变为TRANSITION_ACTIVATE
  // 还要 调用createBondConnection(...)函数
  bool changeStateForNode(
    const std::string & node_name,
    std::uint8_t transition);

  /**
   * @brief For each node in the map, transition to the new target state
   */
  // 被管理的LifecycleNodes状态切换实际操作的函数
   // 内层调用changeStateForNode()
  bool changeStateForAllNodes(std::uint8_t transition);

  // Convenience function to highlight the output on the console
  /**
   * @brief Helper function to highlight the output on the console
   */
  void message(const std::string & msg);

  rclcpp::TimerBase::SharedPtr init_timer_;

  // Timer thread to look at bond connections
  rclcpp::TimerBase::SharedPtr bond_timer_;

  // 用于设置服务器绑定超时时间，
  // 若某个服务器未响应的时间超过了设定的这个超时时间（单位为秒），则会将此管理器管理的所有生命周期节点转换到终结状态。
  std::chrono::milliseconds bond_timeout_;

  // A map of all nodes to check bond connection
  // 在此LifecycleNode中send request, 切换多个LifecycleNodes状态  
  std::map<std::string, std::shared_ptr<bond::Bond>> bond_map_;

  // A map of all nodes to be controlled
  // 所有被管理的LifecycleNodes的 名称 及其 对应的状态转换请求的客户端
  std::map<std::string, std::shared_ptr<nav2_util::LifecycleServiceClient>> node_map_;
  
  // Key是可调用的状态转换,包含 configure, cleanup, activate, deactivate, shutdown
  // Value是 过渡状态,包含 Configuring, Cleaning up, Activating, shutdown
  std::map<std::uint8_t, std::string> transition_label_map_;

  // A map of the expected transitions to primary states
   // Key是可调用的状态转换,包含 configure, cleanup, activate, deactivate, shutdown
   // Value是 基本状态(primary states),包含 unconfigured, inactive, active, shutdown
  std::unordered_map<std::uint8_t, std::uint8_t> transition_state_map_;

  // The names of the nodes to be managed, in the order of desired bring-up
  // 要管理的所有LifecycleNodes的名称集合
  std::vector<std::string> node_names_;

  // Whether to automatically start up the system
  bool autostart_;

  // 用于判断 管理的节点是否处于active状态 的标志位
  bool system_active_{false};
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
