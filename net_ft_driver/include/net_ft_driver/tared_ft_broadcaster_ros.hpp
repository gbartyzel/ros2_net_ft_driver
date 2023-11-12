// Copyright (c) 2023, Grzegorz Bartyzel
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

#ifndef NET_FT_DRIVER__TARED_FT_BROADCASTER_ROS_HPP_
#define NET_FT_DRIVER__TARED_FT_BROADCASTER_ROS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace tared_ft_broadcaster
{
class TaredFTBroadcaster : public rclcpp::Node
{
public:
  TaredFTBroadcaster();
  ~TaredFTBroadcaster() = default;

private:
  void callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void tare(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

  geometry_msgs::msg::Wrench bias_;
  geometry_msgs::msg::Wrench data_;

  bool should_tare_;
};

}  // namespace tared_ft_broadcaster
#endif  // NET_FT_DRIVER__TARED_FT_BROADCASTER_ROS_HPP__
