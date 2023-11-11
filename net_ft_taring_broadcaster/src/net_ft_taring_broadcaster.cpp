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

#include "net_ft_taring_broadcaster/net_ft_taring_broadcaster.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace net_ft_taring_broadcaster
{
NetFtTaringBroadcaster::NetFtTaringBroadcaster() : Node("net_ft_taring_broadcaster"), should_tare_(true)
{
  this->declare_parameter("topic_name", rclcpp::PARAMETER_STRING);
  auto topic_name = this->get_parameter("topic_name").as_string();

  sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      topic_name, 10, std::bind(&NetFtTaringBroadcaster::callback, this, _1));
  pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("tared_" + topic_name, 10);
  srv_ = this->create_service<std_srvs::srv::Trigger>("tare", std::bind(&NetFtTaringBroadcaster::tare, this, _1, _2));
}

void NetFtTaringBroadcaster::callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  data_ = msg->wrench;
  if (should_tare_) {
    bias_ = data_;
    should_tare_ = false;
  }
  data_.force.x -= bias_.force.x;
  data_.force.y -= bias_.force.y;
  data_.force.z -= bias_.force.z;
  data_.torque.x -= bias_.torque.x;
  data_.torque.y -= bias_.torque.y;
  data_.torque.z -= bias_.torque.z;
  auto tared_msg = geometry_msgs::msg::WrenchStamped();
  tared_msg.header = msg->header;
  tared_msg.wrench = data_;
  pub_->publish(tared_msg);
}

void NetFtTaringBroadcaster::tare(const std_srvs::srv::Trigger::Request::SharedPtr req,
                                  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  RCLCPP_INFO(this->get_logger(), "Taring FT sensor");
  should_tare_ = true;
  res->success = true;
}
}  // namespace net_ft_taring_broadcaster
