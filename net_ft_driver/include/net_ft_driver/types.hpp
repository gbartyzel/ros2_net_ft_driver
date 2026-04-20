// Copyright (c) 2022, Grzegorz Bartyzel
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

#ifndef NET_FT_DRIVER__TYPES_HPP_
#define NET_FT_DRIVER__TYPES_HPP_

#include <array>
#include <cstdint>

namespace net_ft_driver
{
namespace types
{
using Vector6D = std::array<double, 6>;

using Vecotr6I32 = std::array<std::int32_t, 6>;

struct SensorData
{
  Vector6D ft_values;
  std::uint32_t lost_packets;
  std::uint32_t packet_count;
  std::uint32_t out_of_order_count;
  std::uint32_t status;
};
}  // namespace types
}  // namespace net_ft_driver

#endif  // NET_FT_DRIVER__TYPES_HPP_
