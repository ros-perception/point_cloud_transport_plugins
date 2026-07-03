/*
 * Copyright (c) 2026, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>

using namespace std::chrono_literals;

namespace
{

sensor_msgs::msg::PointCloud2 makeCloud(uint32_t width)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = width;
  cloud.point_step = 16;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step) * cloud.height);
  for (std::size_t i = 0; i < cloud.data.size(); ++i) {
    cloud.data[i] = static_cast<uint8_t>((i * 31u + 7u) & 0xFFu);
  }
  return cloud;
}

}  // namespace

class ZstdTransportTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

// The zstd transport plugin must be discoverable through pluginlib.
TEST_F(ZstdTransportTest, TransportIsLoadable)
{
  auto node = std::make_shared<rclcpp::Node>("zstd_loadable_test");
  point_cloud_transport::PointCloudTransport pct(*node);

  bool found = false;
  for (const auto & entry : pct.getLoadableTransports()) {
    if (entry.first.find("zstd") != std::string::npos ||
      entry.second.find("zstd") != std::string::npos)
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "zstd transport not found among loadable transports";
}

// Full integration: a cloud published over the zstd transport must arrive,
// decoded, at a subscriber that requested the zstd transport. This drives the
// whole point_cloud_transport layer -- pluginlib load, serialization of the
// CompressedPointCloud2 on the wire, and the decode on receipt.
TEST_F(ZstdTransportTest, PublishSubscribeRoundTrip)
{
  auto node = std::make_shared<rclcpp::Node>("zstd_transport_test");

  sensor_msgs::msg::PointCloud2::ConstSharedPtr received;
  auto sub = point_cloud_transport::create_subscription(
    *node, "test_cloud",
    [&received](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {received = msg;},
    "zstd", rclcpp::SystemDefaultsQoS());

  auto pub = point_cloud_transport::create_publisher(
    *node, "test_cloud", rclcpp::SystemDefaultsQoS());

  rclcpp::executors::SingleThreadedExecutor exec;

  const auto cloud = makeCloud(200);

  // Publish repeatedly and spin until the message is received (or timeout),
  // tolerating in-process discovery latency without being order-dependent.
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (!received && std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    pub.publish(cloud);
    exec.spin_node_some(node);
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_TRUE(received) << "no cloud received through the zstd transport within the timeout";
  EXPECT_EQ(received->data, cloud.data);
  EXPECT_EQ(received->width, cloud.width);
  EXPECT_EQ(received->height, cloud.height);
  EXPECT_EQ(received->point_step, cloud.point_step);
}
