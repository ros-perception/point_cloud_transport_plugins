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

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>

using namespace std::chrono_literals;

namespace
{

struct Point
{
  float x, y, z;
};

// Build an XYZ float32 cloud with `n` distinct points on a diagonal line.
sensor_msgs::msg::PointCloud2 makeXyzCloud(uint32_t n)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = n;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 12;  // 3 * float32
  cloud.row_step = cloud.point_step * n;

  const std::vector<std::string> names = {"x", "y", "z"};
  for (uint32_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i];
    f.offset = i * 4u;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    cloud.fields.push_back(f);
  }

  cloud.data.resize(static_cast<std::size_t>(cloud.row_step) * cloud.height);
  for (uint32_t i = 0; i < n; ++i) {
    const float xyz[3] = {
      static_cast<float>(i), static_cast<float>(i) + 0.25f, static_cast<float>(i) + 0.5f};
    std::memcpy(&cloud.data[i * cloud.point_step], xyz, sizeof(xyz));
  }
  return cloud;
}

std::vector<Point> extractPoints(const sensor_msgs::msg::PointCloud2 & c)
{
  const uint32_t n = c.width * c.height;
  std::vector<Point> pts(n);
  for (uint32_t i = 0; i < n; ++i) {
    std::memcpy(&pts[i], &c.data[i * c.point_step], sizeof(Point));
  }
  return pts;
}

}  // namespace

class DracoTransportTest : public ::testing::Test
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

// The draco transport plugin must be discoverable through pluginlib.
TEST_F(DracoTransportTest, TransportIsLoadable)
{
  auto node = std::make_shared<rclcpp::Node>("draco_loadable_test");
  point_cloud_transport::PointCloudTransport pct(*node);

  bool found = false;
  for (const auto & entry : pct.getLoadableTransports()) {
    if (entry.first.find("draco") != std::string::npos ||
      entry.second.find("draco") != std::string::npos)
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "draco transport not found among loadable transports";
}

// Full integration over the draco transport. Draco is lossy (quantization) and
// reorders points, so the cloud STRUCTURE must be preserved exactly while the
// point set matches the originals (order-independent) within a tolerance.
TEST_F(DracoTransportTest, PublishSubscribeRoundTrip)
{
  auto node = std::make_shared<rclcpp::Node>("draco_transport_test");

  sensor_msgs::msg::PointCloud2::ConstSharedPtr received;
  auto sub = point_cloud_transport::create_subscription(
    *node, "test_cloud",
    [&received](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {received = msg;},
    "draco", rclcpp::SystemDefaultsQoS());

  auto pub = point_cloud_transport::create_publisher(
    *node, "test_cloud", rclcpp::SystemDefaultsQoS());

  rclcpp::executors::SingleThreadedExecutor exec;

  const uint32_t n = 100;
  const auto cloud = makeXyzCloud(n);

  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (!received && std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    pub.publish(cloud);
    exec.spin_node_some(node);
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_TRUE(received) << "no cloud received through the draco transport within the timeout";

  // structure preserved exactly
  EXPECT_EQ(static_cast<uint32_t>(received->width) * received->height, n);
  ASSERT_EQ(received->fields.size(), cloud.fields.size());
  for (std::size_t i = 0; i < cloud.fields.size(); ++i) {
    EXPECT_EQ(received->fields[i].name, cloud.fields[i].name);
    EXPECT_EQ(received->fields[i].datatype, cloud.fields[i].datatype);
  }
  EXPECT_EQ(received->point_step, cloud.point_step);

  // values approximately preserved (sort to be order-independent)
  auto orig = extractPoints(cloud);
  auto got = extractPoints(*received);
  ASSERT_EQ(orig.size(), got.size());
  const auto byX = [](const Point & a, const Point & b) {return a.x < b.x;};
  std::sort(orig.begin(), orig.end(), byX);
  std::sort(got.begin(), got.end(), byX);

  const float tol = 1.0f;
  for (std::size_t i = 0; i < orig.size(); ++i) {
    EXPECT_NEAR(got[i].x, orig[i].x, tol) << "x @ " << i;
    EXPECT_NEAR(got[i].y, orig[i].y, tol) << "y @ " << i;
    EXPECT_NEAR(got[i].z, orig[i].z, tol) << "z @ " << i;
  }
}
