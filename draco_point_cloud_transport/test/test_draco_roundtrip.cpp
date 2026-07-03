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
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <draco_point_cloud_transport/draco_publisher.hpp>
#include <draco_point_cloud_transport/draco_subscriber.hpp>

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

// Extract the points from an XYZ float32 cloud.
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

// Draco is lossy (quantization) and may reorder points. The cloud STRUCTURE must
// be preserved exactly; the point set must match the originals (order-independent)
// within a quantization tolerance.
TEST(DracoPluginRoundTrip, PreservesStructureAndApproxValues)
{
  draco_point_cloud_transport::DracoPublisher pub;
  draco_point_cloud_transport::DracoSubscriber sub;

  const uint32_t n = 100;
  const auto original = makeXyzCloud(n);

  const auto encoded = pub.encodeTyped(original);
  ASSERT_TRUE(encoded.has_value()) << (encoded ? "" : encoded.error());
  ASSERT_TRUE(encoded.value().has_value()) << "encoder returned no message";
  const auto & compressed = *encoded.value();

  const auto decoded = sub.decodeTyped(compressed);
  ASSERT_TRUE(decoded.has_value()) << (decoded ? "" : decoded.error());
  ASSERT_TRUE(decoded.value().has_value()) << "decoder returned no message";
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr out = *decoded.value();

  // --- structure preserved exactly ---
  EXPECT_EQ(static_cast<uint32_t>(out->width) * out->height, n);
  ASSERT_EQ(out->fields.size(), original.fields.size());
  for (std::size_t i = 0; i < original.fields.size(); ++i) {
    EXPECT_EQ(out->fields[i].name, original.fields[i].name);
    EXPECT_EQ(out->fields[i].offset, original.fields[i].offset);
    EXPECT_EQ(out->fields[i].datatype, original.fields[i].datatype);
  }
  EXPECT_EQ(out->point_step, original.point_step);

  // --- values approximately preserved (sort to be order-independent) ---
  auto orig = extractPoints(original);
  auto got = extractPoints(*out);
  ASSERT_EQ(orig.size(), got.size());
  const auto byX = [](const Point & a, const Point & b) {return a.x < b.x;};
  std::sort(orig.begin(), orig.end(), byX);
  std::sort(got.begin(), got.end(), byX);

  const float tol = 1.0f;
  double max_err = 0.0;
  for (std::size_t i = 0; i < orig.size(); ++i) {
    max_err = std::max({max_err,
          std::fabs(static_cast<double>(got[i].x - orig[i].x)),
          std::fabs(static_cast<double>(got[i].y - orig[i].y)),
          std::fabs(static_cast<double>(got[i].z - orig[i].z))});
    EXPECT_NEAR(got[i].x, orig[i].x, tol) << "x @ " << i;
    EXPECT_NEAR(got[i].y, orig[i].y, tol) << "y @ " << i;
    EXPECT_NEAR(got[i].z, orig[i].z, tol) << "z @ " << i;
  }
  std::cout << "[diag] max coordinate error after draco round-trip: " << max_err << "\n";
}
