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

#include <cstdint>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <zstd_point_cloud_transport/zstd_publisher.hpp>
#include <zstd_point_cloud_transport/zstd_subscriber.hpp>

namespace
{

// Build a deterministic PointCloud2 with `width` points of 16 bytes each.
sensor_msgs::msg::PointCloud2 makeCloud(uint32_t width, uint8_t fill_pattern = 0)
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
    cloud.data[i] = fill_pattern ? fill_pattern : static_cast<uint8_t>((i * 31u + 7u) & 0xFFu);
  }
  return cloud;
}

}  // namespace

// zstd is lossless: encode -> decode must reproduce the payload and the cloud
// layout metadata exactly.
TEST(ZstdPluginRoundTrip, PreservesCloud)
{
  zstd_point_cloud_transport::ZstdPublisher pub;
  zstd_point_cloud_transport::ZstdSubscriber sub;

  const auto original = makeCloud(500);

  const auto encoded = pub.encodeTyped(original);
  ASSERT_TRUE(encoded.has_value()) << (encoded ? "" : encoded.error());
  ASSERT_TRUE(encoded.value().has_value()) << "encoder returned no message";
  const auto & compressed = *encoded.value();

  const auto decoded = sub.decodeTyped(compressed);
  ASSERT_TRUE(decoded.has_value()) << (decoded ? "" : decoded.error());
  ASSERT_TRUE(decoded.value().has_value()) << "decoder returned no message";
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr out = *decoded.value();

  EXPECT_EQ(out->data, original.data);
  EXPECT_EQ(out->width, original.width);
  EXPECT_EQ(out->height, original.height);
  EXPECT_EQ(out->point_step, original.point_step);
  EXPECT_EQ(out->row_step, original.row_step);
  EXPECT_EQ(out->is_dense, original.is_dense);
}

// Compression must actually run: a highly repetitive cloud must shrink.
TEST(ZstdPluginRoundTrip, ShrinksCompressibleData)
{
  zstd_point_cloud_transport::ZstdPublisher pub;
  const auto original = makeCloud(4096, 0xAB);

  const auto encoded = pub.encodeTyped(original);
  ASSERT_TRUE(encoded.has_value()) << (encoded ? "" : encoded.error());
  ASSERT_TRUE(encoded.value().has_value());
  EXPECT_LT(encoded.value()->compressed_data.size(), original.data.size());
}
