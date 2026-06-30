/*
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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

#include <zstd.h>

#include <list>
#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <zstd_point_cloud_transport/zstd_subscriber.hpp>

namespace zstd_point_cloud_transport
{
void ZstdSubscriber::declareParameters()
{
}

std::string ZstdSubscriber::getDataType() const
{
  return "point_cloud_interfaces/msg/CompressedPointCloud2";
}

ZstdSubscriber::DecodeResult ZstdSubscriber::decodeTyped(
  const point_cloud_interfaces::msg::CompressedPointCloud2 & msg) const
{
  if (msg.compressed_data.empty()) {
    return tl::make_unexpected("Received compressed Zstd message with zero length.");
  }

  const unsigned long long est_decomp_size = ZSTD_getFrameContentSize(
    msg.compressed_data.data(), msg.compressed_data.size());

  if (est_decomp_size == ZSTD_CONTENTSIZE_ERROR) {
    return tl::make_unexpected("Zstd: input is not a valid compressed frame.");
  }
  if (est_decomp_size == ZSTD_CONTENTSIZE_UNKNOWN) {
    return tl::make_unexpected(
      "Zstd: decompressed size is unknown; streaming frames are not supported.");
  }

  auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();
  result->data.resize(est_decomp_size);

  const size_t decomp_size = ZSTD_decompressDCtx(
    this->zstd_context_.get(),
    result->data.data(),
    est_decomp_size,
    msg.compressed_data.data(),
    msg.compressed_data.size());

  if (ZSTD_isError(decomp_size)) {
    return tl::make_unexpected(
      std::string("Zstd decompression failed: ") + ZSTD_getErrorName(decomp_size));
  }

  result->data.resize(decomp_size);

  result->width = msg.width;
  result->height = msg.height;
  result->row_step = msg.row_step;
  result->point_step = msg.point_step;
  result->is_bigendian = msg.is_bigendian;
  result->is_dense = msg.is_dense;
  result->header = msg.header;
  result->fields = msg.fields;

  return result;
}

}  // namespace zstd_point_cloud_transport
