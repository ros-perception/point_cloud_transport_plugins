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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <zstd_point_cloud_transport/zstd_publisher.hpp>

namespace zstd_point_cloud_transport
{

ZstdPublisher::ZstdPublisher()
{
  this->zstd_context_ = ZSTD_createCCtx();
}

void ZstdPublisher::declareParameters(const std::string & base_topic)
{
  rcl_interfaces::msg::ParameterDescriptor encode_level_paramDescriptor;
  encode_level_paramDescriptor.name = "zstd_encode_level";
  encode_level_paramDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  encode_level_paramDescriptor.description =
    "0 = minimum compression, but the maximum compression 10";
  encode_level_paramDescriptor.set__integer_range(
    {rcl_interfaces::msg::IntegerRange()
      .set__from_value(-1)
      .set__to_value(9)
      .set__step(1)});
  declareParam<int>(
    encode_level_paramDescriptor.name, this->encode_level_,
    encode_level_paramDescriptor);
  getParam<int>(encode_level_paramDescriptor.name, this->encode_level_);

  auto param_change_callback =
    [this](const std::vector<rclcpp::Parameter> & parameters) -> rcl_interfaces::msg::
    SetParametersResult
    {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        if (parameter.get_name().find("zstd_encode_level") != std::string::npos) {
          this->encode_level_ = static_cast<int>(parameter.as_int());
          if (!(this->encode_level_ >= -1 && this->encode_level_ <= 9)) {
            RCLCPP_ERROR_STREAM(
              getLogger(), "encode_level value range should be between [0, 10] ");
          }
          return result;
        }
      }
      return result;
    };
  setParamCallback(param_change_callback);
}

std::string ZstdPublisher::getDataType() const
{
  return "point_cloud_interfaces/msg/CompressedPointCloud2";
}

ZstdPublisher::TypedEncodeResult ZstdPublisher::encodeTyped(
  const sensor_msgs::msg::PointCloud2 & raw) const
{
  size_t est_compress_size = ZSTD_compressBound(raw.data.size());

  point_cloud_interfaces::msg::CompressedPointCloud2 compressed;
  compressed.compressed_data.resize(est_compress_size);

  auto compress_size =
    ZSTD_compressCCtx(
    this->zstd_context_,
    static_cast<void *>(&compressed.compressed_data[0]),
    est_compress_size,
    &raw.data[0],
    raw.data.size(),
    this->encode_level_);

  compressed.compressed_data.resize(compress_size);

  compressed.width = raw.width;
  compressed.height = raw.height;
  compressed.row_step = raw.row_step;
  compressed.point_step = raw.point_step;
  compressed.is_bigendian = raw.is_bigendian;
  compressed.is_dense = raw.is_dense;
  compressed.header = raw.header;
  compressed.fields = raw.fields;
  compressed.format = getTransportName();

  return compressed;
}

}  // namespace zstd_point_cloud_transport
