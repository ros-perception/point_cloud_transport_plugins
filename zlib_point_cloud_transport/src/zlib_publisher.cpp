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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <zlib_point_cloud_transport/zlib_publisher.hpp>

#include "zlib_cpp.hpp"

namespace zlib_point_cloud_transport
{

void ZlibPublisher::declareParameters(const std::string & base_topic)
{
  rcl_interfaces::msg::ParameterDescriptor encode_level_paramDescriptor;
  encode_level_paramDescriptor.name = "encode_level";
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
        if (parameter.get_name().find("encode_level") != std::string::npos) {
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

ZlibPublisher::TypedEncodeResult ZlibPublisher::encodeTyped(
  const sensor_msgs::msg::PointCloud2 & raw) const
{
  zlib::Comp comp(static_cast<zlib::Comp::Level>(this->encode_level_), true);
  auto g_compressed_data =
    comp.Process(&raw.data[0], raw.data.size(), true);

  point_cloud_interfaces::msg::CompressedPointCloud2 compressed;

  size_t total_size = 0;
  for (const auto & data : g_compressed_data) {
    total_size += data->size;
  }

  compressed.compressed_data.resize(total_size);

  size_t index = 0;
  for (const auto & data : g_compressed_data) {
    memcpy(&compressed.compressed_data[index], data->ptr, data->size);
    index += data->size;
  }

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

}  // namespace zlib_point_cloud_transport
