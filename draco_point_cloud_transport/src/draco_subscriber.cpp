/*
 * Copyright (c) 2023, Czech Technical University in Prague
 * Copyright (c) 2019, paplhjak
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

#include <draco/point_cloud/point_cloud.h>
#include <draco/compression/decode.h>
#include <draco_point_cloud_transport/conversion_utilities.h>

#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <rcpputils/tl_expected/expected.hpp>

#include <draco_point_cloud_transport/draco_subscriber.hpp>

namespace draco_point_cloud_transport
{
void DracoSubscriber::declareParameters()
{
  declareParam<bool>(std::string("SkipDequantizationPOSITION"), false);
  getParam<bool>(std::string("SkipDequantizationPOSITION"), config_.SkipDequantizationPOSITION);
  declareParam<bool>(std::string("SkipDequantizationNORMAL"), false);
  getParam<bool>(std::string("SkipDequantizationNORMAL"), config_.SkipDequantizationNORMAL);
  declareParam<bool>(std::string("SkipDequantizationCOLOR"), false);
  getParam<bool>(std::string("SkipDequantizationCOLOR"), config_.SkipDequantizationCOLOR);
  declareParam<bool>(std::string("SkipDequantizationTEX_COORD"), false);
  getParam<bool>(std::string("SkipDequantizationTEX_COORD"), config_.SkipDequantizationTEX_COORD);
  declareParam<bool>(std::string("SkipDequantizationGENERIC"), false);
  getParam<bool>(std::string("SkipDequantizationGENERIC"), config_.SkipDequantizationGENERIC);

  auto param_change_callback =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        if (parameter.get_name().find("SkipDequantizationPOSITION") != std::string::npos) {
          config_.SkipDequantizationPOSITION = parameter.as_bool();
          return result;
        } else if (parameter.get_name().find("SkipDequantizationNORMAL") != std::string::npos) {
          config_.SkipDequantizationNORMAL = parameter.as_bool();
          return result;
        } else if (parameter.get_name().find("SkipDequantizationCOLOR") != std::string::npos) {
          config_.SkipDequantizationCOLOR = parameter.as_bool();
          return result;
        } else if (parameter.get_name().find("SkipDequantizationTEX_COORD") != std::string::npos) {
          config_.SkipDequantizationTEX_COORD = parameter.as_bool();
          return result;
        } else if (parameter.get_name().find("SkipDequantizationGENERIC") != std::string::npos) {
          config_.SkipDequantizationGENERIC = parameter.as_bool();
          return result;
        }
      }
      return result;
    };
  setParamCallback(param_change_callback);
}

//! Method for converting into sensor_msgs::msg::PointCloud2
tl::expected<bool, std::string> convertDracoToPC2(
  const draco::PointCloud & pc,
  const point_cloud_interfaces::msg::CompressedPointCloud2 & compressed_PC2,
  sensor_msgs::msg::PointCloud2 & PC2)
{
  // number of all attributes of point cloud
  int32_t number_of_attributes = pc.num_attributes();

  // number of points in pointcloud
  draco::PointIndex::ValueType number_of_points = pc.num_points();

  PC2.data.resize(number_of_points * compressed_PC2.point_step);

  // for each attribute
  for (int32_t att_id = 0; att_id < number_of_attributes; att_id++) {
    // get attribute
    const draco::PointAttribute * attribute = pc.attribute(att_id);

    // check if attribute is valid
    if (!attribute->IsValid()) {
      return tl::make_unexpected(
        std::string(
          "In point_cloud_transport::DracoToPC2, attribute of Draco pointcloud is not valid!"));
    }

    // get offset of attribute in data structure
    uint32_t attribute_offset = compressed_PC2.fields[att_id].offset;

    // for each point in point cloud
    for (draco::PointIndex::ValueType point_index = 0; point_index < number_of_points;
      point_index++)
    {
      // get pointer to corresponding memory in PointCloud2 data
      uint8_t * out_data =
        &PC2.data[static_cast<int>(compressed_PC2.point_step * point_index + attribute_offset)];

      // read value from Draco pointcloud to out_data
      attribute->GetValue(draco::AttributeValueIndex(point_index), out_data);
    }
  }

  // copy PointCloud2 description (header, width, ...)
  copyCloudMetadata(PC2, compressed_PC2);

  return true;
}

DracoSubscriber::DecodeResult DracoSubscriber::decodeTyped(
  const point_cloud_interfaces::msg::CompressedPointCloud2 & compressed) const
{
  // get size of buffer with compressed data in Bytes
  uint32_t compressed_data_size = static_cast<uint32_t>(compressed.compressed_data.size());

  // empty buffer
  if (compressed_data_size == 0) {
    return tl::make_unexpected("Received compressed Draco message with zero length.");
  }

  draco::DecoderBuffer decode_buffer;
  std::vector<unsigned char> vec_data = compressed.compressed_data;

  // Sets the buffer's internal data. Note that no copy of the input data is
  // made so the data owner needs to keep the data valid and unchanged for
  // runtime of the decoder.
  decode_buffer.Init(reinterpret_cast<const char *>(&vec_data[0]), compressed_data_size);

  // create decoder object
  draco::Decoder decoder;
  // set decoder from dynamic reconfiguration
  if (config_.SkipDequantizationPOSITION) {
    decoder.SetSkipAttributeTransform(draco::GeometryAttribute::POSITION);
  }
  if (config_.SkipDequantizationNORMAL) {
    decoder.SetSkipAttributeTransform(draco::GeometryAttribute::NORMAL);
  }
  if (config_.SkipDequantizationCOLOR) {
    decoder.SetSkipAttributeTransform(draco::GeometryAttribute::COLOR);
  }
  if (config_.SkipDequantizationTEX_COORD) {
    decoder.SetSkipAttributeTransform(draco::GeometryAttribute::TEX_COORD);
  }
  if (config_.SkipDequantizationGENERIC) {
    decoder.SetSkipAttributeTransform(draco::GeometryAttribute::GENERIC);
  }

  // decode buffer into draco point cloud
  const auto res = decoder.DecodePointCloudFromBuffer(&decode_buffer);
  if (!res.ok()) {
    return tl::make_unexpected(
      "Draco decoder returned code " + std::to_string(
        res.status().code()) + ": " + res.status().error_msg() + ".");
  }

  const std::unique_ptr<draco::PointCloud> & decoded_pc = res.value();

  auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
  const auto convertRes = convertDracoToPC2(*decoded_pc, compressed, *message);
  if (!convertRes) {
    return tl::make_unexpected(convertRes.error());
  }

  return message;
}

}  // namespace draco_point_cloud_transport
