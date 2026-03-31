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

#ifndef DRACO_POINT_CLOUD_TRANSPORT__DRACO_PUBLISHER_HPP_
#define DRACO_POINT_CLOUD_TRANSPORT__DRACO_PUBLISHER_HPP_

#include <draco/point_cloud/point_cloud.h>

#include <memory>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/simple_publisher_plugin.hpp>

#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>

namespace draco_point_cloud_transport
{

class DracoPublisher
  : public point_cloud_transport::SimplePublisherPlugin<
    point_cloud_interfaces::msg::CompressedPointCloud2>
{
public:
  void declareParameters(const std::string & base_topic) override;

  std::string getDataType() const override
  {
    return "point_cloud_interfaces/msg/CompressedPointCloud2";
  }

  TypedEncodeResult encodeTyped(const sensor_msgs::msg::PointCloud2 & raw) const override;

  static void registerPositionField(const std::string & field);
  static void registerColorField(const std::string & field);
  static void registerNormalField(const std::string & field);

private:
  tl::expected<std::unique_ptr<draco::PointCloud>, std::string> convertPC2toDraco(
    const sensor_msgs::msg::PointCloud2 & PC2, const std::string & topic, bool deduplicate,
    bool expert_encoding) const;

  class DracoPublisherConfig
  {
public:
    int encode_speed = 7;
    int decode_speed = 7;
    int encode_method = 0;
    bool deduplicate = true;
    bool force_quantization = false;
    int quantization_POSITION = 14;
    int quantization_NORMAL = 14;
    int quantization_COLOR = 14;
    int quantization_TEX_COORD = 14;
    int quantization_GENERIC = 14;
    bool expert_quantization = false;
    bool expert_attribute_types = false;
  };

  DracoPublisherConfig config_;
};
}  // namespace draco_point_cloud_transport

#endif  // DRACO_POINT_CLOUD_TRANSPORT__DRACO_PUBLISHER_HPP_
