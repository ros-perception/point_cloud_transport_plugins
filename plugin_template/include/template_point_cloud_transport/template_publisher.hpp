/*
 * Copyright (c) <Current Year>, <Your Name Here> (if desired)
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

#ifndef TEMPLATE_POINT_CLOUD_TRANSPORT__TEMPLATE_PUBLISHER_HPP_
#define TEMPLATE_POINT_CLOUD_TRANSPORT__TEMPLATE_PUBLISHER_HPP_

#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>

#include <point_cloud_transport/simple_publisher_plugin.hpp>
#include <point_cloud_interfaces/msg/custom_message.hpp>

namespace template_point_cloud_transport
{

// CustomMessage could be any ROS2 message.
// e.g. point_cloud_interfaces::msg::CompressedPointCloud2, or you can make a new one. If you do, please
// add the definiton to point_cloud_interfaces.
class TemplatePublisher
  : public point_cloud_transport::SimplePublisherPlugin<
    point_cloud_interfaces::msg::CustomMessage>
{
public:
  void declareParameters(const std::string & base_topic) override;

  std::string getDataType() const override
  {
    // return the name of whichever message you chose as a string
    return "point_cloud_interfaces/msg/CustomMessage";
  }

  TypedEncodeResult encodeTyped(const sensor_msgs::msg::PointCloud2 & raw) const override;

private:
  // good place to put any internal variables (e.g. compression algo parameters)
};
}  // namespace template_point_cloud_transport

#endif  // TEMPLATE_POINT_CLOUD_TRANSPORT__TEMPLATE_PUBLISHER_HPP_
