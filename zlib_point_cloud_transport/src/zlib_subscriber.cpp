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

#include <list>
#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <zlib_point_cloud_transport/zlib_subscriber.hpp>

#include "zlib_cpp.hpp"

namespace zlib_point_cloud_transport
{
void ZlibSubscriber::declareParameters()
{
}

ZlibSubscriber::DecodeResult ZlibSubscriber::decodeTyped(
  const point_cloud_interfaces::msg::CompressedPointCloud2 & msg) const
{
  auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();

  zlib::Decomp decomp;

  std::shared_ptr<zlib::DataBlock> data = zlib::AllocateData(msg.compressed_data.size());
  memcpy(data->ptr, &msg.compressed_data[0], msg.compressed_data.size());

  std::list<std::shared_ptr<zlib::DataBlock>> out_data_list;
  out_data_list = decomp.Process(data);

  std::shared_ptr<zlib::DataBlock> data2 = zlib::ExpandDataList(out_data_list);

  result->data.resize(data2->size);
  memcpy(&result->data[0], data2->ptr, data2->size);

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

}  // namespace zlib_point_cloud_transport
