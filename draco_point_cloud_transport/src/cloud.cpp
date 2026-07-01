/*
 * Copyright (c) 2023, Czech Technical University in Prague
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

// HACK: we need to access PointCloud2IteratorBase::data_char_ which is private
#include <algorithm>
#include <format>
#include <string>
#include <sstream>
#include <cstring>

#define private protected
#include <sensor_msgs/point_cloud2_iterator.hpp>
#undef private


#include <draco_point_cloud_transport/cloud.hpp>

namespace cras
{

bool hasField(const ::cras::Cloud & cloud, const std::string & fieldName)
{
  return std::any_of(
    cloud.fields.begin(), cloud.fields.end(),
    [&fieldName](const sensor_msgs::msg::PointField & f) {return f.name == fieldName;});
}

sensor_msgs::msg::PointField & getField(::cras::Cloud & cloud, const std::string & fieldName)
{
  for (auto & field : cloud.fields) {
    if (field.name == fieldName) {
      return field;
    }
  }
  throw std::runtime_error(std::format("Field {} does not exist.", fieldName));
}

const sensor_msgs::msg::PointField & getField(
  const ::cras::Cloud & cloud,
  const std::string & fieldName)
{
  for (const auto & field : cloud.fields) {
    if (field.name == fieldName) {
      return field;
    }
  }
  throw std::runtime_error(std::format("Field {} does not exist.", fieldName));
}

size_t sizeOfPointField(const ::sensor_msgs::msg::PointField & field)
{
  return sizeOfPointField(field.datatype);
}

size_t sizeOfPointField(int datatype)
{
  if ((datatype == sensor_msgs::msg::PointField::INT8) ||
    (datatype == sensor_msgs::msg::PointField::UINT8))
  {
    return 1u;
  } else if ((datatype == sensor_msgs::msg::PointField::INT16) ||  // NOLINT
    (datatype == sensor_msgs::msg::PointField::UINT16))
  {
    return 2u;
  } else if ((datatype == sensor_msgs::msg::PointField::INT32) ||  // NOLINT
    (datatype == sensor_msgs::msg::PointField::UINT32) ||
    (datatype == sensor_msgs::msg::PointField::FLOAT32))
  {
    return 4u;
  } else if (datatype == sensor_msgs::msg::PointField::FLOAT64) {
    return 8u;
  } else {
    throw std::runtime_error(
            std::format("PointField of type {} does not exist", datatype));
  }
}

void copyChannelData(const ::cras::Cloud & in, ::cras::Cloud & out, const std::string & fieldName)
{
  if (numPoints(out) < numPoints(in)) {
    throw std::runtime_error(
            "Output cloud needs to be resized to fit the number of points of the input cloud.");
  }

  GenericCloudConstIter dataIn(in, fieldName);
  GenericCloudIter dataOut(out, fieldName);
  for (; dataIn != dataIn.end(); ++dataIn, ++dataOut) {
    dataOut.copyData(dataIn);
  }
}

namespace impl
{

template<typename T, typename TT, typename U, typename C, template<typename> class V>
GenericCloudIteratorBase<T, TT, U, C, V>::GenericCloudIteratorBase(
  C & cloudMsg,
  const std::string & fieldName)
: sensor_msgs::impl::PointCloud2IteratorBase<T, TT, U, C, V>(cloudMsg, fieldName)
{
  this->fieldSize = sizeOfPointField(getField(cloudMsg, fieldName));
}

template<typename T, typename TT, typename U, typename C, template<typename> class V>
U * GenericCloudIteratorBase<T, TT, U, C, V>::rawData() const
{
  return this->data_char_;
}

template<typename T>
void GenericCloudIterator<T>::copyData(const GenericCloudConstIterator<T> & otherIter) const
{
  memcpy(this->rawData(), otherIter.rawData(), this->fieldSize);
}

template<typename T>
void GenericCloudIterator<T>::copyData(const GenericCloudIterator<T> & otherIter) const
{
  memcpy(this->rawData(), otherIter.rawData(), this->fieldSize);
}

// explicitly instantiate
template class GenericCloudIteratorBase<unsigned char, unsigned char, unsigned char,
  sensor_msgs::msg::PointCloud2, GenericCloudIterator>;
template class GenericCloudIteratorBase<unsigned char, const unsigned char, const unsigned char,
  const sensor_msgs::msg::PointCloud2, GenericCloudConstIterator>;

template class GenericCloudIterator<>;
template class GenericCloudConstIterator<>;
}  // namespace impl

}  // namespace cras
