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

#include <draco_point_cloud_transport/conversion_utilities.h>
#include <draco/compression/expert_encode.h>
#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <rcpputils/tl_expected/expected.hpp>

#include <draco_point_cloud_transport/cloud.hpp>
#include <draco_point_cloud_transport/draco_publisher.hpp>

namespace draco_point_cloud_transport
{

static std::unordered_map<std::string, draco::GeometryAttribute::Type> attributeTypes = {
  {"x", draco::GeometryAttribute::Type::POSITION},
  {"y", draco::GeometryAttribute::Type::POSITION},
  {"z", draco::GeometryAttribute::Type::POSITION},
  {"pos", draco::GeometryAttribute::Type::POSITION},
  {"position", draco::GeometryAttribute::Type::POSITION},
  {"vp_x", draco::GeometryAttribute::Type::POSITION},
  {"vp_y", draco::GeometryAttribute::Type::POSITION},
  {"vp_z", draco::GeometryAttribute::Type::POSITION},
  {"rgb", draco::GeometryAttribute::Type::COLOR},
  {"rgba", draco::GeometryAttribute::Type::COLOR},
  {"r", draco::GeometryAttribute::Type::COLOR},
  {"g", draco::GeometryAttribute::Type::COLOR},
  {"b", draco::GeometryAttribute::Type::COLOR},
  {"a", draco::GeometryAttribute::Type::COLOR},
  {"nx", draco::GeometryAttribute::Type::NORMAL},
  {"ny", draco::GeometryAttribute::Type::NORMAL},
  {"nz", draco::GeometryAttribute::Type::NORMAL},
  {"normal_x", draco::GeometryAttribute::Type::NORMAL},
  {"normal_y", draco::GeometryAttribute::Type::NORMAL},
  {"normal_z", draco::GeometryAttribute::Type::NORMAL},
};

void DracoPublisher::declareParameters(const std::string & base_topic)
{
  rcl_interfaces::msg::ParameterDescriptor encode_speed_paramDescriptor;
  encode_speed_paramDescriptor.name = "encode_speed";
  encode_speed_paramDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  encode_speed_paramDescriptor.description =
    "0 = slowest speed, but the best compression 10 = fastest, but the worst compression.";
  encode_speed_paramDescriptor.set__integer_range(
    {rcl_interfaces::msg::IntegerRange()
      .set__from_value(0)
      .set__to_value(10)
      .set__step(1)});
  declareParam<int>(
    encode_speed_paramDescriptor.name, config_.encode_speed,
    encode_speed_paramDescriptor);
  getParam<int>(encode_speed_paramDescriptor.name, config_.encode_speed);

  rcl_interfaces::msg::ParameterDescriptor decode_speed_paramDescriptor;
  decode_speed_paramDescriptor.name = "decode_speed";
  decode_speed_paramDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  decode_speed_paramDescriptor.description =
    "0 = slowest speed, but the best compression 10 = fastest, but the worst compression.";
  decode_speed_paramDescriptor.set__integer_range(
    {rcl_interfaces::msg::IntegerRange()
      .set__from_value(0)
      .set__to_value(10)
      .set__step(1)});
  declareParam<int>(
    decode_speed_paramDescriptor.name, config_.decode_speed,
    decode_speed_paramDescriptor);
  getParam<int>(decode_speed_paramDescriptor.name, config_.decode_speed);

  rcl_interfaces::msg::ParameterDescriptor encode_method_paramDescriptor;
  encode_method_paramDescriptor.name = "encode_method";
  encode_method_paramDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  encode_method_paramDescriptor.description =
    "Encoding process method, 0 = auto, 1 = KD-tree, 2 = sequential";
  encode_method_paramDescriptor.set__integer_range(
    {rcl_interfaces::msg::IntegerRange()
      .set__from_value(0)
      .set__to_value(2)
      .set__step(1)});
  declareParam<int>(
    encode_method_paramDescriptor.name, config_.encode_method,
    encode_method_paramDescriptor);
  getParam<int>(encode_method_paramDescriptor.name, config_.encode_method);

  rcl_interfaces::msg::ParameterDescriptor deduplicate_paramDescriptor;
  deduplicate_paramDescriptor.name = "deduplicate";
  deduplicate_paramDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  deduplicate_paramDescriptor.description =
    "Remove duplicate point entries.";
  declareParam<bool>(deduplicate_paramDescriptor.name, true, deduplicate_paramDescriptor);
  getParam<bool>(deduplicate_paramDescriptor.name, config_.deduplicate);

  rcl_interfaces::msg::ParameterDescriptor force_quantization_paramDescriptor;
  force_quantization_paramDescriptor.name = "force_quantization";
  force_quantization_paramDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  force_quantization_paramDescriptor.description =
    "Force attribute quantization. Attributes of type float32 must be quantized for kd-tree "
    "encoding.";
  declareParam<bool>(
    force_quantization_paramDescriptor.name, config_.force_quantization,
    force_quantization_paramDescriptor);
  getParam<bool>(force_quantization_paramDescriptor.name, config_.force_quantization);

  rcl_interfaces::msg::ParameterDescriptor quantization_POSITION_paramDescriptor;
  quantization_POSITION_paramDescriptor.name = "quantization_POSITION";
  quantization_POSITION_paramDescriptor.type =
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  quantization_POSITION_paramDescriptor.description =
    "Number of bits for quantization of POSITION type attributes.";
  quantization_POSITION_paramDescriptor.set__integer_range(
    {rcl_interfaces::msg::IntegerRange()
      .set__from_value(0)
      .set__to_value(31)
      .set__step(1)});
  declareParam<int>(
    quantization_POSITION_paramDescriptor.name, config_.quantization_POSITION,
    quantization_POSITION_paramDescriptor);
  getParam<int>(quantization_POSITION_paramDescriptor.name, config_.quantization_POSITION);

  rcl_interfaces::msg::ParameterDescriptor quantization_NORMAL_paramDescriptor;
  quantization_NORMAL_paramDescriptor.name = "quantization_NORMAL";
  quantization_NORMAL_paramDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  quantization_NORMAL_paramDescriptor.description =
    "Number of bits for quantization of NORMAL type attributes.";
  quantization_NORMAL_paramDescriptor.set__integer_range(
    {rcl_interfaces::msg::IntegerRange()
      .set__from_value(0)
      .set__to_value(31)
      .set__step(1)});
  declareParam<int>(
    quantization_NORMAL_paramDescriptor.name, config_.quantization_NORMAL,
    quantization_NORMAL_paramDescriptor);
  getParam<int>(quantization_NORMAL_paramDescriptor.name, config_.quantization_NORMAL);

  rcl_interfaces::msg::ParameterDescriptor quantization_COLOR_paramDescriptor;
  quantization_COLOR_paramDescriptor.name = "quantization_COLOR";
  quantization_COLOR_paramDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  quantization_COLOR_paramDescriptor.description =
    "Number of bits for quantization of COLOR type attributes.";
  quantization_COLOR_paramDescriptor.set__integer_range(
    {rcl_interfaces::msg::IntegerRange()
      .set__from_value(0)
      .set__to_value(31)
      .set__step(1)});
  declareParam<int>(
    quantization_COLOR_paramDescriptor.name, config_.quantization_COLOR,
    quantization_COLOR_paramDescriptor);
  getParam<int>(quantization_COLOR_paramDescriptor.name, config_.quantization_COLOR);

  rcl_interfaces::msg::ParameterDescriptor quantization_TEX_COORD_paramDescriptor;
  quantization_TEX_COORD_paramDescriptor.name = "quantization_TEX_COORD";
  quantization_TEX_COORD_paramDescriptor.type =
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  quantization_TEX_COORD_paramDescriptor.description =
    "Number of bits for quantization of TEX_COORD type attributes.";
  quantization_TEX_COORD_paramDescriptor.set__integer_range(
    {rcl_interfaces::msg::IntegerRange()
      .set__from_value(0)
      .set__to_value(31)
      .set__step(1)});
  declareParam<int>(
    quantization_TEX_COORD_paramDescriptor.name, config_.quantization_TEX_COORD,
    quantization_TEX_COORD_paramDescriptor);
  getParam<int>(quantization_TEX_COORD_paramDescriptor.name, config_.quantization_TEX_COORD);

  rcl_interfaces::msg::ParameterDescriptor quantization_GENERIC_paramDescriptor;
  quantization_GENERIC_paramDescriptor.name = "quantization_GENERIC";
  quantization_GENERIC_paramDescriptor.type =
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  quantization_GENERIC_paramDescriptor.description =
    "Number of bits for quantization of GENERIC type attributes.";
  quantization_GENERIC_paramDescriptor.set__integer_range(
    {rcl_interfaces::msg::IntegerRange()
      .set__from_value(0)
      .set__to_value(31)
      .set__step(1)});
  declareParam<int>(
    quantization_GENERIC_paramDescriptor.name, config_.quantization_GENERIC,
    quantization_GENERIC_paramDescriptor);
  getParam<int>(quantization_GENERIC_paramDescriptor.name, config_.quantization_GENERIC);

  rcl_interfaces::msg::ParameterDescriptor expert_quantization_paramDescriptor;
  expert_quantization_paramDescriptor.name = "expert_quantization";
  expert_quantization_paramDescriptor.type =
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  expert_quantization_paramDescriptor.description =
    "WARNING: Apply user specified quantization for PointField entries. User must specify all "
    "entries at parameter server.";
  declareParam<bool>(
    expert_quantization_paramDescriptor.name, config_.expert_quantization,
    expert_quantization_paramDescriptor);
  getParam<bool>(expert_quantization_paramDescriptor.name, config_.expert_quantization);

  rcl_interfaces::msg::ParameterDescriptor expert_attribute_types_paramDescriptor;
  expert_attribute_types_paramDescriptor.name = "expert_attribute_types";
  expert_attribute_types_paramDescriptor.type =
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  expert_attribute_types_paramDescriptor.description =
    "WARNING: Apply user specified attribute types for PointField entries. User must specify all "
    "entries at parameter server.";
  declareParam<bool>(
    expert_attribute_types_paramDescriptor.name, config_.expert_attribute_types,
    expert_attribute_types_paramDescriptor);
  getParam<bool>(expert_attribute_types_paramDescriptor.name, config_.expert_attribute_types);

  // we call get param at runtime to get the latest value for these
  declareParam<std::string>("attribute_mapping.attribute_type.x", "POSITION");
  declareParam<std::string>("attribute_mapping.attribute_type.y", "POSITION");
  declareParam<std::string>("attribute_mapping.attribute_type.z", "POSITION");
  declareParam<int>("attribute_mapping.quantization_bits.x", 16);
  declareParam<int>("attribute_mapping.quantization_bits.y", 16);
  declareParam<int>("attribute_mapping.quantization_bits.z", 16);
  declareParam<int>("attribute_mapping.quantization_bits.rgb", 16);
  declareParam<bool>("attribute_mapping.rgba_tweak.rgb", true);
  declareParam<bool>("attribute_mapping.rgba_tweak.rgba", false);

  auto param_change_callback =
    [this](const std::vector<rclcpp::Parameter> & parameters) -> rcl_interfaces::msg::
    SetParametersResult
    {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        if (parameter.get_name().find("expert_quantization") != std::string::npos) {
          config_.expert_quantization = parameter.as_bool();
          return result;
        } else if (parameter.get_name().find("force_quantization") != std::string::npos) {
          config_.force_quantization = parameter.as_bool();
          return result;
        } else if (parameter.get_name().find("encode_speed") != std::string::npos) {
          config_.encode_speed = static_cast<int>(parameter.as_int());
          if (!(config_.encode_speed >= 0 && config_.encode_speed <= 10)) {
            RCLCPP_ERROR_STREAM(
              getLogger(), "encode_speed value range should be between [0, 10] ");
          }
          return result;
        } else if (parameter.get_name().find("decode_speed") != std::string::npos) {
          config_.decode_speed = static_cast<int>(parameter.as_int());
          if (!(config_.decode_speed >= 0 && config_.decode_speed <= 10)) {
            RCLCPP_ERROR_STREAM(
              getLogger(), "decode_speed value range should be between [0, 10] ");
          }
          return result;
        } else if (parameter.get_name().find("encode_method") != std::string::npos) {
          config_.encode_method = static_cast<int>(parameter.as_int());
          return result;
        } else if (parameter.get_name().find("deduplicate") != std::string::npos) {
          config_.deduplicate = parameter.as_bool();
          return result;
        } else if (parameter.get_name().find("quantization_POSITION") != std::string::npos) {
          config_.quantization_POSITION = static_cast<int>(parameter.as_int());
          if (!(config_.quantization_POSITION >= 1 && config_.quantization_POSITION <= 31)) {
            RCLCPP_ERROR_STREAM(
              getLogger(), "quantization_POSITION value range should be between [1, 31] ");
          }
          return result;
        } else if (parameter.get_name().find("quantization_NORMAL") != std::string::npos) {
          config_.quantization_NORMAL = static_cast<int>(parameter.as_int());
          if (!(config_.quantization_NORMAL >= 1 && config_.quantization_NORMAL <= 31)) {
            RCLCPP_ERROR_STREAM(
              getLogger(), "quantization_NORMAL value range should be between [1, 31] ");
          }
          return result;
        } else if (parameter.get_name().find("quantization_COLOR") != std::string::npos) {
          config_.quantization_COLOR = static_cast<int>(parameter.as_int());
          if (!(config_.quantization_COLOR >= 1 && config_.quantization_COLOR <= 31)) {
            RCLCPP_ERROR_STREAM(
              getLogger(), "quantization_COLOR value range should be between [1, 31] ");
          }
          return result;
        } else if (parameter.get_name().find("quantization_TEX_COORD") != std::string::npos) {
          config_.quantization_TEX_COORD = static_cast<int>(parameter.as_int());
          if (!(config_.quantization_TEX_COORD >= 1 && config_.quantization_TEX_COORD <= 31)) {
            RCLCPP_ERROR_STREAM(
              getLogger(), "quantization_TEX_COORD value range should be between [1, 31] ");
          }
          return result;
        } else if (parameter.get_name().find("quantization_GENERIC") != std::string::npos) {
          config_.quantization_GENERIC = static_cast<int>(parameter.as_int());
          if (!(config_.quantization_GENERIC >= 1 && config_.quantization_GENERIC <= 31)) {
            RCLCPP_ERROR_STREAM(
              getLogger(), "quantization_GENERIC value range should be between [1, 31] ");
          }
          return result;
        } else if (parameter.get_name().find("expert_attribute_types") != std::string::npos) {
          config_.expert_attribute_types = parameter.as_bool();
          return result;
        }
      }
      return result;
    };
  setParamCallback(param_change_callback);
}

tl::expected<std::unique_ptr<draco::PointCloud>, std::string> DracoPublisher::convertPC2toDraco(
  const sensor_msgs::msg::PointCloud2 & PC2, const std::string & topic, bool deduplicate,
  bool expert_encoding) const
{
  // object for conversion into Draco Point Cloud format
  draco::PointCloudBuilder builder;
  // number of points in point cloud
  uint64_t number_of_points = PC2.height * PC2.width;
  // initialize builder object, requires prior knowledge of point cloud size for buffer allocation
  builder.Start(static_cast<unsigned int>(number_of_points));
  // vector to hold IDs of attributes for builder object
  std::vector<int> att_ids;

  // initialize to invalid
  draco::GeometryAttribute::Type attribute_type = draco::GeometryAttribute::INVALID;
  draco::DataType attribute_data_type = draco::DT_INVALID;

  // tracks if rgb/rgba was encountered (4 colors saved as one variable,
  bool rgba_tweak = false;
  bool rgba_tweak_64bit = false;

  // tracks if all necessary parameters were set for expert encoder
  bool expert_settings_ok = true;

  std::string expert_attribute_data_type;

  // fill in att_ids with attributes from PointField[] fields
  for (const auto & field : PC2.fields) {
    if (expert_encoding) {  // find attribute type in user specified parameters
      rgba_tweak = false;
      if (getParam(
          topic + "/attribute_mapping/attribute_type/" + field.name,
          expert_attribute_data_type))
      {
        if (expert_attribute_data_type == "POSITION") {  // if data type is POSITION
          attribute_type = draco::GeometryAttribute::POSITION;
        } else if (expert_attribute_data_type == "NORMAL") {  // if data type is NORMAL
          attribute_type = draco::GeometryAttribute::NORMAL;
        } else if (expert_attribute_data_type == "COLOR") {  // if data type is COLOR
          attribute_type = draco::GeometryAttribute::COLOR;
          getParam(topic + "/attribute_mapping/rgba_tweak/" + field.name, rgba_tweak);
        } else if (expert_attribute_data_type == "TEX_COORD") {  // if data type is TEX_COORD
          attribute_type = draco::GeometryAttribute::TEX_COORD;
        } else if (expert_attribute_data_type == "GENERIC") {  // if data type is GENERIC
          attribute_type = draco::GeometryAttribute::GENERIC;
        } else {
          RCLCPP_ERROR_STREAM(
            getLogger(), "Attribute data type not recognized for " + field.name + " field entry. "
            "Using regular type recognition instead.");
          expert_settings_ok = false;
        }
      } else {
        RCLCPP_ERROR_STREAM(
          getLogger(), "Attribute data type not specified for " + field.name + " field entry."
          "Using regular type recognition instead.");
        RCLCPP_INFO_STREAM(
          getLogger(), "To set attribute type for " + field.name + " field entry, set " + topic +
          "/attribute_mapping/attribute_type/" + field.name);
        expert_settings_ok = false;
      }
    }

    // find attribute type in recognized names
    if ((!expert_encoding) || (!expert_settings_ok)) {
      rgba_tweak = field.name == "rgb" || field.name == "rgba";
      attribute_type = draco::GeometryAttribute::GENERIC;
      const auto & it = attributeTypes.find(field.name);
      if (it != attributeTypes.end()) {
        attribute_type = it->second;
      }
    }

    // attribute data type switch
    switch (field.datatype) {
      case sensor_msgs::msg::PointField::INT8: attribute_data_type = draco::DT_INT8;
        break;
      case sensor_msgs::msg::PointField::UINT8: attribute_data_type = draco::DT_UINT8;
        break;
      case sensor_msgs::msg::PointField::INT16: attribute_data_type = draco::DT_INT16;
        break;
      case sensor_msgs::msg::PointField::UINT16: attribute_data_type = draco::DT_UINT16;
        break;
      case sensor_msgs::msg::PointField::INT32: attribute_data_type = draco::DT_INT32;
        rgba_tweak_64bit = false;
        break;
      case sensor_msgs::msg::PointField::UINT32: attribute_data_type = draco::DT_UINT32;
        rgba_tweak_64bit = false;
        break;
      case sensor_msgs::msg::PointField::FLOAT32: attribute_data_type = draco::DT_FLOAT32;
        rgba_tweak_64bit = false;
        break;
      case sensor_msgs::msg::PointField::FLOAT64: attribute_data_type = draco::DT_FLOAT64;
        rgba_tweak_64bit = true;
        break;
      default: return tl::make_unexpected("Invalid data type in PointCloud2 to Draco conversion");
    }
    // attribute data type switch end

    // add attribute to point cloud builder
    if (rgba_tweak) {
      // attribute is rgb/rgba color
      if (rgba_tweak_64bit) {
        // attribute data type is 64bits long, each color is encoded in 16bits
        att_ids.push_back(builder.AddAttribute(attribute_type, 4 * field.count, draco::DT_UINT16));
      } else {
        // attribute data type is 32bits long, each color is encoded in 8bits
        att_ids.push_back(builder.AddAttribute(attribute_type, 4 * field.count, draco::DT_UINT8));
      }
    } else {
      // attribute is not rgb/rgba color, this is the default behavior
      att_ids.push_back(builder.AddAttribute(attribute_type, field.count, attribute_data_type));
    }
    // Set attribute values for the last added attribute
    if ((!att_ids.empty()) && (attribute_data_type != draco::DT_INVALID)) {
      builder.SetAttributeValuesForAllPoints(
        static_cast<int>(att_ids.back()), &PC2.data[0] + field.offset, PC2.point_step);
    }
  }
  // finalize point cloud *** builder.Finalize(bool deduplicate) ***
  std::unique_ptr<draco::PointCloud> pc = builder.Finalize(deduplicate);

  if (pc == nullptr) {
    return tl::make_unexpected(
      "Conversion from sensor_msgs::msg::PointCloud2 to Draco::PointCloud failed");
  }

  // add metadata to point cloud
  std::unique_ptr<draco::GeometryMetadata> metadata = std::make_unique<draco::GeometryMetadata>();

  if (deduplicate) {
    metadata->AddEntryInt("deduplicate", 1);  // deduplication=true flag
  } else {
    metadata->AddEntryInt("deduplicate", 0);  // deduplication=false flag
  }
  pc->AddMetadata(std::move(metadata));

  if ((pc->num_points() != number_of_points) && !deduplicate) {
    return tl::make_unexpected(
      "Number of points in Draco::PointCloud differs from sensor_msgs::msg::PointCloud2!");
  }

  return std::move(pc);  // std::move() has to be here for GCC 7
}

DracoPublisher::TypedEncodeResult DracoPublisher::encodeTyped(
  const sensor_msgs::msg::PointCloud2 & raw) const
{
  // Remove invalid points if the cloud contains them - draco cannot cope with them
  sensor_msgs::msg::PointCloud2::SharedPtr rawCleaned;
  if (!raw.is_dense) {
    rawCleaned = std::make_shared<sensor_msgs::msg::PointCloud2>();
    CREATE_FILTERED_CLOUD(
      raw, *rawCleaned, false, std::isfinite(*x_it) && std::isfinite(
        *y_it) && std::isfinite(*z_it))
  }

  const sensor_msgs::msg::PointCloud2 & rawDense = raw.is_dense ? raw : *rawCleaned;

  // Compressed message
  point_cloud_interfaces::msg::CompressedPointCloud2 compressed;

  copyCloudMetadata(compressed, rawDense);

  auto res = convertPC2toDraco(
    rawDense, base_topic_, config_.deduplicate,
    config_.expert_attribute_types);
  if (!res) {
    return tl::make_unexpected(res.error());
  }

  const auto & pc = res.value();

  if (config_.deduplicate) {
    compressed.height = 1;
    compressed.width = pc->num_points();
    compressed.row_step = compressed.width * compressed.point_step;
    compressed.is_dense = true;
  }

  draco::EncoderBuffer encode_buffer;

  // tracks if all necessary parameters were set for expert encoder
  bool expert_settings_ok = true;

  // expert encoder
  if (config_.expert_quantization) {
    draco::ExpertEncoder expert_encoder(*pc);
    expert_encoder.SetSpeedOptions(config_.encode_speed, config_.decode_speed);

    // default
    if ((config_.encode_method == 0) && (!config_.force_quantization)) {
      // let draco handle method selection
    } else if ((config_.encode_method == 1) || (config_.force_quantization)) {
      // force kd tree
      if (config_.force_quantization) {
        // keep track of which attribute is being processed
        int att_id = 0;
        int attribute_quantization_bits;

        for (const auto & field : rawDense.fields) {
          if (getParam(
              base_topic_ + "/attribute_mapping/quantization_bits/" + field.name,
              attribute_quantization_bits))
          {
            expert_encoder.SetAttributeQuantization(att_id, attribute_quantization_bits);
          } else {
            RCLCPP_ERROR_STREAM(
              getLogger(), "Attribute quantization not specified for " + field.name +
              " field entry. Using regular encoder instead.");
            RCLCPP_INFO_STREAM(
              getLogger(), "To set quantization for " + field.name + " field entry, set " +
              base_topic_ + "/attribute_mapping/quantization_bits/" + field.name);
            expert_settings_ok = false;
          }
          att_id++;
        }
      }
      expert_encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
    } else {
      // force sequential encoding
      expert_encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
    }

    // encodes point cloud and raises error if encoding fails
    // draco::Status status = encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);
    draco::Status status = expert_encoder.EncodeToBuffer(&encode_buffer);

    if (status.code() != 0) {
      // TODO(anyone): Fix with proper format
      return tl::make_unexpected(
        "Draco encoder returned code " + std::to_string(
          status.code()) + ": " + status.error_msg() + ".");
    }
  }
  // expert encoder end
  // regular encoder
  if ((!config_.expert_quantization) || (!expert_settings_ok)) {
    draco::Encoder encoder;
    encoder.SetSpeedOptions(config_.encode_speed, config_.decode_speed);

    // default
    if ((config_.encode_method == 0) && (!config_.force_quantization)) {
      // let draco handle method selection
    } else if ((config_.encode_method == 1) || (config_.force_quantization)) {
      // force kd tree
      if (config_.force_quantization) {
        encoder.SetAttributeQuantization(
          draco::GeometryAttribute::POSITION,
          config_.quantization_POSITION);
        encoder.SetAttributeQuantization(
          draco::GeometryAttribute::NORMAL,
          config_.quantization_NORMAL);
        encoder.SetAttributeQuantization(
          draco::GeometryAttribute::COLOR,
          config_.quantization_COLOR);
        encoder.SetAttributeQuantization(
          draco::GeometryAttribute::TEX_COORD,
          config_.quantization_TEX_COORD);
        encoder.SetAttributeQuantization(
          draco::GeometryAttribute::GENERIC,
          config_.quantization_GENERIC);
      }
      encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
    } else {
      // force sequential encoding
      encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
    }

    // encodes point cloud and raises error if encoding fails
    draco::Status status = encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);

    if (!status.ok()) {
      return tl::make_unexpected(
        "Draco encoder returned code " + std::to_string(
          status.code()) + ": " + status.error_msg() + ".");
    }
  }
  // regular encoder end

  uint32_t compressed_data_size = static_cast<uint32_t>(encode_buffer.size());
  auto cast_buffer = reinterpret_cast<const unsigned char *>(encode_buffer.data());
  std::vector<unsigned char> vec_data(cast_buffer, cast_buffer + compressed_data_size);
  compressed.compressed_data = vec_data;
  compressed.format = getTransportName();

  return compressed;
}

void DracoPublisher::registerPositionField(const std::string & field)
{
  attributeTypes[field] = draco::GeometryAttribute::Type::POSITION;
}

void DracoPublisher::registerColorField(const std::string & field)
{
  attributeTypes[field] = draco::GeometryAttribute::Type::COLOR;
}

void DracoPublisher::registerNormalField(const std::string & field)
{
  attributeTypes[field] = draco::GeometryAttribute::Type::NORMAL;
}

}  // namespace draco_point_cloud_transport
