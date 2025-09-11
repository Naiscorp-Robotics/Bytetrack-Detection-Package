// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <parameter_traits/parameter_traits.hpp>

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>



namespace yolox_parameters {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

// temporarily needed for backwards compatibility for custom validators
using namespace parameter_traits;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        bool imshow_isshow = false;
        std::string model_path = "example.onnx";
        std::string class_labels_path = "";
        int64_t num_classes = 1;
        bool is_nchw = true;
        bool p6 = false;
        double conf = 0.3;
        double nms = 0.45;
        int64_t tensorrt_device = 0;
        std::string openvino_device = "AUTO";
        bool onnxruntime_use_cuda = true;
        int64_t onnxruntime_device_id = 0;
        bool onnxruntime_use_parallel = false;
        int64_t onnxruntime_inter_op_num_threads = 1;
        int64_t onnxruntime_intra_op_num_threads = 1;
        int64_t tflite_num_threads = 1;
        std::string model_type = "tflite";
        std::string model_version = "0.1.1rc0";
        std::string src_image_topic_name = "image_raw";
        std::string publish_image_topic_name = "yolox/image_raw";
        std::string publish_boundingbox_topic_name = "yolox/bounding_boxes";
        bool use_bbox_ex_msgs = false;
        bool publish_resized_image = false;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        bool imshow_isshow = false;
        int64_t num_classes = 1;
        bool is_nchw = true;
        bool p6 = false;
        double conf = 0.3;
        double nms = 0.45;
        int64_t tensorrt_device = 0;
        bool onnxruntime_use_cuda = true;
        int64_t onnxruntime_device_id = 0;
        bool onnxruntime_use_parallel = false;
        int64_t onnxruntime_inter_op_num_threads = 1;
        int64_t onnxruntime_intra_op_num_threads = 1;
        int64_t tflite_num_threads = 1;
        bool use_bbox_ex_msgs = false;
        bool publish_resized_image = false;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    ParamListener(rclcpp::Node::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("yolox_parameters"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "")
    : prefix_{prefix},
      logger_{std::move(logger)} {
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    /**
     * @brief Tries to update the parsed Params object
     * @param params_in The Params object to update
     * @return true if the Params object was updated, false if it was already up to date or the mutex could not be locked
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_update_params(Params & params_in) const {
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
          return true;
        }
      }
      return false;
    }

    /**
     * @brief Tries to get the current Params object
     * @param params_in The Params object to fill with the current parameters
     * @return true if mutex can be locked, false if mutex could not be locked
     * @note The parameters are only filled, when the mutex can be locked and the params timestamp is different
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_get_params(Params & params_in) const {
      if (mutex_.try_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
        }
        mutex_.unlock();
        return true;
      }
      return false;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.imshow_isshow = params.imshow_isshow;
      output.num_classes = params.num_classes;
      output.is_nchw = params.is_nchw;
      output.p6 = params.p6;
      output.conf = params.conf;
      output.nms = params.nms;
      output.tensorrt_device = params.tensorrt_device;
      output.onnxruntime_use_cuda = params.onnxruntime_use_cuda;
      output.onnxruntime_device_id = params.onnxruntime_device_id;
      output.onnxruntime_use_parallel = params.onnxruntime_use_parallel;
      output.onnxruntime_inter_op_num_threads = params.onnxruntime_inter_op_num_threads;
      output.onnxruntime_intra_op_num_threads = params.onnxruntime_intra_op_num_threads;
      output.tflite_num_threads = params.tflite_num_threads;
      output.use_bbox_ex_msgs = params.use_bbox_ex_msgs;
      output.publish_resized_image = params.publish_resized_image;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "imshow_isshow")) {
            updated_params.imshow_isshow = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "model_path")) {
            updated_params.model_path = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "class_labels_path")) {
            updated_params.class_labels_path = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "num_classes")) {
            updated_params.num_classes = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "is_nchw")) {
            updated_params.is_nchw = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "p6")) {
            updated_params.p6 = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "conf")) {
            updated_params.conf = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "nms")) {
            updated_params.nms = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "tensorrt_device")) {
            updated_params.tensorrt_device = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "openvino_device")) {
            if(auto validation_result = one_of<std::string>(param, {"AUTO", "CPU", "GPU", "NPU", "MYRIAD"});
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.openvino_device = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "onnxruntime_use_cuda")) {
            updated_params.onnxruntime_use_cuda = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "onnxruntime_device_id")) {
            updated_params.onnxruntime_device_id = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "onnxruntime_use_parallel")) {
            updated_params.onnxruntime_use_parallel = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "onnxruntime_inter_op_num_threads")) {
            updated_params.onnxruntime_inter_op_num_threads = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "onnxruntime_intra_op_num_threads")) {
            updated_params.onnxruntime_intra_op_num_threads = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "tflite_num_threads")) {
            updated_params.tflite_num_threads = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "model_type")) {
            updated_params.model_type = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "model_version")) {
            updated_params.model_version = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "src_image_topic_name")) {
            updated_params.src_image_topic_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "publish_image_topic_name")) {
            updated_params.publish_image_topic_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "publish_boundingbox_topic_name")) {
            updated_params.publish_boundingbox_topic_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "use_bbox_ex_msgs")) {
            updated_params.use_bbox_ex_msgs = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "publish_resized_image")) {
            updated_params.publish_resized_image = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
      if (user_callback_) {
         user_callback_(updated_params);
      }
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "imshow_isshow")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable or disable imshow.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.imshow_isshow);
          parameters_interface_->declare_parameter(prefix_ + "imshow_isshow", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "model_path")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Path to the model file.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.model_path);
          parameters_interface_->declare_parameter(prefix_ + "model_path", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "class_labels_path")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Path to the class labels file.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.class_labels_path);
          parameters_interface_->declare_parameter(prefix_ + "class_labels_path", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "num_classes")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Number of classes.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.num_classes);
          parameters_interface_->declare_parameter(prefix_ + "num_classes", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "is_nchw")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable or disable NCHW.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.is_nchw);
          parameters_interface_->declare_parameter(prefix_ + "is_nchw", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "p6")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable or disable P6.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.p6);
          parameters_interface_->declare_parameter(prefix_ + "p6", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "conf")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Confidence threshold.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.conf);
          parameters_interface_->declare_parameter(prefix_ + "conf", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "nms")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "NMS threshold.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.nms);
          parameters_interface_->declare_parameter(prefix_ + "nms", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "tensorrt_device")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "TensorRT device.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.tensorrt_device);
          parameters_interface_->declare_parameter(prefix_ + "tensorrt_device", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "openvino_device")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "OpenVINO device.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.openvino_device);
          parameters_interface_->declare_parameter(prefix_ + "openvino_device", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "onnxruntime_use_cuda")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable or disable CUDA.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.onnxruntime_use_cuda);
          parameters_interface_->declare_parameter(prefix_ + "onnxruntime_use_cuda", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "onnxruntime_device_id")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "ONNXRuntime device ID.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.onnxruntime_device_id);
          parameters_interface_->declare_parameter(prefix_ + "onnxruntime_device_id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "onnxruntime_use_parallel")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable or disable parallel.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.onnxruntime_use_parallel);
          parameters_interface_->declare_parameter(prefix_ + "onnxruntime_use_parallel", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "onnxruntime_inter_op_num_threads")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "ONNXRuntime inter op num threads.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.onnxruntime_inter_op_num_threads);
          parameters_interface_->declare_parameter(prefix_ + "onnxruntime_inter_op_num_threads", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "onnxruntime_intra_op_num_threads")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "ONNXRuntime intra op num threads.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.onnxruntime_intra_op_num_threads);
          parameters_interface_->declare_parameter(prefix_ + "onnxruntime_intra_op_num_threads", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "tflite_num_threads")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "TFLite num threads.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.tflite_num_threads);
          parameters_interface_->declare_parameter(prefix_ + "tflite_num_threads", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "model_type")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Model type.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.model_type);
          parameters_interface_->declare_parameter(prefix_ + "model_type", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "model_version")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Model version.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.model_version);
          parameters_interface_->declare_parameter(prefix_ + "model_version", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "src_image_topic_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Source image topic name.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.src_image_topic_name);
          parameters_interface_->declare_parameter(prefix_ + "src_image_topic_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "publish_image_topic_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Publish image topic name.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.publish_image_topic_name);
          parameters_interface_->declare_parameter(prefix_ + "publish_image_topic_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "publish_boundingbox_topic_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Publish bounding box topic name.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.publish_boundingbox_topic_name);
          parameters_interface_->declare_parameter(prefix_ + "publish_boundingbox_topic_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "use_bbox_ex_msgs")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable or disable bbox_ex_msgs. If true, disable vision_msgs::Detection2DArray.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.use_bbox_ex_msgs);
          parameters_interface_->declare_parameter(prefix_ + "use_bbox_ex_msgs", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "publish_resized_image")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enable or disable resized image.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.publish_resized_image);
          parameters_interface_->declare_parameter(prefix_ + "publish_resized_image", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "imshow_isshow");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.imshow_isshow = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "model_path");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.model_path = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "class_labels_path");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.class_labels_path = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "num_classes");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.num_classes = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "is_nchw");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.is_nchw = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "p6");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.p6 = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "conf");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.conf = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "nms");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.nms = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "tensorrt_device");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.tensorrt_device = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "openvino_device");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = one_of<std::string>(param, {"AUTO", "CPU", "GPU", "NPU", "MYRIAD"});
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'openvino_device': {}", validation_result.error()));
      }
      updated_params.openvino_device = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "onnxruntime_use_cuda");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.onnxruntime_use_cuda = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "onnxruntime_device_id");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.onnxruntime_device_id = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "onnxruntime_use_parallel");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.onnxruntime_use_parallel = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "onnxruntime_inter_op_num_threads");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.onnxruntime_inter_op_num_threads = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "onnxruntime_intra_op_num_threads");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.onnxruntime_intra_op_num_threads = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "tflite_num_threads");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.tflite_num_threads = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "model_type");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.model_type = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "model_version");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.model_version = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "src_image_topic_name");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.src_image_topic_name = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "publish_image_topic_name");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.publish_image_topic_name = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "publish_boundingbox_topic_name");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.publish_boundingbox_topic_name = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "use_bbox_ex_msgs");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.use_bbox_ex_msgs = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "publish_resized_image");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.publish_resized_image = param.as_bool();


      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
    }

    using userParameterUpdateCB = std::function<void(const Params&)>;
    void setUserCallback(const userParameterUpdateCB& callback){
      user_callback_ = callback;
    }

    void clearUserCallback(){
      user_callback_ = {};
    }

    private:
      void update_internal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = std::move(updated_params);
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;
      userParameterUpdateCB user_callback_;

      rclcpp::Logger logger_;
      std::mutex mutable mutex_;
  };

} // namespace yolox_parameters
