#pragma once

#include <cmath>
#include <chrono>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "bboxes_ex_msgs/msg/bounding_box.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"

#include "yolox_cpp/yolox.hpp"
#include "yolox_cpp/utils.hpp"
#include "yolox_param/yolox_param.hpp"

namespace yolox_ros_cpp{
    class YoloXNode : public rclcpp::Node
    {
    public:
        YoloXNode(const rclcpp::NodeOptions &);
    private:
        void onInit();
        void colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &);

        static bboxes_ex_msgs::msg::BoundingBoxes objects_to_bboxes(const cv::Mat &, const std::vector<yolox_cpp::Object> &, const std_msgs::msg::Header &);
        static vision_msgs::msg::Detection2DArray objects_to_detection2d(const std::vector<yolox_cpp::Object> &, const std_msgs::msg::Header &);

    protected:
        std::shared_ptr<yolox_parameters::ParamListener> param_listener_;
        yolox_parameters::Params params_;
    private:
        std::unique_ptr<yolox_cpp::AbcYoloX> yolox_;
        std::vector<std::string> class_names_;

        rclcpp::TimerBase::SharedPtr init_timer_;
        image_transport::Subscriber sub_image_;

        rclcpp::Publisher<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr pub_bboxes_;
        rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_detection2d_;
        image_transport::Publisher pub_image_;
    };
}
