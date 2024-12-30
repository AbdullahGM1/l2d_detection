/*
BSD 3-Clause License

Copyright (c) 2024, Khaled Hammad Gabr, Abdallah AlMusalami
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include <vector>
#include <stdexcept>
#include "pcl/filters/extract_indices.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include <pcl/filters/crop_box.h>
#include <algorithm> 

class PointCloudToDepthMap : public rclcpp::Node
{
public:
    PointCloudToDepthMap()
        : Node("point_cloud_to_depth_map")
    {
        // Declare parameters with default values
        this->declare_parameter<int>("width", 650);
        this->declare_parameter<int>("height", 650);
        this->declare_parameter<float>("MinDepth", 0.2f);
        this->declare_parameter<float>("MaxDepth", 30.0f);

        // Fetch parameters
        this->get_parameter("width", width_);
        this->get_parameter("height", height_);
        this->get_parameter("MinDepth", MinDepth_);
        this->get_parameter("MaxDepth", MaxDepth_);

        // Compute scale. We multiply by 4.f as a design choice 
        // to magnify the projection in the 2D depth image.
        float scale_w = static_cast<float>(width_) / (2.0f * MaxDepth_);
        float scale_h = static_cast<float>(height_) / (2.0f * MaxDepth_);
        scale_ = std::min(scale_w, scale_h) * 4.f;

        RCLCPP_INFO(
            get_logger(),
            "Parameters: width=%d, height=%d, scale=%.2f, MinDepth=%.2f, MaxDepth=%.2f",
            width_, height_, scale_, MinDepth_, MaxDepth_);

        // 1) Simple subscriber to build + publish the "full" depth map.
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scan/points", 10,
            std::bind(&PointCloudToDepthMap::point_cloud_callback, this, std::placeholders::_1));

        // 2) message_filters subscribers for synchronized callback (pointcloud + detections).
        pointcloud_sub_.subscribe(this, "/scan/points");
        detection_sub_.subscribe(this, "/depth_map/tracking");

        // Create synchronization policy
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), pointcloud_sub_, detection_sub_);
        sync_->registerCallback(
            std::bind(&PointCloudToDepthMap::sync_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Publishers
        original_publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>("/depth_map", 10);

        detected_object_publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>("/detected_object_depth_map", 10);

        detected_object_pose_publisher_ =
            this->create_publisher<geometry_msgs::msg::PoseArray>("/detected_object_depthmap_pose", 10);

        RCLCPP_INFO(this->get_logger(), "PointCloud to Depth Map Node has been started.");
    }

private:
    //--------------------------------------------------------------------------
    // Helper function to project a PCL point's (X,Y) into 2D pixel coordinates,
    //--------------------------------------------------------------------------
    std::pair<int, int> projectToPixel(const pcl::PointXYZ &pt) const
    {
        // center_x, center_y
        int cx = width_ / 2;
        int cy = height_ / 2;

        // invert the sign on Y and X to match original approach.
        int pixel_x = cx - static_cast<int>(std::round(pt.y * scale_));
        int pixel_y = cy - static_cast<int>(std::round(pt.x * scale_));

        return std::make_pair(pixel_x, pixel_y);
    }

    //--------------------------------------------------------------------------
    // Helper to fill two single-channel images for +Z and -Z from a filtered cloud.
    //--------------------------------------------------------------------------
    void fillDepthImages(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                         cv::Mat &pos_z_image,
                         cv::Mat &neg_z_image) const
    {
        // pos_z_image and neg_z_image must be CV_8UC1, initialized to zeros.
        for (const auto &point : cloud->points)
        {
            // Project (x, y) into pixel coords
            auto [px, py] = projectToPixel(point);

            // Check bounds
            if (px < 0 || px >= width_ || py < 0 || py >= height_)
                continue;

            if (point.z > 0.0f)  // +Z
            {
                int depth_value = std::clamp(
                    static_cast<int>((point.z / MaxDepth_) * 255), 0, 255);

                // Invert grayscale (closer = brighter)
                pos_z_image.at<uint8_t>(py, px) = static_cast<uint8_t>(255 - depth_value);
            }
            else if (point.z < 0.0f)  // -Z
            {
                // For negative z, consider the magnitude (-point.z)
                int depth_value = std::clamp(
                    static_cast<int>((-point.z / MaxDepth_) * 255), 0, 255);

                neg_z_image.at<uint8_t>(py, px) = static_cast<uint8_t>(255 - depth_value);
            }
        }
    }

    //--------------------------------------------------------------------------
    // subscription callback to publish a "full" depth map (no bounding boxes).
    //--------------------------------------------------------------------------
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 -> PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filter_point_cloud(pcl_cloud);

        // Create single-channel images for +Z and -Z
        cv::Mat pos_z_img(height_, width_, CV_8UC1, cv::Scalar(0));
        cv::Mat neg_z_img(height_, width_, CV_8UC1, cv::Scalar(0));

        // Fill depth images
        fillDepthImages(filtered_cloud, pos_z_img, neg_z_img);

        // Convert to 3-channel BGR for visualization
        cv::Mat pos_bgr, neg_bgr;
        cv::cvtColor(pos_z_img, pos_bgr, cv::COLOR_GRAY2BGR);
        cv::cvtColor(neg_z_img, neg_bgr, cv::COLOR_GRAY2BGR);

        // Combine horizontally
        cv::Mat combined;
        cv::hconcat(pos_bgr, neg_bgr, combined);

        // Convert to ROS message
        auto combined_image_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", combined).toImageMsg();

        // Preserve timestamp/frame_id
        combined_image_msg->header = msg->header;

        // Publish
        original_publisher_->publish(*combined_image_msg);
    }

    //--------------------------------------------------------------------------
    // Filtering logic: CropBox, then pass-through on Z to keep z >= MinDepth or z <= -MinDepth
    //--------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_point_cloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) const
    {
        // 1) CropBox: clamp to [-MaxDepth_, +MaxDepth_] in X/Y/Z
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setInputCloud(input_cloud);
        box_filter.setMin(Eigen::Vector4f(-MaxDepth_, -MaxDepth_, -MaxDepth_, 1.0f));
        box_filter.setMax(Eigen::Vector4f(+MaxDepth_, +MaxDepth_, +MaxDepth_, 1.0f));

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cropped(new pcl::PointCloud<pcl::PointXYZ>);
        box_filter.filter(*temp_cropped);

        // 2) Keep z >= MinDepth_ or z <= -MinDepth_
        pcl::PointCloud<pcl::PointXYZ>::Ptr z_positive(new pcl::PointCloud<pcl::PointXYZ>);
        {
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(temp_cropped);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(MinDepth_, MaxDepth_);
            pass.filter(*z_positive);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr z_negative(new pcl::PointCloud<pcl::PointXYZ>);
        {
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(temp_cropped);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(-MaxDepth_, -MinDepth_);
            pass.filter(*z_negative);
        }

        // Combine
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *final_cloud += *z_positive;
        *final_cloud += *z_negative;

        return final_cloud;
    }

    //--------------------------------------------------------------------------
    // Synchronized callback: (PointCloud2 + YOLO detections).
    //--------------------------------------------------------------------------
    void sync_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg,
        const yolov8_msgs::msg::DetectionArray::ConstSharedPtr detection_msg)
    {
        // Prepare PoseArray
        geometry_msgs::msg::PoseArray detected_object_poses;
        detected_object_poses.header = pointcloud_msg->header;

        // Convert ROS -> PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pointcloud_msg, *pcl_cloud);

        // Filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filter_point_cloud(pcl_cloud);

        // Prepare single-channel images
        cv::Mat pos_z_img(height_, width_, CV_8UC1, cv::Scalar(0));
        cv::Mat neg_z_img(height_, width_, CV_8UC1, cv::Scalar(0));

        for (const auto &bbox : detection_msg->detections)
        {
            double x_center = bbox.bbox.center.position.x;
            double y_center = bbox.bbox.center.position.y;
            double box_w = bbox.bbox.size.x;
            double box_h = bbox.bbox.size.y;

            double x_min = x_center - (box_w / 2.0);
            double x_max = x_center + (box_w / 2.0);
            double y_min = y_center - (box_h / 2.0);
            double y_max = y_center + (box_h / 2.0);

            double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
            int point_count = 0;

            // Iterate over the filtered cloud to see which points fall within the bounding box
            for (const auto &point : filtered_cloud->points)
            {
                auto [px, py] = projectToPixel(point);

                // Check if pixel is within the bounding box range
                if (px >= x_min && px <= x_max &&
                    py >= y_min && py <= y_max)
                {
                    // Mark the depth on the images
                    if (point.z > 0.0f)
                    {
                        int depth_val = std::clamp(
                            static_cast<int>((point.z / MaxDepth_) * 255), 0, 255);
                        pos_z_img.at<uint8_t>(py, px) = static_cast<uint8_t>(255 - depth_val);
                    }
                    else if (point.z < 0.0f)
                    {
                        int depth_val = std::clamp(
                            static_cast<int>((-point.z / MaxDepth_) * 255), 0, 255);
                        neg_z_img.at<uint8_t>(py, px) = static_cast<uint8_t>(255 - depth_val);
                    }

                    // Accumulate for pose
                    sum_x += point.x;
                    sum_y += point.y;
                    sum_z += point.z;
                    point_count++;
                }
            }

            // If we found any points for this detection, compute the average position
            if (point_count > 0)
            {
                geometry_msgs::msg::Pose object_pose;
                object_pose.position.x = sum_x / point_count;
                object_pose.position.y = sum_y / point_count;
                object_pose.position.z = sum_z / point_count;

                // Identity orientation
                object_pose.orientation.x = 0.0;
                object_pose.orientation.y = 0.0;
                object_pose.orientation.z = 0.0;
                object_pose.orientation.w = 1.0;

                detected_object_poses.poses.push_back(object_pose);
            }
        }

        // Publish the poses
        detected_object_pose_publisher_->publish(detected_object_poses);

        // Convert single-channel images to BGR
        cv::Mat pos_bgr, neg_bgr, combined;
        cv::cvtColor(pos_z_img, pos_bgr, cv::COLOR_GRAY2BGR);
        cv::cvtColor(neg_z_img, neg_bgr, cv::COLOR_GRAY2BGR);
        cv::hconcat(pos_bgr, neg_bgr, combined);

        // Convert to ROS image
        auto detected_object_image_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", combined).toImageMsg();
        detected_object_image_msg->header = pointcloud_msg->header;

        detected_object_publisher_->publish(*detected_object_image_msg);
    }

    //--------------------------------------------------------------------------
    // Member variables
    //--------------------------------------------------------------------------
    // 1) subscription for full cloud → depth map
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    // 2) Synchronized subscribers + policy
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_;
    message_filters::Subscriber<yolov8_msgs::msg::DetectionArray> detection_sub_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        yolov8_msgs::msg::DetectionArray>;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr original_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detected_object_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr detected_object_pose_publisher_;

    // Parameters
    int width_;
    int height_;
    float scale_;    
    float MinDepth_;
    float MaxDepth_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToDepthMap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
