//Final Code with Khaled update and sync and filtter all axis and two depth maps for (+z) and (-z)

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

class PointCloudToDepthMap : public rclcpp::Node
{
public:
    PointCloudToDepthMap()
        : Node("point_cloud_to_depth_map")
    {
        // Declare parameters with default values
        this->declare_parameter<int>("width", 650);
        this->declare_parameter<int>("height", 650);
        this->declare_parameter<float>("scale", 50.0);
        this->declare_parameter<float>("MinDepth", 0.2f);
        this->declare_parameter<float>("MaxDepth", 30.0f);

        // Fetch parameters
        this->get_parameter("width", width_);
        this->get_parameter("height", height_);
        this->get_parameter("scale", scale_);
        this->get_parameter("MinDepth", MinDepth_);
        this->get_parameter("MaxDepth", MaxDepth_);

        // Log parameters
        // RCLCPP_INFO(this->get_logger(), "Loaded Parameters: width=%d, height=%d, scale=%f, MinDepth=%f, MaxDepth=%f",
        //             width_, height_, scale_, MinDepth_, MaxDepth_);

        // Subscriber for PointCloud2 messages
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/observer/lidar_points", 10, std::bind(&PointCloudToDepthMap::point_cloud_callback, this, std::placeholders::_1));

        // Create subscribers using message_filters
        pointcloud_sub_.subscribe(this, "/observer/lidar_points");
        detection_sub_.subscribe(this, "/tracking");

        // Create synchronization policy
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), pointcloud_sub_, detection_sub_);

        // Register callback
        sync_->registerCallback(std::bind(&PointCloudToDepthMap::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Publisher for original depth map
        original_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/depth_map", 10);

        // Publisher for detected object depth map
        detected_object_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/detected_object_depth_map", 10);

        // Publisher for detected object poses
        detected_object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected_object_depthmap_pose", 10);

        RCLCPP_INFO(this->get_logger(), "PointCloud to Depth Map Node has been started.");
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
            
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Filter the point cloud 
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filter_point_cloud(pcl_cloud);

        // Create depth maps (+Z) & (-Z)
        cv::Mat positive_depth_map_single = cv::Mat::zeros(height_, width_, CV_8UC1);
        cv::Mat negative_depth_map_single = cv::Mat::zeros(height_, width_, CV_8UC1);
        create_depth_map(filtered_cloud, positive_depth_map_single, negative_depth_map_single);

        // Convert the single-channel depth maps to 3-channel images
        cv::Mat positive_depth_map, negative_depth_map;
        cv::cvtColor(positive_depth_map_single, positive_depth_map, cv::COLOR_GRAY2BGR);
        cv::cvtColor(negative_depth_map_single, negative_depth_map, cv::COLOR_GRAY2BGR);

        // Combine the images horizontally
        cv::Mat combined_depth_map;
        cv::hconcat(positive_depth_map, negative_depth_map, combined_depth_map);

        // Convert the combined depth map to a ROS Image message
        auto combined_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", combined_depth_map).toImageMsg();

        // Add timestamp and frame information to the header
        combined_image_msg->header = msg->header;

        // Publish the combined image
        original_publisher_->publish(*combined_image_msg);
    }

    void create_depth_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, cv::Mat& positive_depth_map_single, cv::Mat& negative_depth_map_single)
    {
        int center_x = width_ / 2;
        int center_y = height_ / 2;

        for (const auto& point : cloud->points)
        {
            int pixel_x = center_x + static_cast<int>(ceil(point.y * scale_) * -1);
            int pixel_y = center_y + static_cast<int>(ceil(point.x * scale_) * -1);

            // Check if pixel coordinates are within bounds
            if (pixel_x >= 0 && pixel_x < width_ && pixel_y >= 0 && pixel_y < height_)
            {
                if (point.z > 0)  // Positive Z
                {
                    int depth_value = std::clamp(static_cast<int>(point.z * 255 / MaxDepth_), 0, 255);
                    positive_depth_map_single.at<uint8_t>(pixel_y, pixel_x) = 255 - depth_value;
                    // RCLCPP_INFO(this->get_logger(), "Positive Z: pixel (%d, %d), depth_value=%d", pixel_x, pixel_y, depth_value);
                }
                else if (point.z < 0)  // Negative Z
                {
                    int depth_value = std::clamp(static_cast<int>(-point.z * 255 / MaxDepth_), 0, 255);
                    negative_depth_map_single.at<uint8_t>(pixel_y, pixel_x) = 255 - depth_value;
                    // RCLCPP_INFO(this->get_logger(), "Negative Z: pixel (%d, %d), depth_value=%d", pixel_x, pixel_y, depth_value);
                }
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

         // Z-axis filtering - Positive range
        pcl::PassThrough<pcl::PointXYZ> pass_z_positive;
        pass_z_positive.setInputCloud(input_cloud);
        pass_z_positive.setFilterFieldName("z");
        pass_z_positive.setFilterLimits(MinDepth_, MaxDepth_);
        pass_z_positive.filter(*temp_cloud);

        // Create a new cloud for combined results
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_z_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        *combined_z_filtered += *temp_cloud;

        // Z-axis filtering - Negative range
        // pcl::PassThrough<pcl::PointXYZ> pass_z_negative;
        // pass_z_negative.setInputCloud(input_cloud);
        // pass_z_negative.setFilterFieldName("z");
        // pass_z_negative.setFilterLimits(-MaxDepth_, -MinDepth_);
        // pass_z_negative.setNegative(false);
        // pass_z_negative.filter(*temp_cloud);

        // *combined_z_filtered += *temp_cloud;

        // X-axis filtering - Negative range
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(combined_z_filtered);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-MaxDepth_, -MinDepth_);
        pass_x.setNegative(false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr x_neg_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pass_x.filter(*x_neg_filtered);

        // X-axis filtering - Positive range
        pass_x.setFilterLimits(MinDepth_, MaxDepth_);
        pass_x.setNegative(false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr x_pos_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pass_x.filter(*x_pos_filtered);

        // Combine X filtered clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr x_combined_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        *x_combined_filtered = *x_neg_filtered + *x_pos_filtered;

        // Y-axis filtering - Negative range
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(x_combined_filtered);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-MaxDepth_, -MinDepth_);
        pass_y.setNegative(false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr y_neg_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pass_y.filter(*y_neg_filtered);

        // Y-axis filtering - Positive range
        pass_y.setFilterLimits(MinDepth_, MaxDepth_);
        pass_y.setNegative(false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr y_pos_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pass_y.filter(*y_pos_filtered);

        // Final filtered cloud combines both Y ranges
        *filtered_cloud = *y_neg_filtered + *y_pos_filtered;

        return filtered_cloud;

    }

    void sync_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg,
                       const yolov8_msgs::msg::DetectionArray::ConstSharedPtr detection_msg)
    {
        
        int center_x = width_ / 2;
        int center_y = height_ / 2;

        geometry_msgs::msg::PoseArray detected_object_poses;
        detected_object_poses.header = pointcloud_msg->header;

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pointcloud_msg, *pcl_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filter_point_cloud(pcl_cloud);

        cv::Mat positive_depth_map_single = cv::Mat::zeros(height_, width_, CV_8UC1);
        cv::Mat negative_depth_map_single = cv::Mat::zeros(height_, width_, CV_8UC1);

        for (const auto& bbox : detection_msg->detections)
        {
            double x_center = bbox.bbox.center.position.x;
            double y_center = bbox.bbox.center.position.y;
            double width = bbox.bbox.size.x;
            double height = bbox.bbox.size.y;

            double x_min = x_center - width / 2.0;
            double x_max = x_center + width / 2.0;
            double y_min = y_center - height / 2.0;
            double y_max = y_center + height / 2.0;

            double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
            int point_count = 0;

            for (const auto& point : filtered_cloud->points)
            {
                int pixel_x = center_x + static_cast<int>(ceil(point.y * scale_) * -1);
                int pixel_y = center_y + static_cast<int>(ceil(point.x * scale_) * -1);

                if (pixel_x >= x_min && pixel_x <= x_max &&
                    pixel_y >= y_min && pixel_y <= y_max)
                {
                    if (point.z > 0) // Positive Z-axis
                    {
                        int depth_value = std::clamp(static_cast<int>(point.z * 255 / MaxDepth_), 0, 255);
                        positive_depth_map_single.at<uint8_t>(pixel_y, pixel_x) = 255 - depth_value;
                    }
                    else if (point.z < 0) // Negative Z-axis
                    {
                        int depth_value = std::clamp(static_cast<int>(-point.z * 255 / MaxDepth_), 0, 255);
                        negative_depth_map_single.at<uint8_t>(pixel_y, pixel_x) = 255 - depth_value;
                    }

                    sum_x += point.x;
                    sum_y += point.y;
                    sum_z += point.z;
                    point_count++;
                }
            }

            if (point_count > 0)
            {
                geometry_msgs::msg::Pose object_pose;
                object_pose.position.x = sum_x / point_count;
                object_pose.position.y = sum_y / point_count;
                object_pose.position.z = sum_z / point_count;
                object_pose.orientation.x = 0.0;
                object_pose.orientation.y = 0.0;
                object_pose.orientation.z = 0.0;
                object_pose.orientation.w = 1.0;

                detected_object_poses.poses.push_back(object_pose);
        }
    }

    detected_object_pose_publisher_->publish(detected_object_poses);

    cv::Mat positive_depth_map, negative_depth_map, combined_depth_map;
    cv::cvtColor(positive_depth_map_single, positive_depth_map, cv::COLOR_GRAY2BGR);
    cv::cvtColor(negative_depth_map_single, negative_depth_map, cv::COLOR_GRAY2BGR);

    cv::hconcat(positive_depth_map, negative_depth_map, combined_depth_map);

    auto detected_object_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", combined_depth_map).toImageMsg();
    detected_object_image_msg->header = pointcloud_msg->header;

    detected_object_publisher_->publish(*detected_object_image_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr original_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detected_object_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr detected_object_pose_publisher_;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_;
    message_filters::Subscriber<yolov8_msgs::msg::DetectionArray> detection_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        yolov8_msgs::msg::DetectionArray>
        SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

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