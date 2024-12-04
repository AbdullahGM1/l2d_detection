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
        RCLCPP_INFO(this->get_logger(), "Loaded Parameters: width=%d, height=%d, scale=%f, MinDepth=%f, MaxDepth=%f",
                    width_, height_, scale_, MinDepth_, MaxDepth_);
        
    
        // Subscriber for PointCloud2 messages
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scan/points", 10, std::bind(&PointCloudToDepthMap::point_cloud_callback, this, std::placeholders::_1));

        // Subscriber for bounding box messages
        bbox_subscription_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
            "/depth_map/tracking", 10, std::bind(&PointCloudToDepthMap::bbox_callback, this, std::placeholders::_1));
        
        // Publisher for original depth map
        original_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/depth_map", 10);
        
        // Publisher for detected object depth map
        detected_object_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/detected_object_depth_map", 10);
        
        // Publisher for detected object poses
        detected_object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected_object_depthmap_pose", 10);
        
        RCLCPP_INFO(this->get_logger(), "PointCloud to Depth Map Node has been started.");
    }

private:

    struct BoundingBox {
        double x_min, y_min, x_max, y_max;
        double sum_x = 0, sum_y = 0, sum_z = 0;  // Sum of coordinates
        int count = 0;  // Number of points within the bounding box
        bool valid = false;  // Indicates if the bbox is valid
        int id = -1;  // ID of the bounding box
    };

    void bbox_callback(const yolov8_msgs::msg::DetectionArray::SharedPtr msg)
    {
        bounding_boxes.clear(); // Clear previous detections

        for (const auto& detection : msg->detections) {
            BoundingBox bbox;
            try {
                bbox.id = std::stoi(detection.id);  // Convert string ID to integer
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert detection ID to integer: %s", e.what());
                continue; // Skip this detection and proceed with the next
            }

            // Calaculating the BB Dimensions detection.bbox provides center.x, center.y, size.x, size.y
            bbox.x_min = detection.bbox.center.position.x - detection.bbox.size.x / 2.0;
            bbox.y_min = detection.bbox.center.position.y - detection.bbox.size.y / 2.0;
            bbox.x_max = detection.bbox.center.position.x + detection.bbox.size.x / 2.0;
            bbox.y_max = detection.bbox.center.position.y + detection.bbox.size.y / 2.0;
            bbox.valid = true;

            bounding_boxes.push_back(bbox);

            // RCLCPP_INFO(this->get_logger(), "Bounding Box ID: %d - xmin: %f, ymin: %f, xmax: %f, ymax: %f", 
            //             bbox.id, bbox.x_min, bbox.y_min, bbox.x_max, bbox.y_max);
        }
    }

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Create empty single-channel depth maps
        cv::Mat original_depth_map_single = cv::Mat::zeros(height_, width_, CV_8UC1);
        cv::Mat detected_object_depth_map_single = cv::Mat::zeros(height_, width_, CV_8UC1);

        // Apply filtering to remove far-away points
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(MinDepth_, MaxDepth_);
        pass.filter(*filtered_cloud);

        // Reset bounding box point accumulation
        for (auto& bbox : bounding_boxes) {
            bbox.sum_x = 0;
            bbox.sum_y = 0;
            bbox.sum_z = 0;
            bbox.count = 0;
        }

        // Define the center of the depth map
        int center_x = width_ / 2;
        int center_y = height_ / 2;

        // Iterate through the filtered point cloud and map points to depth maps
        for (const auto& point : filtered_cloud->points)
        {
            float x = point.x;
            float y = point.y;
            float z = point.z;

            // Map x and y to pixel coordinates
            // int pixel_x = static_cast<int>(center_x + x * scale_);
            // int pixel_y = static_cast<int>(center_y - y * scale_);
            // Map x and y to pixel coordinates
            int pixel_x = center_x + static_cast<int>(ceil(y * scale_) * -1);
            int pixel_y = center_y + static_cast<int>(ceil(x * scale_) * -1);

            // Check if the pixel is within image bounds
            if (pixel_x >= 0 && pixel_x < width_ && pixel_y >= 0 && pixel_y < height_)
            {
                // Normalize depth value (x) to 0-255
                int depth_value = std::clamp(static_cast<int>(z * 255 / MaxDepth_), 0, 255);
                
                // Original depth map for all points
                original_depth_map_single.at<uint8_t>(pixel_y, pixel_x) = 255 - depth_value;

                // Check if point is within any bounding box
                for (auto& bbox : bounding_boxes)
                {
                    if (pixel_x >= bbox.x_min && pixel_x <= bbox.x_max &&
                        pixel_y >= bbox.y_min && pixel_y <= bbox.y_max)
                    {
                        // Depth map for points within bounding boxes
                        detected_object_depth_map_single.at<uint8_t>(pixel_y, pixel_x) = 255 - depth_value;
                        
                        // Accumulate point coordinates for average calculation
                        bbox.sum_x += x;
                        bbox.sum_y += y;
                        bbox.sum_z += z;
                        bbox.count++;
                        
                        break;  // Stop checking other bounding boxes once a match is found
                    }
                }
            }
        }

        // Convert the single-channel depth maps to 3-channel images
        cv::Mat original_depth_map, detected_object_depth_map;
        cv::cvtColor(original_depth_map_single, original_depth_map, cv::COLOR_GRAY2BGR);
        cv::cvtColor(detected_object_depth_map_single, detected_object_depth_map, cv::COLOR_GRAY2BGR);

        // Create PoseArray for detected object poses
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header = msg->header;

        // Calculate and add average poses for each bounding box
        for (const auto& bbox : bounding_boxes)
        {
            if (bbox.count > 0)
            {
                geometry_msgs::msg::Pose pose;
                pose.position.x = bbox.sum_x / bbox.count;
                pose.position.y = bbox.sum_y / bbox.count;
                pose.position.z = bbox.sum_z / bbox.count;
                
                // Set default orientation (you might want to calculate a more meaningful orientation)
                pose.orientation.x = 0;
                pose.orientation.y = 0;
                pose.orientation.z = 0;
                pose.orientation.w = 1;

                pose_array.poses.push_back(pose);

                // RCLCPP_INFO(this->get_logger(), "Bounding Box %d Average Pose - x: %f, y: %f, z: %f (points: %d)", 
                //             bbox.id, pose.position.x, pose.position.y, pose.position.z, bbox.count);
            }
        }

        // Convert the depth maps to ROS Image messages
        sensor_msgs::msg::Image::SharedPtr original_image_msg = 
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", original_depth_map).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr detected_object_image_msg = 
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", detected_object_depth_map).toImageMsg();

        // Publish depth maps and poses
        original_publisher_->publish(*original_image_msg);
        detected_object_publisher_->publish(*detected_object_image_msg);
        detected_object_pose_publisher_->publish(pose_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr bbox_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr original_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detected_object_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr detected_object_pose_publisher_;

    std::vector<BoundingBox> bounding_boxes;
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