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
#include "message_filters/synchronizer.h"

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

         // Initialize message_filters subscribers 
        point_cloud_subscriber_ = std::make_shared<Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/scan/points");
        detection_array_subscriber_ = std::make_shared<Subscriber<yolov8_msgs::msg::DetectionArray>>(this, "/depth_map/tracking");
        
        // Publisher for original depth map
        original_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/depth_map", 10);
        
        // Publisher for detected object depth map
        detected_object_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/detected_object_depth_map", 10);
        
        // Publisher for detected object poses
        detected_object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected_object_depthmap_pose", 10);
        
        RCLCPP_INFO(this->get_logger(), "PointCloud to Depth Map Node has been started.");

        // Set up the synchronizer with a queue size of 10 (modify as needed)
        sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, yolov8_msgs::msg::DetectionArray>>(
        *point_cloud_subscriber_, *detection_array_subscriber_, 10);

        //Register the Sync_Callback  
        sync_->registerCallback(std::bind(&DataSynchronizationNode::synchronized_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "PointCloud to Depth Map Node has been started.");
    }

private:

    //Data structure designed to store information about bounding boxes
    struct BoundingBox {
        double x_min, y_min, x_max, y_max;
        double sum_x = 0, sum_y = 0, sum_z = 0;  // Sum of coordinates
        int count = 0;  // Number of points within the bounding box
        bool valid = false;  // Indicates if the bbox is valid
        int id = -1;  // ID of the bounding box
    };

        void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Create empty single-channel depth maps
        cv::Mat original_depth_map_single = cv::Mat::zeros(height_, width_, CV_8UC1);

        // Apply filtering to remove far-away points
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(MinDepth_, MaxDepth_);
        pass.filter(*filtered_cloud);

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
            }
        }

        // Convert the single-channel depth maps to 3-channel images
        cv::Mat original_depth_map, detected_object_depth_map;
        cv::cvtColor(original_depth_map_single, original_depth_map, cv::COLOR_GRAY2BGR);


        // Convert the depth maps to ROS Image messages
        sensor_msgs::msg::Image::SharedPtr original_image_msg = 
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", original_depth_map).toImageMsg();
        
        // Publish depth maps and poses
        original_publisher_->publish(*original_image_msg);
    }
    
    // Callback for synchronized messages
    void synchronized_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &point_cloud,
        const yolov8_msgs::msg::DetectionArray::ConstSharedPtr &detection_array)
    {
        RCLCPP_INFO(this->get_logger(), "Callback triggered!");
        if (!point_cloud || !detection_array) {
            RCLCPP_ERROR(this->get_logger(), "One of the messages is empty!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Synchronized messages received!");
            RCLCPP_INFO(this->get_logger(), "Synchronized messages received!");

        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*point_cloud, *pcl_cloud);
        

        // Check if the cloud has at least 10 points
        int num_points = std::min(static_cast<int>(pcl_cloud->points.size()), 10);

        // Print the first 10 or fewer points in the point cloud
        for (int i = 0; i < num_points; ++i) {
            const auto& point = pcl_cloud->points[i];
            RCLCPP_INFO(rclcpp::get_logger("pcl_logger"), "Point %d: (%f, %f, %f)", i, point.x, point.y, point.z);
        }
        // Process detections
        for (const auto& detection : detection_array->detections) {
            RCLCPP_INFO(this->get_logger(), "Detection ID: %s, Score: %f", detection.id.c_str(), detection.score);
        }        
    }
    


    //Declare the publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr original_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detected_object_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr detected_object_pose_publisher_;

    //Declare the subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr bbox_subscription_;

    //Declare the Sync Data
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_sub_;
    message_filters::Subscriber<yolov8_msgs::msg::DetectionArray> bbox_sub_;

    //Define Parameters
    int width_, height_;
    float scale_, MinDepth_, MaxDepth_;

    std::vector<BoundingBox> bounding_boxes;
    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PointCloudToDepthMap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}