#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <random>

#include "depthai/depthai.hpp"

class OakPointCloudCpp : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<dai::Device> device_;
    std::shared_ptr<dai::DataOutputQueue> depth_queue_;
    
    // Camera parameters
    double fx_ = 250.0;
    double fy_ = 250.0;
    double cx_, cy_;
    int downsample_factor_ = 2;

public:
    OakPointCloudCpp() : Node("oak_pointcloud_cpp")
    {
        this->declare_parameter<std::string>("frame_id", "oakd_link");
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/oak/points", 10);
        
        setupDepthAIPipeline();
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&OakPointCloudCpp::update, this));
    }
private:
    void setupDepthAIPipeline()
    {
        dai::Pipeline pipeline;
        
        // Create nodes
        auto monoLeft = pipeline.create<dai::node::MonoCamera>();
        auto monoRight = pipeline.create<dai::node::MonoCamera>();
        auto stereo = pipeline.create<dai::node::StereoDepth>();
        auto depthOut = pipeline.create<dai::node::XLinkOut>();
        
        // Configure mono cameras
        monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
        monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
        monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);
        
        // Configure stereo depth - doğru API kullanımı
        stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);
        stereo->setSubpixel(false);
        stereo->setLeftRightCheck(true);
        
        // Config için try-catch kullan
        try {
            auto config = stereo->initialConfig.get();
            // Doğru field adları kullan
            config.algorithmControl.enableLeftRightCheck = true;
            config.algorithmControl.enableSubpixel = false;
            config.postProcessing.median = dai::MedianFilter::KERNEL_7x7;
            config.postProcessing.speckleFilter.enable = true;
            config.postProcessing.spatialFilter.enable = true;
            config.postProcessing.temporalFilter.enable = true;
            stereo->initialConfig.set(config);
            RCLCPP_INFO(this->get_logger(), "Stereo config applied successfully");
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Config failed, using defaults: %s", e.what());
        }
        
        // Set output queue name
        depthOut->setStreamName("depth");
        
        // Link nodes
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);
        stereo->depth.link(depthOut->input);
        
        // Connect to device
        device_ = std::make_shared<dai::Device>(pipeline);
        depth_queue_ = device_->getOutputQueue("depth", 4, false);
        
        RCLCPP_INFO(this->get_logger(), "DepthAI pipeline initialized");
    }
    
    void update()
    {
        auto in_depth = depth_queue_->tryGet<dai::ImgFrame>();
        if (!in_depth) return;
        
        // OpenCV destekli getFrame() kullanımı
        cv::Mat depth_frame = in_depth->getFrame();
        
        if (depth_frame.empty()) return;
        
        // Convert to float and mm->m
        depth_frame.convertTo(depth_frame, CV_32F, 1.0/1000.0);
        
        // Downsample
        cv::Mat depth_downsampled;
        cv::resize(depth_frame, depth_downsampled, cv::Size(), 
                  1.0/downsample_factor_, 1.0/downsample_factor_, cv::INTER_NEAREST);
        
        int height = depth_downsampled.rows;
        int width = depth_downsampled.cols;
        
        cx_ = width / 2.0;
        cy_ = height / 2.0;
        
        // Create point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        cloud->points.reserve(height * width);
        
        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                float depth = depth_downsampled.at<float>(v, u);
                
                // Filter invalid depths
                if (depth > 0.01 && depth < 2.0) {
                    pcl::PointXYZ point;
                    point.z = depth;
                    point.x = (u - cx_) * depth / fx_;
                    point.y = (v - cy_) * depth / fy_;
                    
                    cloud->points.push_back(point);
                }
            }
        }
        
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;
        
        // Additional filtering
        if (cloud->points.size() > 5000) {
            // Statistical outlier removal
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud);
            sor.setMeanK(10);
            sor.setStddevMulThresh(0.5);
            sor.filter(*cloud);
            
            // Random downsampling if still too many points
            if (cloud->points.size() > 5000) {
                std::random_device rd;
                std::mt19937 g(rd());
                std::shuffle(cloud->points.begin(), cloud->points.end(), g);
                cloud->points.resize(5000);
                cloud->width = 5000;
            }
        }
        
        // Convert to ROS message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = this->get_clock()->now();
        std::string frameParam = this->get_parameter("frame_id").as_string();
        cloud_msg.header.frame_id = frameParam;

        publisher_->publish(cloud_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published %zu points", cloud->points.size());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OakPointCloudCpp>());
    rclcpp::shutdown();
    return 0;
}