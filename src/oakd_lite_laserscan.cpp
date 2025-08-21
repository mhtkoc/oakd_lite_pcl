#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>

#include "depthai/depthai.hpp"

class OakLaserScanCpp : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<dai::Device> device_;
    std::shared_ptr<dai::DataOutputQueue> depth_queue_;
    
    // Camera parameters
    double fx_ = 250.0;
    double fy_ = 250.0;
    double cx_, cy_;
    int downsample_factor_ = 2;
    
    // LaserScan parameters
    double angle_min_ = -M_PI/2.0;  // -90 degrees
    double angle_max_ = M_PI/2.0;   // +90 degrees
    double range_min_ = 0.1;        // 10cm
    double range_max_ = 3.0;        // 3m
    int num_beams_ = 360;           // Açı çözünürlüğü
    double height_threshold_ = 1.5; // 1.5 metre yükseklik sınırı
    double camera_height_ = 0.15;   // Kamera yüksekliği (15cm)
    bool debug_enabled_ = false;    // Debug flag

public:
    OakLaserScanCpp() : Node("oak_laserscan_cpp")
    {
        this->declare_parameter<std::string>("frame_id", "oakd_link");
        this->declare_parameter<bool>("debug", false);
        
        debug_enabled_ = this->get_parameter("debug").as_bool();
        
        laser_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/oak/scan", 10);
        
        if (debug_enabled_) {
            debug_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/oak/debug_points", 10);
            RCLCPP_INFO(this->get_logger(), "Debug mode enabled - publishing point cloud");
        }
        
        setupDepthAIPipeline();
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&OakLaserScanCpp::update, this));
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
        
        // Point cloud oluştur ve LaserScan'e çevir
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        sensor_msgs::msg::LaserScan scan_msg;
        
        depthToLaserScanWithCloud(depth_downsampled, scan_msg, cloud);
        
        // LaserScan yayınla
        laser_publisher_->publish(scan_msg);
        
        // Debug modundaysa point cloud da yayınla
        if (debug_enabled_ && !cloud->points.empty()) {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_msg.header.stamp = this->get_clock()->now();
            std::string frameParam = this->get_parameter("frame_id").as_string();
            cloud_msg.header.frame_id = frameParam;
            debug_cloud_publisher_->publish(cloud_msg);
        }
        
        // Valid ranges sayısını hesapla
        int valid_ranges = 0;
        for (const auto& range : scan_msg.ranges) {
            if (std::isfinite(range)) valid_ranges++;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Published LaserScan with %d valid ranges, PointCloud with %zu points", 
                    valid_ranges, cloud->points.size());
    }
    
    void depthToLaserScanWithCloud(const cv::Mat& depth_image, sensor_msgs::msg::LaserScan& scan_msg,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // LaserScan mesaj header'ını ayarla
        scan_msg.header.stamp = this->get_clock()->now();
        std::string frameParam = this->get_parameter("frame_id").as_string();
        scan_msg.header.frame_id = frameParam;
        
        // LaserScan parametrelerini ayarla
        scan_msg.angle_min = angle_min_;
        scan_msg.angle_max = angle_max_;
        scan_msg.angle_increment = (angle_max_ - angle_min_) / num_beams_;
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = 0.05; // 20Hz
        scan_msg.range_min = range_min_;
        scan_msg.range_max = range_max_;
        
        // Range array'ini initialize et
        scan_msg.ranges.assign(num_beams_, std::numeric_limits<float>::infinity());
        
        // Point cloud'u temizle
        cloud->clear();
        cloud->points.reserve(depth_image.rows * depth_image.cols);
        
        int height = depth_image.rows;
        int width = depth_image.cols;
        
        int valid_points = 0;
        int height_filtered = 0;
        int angle_filtered = 0;
        int range_filtered = 0;
        
        // İlk aşama: Tüm geçerli noktaları point cloud'a ekle ve en alt Z değerini bul
        
        
        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                float depth = depth_image.at<float>(v, u);
                
                if (depth <= 0 || depth < range_min_ || depth > range_max_) {
                    range_filtered++;
                    continue;
                }
                
                // Orijinal kamera koordinat sistemi:
                // Z = derinlik (ileri), Y = yükseklik (aşağı pozitif), X = genişlik (sağa pozitif)
                float x = (u - cx_) * depth / fx_;  // Horizontal offset
                float y = (v - cy_) * depth / fy_;  // Vertical offset (yükseklik)
                float z = depth;                    // Derinlik
                
                // Point cloud'a ekle (debug için) - Dönüştürülmüş koordinatlar
                pcl::PointXYZ point;
                point.x = z;   // Kamera Z -> Point cloud X (ileri)
                point.y = x;   // Kamera X -> Point cloud Y (sol/sağ) 
                point.z = -y;  // Kamera -Y -> Point cloud Z (yukarı)
                cloud->points.push_back(point);
                valid_points++;
            }
        }
        
        // İkinci aşama: Kamera yüksekliğine göre LaserScan oluştur
        // Kamera 26cm yukarıda olduğu için yer seviyesi yaklaşık -0.23m'de görünür
        float ground_level_z = -camera_height_ + 0.03;  // Kamera yüksekliği - 3cm tolerance (yaklaşık -0.23m)
        float obstacle_min_height = ground_level_z + 0.03;  // Ground level + 3cm minimum engel yüksekliği

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Camera height: %.2fm, Ground level Z: %.3f, Min obstacle height: %.3f", 
                           camera_height_, ground_level_z, obstacle_min_height);

        // Tüm noktaları kontrol et, sadece ground level üzerindeki objeler LaserScan'e dahil
        for (const auto& point : cloud->points) {
            // LaserScan mantığı: Ground level'in ÜZERİNDE olan objeler için
            // 1. Ground level'den yüksek olmalı (point.z > ground_level_z)  
            // 2. Minimum engel yüksekliğini geçmeli (point.z > obstacle_min_height)
            if (point.z <= obstacle_min_height || point.z <= ground_level_z) {  
                height_filtered++;
                continue;  // Bu nokta LaserScan'e dahil edilmez (yer seviyesinde)
            }
                
                // LaserScan hesaplaması - Point cloud koordinat sisteminde
                // X=ileri, Y=sol/sağ, Z=yukarı/aşağı
                float range = sqrt(point.x * point.x + point.y * point.y);  // XY düzleminde mesafe
                float angle = atan2(point.y, point.x);          // X forward, Y left/right
                
                // Açı limitlerini kontrol et
                if (angle < angle_min_ || angle > angle_max_) {
                    angle_filtered++;
                    continue;
                }
                
                // Beam index hesapla
                int beam_index = (int)((angle - angle_min_) / scan_msg.angle_increment);
                
                // Index limitlerini kontrol et ve EN YAKIN mesafeyi kullan (ilk engel)
                if (beam_index >= 0 && beam_index < num_beams_) {
                    if (scan_msg.ranges[beam_index] > range) {
                        scan_msg.ranges[beam_index] = range;
                    }
                }
            
        }
        
        // Point cloud properties ayarla
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;
        
        // Valid ranges sayısını hesapla
        int laser_points = 0;
        for (const auto& range : scan_msg.ranges) {
            if (std::isfinite(range)) laser_points++;
        }
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                           "Points: %d total, %d ground-level filtered, %d angle filtered, %d range filtered, %d laser points",
                           valid_points, height_filtered, angle_filtered, range_filtered, laser_points);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OakLaserScanCpp>());
    rclcpp::shutdown();
    return 0;
}