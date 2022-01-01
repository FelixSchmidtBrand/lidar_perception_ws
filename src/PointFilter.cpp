#define PCL_NO_PRECOMPILE
#include <memory>
#include <boost/foreach.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "PointXYZVI.hpp"

#include <pcl/PCLHeader.h>
#include <pcl/filters/crop_box.h>
#include <chrono>
#include <functional>

#include "pcl_conversions/pcl_conversions.h"

using namespace std::chrono_literals;

class PointFilter : public rclcpp::Node
{
  public:
    PointFilter()
    : Node("PointFilter")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2> (
      "input_cloud", 10, std::bind(&PointFilter::topic_callback, this, std::placeholders::_1)
      );
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10); 
      cropBoxFilter_ = new pcl::CropBox<PointXYZVI>();

      //parameters

    
      this->declare_parameter<float>("box_min_x", 0.f);
      this->declare_parameter<float>("box_min_y", 0.f);
      this->declare_parameter<float>("box_min_z", -10.f);
      this->declare_parameter<float>("box_max_x", 100.f);
      this->declare_parameter<float>("box_max_y", 30.f);
      this->declare_parameter<float>("box_max_z", 10.f);
      this->declare_parameter<float>("vel_min", -200.f);
      this->declare_parameter<float>("vel_max", 200.f);

      timer_ = this->create_wall_timer(
      1000ms, std::bind(&PointFilter::setParameters, this));
      setParameters();
    }

    void setParameters()
    {
      this->get_parameter("box_min_x", box_min_x);
      this->get_parameter("box_min_y", box_min_y);
      this->get_parameter("box_min_z", box_min_z);
      this->get_parameter("box_max_x", box_max_x);
      this->get_parameter("box_max_y", box_max_y);
      this->get_parameter("box_max_z", box_max_z);
      this->get_parameter("vel_min", vel_min);
      this->get_parameter("vel_max", vel_max);
      min_pt << box_min_x, box_min_y, box_min_z, 1.f;
      max_pt << box_max_x, box_max_y, box_max_z, 1.f;
      cropBoxFilter_->setMin(min_pt);
      cropBoxFilter_->setMax(max_pt);
      RCLCPP_INFO(this->get_logger(), "Filter box parameter set to: x %f - %f, y %f - %f, z %f - %f", box_min_x,box_max_x, box_min_y,box_max_y, box_min_z,box_max_z);
    }

  private:
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    pcl::CropBox<PointXYZVI> * cropBoxFilter_;
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    //parameter
    rclcpp::TimerBase::SharedPtr timer_;
    float box_min_x;
    float box_min_y;
    float box_min_z;
    float box_max_x;
    float box_max_y;
    float box_max_z;
    float vel_min;
    float vel_max;

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs) const
    {    
      pcl::PointCloud<PointXYZVI> cloud_out;
      pcl::PointCloud<PointXYZVI> input_cloud;
      fromROSMsg(*point_cloud2_msgs, input_cloud);
      //I don't like copying the data, but I have not found another way to get the pointcloud into a ConstPtr. Improvements are very much appreciated
      pcl::PointCloud<PointXYZVI>::ConstPtr filter_cloud(new pcl::PointCloud<PointXYZVI>(input_cloud));
      cropBoxFilter_ ->setInputCloud(filter_cloud);
      cropBoxFilter_ ->filter(cloud_out);
      // BOOST_FOREACH (const PointXYZVI& pt, point_cloud)
        //RCLCPP_INFO(this->get_logger(), "Receiving: \t(%f, %f, %f, %f, %f)\n", pt.x, pt.y, pt.z, pt.velocity, pt.intensity); 
      sensor_msgs::msg::PointCloud2 output_cloud;
      toROSMsg(cloud_out, output_cloud);
      publisher_->publish(output_cloud);
    }
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointFilter>());
  rclcpp::shutdown();
  return 0;
}


    
