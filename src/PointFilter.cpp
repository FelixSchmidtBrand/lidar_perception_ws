#define PCL_NO_PRECOMPILE
//The PCL_NO_PRECOMPILE is necessary to use my own Point T template PointXYZVI within the pcl functions
#include <memory>
//#include <boost/foreach.hpp>
#include <functional>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "PointXYZVI.hpp"

#include <pcl/PCLHeader.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>   
#include <pcl/filters/impl/passthrough.hpp>

#include "pcl_conversions/pcl_conversions.h"

using namespace std::chrono_literals;

/* The PointFilter node filters unimportent points based on three criteria:
1. RoI: The region of interest is described by a box. All points inside this box are considered relevant.
2. Velocity: All points with low absolute velocity are filtered (considered to be background - especially useful, if the sensor itself does not move).
3. Ground Plane removal: All points close to the ground plane are removed. 
*/
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
      
      //filter objects and their parameter
      cropBoxFilter_ = new pcl::CropBox<PointXYZVI>();
      vel_filter_ = new pcl::PassThrough<PointXYZVI>();
      vel_filter_ ->setFilterFieldName("velocity");
      vel_filter_ ->setNegative(true);

      this->declare_parameter<float>("box_min_x", 0.f);
      this->declare_parameter<float>("box_min_y", 0.f);
      this->declare_parameter<float>("box_min_z", -10.f);
      this->declare_parameter<float>("box_max_x", 100.f);
      this->declare_parameter<float>("box_max_y", 30.f);
      this->declare_parameter<float>("box_max_z", 10.f);
      this->declare_parameter<float>("vel_min", -0.1f);
      this->declare_parameter<float>("vel_max", 0.1f);

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
      this->get_parameter("vel_low", vel_low);
      this->get_parameter("vel_high", vel_high);
      min_pt << box_min_x, box_min_y, box_min_z, 1.f;
      max_pt << box_max_x, box_max_y, box_max_z, 1.f;
      cropBoxFilter_->setMin(min_pt);
      cropBoxFilter_->setMax(max_pt);
      vel_filter_ ->setFilterLimits(vel_low, vel_high);
      RCLCPP_INFO(this->get_logger(), "Filter box parameter set to: x %f - %f, y %f - %f, z %f - %f \n"
                                      " Velocity filter set to: %f - %f",
                                       box_min_x,box_max_x, box_min_y,box_max_y, box_min_z,box_max_z, vel_low, vel_high);
    }

  private:
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    //filter elements
    pcl::CropBox<PointXYZVI> * cropBoxFilter_;
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    pcl::PassThrough<PointXYZVI> * vel_filter_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    //filter parameter
    float box_min_x;
    float box_min_y;
    float box_min_z;
    float box_max_x;
    float box_max_y;
    float box_max_z;

    float vel_low;
    float vel_high;

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs) const
    {    
      
      //----------deserialize--------------
      pcl::PointCloud<PointXYZVI> cloud_out;
      pcl::PointCloud<PointXYZVI> input_cloud;
      fromROSMsg(*point_cloud2_msgs, input_cloud);
      
      //---------filter-------------
      //I don't like copying the data, but I have not found another way to get the pointcloud behind a shared Ptr. Improvements are very much appreciated.
      pcl::PointCloud<PointXYZVI>::Ptr boxfilter_cloud(new pcl::PointCloud<PointXYZVI>(input_cloud));
      pcl::PointCloud<PointXYZVI>::Ptr vel_cloud(new pcl::PointCloud<PointXYZVI>(cloud_out));
      cropBoxFilter_ ->setInputCloud(boxfilter_cloud);
      cropBoxFilter_ ->filter(*vel_cloud);
      
      vel_filter_ ->setInputCloud(vel_cloud);
      vel_filter_ ->filter(cloud_out);
     
      //--------serialize---------------
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


    
