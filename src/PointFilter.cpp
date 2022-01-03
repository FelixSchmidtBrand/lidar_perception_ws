#define PCL_NO_PRECOMPILE
//The PCL_NO_PRECOMPILE is necessary to use my own Point T template PointXYZVI within the pcl functions
#include <memory>
//#include <boost/foreach.hpp>
#include <functional>
#include <chrono>

#include <Eigen/Geometry> 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include "PointXYZVI.hpp"

#include <pcl/PCLHeader.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>   
#include <pcl/filters/impl/passthrough.hpp>

#include "pcl_conversions/pcl_conversions.h"


using namespace std::chrono_literals;

/* The PointFilter node filters unimportent points based on three criteria:
1. RoI: The region of interest is described by a box. All points inside this box are considered relevant. The parameters of the box
are described in world coordinates. Thus, a transformation from sensor to world needs to be given. It is recommended to set the world 
coordinate system onto the road surface. This allows you to easily remove the ground points by defining a box just a bit higher then the road surface.
2. Velocity: All points with low absolute velocity are filtered (considered to be background - especially useful, if the sensor itself does not move).
**/
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
      
      //tf2 listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      
      //filter objects
      cropBoxFilter_ = new pcl::CropBox<PointXYZVI>();
      vel_filter_ = new pcl::PassThrough<PointXYZVI>();
      vel_filter_ ->setFilterFieldName("velocity");
      vel_filter_ ->setNegative(true);
      
      //parameters
      this->declare_parameter<std::string>("input_frame", "input_frame");
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
      this->get_parameter("input_frame", fromFrameRel);
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
    
    //tf2 listener
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    
    //listener parameters
    std::string fromFrameRel;
    std::string toFrameRel = "world";
    
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
      
      //---------tf2 transformation------------
      //this stuff should be done with pcl_ros as soon as a filter transformation is available.
      geometry_msgs::msg::TransformStamped transformStamped;
      try {
          transformStamped = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel, fromFrameRel, ex.what());
          return;
        }
      
      Eigen::Affine3f transform;
      Eigen::AngleAxisf rotation;
      Eigen::Transform<float,3,Eigen::Affine> t;
      Eigen::Quaternion<float> quat(transformStamped.transform.rotation.w,
                          transformStamped.transform.rotation.x,
                          transformStamped.transform.rotation.y,
                          transformStamped.transform.rotation.z);
      rotation = quat;
      Eigen::Translation<float,3> translation(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
      t = translation * rotation;
      cropBoxFilter_->setTransform(t);  
      
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


    
