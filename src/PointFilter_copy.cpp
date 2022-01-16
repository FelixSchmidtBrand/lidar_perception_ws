//The PCL_NO_PRECOMPILE is necessary to use my own Point T template PointXYZVI within the pcl functions
#include <memory>
//#include <boost/foreach.hpp>
#include <functional>
#include <chrono>

#include <Eigen/Geometry> 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/pcl_base.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <pcl/PCLHeader.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>   
#include <pcl/filters/impl/passthrough.hpp>

#include <pcl/filters/voxel_grid.h>
#include "pcl_conversions/pcl_conversions.h"


using namespace std::chrono_literals;

/* The PointFilterCopy node filters unimportent points based on three criteria:
1. RoI: The region of interest is described by a box. All points inside this box are considered relevant. The parameters of the box
are described in world coordinates. Thus, a transformation from sensor to world needs to be given. It is recommended to set the world 
coordinate system onto the road surface. This allows you to easily remove the ground points by defining a box just a bit higher then the road surface.
2. Velocity: All points with low absolute velocity are filtered (considered to be background - especially useful, if the sensor itself does not move).
**/
class PointFilterCopy : public rclcpp::Node
{
  public:
    PointFilterCopy()
    : Node("PointFilterCopy")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2> (
      "input_cloud", 10, std::bind(&PointFilterCopy::topic_callback, this, std::placeholders::_1)
      );
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10); 
      
      //tf2 listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      
      //filter objects
      cropBoxFilter_ = new pcl::CropBox<pcl::PCLPointCloud2>();
      vel_filter_ = new pcl::PassThrough<pcl::PCLPointCloud2>();
      vel_filter_ ->setFilterFieldName("velocity");
      vel_filter_ ->setNegative(true);
      sor_ = std::make_unique<pcl::VoxelGrid<pcl::PCLPointCloud2>>();
      
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
      this->declare_parameter<float>("voxel_size", 0.3f);

      timer_ = this->create_wall_timer(
      10000ms, std::bind(&PointFilterCopy::setParameters, this));
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
      this->get_parameter("voxel_size", voxel_size);
      min_pt << box_min_x, box_min_y, box_min_z, 1.f;
      max_pt << box_max_x, box_max_y, box_max_z, 1.f;
      cropBoxFilter_->setMin(min_pt);
      cropBoxFilter_->setMax(max_pt);
      vel_filter_ ->setFilterLimits(vel_low, vel_high);
      sor_->setLeafSize (voxel_size, voxel_size, voxel_size);
      //RCLCPP_INFO(this->get_logger(), "Filter box parameter set to: x %f - %f, y %f - %f, z %f - %f \n"
      //                                " Velocity filter set to: %f - %f",
      //                                 box_min_x,box_max_x, box_min_y,box_max_y, box_min_z,box_max_z, vel_low, vel_high);
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
    pcl::CropBox<pcl::PCLPointCloud2> * cropBoxFilter_;
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    pcl::PassThrough<pcl::PCLPointCloud2> * vel_filter_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<pcl::VoxelGrid<pcl::PCLPointCloud2>> sor_;
    
    //filter parameter
    float box_min_x;
    float box_min_y;
    float box_min_z;
    float box_max_x;
    float box_max_y;
    float box_max_z;

    float vel_low;
    float vel_high;

    float voxel_size;

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs) const
    {    
      using std::chrono::high_resolution_clock;
      using std::chrono::duration_cast;
      using std::chrono::duration;
      using std::chrono::milliseconds;

      auto t1 = high_resolution_clock::now();
      //----------deserialize--------------
      //pcl::PointCloud<PointXYZVI> cloud_out;
      //pcl::PointCloud<PointXYZVI> input_cloud;
      //fromROSMsg(*point_cloud2_msgs, input_cloud);
      
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
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
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
      
      //---------filter-----------
      //I don't like copying the data, but I have not found another way to get the pointcloud behind a shared Ptr.
      //pcl::PointCloud<PointXYZVI>::Ptr boxfilter_cloud(new pcl::PointCloud<PointXYZVI>(input_cloud));
      //pcl::PointCloud<PointXYZVI>::Ptr vel_cloud(new pcl::PointCloud<PointXYZVI>(cloud_out));
      
      pcl::PCLPointCloud2::Ptr boxfiltered_cloud (new pcl::PCLPointCloud2());
      pcl::PCLPointCloud2::Ptr velocity_filtered_cloud (new pcl::PCLPointCloud2());
      pcl::PCLPointCloud2::Ptr output_cloud (new pcl::PCLPointCloud2());
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2(); 
      //pcl::PCLPointCloud2 cloud_filtered;
      //Performance note: Not sure if PCLPointCloud2ConstPtr copies the data - if so this should be optimized
      pcl_conversions::toPCL(*point_cloud2_msgs, *cloud);
      //pcl::PCLPointCloud2::Ptr input_cloud (new pcl::PCLPointCloud2());
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
      cropBoxFilter_ ->setInputCloud(cloudPtr);
      cropBoxFilter_ ->filter(*boxfiltered_cloud);
      //----------velocity based filtering-------------
      pcl::PCLPointCloud2ConstPtr vel_input_cloudPtr(boxfiltered_cloud);
      vel_filter_ ->setInputCloud(vel_input_cloudPtr);
      vel_filter_ ->filter(*velocity_filtered_cloud);
      //----------down sampling------------
      pcl::PCLPointCloud2ConstPtr voxel_input_cloudPtr(velocity_filtered_cloud);
      sor_->setInputCloud (voxel_input_cloudPtr);
      sor_->filter (*output_cloud);

      //----------serialize---------------
      sensor_msgs::msg::PointCloud2 output;
      pcl_conversions::fromPCL(*output_cloud, output);
      publisher_->publish(output);

      //Get runtime info
      auto t2 = high_resolution_clock::now();
      duration<double, std::milli> ms_double = t2 - t1;

      RCLCPP_INFO(
        this->get_logger(), "Pointfilter took %f ms to complete",
            ms_double.count());
    }
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointFilterCopy>());
  rclcpp::shutdown();
  return 0;
}


    
