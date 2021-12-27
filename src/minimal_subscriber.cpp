#include <memory>
#include <boost/foreach.hpp>



#include "minimal_subscriber.hpp"
#include "pcl/PCLHeader.h"
//#include "pcl/conversions.h"
#include "pcl_conversions/pcl_conversions.h"


MinimalSubscriber::MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_cloud", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

void MinimalSubscriber::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs) const
    {
      pcl::PointCloud<pcl::PointXYZ> point_cloud;
      fromROSMsg(*point_cloud2_msgs, point_cloud);
      BOOST_FOREACH (const pcl::PointXYZ& pt, point_cloud)
        RCLCPP_INFO(this->get_logger(), "Publishing: \t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    }
    
