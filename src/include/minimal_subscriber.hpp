#ifndef MINIMAL_SUBSCRIBER_HPP
#define MINIMAL_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber();

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs) const;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

#endif