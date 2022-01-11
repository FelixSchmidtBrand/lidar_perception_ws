#define PCL_NO_PRECOMPILE

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/search/kdtree.h>

#include <pcl/common/distances.h>
#include "pcl_conversions/pcl_conversions.h"
#include "PointXYZVI.hpp"

using namespace std::chrono_literals;

/* 
This node takes in a point cloud and creates clusters based on euclidean distance
and the doppler velocity difference of the points. 

**/

class ClusterBuilder : public rclcpp::Node
{
  public:
    ClusterBuilder()
    : Node("ClusterBuilder")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2> (
      "filtered_cloud", 10, std::bind(&ClusterBuilder::topic_callback, this, std::placeholders::_1)
      );
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("segmented_cloud", 10);

       //cluster objects     
      euclidean_cluster_extractor_ = std::make_unique<pcl::ConditionalEuclideanClustering<PointXYZVI>>();
      clusters_ = pcl::IndicesClustersPtr(new pcl::IndicesClusters());
      pcl::search::KdTree<PointXYZVI>::Ptr kd_tree_ (new pcl::search::KdTree<PointXYZVI>);
      euclidean_cluster_extractor_->setSearchMethod(kd_tree_);

      //parameters
      //this->declare_parameter<std::string>("input_frame", "input_frame");
      this->declare_parameter<int>("rec_min_cluster_size", 5);
      this->declare_parameter<int>("rec_max_cluster_size", 100);
      this->declare_parameter<float>("max_vel_deviation", 1.f);
      this->declare_parameter<float>("tolerance", 500.f);
     
      timer_ = this->create_wall_timer(
        1000ms, std::bind(&ClusterBuilder::setParameters, this));
      setParameters();  
    }

    static float max_vel_deviation_static;
    static bool enforceDopplerVelocitySimilarity (const PointXYZVI& point_a, const PointXYZVI& point_b, float /*squared_distance*/)
    {
      if (std::abs(point_a.velocity - point_b.velocity) < max_vel_deviation_static)
        return (true);
      else
        return (false);
    }
    
    
    private:
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    //Euclidean Cluster
    std::unique_ptr<pcl::ConditionalEuclideanClustering<PointXYZVI>> euclidean_cluster_extractor_;
    pcl::IndicesClustersPtr clusters_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //filter parameter
    int rec_min_cluster_size;
    int rec_max_cluster_size;
    float max_vel_deviation;
    float tolerance;

    void setParameters()
    {
      //this->get_parameter("input_frame", fromFrameRel);
      this->get_parameter("rec_min_cluster_size", rec_min_cluster_size);
      this->get_parameter("rec_max_cluster_size", rec_max_cluster_size);
      this->get_parameter("max_vel_deviation", max_vel_deviation);
      max_vel_deviation_static = max_vel_deviation;
      this->get_parameter("tolerance", tolerance);
      euclidean_cluster_extractor_->setMinClusterSize(rec_min_cluster_size);
      euclidean_cluster_extractor_->setMaxClusterSize(rec_max_cluster_size);
      euclidean_cluster_extractor_->setConditionFunction(&ClusterBuilder::enforceDopplerVelocitySimilarity);
      euclidean_cluster_extractor_->setClusterTolerance(tolerance);
      RCLCPP_INFO(this->get_logger(), "Cluster points min-max set to: x %i - %i, \n"
                                      " Velocity max deviation set to: %f",
                                       rec_min_cluster_size,rec_max_cluster_size, max_vel_deviation);
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs) const
    {   
      //----------deserialize--------------

      pcl::PointCloud<PointXYZVI> input_cloud;
      fromROSMsg(*point_cloud2_msgs, input_cloud);
      RCLCPP_INFO(this->get_logger(), "Received point cloud length: %i", input_cloud.size());
      //----------------cluster--------------
      pcl::PointCloud<PointXYZVI>::Ptr cloud_to_cluster_(new pcl::PointCloud<PointXYZVI>(input_cloud));
      euclidean_cluster_extractor_->setInputCloud(cloud_to_cluster_);
      euclidean_cluster_extractor_->segment(*clusters_);
      
      //quick hack visualization over intensity channel
      for (const auto& cluster : (*clusters_))
      {
        int label = rand () % 8;
        for (const auto& j : cluster.indices)
          (*cloud_to_cluster_)[j].intensity = label;
      }

      //--------serialize---------------
      pcl::PointCloud<PointXYZVI> cloud_out;
      sensor_msgs::msg::PointCloud2 output_cloud;
      pcl::copyPointCloud(*cloud_to_cluster_, *clusters_, cloud_out); 		
      toROSMsg(cloud_out, output_cloud);
      publisher_->publish(output_cloud);
    }
};

float ClusterBuilder::max_vel_deviation_static = 1.f;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClusterBuilder>());
  rclcpp::shutdown();
  return 0;
}