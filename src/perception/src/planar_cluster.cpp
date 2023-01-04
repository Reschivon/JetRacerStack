#include <iostream>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dirty_conversions.hpp"

class PlanarCluster : public rclcpp::Node
{

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;

public:
    explicit PlanarCluster(const rclcpp::NodeOptions& options)
	: Node("PlanarCluster", rclcpp::NodeOptions(options).use_intra_process_comms(true))
    {
        RCLCPP_DEBUG(this->get_logger(), "Creating subscribers and publishers");

        cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>("input", 10, 
                std::bind(&PlanarCluster::cloudcb, this, std::placeholders::_1));
        cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("output", 1);

        declare_parameter("tolerance", 0.1);
        declare_parameter("minSize", 3);
        declare_parameter("maxSize", 100);
    }

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::msg::PointCloud2::UniquePtr input)
    {
        double tolerance;
        int minSize, maxSize;

        tolerance = get_parameter("tolerance").as_double();
        minSize = get_parameter("minSize").as_int();
        maxSize = get_parameter("maxSize").as_int();

     	/* NODELET_INFO(
	          "[%s::input_indices_callback] PointCloud with %d data points and frame %s on "
	          "topic %s received.",
	          getName().c_str(), input->width * input->height,
	          input->header.frame_id.c_str(), n_.resolveName("input").c_str()); */
	      
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	    dirty_conversion::fromROSMsg(*(input), *(cloud));
	
        // create a vector for storing the indices of the clusters
        std::vector<pcl::PointIndices> cluster_indices;

        // setup extraction:
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(minSize);
        ec.setMaxClusterSize(maxSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        // perform cluster extraction
        ec.extract(cluster_indices);


        int number_clusters = (int) cluster_indices.size();
        RCLCPP_DEBUG(this->get_logger(), "Number of clusters found: %d", number_clusters);
            
        // We will fill this cloud with centroids
        pcl::PointCloud<pcl::PointXYZ> centroids;
	    pcl::PointCloud<pcl::PointXYZ> curr_cluster;
	
	    // Each index represents one cluster
	    // Iterate through indexes
        for(const auto & indexes : cluster_indices)
        {
            // Convert indexes to points and store in curr_cluster
            curr_cluster.clear();
            curr_cluster.points.reserve(indexes.indices.size());
            for (const auto & point : indexes.indices)
                curr_cluster.points.push_back(cloud->points[point]);
                
            curr_cluster.width = curr_cluster.points.size();
            curr_cluster.height = 1;
            curr_cluster.is_dense = true;

            // compute centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(curr_cluster, centroid);
            auto centroid_point = pcl::PointXYZ(centroid(0), centroid(1), centroid(2));
            centroids.points.push_back(centroid_point);
        }
        
        sensor_msgs::msg::PointCloud2::UniquePtr publish_cloud(new sensor_msgs::msg::PointCloud2());
        dirty_conversion::toROSMsg(centroids, *publish_cloud);
        
        publish_cloud->header.stamp = input->header.stamp;
        publish_cloud->header.frame_id = input->header.frame_id;

        cloud_pub->publish(std::move(publish_cloud));
    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PlanarCluster)
