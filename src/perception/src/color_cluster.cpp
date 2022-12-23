#include <iostream>
#include <fstream>
#include <memory>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/conditional_removal.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ColorCluster {
class ColorCluster : public nodelet::Nodelet
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

    int lowH, highH;

    using Point = pcl::PointXYZRGB;
    using PointHSV = pcl::PointXYZHSV;
    using CloudPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;
    using CloudHSVPtr = pcl::PointCloud<PointHSV>::Ptr;

public:
    virtual void onInit() 
    {
        NODELET_DEBUG("Creating subscribers and publishers");
        
        n_ = getPrivateNodeHandle();
        cloud_sub = n_.subscribe("input", 10, &ColorCluster::cloudcb, this);
        cloud_pub = n_.advertise<sensor_msgs::PointCloud2>("output", 1);

        n_.param("lowH", lowH, 10);
        n_.param("highH", highH, 75);
    }

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::PointCloud2::ConstPtr &input)
    {
     	/* NODELET_INFO(
	          "[%s::input_indices_callback] PointCloud with %d data points and frame %s on "
	          "topic %s received.",
	          getName().c_str(), input->width * input->height,
	          input->header.frame_id.c_str(), n_.resolveName("input").c_str()); */
            
        NODELET_INFO(
	          "[%s::cloudcd] Low: %d, high: %d",
	          getName().c_str(), lowH, highH);
	      
        // Convert PointCloud2 and store in reference of boost_ptr<pcl pointcloud>
        CloudPtr cloudRGB(new pcl::PointCloud<Point>);
	    pcl::fromROSMsg(*input, *cloudRGB);
        const pcl::PointCloud<pcl::PointXYZRGB> &cloudRGBConst(*cloudRGB);

        CloudHSVPtr hsvCloud(new pcl::PointCloud<PointHSV>);
        pcl::PointCloudXYZRGBtoXYZHSV(cloudRGBConst, *hsvCloud);
        const pcl::PointCloud<PointHSV>::ConstPtr hsvCloudConst(hsvCloud);

        CloudHSVPtr cloud_filtered(new pcl::PointCloud<PointHSV>);

        NODELET_INFO(
	          "[%s::cloudcd] HSV Cloud size: %lu",
	          getName().c_str(), hsvCloudConst->size());
                
        pcl::ConditionOr<PointHSV>::Ptr range_cond(new pcl::ConditionOr<PointHSV>());
        range_cond->addComparison(pcl::FieldComparison<PointHSV>::ConstPtr(new pcl::FieldComparison<PointHSV>("h", pcl::ComparisonOps::LT, lowH)));
        range_cond->addComparison(pcl::FieldComparison<PointHSV>::ConstPtr(new pcl::FieldComparison<PointHSV>("h", pcl::ComparisonOps::GT, highH)));
    
        pcl::ConditionalRemoval<PointHSV> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(hsvCloudConst);
        condrem.setKeepOrganized(true);
        condrem.filter(*cloud_filtered);

        const pcl::PointCloud<PointHSV>::ConstPtr CloudFilteredConst(cloud_filtered);

        NODELET_INFO(
	          "[%s::cloudcd] Cloud filtered size: %lu",
	          getName().c_str(), CloudFilteredConst->size());
    
        // create a vector for storing the indices of the clusters
        std::vector<pcl::PointIndices> cluster_indices;

        // setup extraction:
        pcl::search::KdTree<PointHSV>::Ptr tree (new pcl::search::KdTree<PointHSV>);
        pcl::EuclideanClusterExtraction<PointHSV> ec;
        ec.setClusterTolerance(1); // cm
        ec.setMinClusterSize(3);
        ec.setMaxClusterSize(5000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(CloudFilteredConst);
        // perform cluster extraction
        ec.extract(cluster_indices);


        int number_clusters = (int) cluster_indices.size();
        NODELET_INFO("Number of clusters found: %d", number_clusters);
            
        // We will fill this cloud with centroids
        pcl::PointCloud<pcl::PointXYZ> centroids;
	    pcl::PointCloud<PointHSV> curr_cluster;
	
	    // Each index represents one cluster
	    // Iterate through indexes
        for(const auto & indexes : cluster_indices)
        {
            // Convert indexes to points and store in curr_cluster
            curr_cluster.clear();
            curr_cluster.points.reserve(indexes.indices.size());
            for (const auto & point : indexes.indices)
                curr_cluster.points.push_back(CloudFilteredConst->points[point]);
                
            curr_cluster.width = curr_cluster.points.size();
            curr_cluster.height = 1;
            curr_cluster.is_dense = true;

            // compute centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(curr_cluster, centroid);
            auto centroid_point = pcl::PointXYZ(centroid(0), centroid(1), centroid(2));
            centroids.points.push_back(centroid_point);
        }
        
        sensor_msgs::PointCloud2 publish_cloud;
        // pcl::toROSMsg(centroids, publish_cloud);
        pcl::toROSMsg(*cloud_filtered, publish_cloud);
        
        publish_cloud.header.stamp = input->header.stamp;
        publish_cloud.header.frame_id = input->header.frame_id;

        cloud_pub.publish(boost::make_shared<sensor_msgs::PointCloud2>(publish_cloud));
    }
};

}

PLUGINLIB_EXPORT_CLASS(ColorCluster::ColorCluster, nodelet::Nodelet);
