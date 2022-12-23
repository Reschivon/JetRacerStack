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


template <typename PointT>
class ConditionThresholdHSV : public pcl::ConditionBase<PointT>
{
  public:
    typedef boost::shared_ptr<ConditionThresholdHSV<PointT> > Ptr;
    
    ConditionThresholdHSV (float min_h, float max_h, float min_s, float max_s, float min_v, float max_v) :
      min_h_(min_h), max_h_(max_h), min_s_(min_s), max_s_(max_s), min_v_(min_v), max_v_(max_v)
    {
      // Make min_h_ and max_h_ fall within [0, 360)
      assert (!std::isnan(min_h) && !std::isnan(max_h));
      while (min_h_ < 0) min_h_ += 360;
      while (min_h_ >= 360) min_h_ -= 360;
      while (max_h_ < 0) max_h_ += 360;
      while (max_h_ >= 360) max_h_ -= 360;
    }
    
    // Evaluate whether the color of the given point falls within the specified thresholds
    virtual bool evaluate(const PointT & p) const
    {
      float h, s, v;
      rgb2hsv (p.r, p.g, p.b, h, s, v);
      return (!std::isnan(h) && !std::isnan(s) && !std::isnan(v) && 
              ((min_h_ < max_h_) ? ((min_h_ <= h) && (h <= max_h_)) : ((min_h_ <= h) || (h <= max_h_))) &&
              (min_s_ <= s) && (s <= max_s_) &&
              (min_v_ <= v) && (v <= max_v_));
    }
    
    void rgb2hsv (std::uint8_t r, std::uint8_t g, std::uint8_t b, float & h, float & s, float & v) const
    {
      float maxval = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
      float minval = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
      float minmaxdiff = maxval - minval;
      
      if (maxval == minval)
      {
        h = 0;
        s = 0;
        v = maxval;
        return;
      }   
      else if (maxval == r)
      {
        h = 60.0*((g - b)/minmaxdiff);
        if (h < 0) h += 360.0;
      }
      else if (maxval == g)
      {
        h = 60.0*((b - r)/minmaxdiff + 2.0);
      }
      else // (maxval == b)
      {
        h = 60.0*((r - g)/minmaxdiff + 4.0);
      }
      s = 100.0 * minmaxdiff / maxval;
      v = maxval;
    }

  protected:
    float min_h_, max_h_, min_s_, max_s_, min_v_, max_v_;
};


class ColorCluster : public nodelet::Nodelet
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub, cluster_pub;

    int lowH, highH, lowS, highS, lowV, highV;

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
        cluster_pub = n_.advertise<sensor_msgs::PointCloud2>("cluster", 1);

        n_.param("lowH", lowH, 10);
        n_.param("highH", highH, 75);
        n_.param("lowS", lowS, 10);
        n_.param("highS", highS, 75);
        n_.param("lowV", lowV, 10);
        n_.param("highV", highV, 75);
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

        NODELET_INFO(
	          "[%s::cloudcd] Initial size: %lu",
	          getName().c_str(), cloudRGB->size());
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::ConditionalRemoval<pcl::PointXYZRGB> removal_filter;
        removal_filter.setKeepOrganized (false);
        ConditionThresholdHSV<pcl::PointXYZRGB>::Ptr condition (new ConditionThresholdHSV<pcl::PointXYZRGB> (lowH,highH, lowS,highS, lowV,highV));
        removal_filter.setCondition (condition);
        removal_filter.setInputCloud (cloudRGB);
        removal_filter.filter (*filtered);
        
        NODELET_INFO(
	          "[%s::cloudcd] Cloud filtered size: %lu",
	          getName().c_str(), filtered->size());
    
        // create a vector for storing the indices of the clusters
        std::vector<pcl::PointIndices> cluster_indices;

        // setup extraction:
        pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
        pcl::EuclideanClusterExtraction<Point> ec;
        ec.setClusterTolerance(1); // cm
        ec.setMinClusterSize(3);
        ec.setMaxClusterSize(5000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(filtered);
        // perform cluster extraction
        ec.extract(cluster_indices);


        int number_clusters = (int) cluster_indices.size();
        NODELET_INFO("Number of clusters found: %d", number_clusters);
            
        // We will fill this cloud with centroids
        pcl::PointCloud<pcl::PointXYZ> centroids;
	    pcl::PointCloud<Point> curr_cluster;
	
	    // Each index represents one cluster
	    // Iterate through indexes
        for(const auto & indexes : cluster_indices)
        {
            // Convert indexes to points and store in curr_cluster
            curr_cluster.clear();
            curr_cluster.points.reserve(indexes.indices.size());
            for (const auto & point : indexes.indices)
                curr_cluster.points.push_back(filtered->points[point]);
                
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
        pcl::toROSMsg(*filtered, publish_cloud);
        
        publish_cloud.header.stamp = input->header.stamp;
        publish_cloud.header.frame_id = input->header.frame_id;

        cloud_pub.publish(boost::make_shared<sensor_msgs::PointCloud2>(publish_cloud));


        sensor_msgs::PointCloud2 publish_cluster;
        pcl::toROSMsg(centroids, publish_cluster);
        
        publish_cluster.header.stamp = input->header.stamp;
        publish_cluster.header.frame_id = input->header.frame_id;

        cluster_pub.publish(boost::make_shared<sensor_msgs::PointCloud2>(publish_cluster));
    }
};

}

PLUGINLIB_EXPORT_CLASS(ColorCluster::ColorCluster, nodelet::Nodelet);
