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
#include <pcl/point_types_conversion.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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


class ColorCluster : public rclcpp::Node
{

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub, cluster_pub;

    using Point = pcl::PointXYZRGB;
    using PointHSV = pcl::PointXYZHSV;
    using CloudPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;
    using CloudHSVPtr = pcl::PointCloud<PointHSV>::Ptr;

public:
    explicit ColorCluster(const rclcpp::NodeOptions& options)
	: Node("ColorCluster", rclcpp::NodeOptions(options).use_intra_process_comms(true))
    {
        RCLCPP_DEBUG(this->get_logger(), "Creating subscribers and publishers");
        
        cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>("input", 10, 
            std::bind(&ColorCluster::cloudcb, this, std::placeholders::_1));
        cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("output", 1);

        declare_parameter("lowH", 10);
        declare_parameter("highH", 75);
        declare_parameter("lowS", 10);
        declare_parameter("highS", 75);
        declare_parameter("lowV", 10);
        declare_parameter("highV", 75);
    }

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::msg::PointCloud2::UniquePtr input)
    {

        int lowH, highH, lowS, highS, lowV, highV;

        lowH = get_parameter("lowH").as_int();
        highH = get_parameter("g").as_int();
        lowS = get_parameter("lowS").as_int();
        highS = get_parameter("highS").as_int();
        lowV = get_parameter("lowV").as_int();
        highV = get_parameter("highV").as_int();

     	  /* NODELET_INFO(
	          "[%s::input_indices_callback] PointCloud with %d data points and frame %s on "
	          "topic %s received.",
	          getName().c_str(), input->width * input->height,
	          input->header.frame_id.c_str(), n_.resolveName("input").c_str()); */
            
        /* NODELET_INFO(
	          "[%s::cloudcd] Low: %d, high: %d",
	          getName().c_str(), lowH, highH); */

 
	      
        // Convert PointCloud2 and store in reference of boost_ptr<pcl pointcloud>
        CloudPtr cloudRGB(new pcl::PointCloud<Point>);
	      pcl::fromROSMsg(*input, *cloudRGB);

        /* NODELET_INFO(
	          "[%s::cloudcd] Initial size: %lu",
	          getName().c_str(), cloudRGB->size()); */
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::ConditionalRemoval<pcl::PointXYZRGB> removal_filter;
        removal_filter.setKeepOrganized (false);
        ConditionThresholdHSV<pcl::PointXYZRGB>::Ptr condition (new ConditionThresholdHSV<pcl::PointXYZRGB> (lowH,highH, lowS,highS, lowV,highV));
        removal_filter.setCondition (condition);
        removal_filter.setInputCloud (cloudRGB);
        removal_filter.filter (*filtered);
        
        /* NODELET_INFO(
	          "[%s::cloudcd] Cloud filtered size: %lu",
	          getName().c_str(), filtered->size()); */
        
        sensor_msgs::msg::PointCloud2::UniquePtr publish_cloud(new sensor_msgs::msg::PointCloud2());
        pcl::toROSMsg(*filtered, *publish_cloud);
        
        publish_cloud->header.stamp = input->header.stamp;
        publish_cloud->header.frame_id = input->header.frame_id;

        cloud_pub->publish(std::move(publish_cloud));
    }
};


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ColorCluster)

