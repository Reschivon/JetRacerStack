#include <iostream>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <boost/assign/list_of.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <pcl/conversions.h>

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "delaunator.hpp"

#include "dirty_conversions.hpp"

static const rmw_qos_profile_t latched_map_data_profile =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  5,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};


class BorderFinder : public rclcpp::Node
{

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cloud_pub;

    using Point = pcl::PointXYZ;
    using Cloud = pcl::PointCloud<Point>;

public:
    explicit BorderFinder(const rclcpp::NodeOptions& options)
	: Node("BorderFinder", rclcpp::NodeOptions(options).use_intra_process_comms(true))
    {
        RCLCPP_DEBUG(this->get_logger(), "Creating subscribers and publishers");
        
        cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>("input", 10, 
                std::bind(&BorderFinder::cloudcb, this, std::placeholders::_1));
        cloud_pub = create_publisher<nav_msgs::msg::OccupancyGrid>("output", 
                rclcpp::QoS(rclcpp::KeepLast(1), latched_map_data_profile));

        declare_parameter("track_width", 0.2);
        declare_parameter("cone_spacing", 0.15);
        declare_parameter("deviation_ratio", 0.2);
        declare_parameter("resolution", 0.03);
    }

    void cloudcb(const sensor_msgs::msg::PointCloud2::UniquePtr input) {
        double track_width = get_parameter("track_width").as_double();
        double cone_spacing = get_parameter("cone_spacing").as_double();
        double deviation_ratio = get_parameter("deviation_ratio").as_double();
        double resolution = get_parameter("resolution").as_double();
    
        Cloud cloud;
	    dirty_conversion::fromROSMsg(*input, cloud);

        if (cloud.empty()) {
            RCLCPP_WARN_STREAM(get_logger(), "Received a landmark cloud with no points, ignoring");
            return;
        }

        // convert cloud to 2D points
        std::vector<double> coords;
        coords.reserve(cloud.size() * 2);

        double minX = 0, minY = 0, maxX = 0, maxY = 0;
        for(const Point& point : cloud) {
            // TODO this is hardcoded for a wall mounted track
            float x = point.z;
            float y = point.y;

            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;

            coords.emplace_back(x);
            coords.emplace_back(y);
        }

        // minX -= 1;
        // maxX += 1;
        // minY -= 1;
        // maxY += 1;

        double width = maxX - minX;
        double height = maxY - minY;

        // Dest occupancy map
        cv::Mat occupancy(width / resolution, height / resolution, CV_8S, cv::Scalar(0));

        const auto meters_to_pixel = [&](double x, double y) -> std::pair<double, double>{
            return {(x - minX) / resolution,
                    (y - minY) / resolution};
        };

        // Actually compute delauney triangles
        delaunator::Delaunator d(coords);

        double ideal_centroid_deviation = 
                2 * sqrt((cone_spacing/2) * (cone_spacing/2) + (track_width/3) * (track_width/3)) +
                (track_width * 2.0/3);

        for(std::size_t i = 0; i < d.triangles.size(); i+=3) {
            double x1 = d.coords[2 * d.triangles[i]];        
            double y1 = d.coords[2 * d.triangles[i] + 1];    
            double x2 = d.coords[2 * d.triangles[i + 1]];    
            double y2 = d.coords[2 * d.triangles[i + 1] + 1];
            double x3 = d.coords[2 * d.triangles[i + 2]];    
            double y3 = d.coords[2 * d.triangles[i + 2] + 1];

            double centroidX = (x1 + x2 + x3) / 3;
            double centroidY = (y1 + y2 + y3) / 3;

            double centroidDeviation = 
                    hypot(x1 - centroidX, y1 - centroidY) + 
                    hypot(x2 - centroidX, y2 - centroidY) + 
                    hypot(x3 - centroidX, y3 - centroidY); 

            auto point1pix = meters_to_pixel(x1, y1);
            auto point2pix = meters_to_pixel(x2, y2);
            auto point3pix = meters_to_pixel(x3, y3);
            std::vector<cv::Point> points = boost::assign::list_of(cv::Point(point1pix.first, point1pix.second))
                                (cv::Point(point2pix.first, point2pix.second))
                                (cv::Point(point3pix.first, point3pix.second));
        
            if (abs(centroidDeviation / ideal_centroid_deviation - 1) < deviation_ratio) {
                std::cout << 127 * (centroidDeviation / ideal_centroid_deviation) << std::endl;
                // OK, this triangle works for us
                cv::fillConvexPoly(occupancy, points, cv::Scalar(50 * (centroidDeviation / ideal_centroid_deviation)));
            }
        }

        cv::imwrite("immmmm.png", occupancy);

        // Copy the cv matrix to occupancy grid
        nav_msgs::msg::OccupancyGrid::UniquePtr map(new nav_msgs::msg::OccupancyGrid());
        
        // Copy the image data into the map structure
        map->info.width = occupancy.cols;
        map->info.height = occupancy.rows;
        map->info.resolution = resolution;
        map->info.origin.position.x = minX;
        map->info.origin.position.y = minY;
        map->info.origin.position.z = 0.0;
        tf2::Quaternion q;
        q.setEuler(-1.51, 0, 0);
        map->info.origin.orientation.x = q.getX();
        map->info.origin.orientation.y = q.getY();
        map->info.origin.orientation.z = q.getZ();
        map->info.origin.orientation.w = q.getW();

        // Allocate space to hold the data
        map->data.reserve(occupancy.total());
        map->data.insert(map->data.begin(), occupancy.data, occupancy.data + occupancy.total());

        map->header.frame_id = "map";

        std::cout << "PUBLISHEDD!!!!" << std::endl;
        cloud_pub->publish(std::move(map));
    }

private:

    std::pair<double, double> norm(std::pair<double, double> coord) {
        double hypotLen = hypot(coord.first, coord.second);
        return {coord.first / hypotLen, coord.second / hypotLen};
    }
    
};


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BorderFinder)
