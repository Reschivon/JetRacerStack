#include <iostream>
#include <fstream>
#include <memory>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/assign/list_of.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/imgproc.hpp>
#include "delaunator.hpp"

namespace BorderFinder {

class BorderFinder : public nodelet::Nodelet
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

    double track_width, cone_spacing;
    double deviation_ratio;

    using Point = pcl::PointXYZ;
    using Cloud = pcl::PointCloud<Point>;

public:
    virtual void onInit() 
    {
        NODELET_DEBUG("Creating subscribers and publishers");
        
        n_ = getPrivateNodeHandle();
        cloud_sub = n_.subscribe("input", 10, &BorderFinder::cloudcb, this);
        cloud_pub = n_.advertise<nav_msgs::OccupancyGrid>("output", 1);

        n_.param("track_width", track_width, 0.2);
        n_.param("cone_spacing", cone_spacing, 0.15);
        n_.param("deviation_ratio", deviation_ratio, 0.2);
    }

    void cloudcb(const sensor_msgs::PointCloud2::ConstPtr &input) {
        Cloud cloud;
	    pcl::fromROSMsg(*input, cloud);

        if (cloud.empty()) {
            NODELET_WARN_STREAM_NAMED("Border Finder", "Received a landmark cloud with no points, ignoring");
            return;
        }

        // convert cloud to 2D points
        std::vector<double> coords;
        coords.reserve(cloud.size() * 2);

        int minX = 0, minY = 0, maxX = 0, maxY = 0;
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
            std::cout << "PPPPP " << point.z << " " << point.y << " " << point.z << std::endl;
        }

        int width = maxX - minX;
        int height = maxY - minY;
        double resolution = 0.1;

        // Dest occupancy map
        cv::Mat occupancy(width / resolution, height / resolution, CV_8S, cv::Scalar(255));

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
                // OK, this triangle works for us
                cv::fillConvexPoly(occupancy, points, cv::Scalar(255));
            }
        }

        // Copy the cv matrix to occupancy grid
        nav_msgs::OccupancyGridPtr map = nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid());
        
        // Copy the image data into the map structure
        map->info.width = occupancy.cols;
        map->info.height = occupancy.rows;
        map->info.resolution = resolution;
        map->info.origin.position.x = minX;
        map->info.origin.position.y = minY;
        map->info.origin.position.z = 0.0;
        tf2::Quaternion q;
        q.setEuler(0, 0, 0);
        map->info.origin.orientation.x = q.getX();
        map->info.origin.orientation.y = q.getY();
        map->info.origin.orientation.z = q.getZ();
        map->info.origin.orientation.w = q.getW();

        // Allocate space to hold the data
        map->data.reserve(occupancy.total());
        map->data.insert(map->data.begin(), occupancy.data, occupancy.data + occupancy.total());

        cloud_pub.publish(map);
    }

private:

    std::pair<double, double> norm(std::pair<double, double> coord) {
        double hypotLen = hypot(coord.first, coord.second);
        return {coord.first / hypotLen, coord.second / hypotLen};
    }
    
};

}

PLUGINLIB_EXPORT_CLASS(BorderFinder::BorderFinder, nodelet::Nodelet);
