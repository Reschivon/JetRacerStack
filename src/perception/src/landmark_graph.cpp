#include <iostream>
#include <fstream>
#include <memory>
#include <stdint.h>
#include <mutex>
#include <utility>
	
#include <rclcpp/rclcpp.hpp>

#include <rclcpp/qos.hpp>
// #include <rclcpp/clock.hpp>
// #include <rclcpp/cock.hpp> heh heh hehehe

#include <Eigen/Geometry> 

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/common/transforms.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/conditional_removal.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>

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

class LandmarkGraph : public rclcpp::Node
{

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cum_path_pub;

    // This node has two modes
    // first accumulate landmark pointclouds and store them
    // then after one lap, stop listening and broadcast a latched corrected
    // map of all landmarks. Then stop
    bool accumulate_mode = true;

    // TF listener continuously writes to buffer in its own thread
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    // Maintain a queue of most recent 10 paths
    std::deque<nav_msgs::msg::Path::ConstSharedPtr> paths;
    std::mutex paths_lock;
    size_t paths_size = 10;

    // Snapshots are records of landmark poses relative to a central coordinate  
    using Point = pcl::PointXYZ;
    using Cloud = pcl::PointCloud<Point>;

    // Store snapshots and their corresponding path_map poses (defined as their timestamp)
    std::vector<std::tuple<rclcpp::Time, geometry_msgs::msg::Pose, Cloud>> snapshots;

    geometry_msgs::msg::Pose begin_pose;

    // Needed to find translation for triggering snapshot-taking code
    geometry_msgs::msg::Pose last_snapshot_pose;

    std::string publish_frame;

public:
    explicit LandmarkGraph(const rclcpp::NodeOptions& options)
	: Node("LandmarkGraph", rclcpp::NodeOptions(options).use_intra_process_comms(true))
    {
        RCLCPP_DEBUG(this->get_logger(), "Creating subscribers and publishers");
        
        path_sub = create_subscription<nav_msgs::msg::Path>("path", 1, 
            std::bind(&LandmarkGraph::pathcb, this, std::placeholders::_1));
        cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>("cloud", 1, 
            std::bind(&LandmarkGraph::cloudcb, this, std::placeholders::_1));
        cum_path_pub = create_publisher<sensor_msgs::msg::PointCloud2>("cum_path",
            rclcpp::QoS(rclcpp::KeepLast(1), latched_map_data_profile));

        declare_parameter("lap_proximity_meters", 4.0);
        declare_parameter("min_snapshots_in_lap", 4);       
        declare_parameter("publish_frame", "map"); 
        declare_parameter("distance_for_snapshot", 0.1);    

        // This sucker needs to be inited after the node
        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    }

    // Keep a queue of the most recent 10 Paths so when the (much slower)
    // pointcloud topic publishes, it can select the closest matching path from the queue
    void pathcb(const nav_msgs::msg::Path::ConstSharedPtr input)
    {
        std::lock_guard<std::mutex> scoped(paths_lock);

        if(paths.empty()) {
            paths.push_back(input);
            return;
        }

        // do not add if it is not sequential
        if(input->header.stamp.sec < paths.back()->header.stamp.sec)
            return;
        else if(input->header.stamp.sec == paths.back()->header.stamp.sec && 
                input->header.stamp.nanosec < paths.back()->header.stamp.nanosec)
            return;

        // insert
        paths.push_back(input);

        if (paths.size() > paths_size)
            paths.pop_front();
    }

    void cloudcb(const sensor_msgs::msg::PointCloud2::UniquePtr cloud)
    {

        double lap_proximity_meters = get_parameter("lap_proximity_meters").as_double();
        int min_snapshots_in_lap = get_parameter("min_snapshots_in_lap").as_int();
        publish_frame = get_parameter("publish_frame").as_string();
        double distance_for_snapshot = get_parameter("distance_for_snapshot").as_double(); // meters

        if (!accumulate_mode)
            return;

        // find latest path message that comes before the pointcloud,
        // then extract the last pose from that path message
        // This pose most closely aligns with there the camera was when the
        // cones were observed
        nav_msgs::msg::Path::ConstSharedPtr current_path;
        geometry_msgs::msg::PoseStamped current_pose;
        {
            std::lock_guard<std::mutex> scoped(paths_lock);

            // No latest_path yet
            if(paths.empty())  {
                return;
            }

            for(int i = paths.size() - 1; ; i--) {
                if (i < 0) {
                    // either 'paths' is empty or there are no paths before `cloud`
                    return;
                }

                if(rclcpp::Time(paths.at(i)->header.stamp) < rclcpp::Time(cloud->header.stamp)) {
                    current_path = paths.at(i);
                    current_pose = paths.at(i)->poses.back();
                    break;
                }
            }
        } // end mutexed scope

        // Continue to take snapshots of the landmarks if the pose has changed alot
        // Note current_pose is derived from path_map, which can contain loop closing jumps,
        // rendering the distance pretty unpredicatable. However, I dont really care

        auto current_position = current_pose.pose.position;
        double distance_from_last = dist(current_position, last_snapshot_pose.position);
        double distance_from_start = dist(current_position, begin_pose.position);
                

        /* // Should we stop listening and compute the final map
        if(distance_from_start < lap_proximity_meters && paths.size() > min_snapshots_in_lap) {
            accumulate_mode = false;
            cum_path_pub.publish(compute_final_map(current_path));
            return;
        } */
        // TODO remove so this only happens once
        publish_final_map(current_path);
        
        if (distance_from_last > distance_for_snapshot)
            last_snapshot_pose = current_pose.pose;
        else
            return;

        // convert from cloud frame to camera pose frame
        geometry_msgs::msg::TransformStamped cloud_to_cam;
        try {

            cloud_to_cam = tfBuffer->lookupTransform(
                current_pose.header.frame_id,   // dest (map)
                cloud->header.frame_id,         // source (zed2_left_camera_optical_frame)
                cloud->header.stamp);
            
            // std::cout << "From " << cloud->header.frame_id << " To " << current_pose.header.frame_id <<
            // "\ntf: " << transformStamped << std::endl;

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "[Landmark Graph Error] %s", ex.what());
            // lmao the github issues for this in ros2 was such a fun rabbit hole 
            // although Clock::sleep_for was implemented in 2021, it didn;t make it into foxy
            // rclcpp::Time(1.0 * 10e9).sleep();
            return;
        }

        Cloud::Ptr cloud_pcl(new pcl::PointCloud<Point>);
	    dirty_conversion::fromROSMsg(*cloud, *cloud_pcl);
        const Cloud::Ptr cloud_pcl_const(cloud_pcl);

        Eigen::Affine3d c2ctEigen = tf2::transformToEigen(cloud_to_cam);

        Cloud cloud_transformed;
        pcl::transformPointCloud(*cloud_pcl_const, cloud_transformed, c2ctEigen);

        RCLCPP_INFO(get_logger(), "[Landmark Graph] Storing new snapshot of size %lu", cloud_transformed.size());
        snapshots.emplace_back(std::make_tuple(
            current_path->header.stamp, 
            current_pose.pose, 
            cloud_transformed));
    }

private:

    void publish_final_map(nav_msgs::msg::Path::ConstSharedPtr path) {        
        auto curr_path_entry = path->poses.begin();

        Cloud::Ptr accum_landmarks(new Cloud);

        // Correct the pointcloud in each snapshot with updated poses from path
        // I considered enabling c++17 for auto[tuple] but it would hit compile times
        for(const auto& tuple : snapshots) {
            // Get snapshot time, pose (global static frame), and cloud
            const auto& stamp = std::get<0>(tuple);
            const auto& pose = std::get<1>(tuple);
            const auto& cloud = std::get<2>(tuple);

            // find corrected pose with the same stamp as snapshot pose
            while (stamp != rclcpp::Time(curr_path_entry->header.stamp)) {
                curr_path_entry++;

                if (curr_path_entry == path->poses.end()) {
                    // ran out of paths (should not happen)
                    RCLCPP_WARN_STREAM(get_logger(), "In parsing path_map data, could not find "
                                        "corresponding poses for all snapshots"); 
                    return;
                }
            }

            // get current pose (global static frame)
            const auto& curr_pose = curr_path_entry->pose;
            tf2::Transform tfpose, tfcurr_pose;
            // convert snapshot pose and curr_pose to tf::Pose for its inverse() function
            tf2::fromMsg(pose, tfpose);
            tf2::fromMsg(curr_pose, tfcurr_pose);

            // Get disparity between recorded pose and corrected pose
            const tf2::Transform disparity = tfpose.inverseTimes(tfcurr_pose);
            Eigen::Affine3d disparityEigen = tf2::transformToEigen(tf2::toMsg(disparity));
            
            
            // transform snapshot pointcloud by disparity
            Cloud cloud_transformed;
            pcl::transformPointCloud(cloud, cloud_transformed, disparityEigen);

            // add corrected pointcloud to Mother
            accum_landmarks->operator+=(cloud_transformed);

            RCLCPP_DEBUG_STREAM(get_logger(), "Append cum path; points: " << cloud_transformed.size());
        }

        RCLCPP_DEBUG_STREAM(get_logger(), "Publish cum path; sipointsze: " << accum_landmarks->size());

        // convert accum_landmarks to ros message
        sensor_msgs::msg::PointCloud2::UniquePtr publish_cloud(new sensor_msgs::msg::PointCloud2);
        dirty_conversion::toROSMsg(*accum_landmarks, *publish_cloud);
        publish_cloud->header.stamp = rclcpp::Time(0); // now
        publish_cloud->header.frame_id = publish_frame;

        cum_path_pub->publish(std::move(publish_cloud));
    }

    double dist(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b) {
        return std::sqrt(
            ((a.x - b.x) * (a.x - b.x)) + 
            ((a.y - b.y) * (a.y - b.y)) +
            ((a.z - b.z) * (a.z - b.z)));
    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(LandmarkGraph)