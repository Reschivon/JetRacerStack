#include <iostream>
#include <fstream>
#include <memory>
#include <stdint.h>
#include <mutex>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/conditional_removal.h>

namespace LandmarkGraph {


class LandmarkGraph : public nodelet::Nodelet
{

private:
    ros::NodeHandle n_;
    ros::Subscriber path_sub, cloud_sub;
    ros::Publisher cum_path_pub;

    // This node has two modes
    // first accumulate landmark pointclouds and store them
    // then after one lap, stop listening and broadcast a latched corrected
    // map of all landmarks. Then stop
    bool accumulate_mode = true;

    // TF listener continuously writes to buffer in its own thread
    tf2_ros::Buffer tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;

    // Maintain a queue of most recent 10 paths
    std::deque<nav_msgs::Path::ConstPtr> paths;
    std::mutex paths_lock;
    int paths_size = 10;

    // Snapshots are records of landmark poses relative to a central coordinate  
    using Point = pcl::PointXYZ;
    using Cloud = pcl::PointCloud<Point>;
    // Store snapshots and their corresponding path_map poses (defined as their timestamp)
    std::vector<std::tuple<ros::Time, geometry_msgs::Pose, Cloud>> snapshots;

    geometry_msgs::Pose begin_pose;
    double lap_proximity_meters;
    int min_snapshots_in_lap;
    std::string publish_frame;

    // Needed to find translation for triggering snapshot-taking code
    geometry_msgs::Pose last_snapshot_pose;
    double distance_for_snapshot = 0.1; // meters

public:
    virtual void onInit() 
    {
        NODELET_DEBUG("Creating subscribers and publishers");
        
        n_ = getPrivateNodeHandle();
        path_sub = n_.subscribe("path", 1, &LandmarkGraph::pathcb, this);
        cloud_sub = n_.subscribe("cloud", 1, &LandmarkGraph::cloudcb, this);
        cum_path_pub = n_.advertise<sensor_msgs::PointCloud2>("cum_path", 1, true);

        n_.param("lap_proximity_meters", lap_proximity_meters, 4.0);
        n_.param("min_snapshots_in_lap", min_snapshots_in_lap, 4);       
        n_.param<std::string>("publish_frame", publish_frame, "map");        

        // This sucker needs to be inited after the node
        tfListener = std::make_unique<tf2_ros::TransformListener>(tfBuffer);
    }

    // Keep a queue of the most recent 10 Paths so when the (much slower)
    // pointcloud topic publishes, it can select the closest matching path from the queue
    void pathcb(const nav_msgs::Path::ConstPtr &input)
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
                input->header.stamp.nsec < paths.back()->header.stamp.nsec)
            return;

        // insert
        paths.push_back(input);

        if (paths.size() > paths_size)
            paths.pop_front();
    }

    void cloudcb(const sensor_msgs::PointCloud2::ConstPtr &cloud)
    {
        if (!accumulate_mode)
            return;

        // find latest path message that comes before the pointcloud,
        // then extract the last pose from that path message
        // This pose most closely aligns with there the camera was when the
        // cones were observed
        nav_msgs::PathConstPtr current_path;
        geometry_msgs::PoseStamped current_pose;
        {
            std::lock_guard<std::mutex> scoped(paths_lock);

            // No latest_path yet
            if(paths.empty())  {
                return;
            }

            const auto lessThan = [](ros::Time a, ros::Time b) {
                if(a.sec < b.sec)
                    return true;
                if(a.sec == b.sec && a.nsec < b.nsec)
                    return true;
                return false;
            };

            for(int i = paths.size() - 1; ; i--) {
                if (i < 0) {
                    // either 'paths' is empty or there are no paths before `cloud`
                    return;
                }

                if(lessThan(paths.at(i)->header.stamp, cloud->header.stamp)) {
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
        cum_path_pub.publish(compute_final_map(current_path));
        
        if (distance_from_last > distance_for_snapshot)
            last_snapshot_pose = current_pose.pose;
        else
            return;

        // convert from cloud frame to camera pose frame
        geometry_msgs::TransformStamped cloud_to_cam;
        try {

            cloud_to_cam = tfBuffer.lookupTransform(
                current_pose.header.frame_id,   // dest (map)
                cloud->header.frame_id,         // source (zed2_left_camera_optical_frame)
                cloud->header.stamp);
            
            // std::cout << "From " << cloud->header.frame_id << " To " << current_pose.header.frame_id <<
            // "\ntf: " << transformStamped << std::endl;

        } catch (tf2::TransformException &ex) {
            ROS_WARN("[Landmark Graph Error] %s", ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

        Cloud::Ptr cloud_pcl(new pcl::PointCloud<Point>);
	    pcl::fromROSMsg(*cloud, *cloud_pcl);
        const Cloud::Ptr cloud_pcl_const(cloud_pcl);

        Cloud cloud_transformed;
        pcl_ros::transformPointCloud(*cloud_pcl_const, cloud_transformed, cloud_to_cam.transform);

        NODELET_INFO("[Landmark Graph] Storing new snapshot of size %lu", cloud_transformed.size());
        snapshots.emplace_back(std::make_tuple(
            current_path->header.stamp, 
            current_pose.pose, 
            cloud_transformed));
    }

private:

    sensor_msgs::PointCloud2 compute_final_map(nav_msgs::PathConstPtr path) {        
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
            while (!equal(stamp, curr_path_entry->header.stamp)) {
                curr_path_entry++;

                if (curr_path_entry == path->poses.end()) {
                    // ran out of paths (should not happen)
                    ROS_WARN_STREAM_NAMED("Landmark Graph", "In parsing path_map data, could not find "
                                        "corresponding poses for all snapshots"); 
                    return sensor_msgs::PointCloud2();
                }
            }

            // get current pose (global static frame)
            const auto& curr_pose = curr_path_entry->pose;
            tf::Pose tfpose, tfcurr_pose;
            // convert snapshot pose and curr_pose to tf::Pose for its inverse() function
            tf::poseMsgToTF(pose, tfpose);
            tf::poseMsgToTF(curr_pose, tfcurr_pose);

            // Get disparity between recorded pose and corrected pose
            const auto &disparity = tfpose.inverseTimes(tfcurr_pose);
            
            // transform snapshot pointcloud by disparity
            Cloud cloud_transformed;
            pcl_ros::transformPointCloud(cloud, cloud_transformed, disparity);

            // add corrected pointcloud to Mother
            accum_landmarks->operator+=(cloud_transformed);

            ROS_DEBUG_STREAM_NAMED("Landmark Graph", "Append cum path; points: " << cloud_transformed.size());
        }

        ROS_DEBUG_STREAM_NAMED("Landmark Graph", "Publish cum path; sipointsze: " << accum_landmarks->size());

        // convert accum_landmarks to ros message
        sensor_msgs::PointCloud2 publish_cloud;
        pcl::toROSMsg(*accum_landmarks, publish_cloud);
        publish_cloud.header.stamp = ros::Time(0); // now
        publish_cloud.header.frame_id = publish_frame;

        return publish_cloud;
    }

    double dist(geometry_msgs::Point a, geometry_msgs::Point b) {
        return std::sqrt(
            ((a.x - b.x) * (a.x - b.x)) + 
            ((a.y - b.y) * (a.y - b.y)) +
            ((a.z - b.z) * (a.z - b.z)));
    }

    bool lessThan(ros::Time a, ros::Time b) {
        if(a.sec < b.sec)
            return true;
        if(a.sec == b.sec && a.nsec < b.nsec)
            return true;
        return false;
    };

    bool equal(ros::Time a, ros::Time b) {
        return (a.sec == b.sec && a.nsec == b.nsec);
    };
    
};

}

PLUGINLIB_EXPORT_CLASS(LandmarkGraph::LandmarkGraph, nodelet::Nodelet);
