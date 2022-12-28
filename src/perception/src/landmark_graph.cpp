#include <iostream>
#include <fstream>
#include <memory>
#include <stdint.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>

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

    // Yes, there are possible multithread read/write bugs, but
    // the topics go slow enough that they're unnoticeable and negligible
    // we can simply drop some and that's fine
    nav_msgs::Path::ConstPtr latest_path;

public:
    virtual void onInit() 
    {
        NODELET_DEBUG("Creating subscribers and publishers");
        
        n_ = getPrivateNodeHandle();
        path_sub = n_.subscribe("path", 10, &LandmarkGraph::pathcb, this);
        cloud_sub = n_.subscribe("cloud", 10, &LandmarkGraph::cloudcb, this);
    }

    void pathcb(const nav_msgs::Path::ConstPtr &input)
    {
        NODELET_INFO("New Path at time: %u %u", input->header.stamp.sec, input->header.stamp.nsec);
        
        // if it doesnt exit just unconditionally set it
        if(!latest_path) {
            latest_path = input;
            return;
        }

        if(input->header.stamp.sec > latest_path->header.stamp.sec)
            latest_path = input;
        else if(input->header.stamp.sec == latest_path->header.stamp.sec && 
                input->header.stamp.nsec > latest_path->header.stamp.nsec)
            latest_path = input;
    }

    void cloudcb(const sensor_msgs::PointCloud2::ConstPtr &input)
    {
        NODELET_INFO("New Cone Cloud at time: %u %u", input->header.stamp.sec, input->header.stamp.nsec);

        // No latest_path yet
        if(!latest_path) 
            return;

        // is input time > latest_path time?
        if (input->header.stamp.sec < latest_path->header.stamp.sec)
            return;
        if (input->header.stamp.sec == latest_path->header.stamp.sec && 
            input->header.stamp.nsec < latest_path->header.stamp.nsec)
            return;
        
        // time difference not > 1 sec
        if (input->header.stamp.sec - latest_path->header.stamp.sec > 1)
            return;

        if(input->header.frame_id != latest_path->header.frame_id) {
            NODELET_INFO("The frames %s and %s do not match. Consider implementing tf transforms if you need it!", 
                input->header.frame_id.c_str(), latest_path->header.frame_id.c_str());
            return;
        }

        if (latest_path->poses.empty())
            return;

        geometry_msgs::PoseStamped latest_pose = latest_path->poses.back();

    }
};

}

PLUGINLIB_EXPORT_CLASS(LandmarkGraph::LandmarkGraph, nodelet::Nodelet);
