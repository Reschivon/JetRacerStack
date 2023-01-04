
#ifndef DIRTY_CONVERSIONS
#define DIRTY_CONVERSIONS

#include <rclcpp/rclcpp.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace dirty_conversion {
inline
void toPCL(const sensor_msgs::msg::PointField &pf, pcl::PCLPointField &pcl_pf)
{
    pcl_pf.name = pf.name;
    pcl_pf.offset = pf.offset;
    pcl_pf.datatype = pf.datatype;
    pcl_pf.count = pf.count;
}

 inline
  void toPCL(const std::vector<sensor_msgs::msg::PointField> &pfs, std::vector<pcl::PCLPointField> &pcl_pfs)
  {
    pcl_pfs.resize(pfs.size());
    std::vector<sensor_msgs::msg::PointField>::const_iterator it = pfs.begin();
    int i = 0;
    for(; it != pfs.end(); ++it, ++i) {
      toPCL(*(it), pcl_pfs[i]);
    }
  }

  
  inline
  void toPCL(const rclcpp::Time &stamp, std::uint64_t &pcl_stamp)
  {
    pcl_stamp = stamp.nanoseconds() / 1000ull;  // Convert from ns to us
  }

 inline
  void toPCL(const std_msgs::msg::Header &header, pcl::PCLHeader &pcl_header)
  {
    toPCL(header.stamp, pcl_header.stamp);
    // TODO(clalancette): Seq doesn't exist in the ROS2 header
    // anymore.  wjwwood suggests that we might be able to get this
    // information from the middleware in the future, but for now we
    // just set it to 0.
    pcl_header.seq = 0;
    pcl_header.frame_id = header.frame_id;
  }

inline
  void copyPointCloud2MetaData(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
  {
    toPCL(pc2.header, pcl_pc2.header);
    pcl_pc2.height = pc2.height;
    pcl_pc2.width = pc2.width;
    toPCL(pc2.fields, pcl_pc2.fields);
    pcl_pc2.is_bigendian = pc2.is_bigendian;
    pcl_pc2.point_step = pc2.point_step;
    pcl_pc2.row_step = pc2.row_step;
    pcl_pc2.is_dense = pc2.is_dense;
  }

inline
  void toPCL(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
  {
    copyPointCloud2MetaData(pc2, pcl_pc2);
    pcl_pc2.data = pc2.data;
  }

template<typename T>
void fromROSMsg(const sensor_msgs::msg::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud)
{
    pcl::PCLPointCloud2 pcl_pc2;
    toPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
}






  inline
  void fromPCL(const std::uint64_t &pcl_stamp, rclcpp::Time &stamp)
  {
    stamp = rclcpp::Time(pcl_stamp * 1000ull); // Convert from us to ns
  }

 /** PCLPointField <=> PointField **/

  inline
  void fromPCL(const pcl::PCLPointField &pcl_pf, sensor_msgs::msg::PointField &pf)
  {
    pf.name = pcl_pf.name;
    pf.offset = pcl_pf.offset;
    pf.datatype = pcl_pf.datatype;
    pf.count = pcl_pf.count;
  }

 inline
  rclcpp::Time fromPCL(const std::uint64_t &pcl_stamp)
  {
    rclcpp::Time stamp;
    fromPCL(pcl_stamp, stamp);
    return stamp;
  }
   inline
  void fromPCL(const std::vector<pcl::PCLPointField> &pcl_pfs, std::vector<sensor_msgs::msg::PointField> &pfs)
  {
    pfs.resize(pcl_pfs.size());
    std::vector<pcl::PCLPointField>::const_iterator it = pcl_pfs.begin();
    int i = 0;
    for(; it != pcl_pfs.end(); ++it, ++i) {
      fromPCL(*(it), pfs[i]);
    }
  }

 inline
  void fromPCL(const pcl::PCLHeader &pcl_header, std_msgs::msg::Header &header)
  {
    header.stamp = fromPCL(pcl_header.stamp);
    header.frame_id = pcl_header.frame_id;
  }

 inline
  void copyPCLPointCloud2MetaData(const pcl::PCLPointCloud2 &pcl_pc2, sensor_msgs::msg::PointCloud2 &pc2)
  {
    fromPCL(pcl_pc2.header, pc2.header);
    pc2.height = pcl_pc2.height;
    pc2.width = pcl_pc2.width;
    fromPCL(pcl_pc2.fields, pc2.fields);
    pc2.is_bigendian = pcl_pc2.is_bigendian;
    pc2.point_step = pcl_pc2.point_step;
    pc2.row_step = pcl_pc2.row_step;
    pc2.is_dense = pcl_pc2.is_dense;
  }

inline
  void moveFromPCL(pcl::PCLPointCloud2 &pcl_pc2, sensor_msgs::msg::PointCloud2 &pc2)
  {
    copyPCLPointCloud2MetaData(pcl_pc2, pc2);
    pc2.data.swap(pcl_pc2.data);
  }


  template<typename T>
  void toROSMsg(const pcl::PointCloud<T> &pcl_cloud, sensor_msgs::msg::PointCloud2 &cloud)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
    moveFromPCL(pcl_pc2, cloud);
  }
}

#endif