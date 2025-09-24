#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <mutex>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class FullMapSaver
{
public:
  FullMapSaver()
  {
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("topic", topic_, "/registered_scan");
    pnh.param<std::string>("output", output_file_, "full_map.pcd");

    sub_ = nh_.subscribe(topic_, 10, &FullMapSaver::cloudCallback, this);
  }

  ~FullMapSaver()
  {
    saveMap();
  }

  void spin() { ros::spin(); }

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*msg, *cloud);
    if (!map_) map_.reset(new PointCloud);
    *map_ += *cloud;
  }

  void saveMap()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (map_ && !map_->empty())
    {
      ROS_INFO("Saving %zu points to %s", map_->size(), output_file_.c_str());
      pcl::io::savePCDFileBinary(output_file_, *map_);
    }
    else
    {
      ROS_WARN("No data to save.");
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::string topic_, output_file_;
  PointCloud::Ptr map_;
  std::mutex mutex_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "full_map_saver");
  FullMapSaver saver;
  saver.spin();
  return 0;
}
