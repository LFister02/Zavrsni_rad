#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include "System.h"
#include "Map.h"
#include "utility.hpp"
#include <memory>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>



class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_mappoints_pub;

    sensor_msgs::msg::PointCloud2 convertMapPointsToPointCloud(const std::vector<ORB_SLAM3::MapPoint*>& mapPoints);
    
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initial_pose_sub;
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    Eigen::Vector3f position_;
    Eigen::Quaternionf orientation_;
};

#endif
