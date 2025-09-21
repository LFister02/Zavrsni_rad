#include "monocular-slam-node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;

    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;
    
    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    m_mappoints_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("orbslam/map_points", 10);

    m_initial_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",
        10,
        std::bind(&MonocularSlamNode::initialPoseCallback, this, std::placeholders::_1));
    
    position_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    orientation_ = Eigen::Quaternionf::Identity();
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Node shutdown completed.");
}


void MonocularSlamNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // Prihvat poze iz /initialpose
    auto pos = msg->pose.pose.position;
    auto ori = msg->pose.pose.orientation;

    position_(0) = static_cast<float>(pos.x);
    position_(1) = static_cast<float>(pos.y);
    position_(2) = static_cast<float>(pos.z);

    orientation_ = Eigen::Quaternionf(static_cast<float>(ori.w),
                                     static_cast<float>(ori.x),
                                     static_cast<float>(ori.y),
                                     static_cast<float>(ori.z));

    m_SLAM->Reset();

    RCLCPP_INFO(this->get_logger(), "Nova početna pozicija primljena: x=%.2f y=%.2f z=%.2f", position_(0), position_(1), position_(2));
}


void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    try
    {
        // Pretvori ROS sliku u OpenCV format
        m_cvImPtr = cv_bridge::toCvCopy(msg);

        // Praćenje monocular SLAM-a
        Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
        Sophus::SE3f Twc = Tcw.inverse();

        //  Usklađivanje osi ORB - ROS 2 
        Eigen::Vector3f t_orb = Twc.translation();
        Eigen::Vector3f t_ros;
        t_ros << t_orb(0),
                 t_orb(2),
                -t_orb(1);

        Eigen::Matrix3f R_orb = Twc.rotationMatrix();
        double yaw = std::atan2(R_orb(0,2), R_orb(2,2));
        Eigen::Quaternionf q_fixed(Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()));

        Eigen::Quaternionf q_rot(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()));

        Eigen::Quaternionf q_corrected = q_rot * q_fixed;

        Eigen::Quaternionf q_corrected_inv = q_corrected.conjugate();

        float scale = 1.85f;    // scale faktor
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link_1";

        tf_msg.transform.translation.x = position_(0) + t_ros(0) * scale;
        tf_msg.transform.translation.y = position_(1) + t_ros(1) * scale;
        tf_msg.transform.translation.z = position_(2) + t_ros(2) * scale;

        tf_msg.transform.rotation.x = q_corrected_inv.x();
        tf_msg.transform.rotation.y = q_corrected_inv.y();
        tf_msg.transform.rotation.z = q_corrected_inv.z();
        tf_msg.transform.rotation.w = q_corrected_inv.w();

        // Objava transformacije
        m_tf_broadcaster->sendTransform(tf_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Objavi map points kao PointCloud2
    std::vector<ORB_SLAM3::MapPoint*> mapPoints = m_SLAM->GetMap()->GetAllMapPoints();
    sensor_msgs::msg::PointCloud2 cloud_msg = convertMapPointsToPointCloud(mapPoints);
    m_mappoints_pub->publish(cloud_msg);
}

sensor_msgs::msg::PointCloud2 MonocularSlamNode::convertMapPointsToPointCloud(
    const std::vector<ORB_SLAM3::MapPoint*>& mapPoints)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = this->get_clock()->now();

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(mapPoints.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    // Transformacija ORB_SLAM3 - ROS
    Eigen::Matrix3f t_orb_ros;
    t_orb_ros << 1, 0, 0,
                 0, 0, 1,
                 0,-1, 0;

    float scale = 1.85f;     // scale faktor

    for (auto* mp : mapPoints)
    {
        if (!mp || mp->isBad()) continue;

        Eigen::Vector3f pos_orb = mp->GetWorldPos() * scale;
        Eigen::Vector3f pos_ros = t_orb_ros * pos_orb;

        *iter_x = pos_ros.x();
        *iter_y = pos_ros.y();
        *iter_z = pos_ros.z();

        ++iter_x; ++iter_y; ++iter_z;
    }

    return cloud_msg;
}

