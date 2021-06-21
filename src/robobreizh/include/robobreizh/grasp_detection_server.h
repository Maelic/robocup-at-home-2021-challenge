

#ifndef GRASP_DETECTION_SERVER_H_
#define GRASP_DETECTION_SERVER_H_


// ROS
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// GPD
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>
#include <gpd/candidate/hand.h>

// this project (services)
#include <robobreizh/detect_grasps.h>

// this project (headers)
#include <robobreizh/grasp_plotter.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class GraspDetectionServer
{
public:
  struct GraspPose{
    geometry_msgs::Pose pre;
    geometry_msgs::Pose actual;
    geometry_msgs::Pose after;
    float distance;
  };

  /**
   * \brief Constructor.
   * \param node the ROS node
  */
  GraspDetectionServer(ros::NodeHandle& node);

  /**
   * \brief Destructor.
  */
  ~GraspDetectionServer()
  {
    delete cloud_camera_;
    delete grasp_detector_;
  }

  /**
   * \brief Service callback for detecting grasps.
   * \param req the service request
   * \param res the service response
   */
  bool detectGrasps(robobreizh::detect_grasps::Request& req, robobreizh::detect_grasps::Response& res);
  bool detectAllGrasps(robobreizh::detect_grasps::Request& req, robobreizh::detect_grasps::Response& res);
  bool detectGraspsWithoutGPD(robobreizh::detect_grasps::Request& req, robobreizh::detect_grasps::Response& res);
  void publish_frame(const gpd::candidate::Hand& hand);

  robobreizh::GraspConfigList createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::Header& header);
  robobreizh::GraspConfig convertToGraspMsg(const gpd::candidate::Hand& hand);
  void PointCloudXYZRGBtoXYZRGBA(pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGBA>& out);
  GraspDetectionServer::GraspPose createPickingEEFPose(robobreizh::GraspConfig grasp_msg);

private:

  ros::Publisher grasps_pub_; ///< ROS publisher for grasp list messages

  std_msgs::Header cloud_camera_header_; ///< stores header of the point cloud
  std::string frame_; ///< point cloud frame

  gpd::GraspDetector* grasp_detector_; ///< used to run the grasp pose detection
  gpd::util::Cloud* cloud_camera_; ///< stores point cloud with (optional) camera information and surface normals

  bool use_rviz_; ///< if rviz is used for visualization instead of PCL
  std::vector<double> workspace_; ///< workspace limits
  Eigen::Vector3d view_point_; ///< (input) view point of the camera onto the point cloud
  GraspPlotter* rviz_plotter_; ///< used to plot detected grasps in rviz
  tf::TransformListener listener_; 

};

#endif /* GRASP_DETECTION_SERVER_H_ */
