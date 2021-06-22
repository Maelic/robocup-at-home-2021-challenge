#include <robobreizh/grasp_detection_server.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <pcl/filters/filter.h>

struct GraspPose{
  geometry_msgs::Pose pre;
  geometry_msgs::Pose actual;
  geometry_msgs::Pose after;
  float distance;
};


GraspDetectionServer::GraspDetectionServer(ros::NodeHandle& node)
{
  cloud_camera_ = NULL;

  // set camera viewpoint to default origin
  auto camera_position = std::vector<double> {0.0, 0.0, 0.0};
  node.getParam("camera_position", camera_position);
  view_point_ << camera_position[0], camera_position[1], camera_position[2];

  std::string cfg_file;
  node.param("config_file", cfg_file, std::string("/workspace/src/robobreizh/cfg/ros_eigen_params.cfg"));
  grasp_detector_ = new gpd::GraspDetector(cfg_file);

  rviz_plotter_ = new GraspPlotter(node, grasp_detector_->getHandSearchParameters().hand_geometry_);

  // Advertise ROS topic for detected grasps.
  grasps_pub_ = node.advertise<robobreizh::GraspConfigList>("clustered_grasps", 10);

  node.getParam("workspace", workspace_);
}

geometry_msgs::PoseStamped vect3ToPoseStamped(geometry_msgs::Vector3 pose) {
  geometry_msgs::PoseStamped out;
  out.header.frame_id = "head_rgbd_sensor_rgb_frame";
  out.pose.position.x = pose.x;
  out.pose.position.y = pose.y;
  out.pose.position.z = pose.z;
  out.pose.orientation.x = 0;
  out.pose.orientation.y = 0;
  out.pose.orientation.z = 0;
  out.pose.orientation.w = 1;

  return out;
}

geometry_msgs::Vector3 poseStampedToVect3(geometry_msgs::PoseStamped pose) {
  geometry_msgs::Vector3 out;
  out.x = pose.pose.position.x;
  out.y = pose.pose.position.y;
  out.z = pose.pose.position.z;

  return out;
}

// Function for creating end-effector poses from GPD msg.
GraspDetectionServer::GraspPose GraspDetectionServer::createPickingEEFPose(robobreizh::GraspConfig grasp_msg) {
  GraspPose grasp_pose;
  tf::StampedTransform tf_base_odom;

  // TODO: try transformPose : http://docs.ros.org/indigo/api/tf/html/c++/classtf_1_1Transformer.html#a0a7b72eb4cc62194164d030afbefda9a
  tf::Matrix3x3 rot_matrix_grasp_base(-grasp_msg.axis.x, grasp_msg.binormal.x, grasp_msg.approach.x,
                                      -grasp_msg.axis.y, grasp_msg.binormal.y, grasp_msg.approach.y,
                                      -grasp_msg.axis.z, grasp_msg.binormal.z, grasp_msg.approach.z);

  tf::Vector3 tr_grasp_base(grasp_msg.position.x, grasp_msg.position.y, grasp_msg.position.z);
  tf::Transform tf_grasp_base(rot_matrix_grasp_base, tr_grasp_base);

  try {
      listener_.waitForTransform("odom", "head_rgbd_sensor_rgb_frame", ros::Time(0), ros::Duration(3.0) );
      listener_.lookupTransform("odom", "head_rgbd_sensor_rgb_frame", ros::Time(0), tf_base_odom);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }

    // Find grasp pose
    tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base;
    tf::poseTFToMsg(tf_grasp_odom, grasp_pose.actual);

    // Find pre-grasp pose
    tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.1));
    tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
    tf::poseTFToMsg(tf_pregrasp_odom, grasp_pose.pre);
    grasp_pose.pre.position.z = grasp_pose.pre.position.z + 0.1;

    // Find after-grasp pose
    // tf::Transform tf_after_grasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.15));
    // tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
    // tf::poseTFToMsg(tf_pregrasp_odom, pre_grasp_pose);

    // Find after-grasp pose
    grasp_pose.after = grasp_pose.actual;
    // ROS_INFO_STREAM("after grasp pose : " << grasp_pose.after);
    // if (grasp_pose.after.position.x < 0 ) {
    //   grasp_pose.after.position.x = grasp_pose.after.position.x + 0.10;
    // } else grasp_pose.after.position.x = grasp_pose.after.position.x - 0.10;
    // if (grasp_pose.after.position.y < 0 ) {
    //   grasp_pose.after.position.y = grasp_pose.after.position.y + 0.10;
    // } else grasp_pose.after.position.y = grasp_pose.after.position.y - 0.10;

    grasp_pose.after.position.z = grasp_pose.after.position.z + 0.15;
    tf::StampedTransform tf_hand_odom;
    try {
        listener_.waitForTransform("odom", "hand_palm_link", ros::Time(0), ros::Duration(3.0) );
        listener_.lookupTransform("odom", "hand_palm_link", ros::Time(0), tf_hand_odom);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
      }

    /*Eigen::VectorXf grasp_position(2);
    Eigen::VectorXf hand_position(2);*/

    // 2D Euclidian distance from hand to grasp pose
  /*  grasp_position << grasp_msg.position.x, grasp_msg.position.y;
    hand_position << tf_hand_odom.getOrigin().getX(), tf_hand_odom.getOrigin().getY();*/
    //grasp_pose.distance = (grasp_position - hand_position).squaredNorm();

    return grasp_pose;

} 

bool GraspDetectionServer::detectAllGrasps(robobreizh::detect_grasps::Request& req, robobreizh::detect_grasps::Response& res){
  ROS_INFO("Received service request ...");
  cloud_camera_ = NULL;
  Eigen::Matrix3Xd view_points(3,1);
  view_points.col(0) = view_point_;
  
  PointCloudRGBA::Ptr cloud_init(new PointCloudRGBA);
  pcl::fromROSMsg(req.request.global_cloud, *cloud_init);

  cloud_camera_ = new gpd::util::Cloud(cloud_init, 0, view_points);
  cloud_camera_header_ = req.request.global_cloud.header;
  ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");

  frame_ = req.request.global_cloud.header.frame_id;
  
  // preprocess the point cloud
  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  // detect grasps in the point cloud
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_);

 
  // Visualize the detected grasp in rviz.
  frame_ = req.request.global_cloud.header.frame_id;
  rviz_plotter_->drawGrasps(grasps, frame_);

  // For each object, get only the best grasp inside the bounding box


   // Publish the detected grasps.
  robobreizh::GraspConfigList selected_grasps_msg = GraspDetectionServer::createGraspListMsg(grasps, cloud_camera_header_);
  res.grasp_configs = selected_grasps_msg;
  ROS_INFO_STREAM("Detected " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");
  ROS_INFO_STREAM("Best Grasp: " << selected_grasps_msg.grasps[0] << " .");

}

bool GraspDetectionServer::detectGrasps(robobreizh::detect_grasps::Request& req, robobreizh::detect_grasps::Response& res){
   ROS_INFO("Received service request ...");

  // 1. Initialize cloud camera.
  cloud_camera_ = NULL;

  Eigen::Matrix3Xd view_points(3,1);
  view_points.col(0) = view_point_;

  PointCloudRGBA::Ptr cloud_init(new PointCloudRGBA);
  pcl::fromROSMsg(req.request.global_cloud, *cloud_init);

  cloud_camera_ = new gpd::util::Cloud(cloud_init, 0, view_points);
  cloud_camera_header_ = req.request.global_cloud.header;
  ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");
  
  PointCloudRGB::Ptr cloud (new PointCloudRGB);
  pcl::fromROSMsg(req.request.global_cloud, *cloud);

  PointCloudRGB::Ptr cloud_filtered (new PointCloudRGB);

  //pcl::copyPointCloud(*cloud, *cloud_init);
  std::cout << "PointCloud after coppying has: " << cloud->size()  << " data points." << std::endl;

  int minX = req.request.bounding_box.x_min.data;
  int minY = req.request.bounding_box.y_min.data;
  int maxX = req.request.bounding_box.x_max.data;
  int maxY = req.request.bounding_box.y_max.data;
  ROS_INFO_STREAM("Received bounding box with min_x: " << minX << " min_y: " << minY << " max_x: " << maxX << " max_y: " << maxY);

  // Extract the indices of the pointcloud subset

  std::vector<int> indices;
  for(int i = minX; i < maxX; i++){
    for(int j = minY; j < maxY; j++){
      indices.push_back(i + (j*640));
    }
  }

  boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(indices);
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (index_ptr);
  extract.setNegative (false); // If set to true, you can extract point clouds outside the specified index
  extract.filter (*cloud_filtered);

  std::cout << "PointCloud after cropping has: " << cloud_filtered->size()  << " data points." << std::endl; 

  // Removing NaN values to avoid Exception with filter
  PointCloudRGB::Ptr cloud_out (new PointCloudRGB);
 
  std::vector<int> ind;
  pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_out, ind);
  std::cout << "size: " << cloud_out->points.size () << std::endl;

  if(cloud_out->points.size() == 0){
    ROS_INFO_STREAM("Points cloud empty, exiting.");
    return false;
  }

  //Two steps: first segmentation on the z axis (groud or table) and then clustering
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

  PointCloudRGB::Ptr cloud_f (new PointCloudRGB);

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.01);
  
  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_out);
  seg.segment (*inliers, *coefficients);

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extraction;
  extraction.setInputCloud (cloud_out);
  extraction.setIndices (inliers);

  // Remove the planar inliers, extract the rest
  extraction.setNegative (true);
  extraction.filter (*cloud_f);
  *cloud_filtered = *cloud_f;
  std::cout << "size: " << cloud_filtered->points.size () << std::endl;

  if(cloud_filtered->points.size() == 0){
    ROS_INFO_STREAM("No objects detected, exiting.");
    return false;
  }

  PointCloudRGBA::Ptr temp_cloud (new PointCloudRGBA);
  GraspDetectionServer::PointCloudXYZRGBtoXYZRGBA(*cloud_filtered, *temp_cloud);

  frame_ = req.request.global_cloud.header.frame_id;

  cloud_camera_ = new gpd::util::Cloud(temp_cloud, 0, view_points);
  cloud_camera_header_ = req.request.global_cloud.header;

  // preprocess the point cloud
  try{
    grasp_detector_->preprocessPointCloud(*cloud_camera_);
  }
  catch(...) {
    ROS_INFO_STREAM("Pre-Processing of the point cloud failed.");
    return false;
  }

  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;
  // detect grasps in the point cloud
  try{
    grasps = grasp_detector_->detectGrasps(*cloud_camera_);
  }
  catch(...) {
    ROS_INFO_STREAM("No grasp position detected, exiting.");
    return false;
  }
 
  // Visualize the detected grasp in rviz.
  frame_ = req.request.global_cloud.header.frame_id;
  rviz_plotter_->drawGrasps(grasps, frame_);

  // Publish the detected grasps.
  robobreizh::GraspConfigList selected_grasps_msg = GraspDetectionServer::createGraspListMsg(grasps, cloud_camera_header_);
  res.grasp_configs = selected_grasps_msg;
  ROS_INFO_STREAM("Detected " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");
  ROS_INFO_STREAM("Best Grasp: " << selected_grasps_msg.grasps[0] << " .");

  return true;
}


void GraspDetectionServer::PointCloudXYZRGBtoXYZRGBA(pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGBA>& out)
{
  out.width   = in.width;
  out.height  = in.height;
  out.points.resize(in.points.size());
  for (size_t i = 0; i < in.points.size (); i++)
  {
    out.points[i].x = in.points[i].x;
    out.points[i].y = in.points[i].y;
    out.points[i].z = in.points[i].z;
    out.points[i].r = in.points[i].r;
    out.points[i].g = in.points[i].g;
    out.points[i].b = in.points[i].b;
    out.points[i].a = 255;
  }
}


robobreizh::GraspConfigList GraspDetectionServer::createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::Header& header)
{
  robobreizh::GraspConfigList msg;

  for (int i = 0; i < hands.size(); i++) {
    msg.grasps.push_back(convertToGraspMsg(*hands[i]));
  }

  msg.header = header;

  return msg;
}

robobreizh::GraspConfig GraspDetectionServer::convertToGraspMsg(const gpd::candidate::Hand& hand)
{
  robobreizh::GraspConfig msg;

  tf::pointEigenToMsg(hand.getPosition(), msg.position);
  tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
  tf::vectorEigenToMsg(hand.getBinormal(), msg.binormal);
  tf::vectorEigenToMsg(hand.getAxis(), msg.axis);

  msg.width.data = hand.getGraspWidth();
  msg.score.data = hand.getScore();
  tf::pointEigenToMsg(hand.getSample(), msg.sample);

  GraspPose grasp_pose;
  grasp_pose = createPickingEEFPose(msg);

  msg.pre_pose = grasp_pose.pre;
  msg.actual_pose = grasp_pose.actual;
  msg.after_pose = grasp_pose.after;

  return msg;
}


bool GraspDetectionServer::detectGraspsWithoutGPD(robobreizh::detect_grasps::Request& req, robobreizh::detect_grasps::Response& res)
{
  ROS_INFO("Received service request ...");

  // 1. Initialize cloud camera.
  cloud_camera_ = NULL;

  Eigen::Matrix3Xd view_points(3,1);
  view_points.col(0) = view_point_;

  PointCloudRGBA::Ptr cloud_init(new PointCloudRGBA);
  pcl::fromROSMsg(req.request.global_cloud, *cloud_init);

  cloud_camera_ = new gpd::util::Cloud(cloud_init, 0, view_points);
  cloud_camera_header_ = req.request.global_cloud.header;
  ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");
  
  PointCloudRGB::Ptr cloud (new PointCloudRGB);
  pcl::fromROSMsg(req.request.global_cloud, *cloud);

  PointCloudRGB::Ptr cloud_filtered (new PointCloudRGB);

  //pcl::copyPointCloud(*cloud, *cloud_init);
  std::cout << "PointCloud after coppying has: " << cloud->size()  << " data points." << std::endl;

  int minX = req.request.bounding_box.x_min.data;
  int minY = req.request.bounding_box.y_min.data;
  int maxX = req.request.bounding_box.x_max.data;
  int maxY = req.request.bounding_box.y_max.data;
  ROS_INFO_STREAM("Received bounding box with min_x: " << minX << " min_y: " << minY << " max_x: " << maxX << " max_y: " << maxY);

  // Extract the indices of the pointcloud subset

  std::vector<int> indices;
  for(int i = minX; i < maxX; i++){
    for(int j = minY; j < maxY; j++){
      indices.push_back(i + (j*640));
    }
  }

  boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(indices);
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (index_ptr);
  extract.setNegative (false); // If set to true, you can extract point clouds outside the specified index
  extract.filter (*cloud_filtered);

  /* Old method: Unusued
  pcl::CropBox<pcl::PointXYZRGB> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1));
  boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1));
  boxFilter.setInputCloud(cloud);
  boxFilter.filter(*cloud_cropped);*/

  std::cout << "PointCloud after cropping has: " << cloud_filtered->size()  << " data points." << std::endl; 

  //Two steps: first segmentation on the z axis (groud or table) and then clustering

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  /*pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  PointCloudRGB::Ptr cloud_filtered (new PointCloudRGB);
  vg.setInputCloud (cloud_cropped);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl;
*/
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

  PointCloudRGB::Ptr cloud_f (new PointCloudRGB);
  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (300);
  seg.setAxis(axis);
  seg.setEpsAngle(10.0f * (M_PI/180.0f));
  seg.setDistanceThreshold (0.01);
  
  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extraction;
  extraction.setInputCloud (cloud_filtered);
  extraction.setIndices (inliers);

  // Remove the planar inliers, extract the rest
  extraction.setNegative (true);
  extraction.filter (*cloud_f);
  *cloud_filtered = *cloud_f;
  

  // Optional: try to segment the wall (y axis), can destroy the object if it has more planar surface on the y axis

  /*Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (300);
  seg.setAxis(axis);
  seg.setEpsAngle(10.0f * (M_PI/180.0f));
  seg.setDistanceThreshold (0.01);
  
  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_f);
  *cloud_filtered = *cloud_f;*/

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  int max_cluster_size = 0;
  PointCloudRGB::Ptr max_cluster (new PointCloudRGB);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PointCloudRGB::Ptr cloud_cluster (new PointCloudRGB);
    for (const auto& idx : it->indices)
      cloud_cluster->push_back ((*cloud_filtered)[idx]); //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;

    if(cloud_cluster->size () > max_cluster_size){
      max_cluster_size = cloud_cluster->size ();
      max_cluster = cloud_cluster;
    }

    j++;
  }
  PointCloudRGBA::Ptr temp_cloud (new PointCloudRGBA);
  GraspDetectionServer::PointCloudXYZRGBtoXYZRGBA(*max_cluster, *temp_cloud);

  cloud_camera_ = new gpd::util::Cloud(temp_cloud, 0, view_points);
  cloud_camera_header_ = req.request.global_cloud.header;

  frame_ = req.request.global_cloud.header.frame_id;

  // preprocess the point cloud
  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  // detect grasps in the point cloud
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_);

  // Publish the detected grasps.
  robobreizh::GraspConfigList selected_grasps_msg = GraspDetectionServer::createGraspListMsg(grasps, cloud_camera_header_);
  res.grasp_configs = selected_grasps_msg;
  ROS_INFO_STREAM("Detected " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");
  ROS_INFO_STREAM("Best Grasp: " << selected_grasps_msg.grasps[0] << " .");

  // Visualize the detected grasp in rviz.
  frame_ = req.request.global_cloud.header.frame_id;
  rviz_plotter_->drawGrasps(grasps, frame_);

  return true;
  }

int main(int argc, char** argv)
{
  // seed the random number generator
  std::srand(std::time(0));

  // initialize ROS
  ros::init(argc, argv, "detect_grasps_server");
  ros::NodeHandle node("~");
  tf::TransformListener listener_; 

  GraspDetectionServer grasp_detection_server(node);

  ros::ServiceServer service = node.advertiseService("detect_grasps", &GraspDetectionServer::detectGrasps,
                                                     &grasp_detection_server);
  ROS_INFO("Grasp detection service is waiting for a point cloud ...");

  ros::spin();

  return 0;
}
