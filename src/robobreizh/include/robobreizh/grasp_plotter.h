

#ifndef GRASP_PLOTTER_H_
#define GRASP_PLOTTER_H_

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// GPG
#include <gpd/candidate/hand.h>
#include <gpd/candidate/hand_geometry.h>


/** GraspPlotter class
 *
 * \brief Draw grasps in rviz.
 *
 * This class provides functions to draw grasps in rviz.
 *
 */
class GraspPlotter
{
public:

  /**
   * \brief Constructor.
   * \param node the ROS node
  */
  GraspPlotter(ros::NodeHandle& node, const gpd::candidate::HandGeometry& params);

  /**
   * \brief Visualize grasps in rviz.
   * \param hands the grasps to be visualized
   * \param frame the frame that the grasps are in
   */
  void drawGrasps(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std::string& frame);

  /**
   * \brief Convert a list of grasps to a ROS message that can be published to rviz.
   * \param hands list of grasps
   * \param frame_id the name of the frame that the grasp is in
   */
  visualization_msgs::MarkerArray convertToVisualGraspMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
    const std::string& frame_id);

  /**
   * \brief Convert a list of grasps to a ROS message that can be published to rviz.
   * \param hands list of grasps
   * \param center the center of the finger
   * \param rot the orientation of the hand
   * \param length the length of the finger
   * \param width the width of the finger
   * \param height the height of the finger
   * \param frame_id the name of the frame that the grasp is in
   */
  visualization_msgs::Marker createFingerMarker(const Eigen::Vector3d& center, const Eigen::Matrix3d& rot,
    const Eigen::Vector3d& lwh, int id, const std::string& frame_id);

  /**
   * \brief Convert a list of grasps to a ROS message that can be published to rviz.
   * \param hands list of grasps
   * \param start
   * \param frame_id the name of the frame that the grasp is in
   */
  visualization_msgs::Marker createHandBaseMarker(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Matrix3d& frame, double length, double height, int id, const std::string& frame_id);


private:

  ros::Publisher rviz_pub_; ///< ROS publisher for grasps in rviz (visualization)

  double outer_diameter_;
  double hand_depth_;
  double finger_width_;
  double hand_height_;
};

#endif /* GRASP_PLOTTER_H_ */
