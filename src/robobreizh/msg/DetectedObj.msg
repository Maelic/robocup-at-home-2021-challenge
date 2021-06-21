# This message stores the detected objects

# The list of object names.
std_msgs/String[] object_names

# All the objects bounding boxes coordinates.
robobreizh/BoundingBoxCoord[] objects_bb

# The source cloud used.
sensor_msgs/PointCloud2 cloud

# All the objects poses in the map frame.
sensor_msgs/PointCloud2 object_poses

geometry_msgs/Pose[] object_posesXYZ