# header for timestamp and extra info
Header header

# point cloud point positions
geometry_msgs/Vector3[] positions

# point cloud point normals
geometry_msgs/Vector3[] normals

# if the message contains no information
bool empty

# the colors of the points in the pointcloud cloud, this uses float 64 rather than 32
ros_utils_py/Color[] colors