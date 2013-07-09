Header header
# Number of points in both direction
uint16 num_x
uint16 num_y
# List of point, such that point[x,y] = points[y*num_x + x]
geometry_msgs/Point[] points
geometry_msgs/Point[] model
