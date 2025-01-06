import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import math
import std_msgs.msg
from matplotlib import cm



def convert_data(_data, _min, _max, _range):
    if _data < _min:
        return [0, 0, 0]
    r = (_data - _min) / _range
    step = int(_range / 5)
    idx = int(r * 5)
    h = (idx + 1) * step + _min
    m = idx * step + _min
    local_r = (_data - m) / (h - m)
    if _data < _min:
        return [0, 0, 0]
    if _data > _max:
        return [255, 255, 255]
    if idx == 0:
        return [0, int(local_r * 255), 255]
    if idx == 1:
        return [0, 255, int((1 - local_r) * 255)]
    if idx == 2:
        return [int(local_r * 255), 255, 0]
    if idx == 3:
        return [255, int((1 - local_r) * 255), 0]
    if idx == 4:
        return [255, 0, int(local_r * 255)]




rospy.init_node('trajectory_marker')

marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

marker = Marker()
marker.header.frame_id = "map"
marker.header.stamp = rospy.Time.now()

marker.ns = "trajectory"
marker.id = 1

marker.type = Marker.LINE_STRIP
marker.action = Marker.ADD

# Scale of the trajectory line
marker.scale.x = 0.2 # Adjust the thickness of the line

marker.pose.orientation.w = 1.0 # No orientation (flat)

# Set some example trajectory points and velocity magnitudes
trajectory_points = [] # Each tuple: (x, y, z, velocity)

file_pose = open("pose.txt", "r")
pose_data = file_pose.readlines()

file_vel = open("vel.txt", "r")
vel_data = file_vel.readlines()

max_vel = 0
min_vel = 100
for i in range(30,len(pose_data)-100):
    pose_list = pose_data[i].split(",")
    vel_list = vel_data[i].split(",")
    velocity = math.sqrt(float(vel_list[4])**2 + float(vel_list[5])**2 + float(vel_list[6])**2)
    if velocity > max_vel:
        max_vel = velocity
    elif velocity < min_vel:
        min_vel = velocity
    trajectory_points.append((float(pose_list[4]), float(pose_list[5]), float(pose_list[6]), 
                              velocity,pose_list[7],pose_list[8],pose_list[9],pose_list[10]))

viridis = cm.get_cmap('jet', len(trajectory_points))
print(max_vel, min_vel)  #0.18012   6.61589
# Define colors based on velocity (velocity ranges from 0 to 3 for this example)
for (x, y, z, velocity,ox,oy,oz,ow) in trajectory_points:
# Add points to trajectory
    point = Point()
    # point.position.x = x
    # point.position.y = y
    # point.position.z = z
    # point.orientation.x = ox
    # point.orientation.y = oy
    # point.orientation.z = oz
    # point.orientation.w = ow
    point.x = x
    point.y = y
    point.z = z

    
    
    marker.points.append(point)
    
    
    # Color based on velocity
    # r, g, b = convert_data(velocity, min_vel, max_vel, len(trajectory_points))
    # print(r, g, b)
    color = std_msgs.msg.ColorRGBA()
    r,g,b,a = viridis((velocity - min_vel)/(max_vel - min_vel))
    color.r = r
    color.g = g
    color.b = b
    color.a = a
    marker.colors.append(color)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    marker_pub.publish(marker)
    rate.sleep()
