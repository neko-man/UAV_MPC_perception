## type 1 r=0.1 l=5
##      2 r=0.3 l=3
##      3 r=0.5 l=3


import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField


rospy.init_node('obstacle_marker')

cloud = PointCloud2()
cloud.header.frame_id = "map"
cloud.header.stamp = rospy.Time.now()
cloud.height = 1
cloud.is_dense = True
fields = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1)
]
cloud.fields = fields

cloud_pub = rospy.Publisher('visualization_cloud', PointCloud2, queue_size=50)

marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

marker = Marker()
marker.header.frame_id = "map" # Set your desired reference frame
marker.header.stamp = rospy.Time.now()

marker.ns = "obstacles"

marker.type = Marker.CYLINDER
marker.action = Marker.ADD

marker_1 = Marker()
marker_1.header.frame_id = "map" # Set your desired reference frame
marker_1.header.stamp = rospy.Time.now()

marker_1.ns = "obstacles"

marker_1.type = Marker.CUBE
marker_1.action = Marker.ADD

marker_2 = Marker()
marker_2.header.frame_id = "map" # Set your desired reference frame
marker_2.header.stamp = rospy.Time.now()

marker_2.ns = "arrow"

marker_2.type = Marker.ARROW
marker_2.action = Marker.ADD

marker.pose.orientation.w = 1.0
marker_1.pose.orientation.w = 1.0
marker_2.pose.orientation.w = 1.0


import numpy as np
from numpy import random
import math

def generate_cylinder_with_small_cylinders(center, radius, height, rgb, resolution = 0.05):

    x_center, y_center, z_center = center
    r_,g_,b_ = rgb
    # print(rgb)
    point_cloud_list = []
    
    x = math.floor(x_center / resolution) * resolution + resolution * 2.0
    y = math.floor(y_center / resolution) * resolution + resolution * 2.0
    
    widNum = int(math.ceil(radius * 2 / resolution)) + 10

    for r in range(-widNum/2,widNum/2):
        for s in range(-widNum/2,widNum/2):
            h = 2 * random.rand() + 3
            heiNum = int(math.ceil(h / resolution))
            for t in range(0, heiNum):
                temp_x = x + (r + 0.5) * resolution + 1e-2
                temp_y = y + (s + 0.5) * resolution + 1e-2
                temp_z = (t + 0.5) * resolution + 1e-2
                
                if (math.sqrt(abs(temp_x - x_center)**2 + abs(temp_y - y_center)**2) > radius): continue
                
                color = (int(b_) << 16) | (int(g_) << 8) | int(r_)

                
                point_cloud_list.append([temp_x,temp_y,temp_z,color])

    return point_cloud_list


data = []
file = open("coordinates.txt", "r")
file_data = file.readlines()
for line in file_data:
    tmp_list = line.split(" ")
    tmp_list[-1] = tmp_list[-1].replace("\n", "")
    data.append(tmp_list)
file.close()
    
length = len(data)
# # Position of the obstacle
# marker.pose.position.x = 1.0
# marker.pose.position.y = 1.0
# marker.pose.position.z = 2.5

# # Size of the obstacle
# marker.scale.x = 1.0
# marker.scale.y = 1.0
# marker.scale.z = 5.0

# # Color of the obstacle (e.g., red)
# marker.color.r = 1.0
# marker.color.g = 0.0
# marker.color.b = 0.0
# marker.color.a = 0.5

rate = rospy.Rate(10)
cloud_list_all = []
for i in range(length):
    
    z = 0
    radius = 0
    height = 0
    x,y,c_type = data[i]
    if c_type == "1":
        # marker.pose.position.z = 2.5
        # marker.scale.x = 0.3
        # marker.scale.y = 0.3
        # marker.scale.z = 5
        z = 2.5
        radius = 0.3
        height = 5
    elif c_type == "2":
        # marker.pose.position.z = 1.5
        # marker.scale.x = 0.5
        # marker.scale.y = 0.5
        # marker.scale.z = 3
        z = 1.5
        radius = 0.5
        height = 3
    else:
        # marker.pose.position.z = 1.5
        # marker.scale.x = 0.7
        # marker.scale.y = 0.7
        # marker.scale.z = 3
        z = 1.5
        radius = 0.7
        height = 3
    # marker.pose.position.x = float(x)
    # marker.pose.position.y = float(y)
    if i in [0,3,7,9,15,16,20,24]:
        cloud_list = generate_cylinder_with_small_cylinders((float(x), float(y), z), radius, height, (0,255,0))
    else:
        cloud_list = generate_cylinder_with_small_cylinders((float(x), float(y), z), radius, height, (255,10,10))
    # print(cloud_list[0])

    cloud_list_all = cloud_list_all + cloud_list
    i = i + 1
# print(cloud_list_all[1])
cloud.data = np.asarray(cloud_list_all,dtype = np.float32).tobytes()
cloud.width = len(cloud_list_all) 
cloud.point_step = 16
i = 0
while not rospy.is_shutdown():
    
    if i == length:
        marker_1.id = i + 1
        marker_1.pose.position.x = 18
        marker_1.pose.position.y = 5.25
        marker_1.pose.position.z = 1.5
        marker_1.scale.x = 0.2
        marker_1.scale.y = 4.5
        marker_1.scale.z = 3
        marker_1.color.r = 0.5
        marker_1.color.g = 0.8
        marker_1.color.b = 1.0
        marker_1.color.a = 1
        marker_pub.publish(marker_1)
        
        marker_1.id = i + 2
        marker_1.pose.position.y = -0.75
        marker_pub.publish(marker_1)

        marker_2.scale.x = 0.05
        marker_2.scale.y = 0.5
        marker_2.scale.z = 1
        marker_2.color.g = 1.0
        marker_2.color.a = 0.5
        p1 = Point()
        p2 = Point()
        p1.x = 0
        p1.y = 0
        p1.z = 0
        p2.x = 50
        p2.y = 0
        p2.z = 0
        marker_2.points.append(p1)
        marker_2.points.append(p2)
        marker_pub.publish(marker_2)
        marker_2.points = []

        i = 0
        
    marker.id = i


    
    

    # print(cloud_list)
    # cloud.width = len(cloud_list)
    # for i in range(len(cloud_list)):
    #     cloud.points[i].x = cloud_list[i][0]
    #     cloud.points[i].y = cloud_list[i][1]
    #     cloud.points[i].z = cloud_list[i][2]


    # print(cloud.data.shape)
    # marker.color.r = 0.5
    # marker.color.g = 0.8
    # marker.color.b = 1.0
    # marker.color.a = 1
    
    # marker_pub.publish(marker)
    i = i + 1
    cloud_pub.publish(cloud)
    rate.sleep()
