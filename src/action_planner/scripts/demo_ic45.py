#!/usr/bin/env python3
import math
import rospy # New
from geometry_msgs.msg import *
from std_msgs.msg import *
from std_srvs.srv import *
import tf

def move_distance(goal_dist, goal_angle, pub_goal_dist):
    msg_dist = Float32MultiArray()
    msg_dist.data = [goal_dist, goal_angle]
    print("Publishing distance")
    pub_goal_dist.publish(msg_dist)
    

def detect_waving():
    detect_waving_client = rospy.ServiceProxy('/detect_waving', Trigger) 
    res = detect_waving_client()
    print(res.message)
    x,y,z = res.message[1:-1].split(', ')[0:3]
    x,y,z = float(x), float(y), float(z)
    if not res.success or math.isnan(x) or math.isnan(y) or math.isnan(z):
        return False,None
    p = PointStamped()
    p.header.frame_id = 'head_rgbd_sensor_link'
    p.point.x, p.point.y, p.point.z = x,y,z
    listener = tf.TransformListener()
    listener.waitForTransform('/base_footprint', 'head_rgbd_sensor_link', rospy.Time(), rospy.Duration(4.0))
    p = listener.transformPoint('base_footprint', p)
    return True, p.point

def main():
    print("Maquina de estados de Marcosoft")
    rospy.init_node("sm_demo_ic45")
    pub_goal_dist = rospy.Publisher("/simple_move/goal_dist_angle", Float32MultiArray, queue_size=10)
    success, waiving_point = detect_waving()
    print(waiving_point)
    distance = math.sqrt(waiving_point.x**2 + waiving_point.y**2) - 1.0
    angle = math.atan2(waiving_point.y, waiving_point.x)
    move_distance(distance, angle, pub_goal_dist)
    rospy.spin()


if __name__ == "__main__":
    main()
