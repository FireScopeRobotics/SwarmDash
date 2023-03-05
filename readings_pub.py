import rospy
import requests
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray 
import random

odom_poses = {}
pressures = {}
temps = {}

sessionID = 160#random.getrandbits(16)
        
def post_odom_callback(event):
    for k, v in odom_poses.items():
        try:
            url = f"http://0.0.0.0:8000/db/add/{sessionID}/{k}?pressure={pressures['2']}&temperature={temps['2']}&x={v.pose.pose.position.x}&y={v.pose.pose.position.y}"
            response = requests.put(url=url)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            print(e)
            raise
def get_odom_callback(pose, robot_name):
    odom_poses[robot_name] = pose

def get_sensor_callback(msg, robot_name):
    pressures[robot_name] = round(msg.data[0],2)
    temps[robot_name] = round(msg.data[1],2)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/carter1/odom", Odometry, get_odom_callback, "1")
    rospy.Subscriber("/carter1/sensor_topic", Float64MultiArray, get_sensor_callback, "1")
    rospy.Subscriber("/carter2/odom", Odometry, get_odom_callback, "2")
    rospy.Subscriber("/carter2/sensor_topic", Float64MultiArray, get_sensor_callback, "2")
    rospy.Subscriber("/carter3/odom", Odometry, get_odom_callback, "3")
    rospy.Subscriber("/carter3/sensor_topic", Float64MultiArray, get_sensor_callback, "3")
    rospy.Timer(rospy.Duration(4), post_odom_callback)

    rospy.spin()
