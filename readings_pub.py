import rospy
import requests
from nav_msgs.msg import Odometry
import random

odom_poses = {}
sessionID = random.getrandbits(16)
        
def post_odom_callback(event):
    for k, v in odom_poses.items():
        try:
            url = f"http://0.0.0.0:8000/db/add/{sessionID}/{k}?pressure=34&temperature=45&x={v.pose.pose.position.x}&y={v.pose.pose.position.y}"
            response = requests.put(url=url)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            print(e)
            raise
def get_odom_callback(pose, robot_name):
    odom_poses[robot_name] = pose

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/carter1/odom", Odometry, get_odom_callback, "1")
    rospy.Subscriber("/carter2/odom", Odometry, get_odom_callback, "2")
    rospy.Subscriber("/carter3/odom", Odometry, get_odom_callback, "3")
    rospy.Timer(rospy.Duration(3), post_odom_callback)

    rospy.spin()
