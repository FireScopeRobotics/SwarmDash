import rospy
import requests
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped 
from tf import TransformListener
import cv2
from skimage.measure import block_reduce
import numpy as np
import random
import time


sessionID = None
fire_found = False
camera = cv2.VideoCapture('udpsrc blocksize=2304 port=5001 ! rawvideoparse use-sink-caps=false width=32 height=24 format=rgb framerate=16/1 ! videoconvert ! videoscale ! video/x-raw,format=GRAY8,width=32,height=24 ! queue ! appsink', cv2.CAP_GSTREAMER)

    
def get_odom_callback(pose, args):
    global fire_found
    robot_name = args[0]
    tf_listener = args[1]
    tf_listener.waitForTransform(f"carter{robot_name}/odom", "map", rospy.Time(0),rospy.Duration(4.0))
    laser_point=PointStamped()
    laser_point.header.frame_id = f"/carter{robot_name}/odom"
    laser_point.header.stamp =rospy.Time(0)
    laser_point.point.x=pose.pose.pose.position.x
    laser_point.point.y=pose.pose.pose.position.y
    p=tf_listener.transformPoint("map",laser_point)
    current_pose = p.point
    ret, frame = camera.read()
    if ret == False:
        return
    test = block_reduce(frame, (2,2), np.min)
    if np.max(test) > 210:
        if not fire_found:
            try:
                url = f"http://0.0.0.0:8000/db/add/fires/{sessionID}/{robot_name}?x={current_pose.x}&y={current_pose.y}"
                response = requests.put(url=url)
                response.raise_for_status()
            except requests.exceptions.RequestException as e:
                print(e)
                raise

            fire_found = True

if __name__ == '__main__':
    rospy.init_node('fire_listener', anonymous=True)
    while True:
        api_url = f"http://localhost:8000/db/session/get" 
        resp = requests.get(api_url)
        session = resp.json()['Session']
        if session is not None:
            break
        else:
            time.sleep(3)
    sessionID = session
    # Used for finding TF between base_link frame and map (i.e. robot position)
    # See https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html#write-the-listener-node
    tf_listener = TransformListener()
    rospy.Subscriber("/carter2/odom", Odometry, get_odom_callback, ("2", tf_listener))
    rospy.spin()
