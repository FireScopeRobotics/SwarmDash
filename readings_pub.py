import rospy
import requests
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped 
from tf import TransformListener
import random
import time
poses = {}
pressures = {}
temps = {}
sessionID = None
        
def post_odom_callback(event):
    for k, v in poses.items():
        try:
            url = f"http://0.0.0.0:8000/db/add/{sessionID}/{k}?pressure={pressures[k]}&temperature={temps[k]}&x={v.x}&y={v.y}"
            response = requests.put(url=url)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            print(e)
            raise

def get_odom_callback(pose, args):
    robot_name = args[0]
    tf_listener = args[1]
    tf_listener.waitForTransform(f"carter{robot_name}/odom", "map", rospy.Time(0),rospy.Duration(4.0))
    laser_point=PointStamped()
    laser_point.header.frame_id = f"/carter{robot_name}/odom"
    laser_point.header.stamp =rospy.Time(0)
    laser_point.point.x=pose.pose.pose.position.x
    laser_point.point.y=pose.pose.pose.position.y
    p=tf_listener.transformPoint("map",laser_point)
    poses[robot_name] = p.point

def get_sensor_callback(msg, robot_name):
    pressures[robot_name] = round(msg.data[0],2)
    temps[robot_name] = round(msg.data[1],2)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
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
    rospy.Subscriber("/carter1/odom", Odometry, get_odom_callback, ("1", tf_listener))
    rospy.Subscriber("/carter1/sensor_topic", Float64MultiArray, get_sensor_callback, "1")
    rospy.Subscriber("/carter2/odom", Odometry, get_odom_callback, ("2", tf_listener))
    rospy.Subscriber("/carter2/sensor_topic", Float64MultiArray, get_sensor_callback, "2")
    rospy.Subscriber("/carter3/odom", Odometry, get_odom_callback, ("3", tf_listener))
    rospy.Subscriber("/carter3/sensor_topic", Float64MultiArray, get_sensor_callback, "3")
    rospy.Timer(rospy.Duration(3), post_odom_callback)

    rospy.spin()