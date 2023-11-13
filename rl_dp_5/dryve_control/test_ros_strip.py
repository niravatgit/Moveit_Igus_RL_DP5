# ROS node
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState 
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
import math

# Publisher for joint status
joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10) 

# Service for setting operation mode
from rl_comm.srv import SetInt, SetIntResponse
def set_mode_handler(req):
    mode = req.data
    # Call set_mode() from dryve_D1.py
    resp = SetIntResponse()
    resp.success = True
    return resp
set_mode_service = rospy.Service('/set_mode', SetInt, set_mode_handler)

# Service for homing
from std_srvs.srv import Trigger, TriggerResponse 
def home_handler(req):
    # Call homing() from dryve_D1.py
    resp = TriggerResponse()
    resp.success = True
    resp.message = "Homing completed"
    return resp
home_service = rospy.Service('/home', Trigger, home_handler) 

# Subscriber for position commands
def set_position_handler(msg):
    position = msg.data
    # Call profile_pos_mode() from dryve_D1.py
set_position_sub = rospy.Subscriber('/cmd/set_joint_position', Float64, set_position_handler)

# Main loop
rospy.init_node('d1_node')
rate = rospy.Rate(10)
while not rospy.is_shutdown():

    # Publish joint status
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    # Populate position, velocity, effort
    joint_state_pub.publish(joint_state)
    
    rate.sleep()

# New testing 12:00 PM 13-11-2023
# ROS node 
import rospy
from std_msgs.msg import Float64

# Publishers for velocity commands
pubs = []
for i in range(5):
  topic = f'/axis_{i+1}/cmd_vel'
  pub = rospy.Publisher(topic, Float64, queue_size=10)
  pubs.append(pub)

# Callback for anti-clockwise button  
def anticlockwise_callback(axis):
  velocity = -500 # negative for anticlockwise
  pubs[axis].publish(velocity)
  
# Callback for stop jogging  
def stop_callback(axis):
  velocity = 0
  pubs[axis].publish(velocity)
  
# Subscribers for button presses  
for i in range(5):
  rospy.Subscriber(f'/axis_{i+1}/anticlockwise', std_msgs.msg.Empty, 
                  lambda msg, axis=i: anticlockwise_callback(axis))
                  
  rospy.Subscriber(f'/axis_{i+1}/stop', std_msgs.msg.Empty, 
                  lambda msg, axis=i: stop_callback(axis))

rospy.init_node('gui_node')
rospy.spin()
