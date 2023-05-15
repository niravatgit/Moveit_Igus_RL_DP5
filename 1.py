import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def publish_transform():
    rospy.init_node('transform_publisher_node', anonymous=True)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        # Create a transform message
        transform_msg = TransformStamped()

        # Set the header
        transform_msg.header.stamp = rospy.Time.now()
        transform_msg.header.frame_id = 'world'
        transform_msg.child_frame_id = 'camera1_depth_optical_frame'

        # Set the transformation values
        transform_msg.transform.translation.x = 0.0  # Set appropriate translation values
        transform_msg.transform.translation.y = 0.0
        transform_msg.transform.translation.z = 0.0
        transform_msg.transform.rotation.x = 0.0  # Set appropriate rotation values
        transform_msg.transform.rotation.y = 0.0
        transform_msg.transform.rotation.z = 0.0
        transform_msg.transform.rotation.w = 1.0

        # Publish the transform message
        tf_broadcaster.sendTransform(transform_msg)

        # Sleep for a while before publishing the next transform
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass

