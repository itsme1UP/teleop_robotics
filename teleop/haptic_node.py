import rospy
from std_msgs.msg import Float32

class HapticNode:
    def __init__(self):
        rospy.init_node('haptic_node', anonymous=True)

        # Placeholder subscriber, e.g., from velocity or impedance output
        rospy.Subscriber('/ur10e/cmd_vel', Float32, self.callback)

        # Publisher for haptic feedback (placeholder)
        self.pub = rospy.Publisher('/haptic/feedback', Float32, queue_size=10)

        rospy.loginfo('Haptic node started')
        self.run()

    def callback(self, msg):
        # For now just pass velocity magnitude as haptic intensity
        intensity = abs(msg.data)
        self.pub.publish(Float32(data=intensity))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        HapticNode()
    except rospy.ROSInterruptException:
        pass
