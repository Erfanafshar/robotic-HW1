#! /usr/bin/env python3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
from gazebo_msgs.srv import GetModelState
import rospy


class PoseMonitor:
    def __init__(self):
        rospy.init_node('pose_monitor', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)
        self.vel_change_sub = rospy.Subscriber('/change', Twist, self.callback_velocity_change)
        self.report_pose = False
        rospy.wait_for_service("gazebo/get_model_state")
        print("gazebo detected")
        self.get_ground_truth = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        self.faults = []

    def callback_velocity_change(self, msg):
        rospy.loginfo("Velocity has changed, now: %5.3f, %5.3f", msg.linear.x, msg.angular.z)
        rospy.sleep(0.75)
        self.report_pose = True

    def quaternion_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        print("Roll: %5.2f Pitch: %5.2f Yaw: %5.2f" % (roll, pitch, yaw))

    def get_fault(self, cx, cy):
        dis1 = abs(cy - 1.5)
        dis2 = abs(cy + 1.5)
        dis3 = abs(cx - 1.5)
        dis4 = abs(cx + 1.5)
        m_dis = min(dis1, dis2, dis3, dis4)
        return m_dis

    def get_avg(self):
        return sum(self.faults) / len(self.faults)

    def callback_odometry(self, msg):
        if self.report_pose:
            print("Position: (%5.2f, %5.2f, %5.2f)" %
                  (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
            self.quaternion_to_euler(msg)
            print("Linear twist: (%5.2f, %5.2f, %5.2f)" %
                  (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z))
            print("Angular twist: (%5.2f, %5.2f, %5.2f)" %
                  (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z))
            # print("Ground Truth: ", self.get_ground_truth("mobile-base", "world"))
            cx = msg.pose.pose.position.x
            cy = msg.pose.pose.position.y
            f_distance = self.get_fault(cx, cy)
            self.faults.append(f_distance)
            c_avg = self.get_avg()
            print("current fault : " + str(f_distance))
            print("average fault until now  : " + str(c_avg))
            print()
            self.report_pose = False


if __name__ == '__main__':
    PoseMonitor()
    rospy.spin()
