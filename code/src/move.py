#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math


class Move:
    def __init__(self):
        rospy.init_node('move')

        self.vel = Twist()
        self.itr = 0

        rospy.wait_for_service("/gazebo/set_model_state")
        print("gazebo detected")
        state_msg = ModelState()
        state_msg.model_name = "turtlebot3_burger"
        state_msg.pose.position.x = -1.5
        state_msg.pose.position.y = 1.5
        self.destination = [1.5, 1.5]
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        set_state(state_msg)
        rospy.on_shutdown(self.shutdown)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)
        self.change_pub = rospy.Publisher('/change', Twist, queue_size=5)

    def set_next_destination(self):
        if self.destination[0] == -1.5 and self.destination[1] == 1.5:
            self.destination[0] = 1.5
            self.destination[1] = 1.5
        elif self.destination[0] == 1.5 and self.destination[1] == 1.5:
            self.destination[0] = 1.5
            self.destination[1] = -1.5
        elif self.destination[0] == 1.5 and self.destination[1] == -1.5:
            self.destination[0] = -1.5
            self.destination[1] = -1.5
        elif self.destination[0] == -1.5 and self.destination[1] == -1.5:
            self.destination[0] = -1.5
            self.destination[1] = 1.5

    def reached(self, x, y, xd, yd):
        dist_to_dest = max(abs(x - xd), abs(y - yd))
        if dist_to_dest < 1.2:
            return True
        else:
            return False

    def check_position(self, x, y, xd, yd):
        if self.reached(x, y, xd, yd):
            self.itr += 1
            if self.itr == 40:
                print("10 laps finished")
                self.shutdown()
            else:
                self.set_next_destination()

    def callback_odometry(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        t = self.quaternion_to_euler(msg)
        xd, yd = self.destination

        self.check_position(x, y, xd, yd)
        xd, yd = self.destination

        atan2 = math.atan2(yd - y, xd - x)
        omega = min((atan2 - t, atan2 - 2 * math.pi - t, atan2 + 2 * math.pi - t), key=abs)
        self.vel.linear.x = 0.8
        self.vel.angular.z = omega
        self.vel_pub.publish(self.vel)
        self.change_pub.publish(self.vel)

    def quaternion_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def shutdown(self):
        print("shutdown occurred")
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        move = Move()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("error occurred")
