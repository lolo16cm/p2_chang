#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class BallFollower:
    def __init__(self):
        self.bridge = CvBridge()

        self.cmd_pub   = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.image_pub = rospy.Publisher('/image_converter/output_video', Image, queue_size=1)

        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

        self.ball_col    = None
        self.ball_row    = None
        self.ball_size   = None
        self.front_dist  = None
        self.depth_image = None

        self.target_dist = 1.0
        self.tolerance   = 0.05

        rospy.loginfo("Ball Follower Started!")

    def depth_callback(self, msg):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self.depth_image = depth_raw.astype(np.float32) / 1000.0
        except:
            try:
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            except Exception as e:
                rospy.logerr("Depth error: %s", str(e))

    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("RGB error: %s", str(e))
            return

        # Find most-red pixel
        b = cv_image[:, :, 0].astype(np.int32)
        g = cv_image[:, :, 1].astype(np.int32)
        r = cv_image[:, :, 2].astype(np.int32)

        dis      = (r - 255)**2 + g**2 + b**2
        min_idx  = np.unravel_index(np.argmin(dis), dis.shape)
        best_row = min_idx[0]
        best_col = min_idx[1]

        # HSV masking for red ball
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_r1 = np.array([0,   100, 100])
        upper_r1 = np.array([10,  255, 255])
        lower_r2 = np.array([160, 100, 100])
        upper_r2 = np.array([180, 255, 255])

        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_r1, upper_r1),
            cv2.inRange(hsv, lower_r2, upper_r2))

        # Find contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours: #find the circular shape
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            area = cv2.contourArea(c)

            # circularity check — 1.0 = perfect circle
            circularity = area / (3.14159 * radius * radius) if radius > 0 else 0

            # radius in pixels: ball is 15cm diameter, at 1m distance ~50px radius
            if circularity > 0.4 and radius > 25:
                self.ball_col  = int(x)
                self.ball_row  = int(y)
                self.ball_size = radius

                if self.depth_image is not None:
                    y1 = max(0, int(y) - 2)
                    y2 = min(self.depth_image.shape[0], int(y) + 2)
                    x1 = max(0, int(x) - 2)
                    x2 = min(self.depth_image.shape[1], int(x) + 2)
                    region = self.depth_image[y1:y2, x1:x2]
                    valid  = region[(region > 0.1) & (region < 5.0) & np.isfinite(region)]
                    if len(valid) > 0:
                        self.front_dist = float(np.mean(valid))
                    else:
                        self.front_dist = None

                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            else:
                self.ball_col   = None
                self.ball_row   = None
                self.ball_size  = None
                self.front_dist = None
        else:
            self.ball_col   = None
            self.ball_row   = None
            self.ball_size  = None
            self.front_dist = None

        # red dot at most-red pixel 
        cv2.circle(cv_image, (best_col, best_row), 10, (0, 0, 255), 2)

        # text notification
        dist_text = "{:.2f}m".format(self.front_dist) if self.front_dist else "N/A"
        size_text = "{:.1f}px".format(self.ball_size) if self.ball_size else "N/A"

        cv2.putText(cv_image, "dist: " + dist_text,
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(cv_image, "size: " + size_text,
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(cv_image, "target: 1.0m",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

    def follow(self):
        rate      = rospy.Rate(10)
        img_width = 640

        while not rospy.is_shutdown():
            twist = Twist()

            if self.ball_col is not None:
                error = self.ball_col - img_width // 2
                twist.angular.z = -float(error) / 300.0

                if self.front_dist is not None:
                    dist_error = self.front_dist - self.target_dist

                    if dist_error > self.tolerance:
                        twist.linear.x = min(0.4, dist_error * 0.4)
                    elif dist_error < -self.tolerance:
                        twist.linear.x = max(-0.4, dist_error * 0.4)
                    else:
                        twist.linear.x = 0.0

                    rospy.loginfo("dist: %.2fm | error: %.2fm | col: %d",
                                  self.front_dist, dist_error, self.ball_col)
                else:
                    twist.linear.x = 0.05
                    rospy.loginfo("Ball col: %d | no depth", self.ball_col)
            else:
                twist.linear.x  = 0.0
                twist.angular.z = 0.0
                rospy.loginfo("No ball detected, stopping...")

            self.cmd_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ball_follower')
    follower = BallFollower()
    follower.follow()
