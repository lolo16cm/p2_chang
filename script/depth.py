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

        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

        self.ball_col    = None
        self.ball_row    = None
        self.ball_size   = None
        self.front_dist  = None

        self.target_dist = 1.1
        self.tolerance   = 0.05

        rospy.loginfo("Ball Follower (Depth Only) Started!")

    def depth_callback(self, msg):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            depth = depth_raw.astype(np.float32) / 1000.0
        except:
            try:
                depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            except Exception as e:
                rospy.logerr("Depth error: %s", str(e))
                return

        # ── Threshold depth to find nearby objects (0.3m - 3.0m) ──
        valid_mask = ((depth > 0.3) & (depth < 3.0)).astype(np.uint8) * 255

        # ── Find contours ──
        contours, _ = cv2.findContours(
            valid_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # ── Build visualization image from depth ──
        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = np.uint8(depth_vis)
        vis_image = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

        best = None
        best_circularity = 0

        for c in contours:
            area = cv2.contourArea(c)
            if area < 500:
                continue

            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius < 15:
                continue

            circularity = area / (3.14159 * radius * radius) if radius > 0 else 0

            if circularity > 0.55 and circularity > best_circularity:
                best_circularity = circularity
                best = (x, y, radius, c)

        if best is not None:
            x, y, radius, c = best
            self.ball_col = int(x)
            self.ball_row = int(y)
            self.ball_size = radius

            # Get distance at ball center
            y1 = max(0, int(y) - 2)
            y2 = min(depth.shape[0], int(y) + 2)
            x1 = max(0, int(x) - 2)
            x2 = min(depth.shape[1], int(x) + 2)
            region = depth[y1:y2, x1:x2]
            valid = region[(region > 0.1) & (region < 5.0) & np.isfinite(region)]
            if len(valid) > 0:
                self.front_dist = float(np.mean(valid))
            else:
                self.front_dist = None

            cv2.circle(vis_image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
        else:
            self.ball_col   = None
            self.ball_row   = None
            self.ball_size  = None
            self.front_dist = None

        # ── Draw info text ──
        dist_text = "{:.2f}m".format(self.front_dist) if self.front_dist else "N/A"
        size_text = "{:.1f}px".format(self.ball_size) if self.ball_size else "N/A"

        cv2.putText(vis_image, "dist: " + dist_text,
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(vis_image, "size: " + size_text,
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(vis_image, "target: 1.0m",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(vis_image, "MODE: DEPTH ONLY",
                    (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(vis_image, 'bgr8'))

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
                        twist.linear.x = min(0.2, dist_error * 0.4)
                    elif dist_error < -self.tolerance:
                        twist.linear.x = max(-0.2, dist_error * 0.4)
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