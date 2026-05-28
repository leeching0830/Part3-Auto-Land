#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math

from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from apriltag_ros.msg import AprilTagDetectionArray

class TagPoseLanding:

    def __init__(self):
        rospy.init_node("tag_pose_landing")

        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber("/mavros/state", State, self.state_callback, queue_size=1)
        
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.current_state = State()
        self.tag_seen = False
        self.ex = 0.0
        self.ey = 0.0
        self.ez = 0.0
        self.current_z = 0.0  
        self.cmd = TwistStamped()

        
        self.kp_xy = 0.45
        self.max_xy = 0.15
        self.kp_z = 0.5       
        self.target_z = 2.5   
        
        
        self.center_threshold = 0.15

        self.search_time = 0.0
        self.brake_counter = 0 
        self.lost_tag_frames = 0
        self.max_lost_frames = 30 
        self.center_counter = 0    
        
        self.last_req = rospy.Time.now()

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_z = msg.pose.position.z

    def tag_callback(self, msg):
        if len(msg.detections) == 0:
            return

        det = msg.detections[0]
        pose = det.pose.pose.pose
        
        raw_x = pose.position.x  
        raw_y = pose.position.y  
        self.ez = pose.position.z
        
       
        self.ex = raw_x + 1.31
        self.ey = raw_y + 1.07
        
        self.lost_tag_frames = 0
        
        if not self.tag_seen:
            self.tag_seen = True
            self.search_time = 0.0
            self.brake_counter = 6 

    def limit(self, v, max_v):
        return max(-max_v, min(max_v, v))

    def control(self):
        rate = rospy.Rate(20)

        try:
            while not rospy.is_shutdown():
                self.cmd.header.stamp = rospy.Time.now()

                
                if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req) > rospy.Duration(2.0):
                    try:
                        resp = self.set_mode_client(custom_mode="OFFBOARD")
                        if resp.mode_sent:
                            rospy.loginfo("Requesting OFFBOARD...")
                    except rospy.ServiceException as e:
                        pass
                    self.last_req = rospy.Time.now()

                self.lost_tag_frames += 1
                if self.lost_tag_frames > self.max_lost_frames:
                    self.tag_seen = False

                z_error = self.target_z - self.current_z
                vz_stabilize = self.limit(self.kp_z * z_error, 0.3)

                
                if not self.tag_seen:
                    self.search_time += 0.05
                    rospy.loginfo_throttle(2.0, f"Searching for Tag...")
                    
                    omega = 0.5  
                    a = 0.03     
                    t = self.search_time
                    
                    self.cmd.twist.linear.x = self.limit(a * math.cos(omega * t) - a * t * omega * math.sin(omega * t), 0.15)
                    self.cmd.twist.linear.y = self.limit(a * math.sin(omega * t) + a * t * omega * math.cos(omega * t), 0.15)
                    self.cmd.twist.linear.z = vz_stabilize  
                    self.cmd.twist.angular.z = 0.15
                    self.vel_pub.publish(self.cmd)

                
                elif self.brake_counter > 0:
                    self.cmd.twist.linear.x = 0.0
                    self.cmd.twist.linear.y = 0.0
                    self.cmd.twist.linear.z = vz_stabilize  
                    self.cmd.twist.angular.z = 0.0
                    self.vel_pub.publish(self.cmd)
                    self.brake_counter -= 1

                
                else:
                    if self.lost_tag_frames > 1:
                        self.cmd.twist.linear.x = 0.0
                        self.cmd.twist.linear.y = 0.0
                        self.cmd.twist.linear.z = vz_stabilize
                        self.cmd.twist.angular.z = 0.0
                        self.vel_pub.publish(self.cmd)
                    
                    else:
                        
                        vx_body = self.kp_xy * self.ey
                        vy_body = self.kp_xy * self.ex

                        self.cmd.twist.linear.x = self.limit(vx_body, self.max_xy)
                        self.cmd.twist.linear.y = self.limit(vy_body, self.max_xy)
                        self.cmd.twist.linear.z = vz_stabilize  
                        self.cmd.twist.angular.z = 0.0

                        
                        if abs(self.ex) < self.center_threshold and abs(self.ey) < self.center_threshold:
                            self.center_counter += 1
                            rospy.loginfo_throttle(0.2, f" Stabilizing ({self.center_counter}/10)")

                            if self.center_counter >= 10:
                                rospy.loginfo(" Switching to Land...")
                                self.cmd.twist.linear.x = 0.0
                                self.cmd.twist.linear.y = 0.0
                                self.cmd.twist.linear.z = 0.0
                                self.cmd.twist.angular.z = 0.0
                                for _ in range(12): 
                                    self.vel_pub.publish(self.cmd)
                                    rate.sleep()
                                
                                try:
                                    response = self.set_mode_client(custom_mode="AUTO.LAND")
                                    if response.mode_sent:
                                        rospy.loginfo("Land")
                                        break
                                except rospy.ServiceException as e:
                                    rospy.logerr("Land failed" )
                            else:
                                self.vel_pub.publish(self.cmd)
                        
                        else:
                            self.center_counter = 0  
                            rospy.loginfo_throttle(0.5, f"Track, X: {self.ex:.2f}m, Y: {self.ey:.2f}m")
                            self.vel_pub.publish(self.cmd)

                rate.sleep()

        except rospy.ROSInterruptException:
            rospy.loginfo("Node closed")

if __name__ == "__main__":
    node = TagPoseLanding()
    node.control()