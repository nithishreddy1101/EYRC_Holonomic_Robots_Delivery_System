#!/usr/bin/env python3

'''
# Team ID:          1060
# Theme:            Holo Battalion
# Author List:      A.Nithish, Rohan A Khamitkar
# Filename:         holonomic_controller.py
# Functions:        pose_bot_cb, pose_crate_cb, control_cb, publish_wheel_velocities, inverse_kinematics
# Global variables: None
'''

# ---------------------- Import Required Libraries ----------------------------
import rclpy
from rclpy.node import Node
from hb_interfaces.msg import Pose2D, Poses2D
from hb_interfaces.msg import BotCmd, BotCmdArray
import numpy as np
import math
from functools import partial

import sys 
import paho.mqtt.client as mqtt
from hb_control.srv import Attach #custom service 

broker_ip = "172.29.122.156"

def angle_error_rad(goal_rad, current_rad):
    '''
    Purpose:
    ---
    Computes the shortest signed angular error between goal and current orientation.

    This ensures the angle difference stays in the range [-π, π], preventing 
    discontinuities when angles wrap around 0° or 360°.

    Example:
    If goal = 350° and current = 10°, naive subtraction gives -340°,
    but the shortest path is actually +20°.

    Returns:
    ---
    error : float
        Angular error in radians (within [-π, π])
    '''
    error = goal_rad - current_rad 
    while error > math.pi:
        error -= 2.0 * math.pi
    while error < -math.pi:
        error += 2.0 * math.pi
    return error


# ---------------------- PID Controller Class --------------------------------
class PID:
    '''
    Purpose:
    ---
    Implements a simple PID (Proportional–Integral–Derivative) controller.
    Each axis (x, y, θ) uses its own PID instance.

    Attributes:
        kp, ki, kd : PID constants
        max_out    : Maximum output value to limit command velocity
        integral   : Sum of past errors
        prev_error : Error from previous iteration (for derivative term)
    '''
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        '''
        Purpose:
        ---
        Compute PID output based on current error and time difference.

        Formula:
        output = (kp * error) + (ki * integral of error) + (kd * rate of change of error)
        '''
        
        self.integral += error*dt

        derivative = (error - self.prev_error) / dt

        v = self.kp*error + self.ki*self.integral + self.kd*derivative

        self.prev_error = error

        v = max(min(v, self.max_out), -self.max_out)

        return v
    
    
    def reset(self):
        '''Reset integral and previous error when moving to a new goal.'''
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Main Node Class -------------------------------------
class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')  # initializing ros node

        self.max_vel = 500.0
        self.last_time = self.get_clock().now()
        self.current_pose = None
        self.current_bot_poses = {} 
        self.crate_poses = {}
        self.current_index_ = 0
        self.state = 0
        self.wait_start_time = None
        self.wait_duration = 5.0
        self.bias = 180.5

        self.initial_poses = None
        self.task = 0

        self.ir_msg = 1


        # ---------------- Goal Definitions ----------------
        self.goals = [(1219.2, 1219.0, 0.0),
                      (1219.2, 1219.2, 0.0),
                      (1218.2, 205.2, 0.0)]
        
        self.pid_params = {
            'x': {'kp': 10.8, 'ki': 0.00, 'kd':0 , 'max_out': self.max_vel},
            'y': {'kp': 10.8, 'ki': 0.00, 'kd': 0, 'max_out': self.max_vel},
            'theta': {'kp': 1000.0, 'ki': 0.0, 'kd': 0.0, 'max_out': self.max_vel*2 }
        }

        # Initialize PIDs
        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        # ---------------- ROS 2 Publishers & Subscribers ----------------

        self.bot_subsriber = self.create_subscription(Poses2D, "/bot_pose", self.pose_bot_cb, 10)
        self.crate_pose = self.create_subscription(Poses2D, "/crate_pose", self.pose_crate_cb, 10)

        self.client = mqtt.Client()

        self.client.on_message = self.on_message
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.connect(broker_ip, 1883, 60)
        self.client.loop_start()
        
        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.05, self.control_cb)  # ~50ms = 20 Hz

        self.get_logger().info(f'Holonomic PID Controller started. Goals: {self.goals}')


    # ---------------- Subscriber Callback ----------------
    def pose_bot_cb(self, msg: Poses2D):
        '''
        Purpose:
        ---
        Callback function for /bot_pose topic.
        Updates the current robot pose whenever a new pose message arrives.
        '''
        poses = msg.poses
        if not poses or len(poses) == 0:
            return
        for pose in poses:
            self.current_bot_poses[pose.id] =(pose.x, pose.y, math.radians(pose.w))

    def pose_crate_cb(self, msg):
        poses = msg.poses
        if not poses or len(poses) == 0:
            return
        for pose in poses:
            self.crate_poses[pose.id] =(pose.x, pose.y, pose.w)
            self.goals[0] = self.crate_poses[pose.id]


    # ---------------- Control Loop ----------------
    def control_cb(self):
        '''
        Purpose:
        ---
        Main control loop called periodically via ROS timer.
        It computes PID-based body velocities and converts them to wheel velocities.
        '''

        if not self.current_bot_poses:
            topic = f"bot_cmd/0" 
            motor_msg = "1500,1500,1500,100,50"
            self.client.publish(topic, motor_msg, qos=0)
            self.get_logger().info("No bot poses received yet.")
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        pose_x, pose_y, pose_w = self.current_bot_poses[0]
    
        if self.current_index_ >= len(self.goals):
            wheel_vel = [0.0, 0.0, 0.0]
            self.publish_wheel_velocities(wheel_vel, 0, 100, 50)
            return
      
        goal_x, goal_y, goal_w = self.goals[self.current_index_]
        goal_w = math.radians(goal_w)
        goal_w = 0

        print(f"{self.goals[self.current_index_]}")

        error = [goal_x - pose_x,
                goal_y - pose_y-self.bias,
                angle_error_rad(goal_w, pose_w)]
        
        error_x, error_y = self.rotation_matrix(error[0], error[1], pose_w) 

        print(f"Error: {error[0]} {error[1]}")
        
        vx = self.pid_x.compute(error_x, dt)
        vy = self.pid_y.compute(error_y, dt)
        vw = self.pid_theta.compute(error[2], dt)

        vel_bot = np.array([vx, vy, vw])

        wheel_vel = self.inverse_kinematics(vel_bot)

        print(f"{wheel_vel}")

        if (self.state == 0):
            self.publish_wheel_velocities(wheel_vel, 0, 90, 0)
            if (abs(error[0]) < 5.0 and abs(error[1]) < 5.0 and abs(error[2]) < math.radians(5.0)):
                    self.get_logger().info(f"Goal {self.current_index_+1} reached.")
                    wheel_vel = [0.0, 0.0, 0.0]
                    self.publish_wheel_velocities(wheel_vel, 0, 70, 50)
                    self.state = 1
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    self.current_index_ += 1
                    self.bias = 150.0
                    return
        if(self.state == 1):
            if self.wait_start_time is None:
                self.wait_start_time = self.get_clock().now()
            else:
                elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
                if elapsed >= self.wait_duration:
                    self.wait_start_time = None
                    wheel_vel = [0.0, 0.0, 0.0]
                    self.publish_wheel_velocities(wheel_vel, 0, 165, 90) 
                    self.call_link_service(0,"attach")
                    self.state = 2
                    return   
        if(self.state == 2):
            if self.wait_start_time is None:
                self.wait_start_time = self.get_clock().now()
            else:
                elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
                if elapsed >= self.wait_duration:
                    self.wait_start_time = None
                    wheel_vel = [0.0, 0.0, 0.0]
                    self.publish_wheel_velocities(wheel_vel, 0, 70, 60) 
                    self.state = 3
                    return              
        if(self.state == 3):
            self.publish_wheel_velocities(wheel_vel, 0, 70, 60)
            if (abs(error[0]) < 5.0 and abs(error[1]) < 5.0 and abs(error[2]) < 5.0):
                    self.get_logger().info(f"Goal {self.current_index_+1} reached.")
                    self.publish_wheel_velocities([0.0, 0.0, 0.0], 0, 160, 90)
                    self.current_index_ += 1
                    self.bias = 0.0
                    self.state = 4
                    return
        if(self.state == 4):
            if self.wait_start_time is None:
                self.wait_start_time = self.get_clock().now()
            else:
                elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
                if elapsed >= self.wait_duration:
                    self.wait_start_time = None
                    self.call_link_service(0, "detach")
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    self.state = 5
                    return
                
        if (self.state == 5):
            self.publish_wheel_velocities(wheel_vel, 0, 90, 50)
            if (abs(error[0]) < 5.0 and abs(error[1]) < 5.0 and abs(error[2]) < 5.0):
                self.current_index_ += 1
                   

    def call_link_service(self, bot_id, action):

        if action == "attach":
            self.attach_cli = self.create_client(Attach, "/attach")
            while not self.attach_cli.wait_for_service(1.0):
                self.get_logger().warn("Waiting for the service.. ")

            self.attach_req = Attach.Request()
            self.attach_req.attach =True
            self.attach_req.bot_id = bot_id
            future = self.attach_cli.call_async(self.attach_req)

        elif action == "detach":
            self.detach_cli = self.create_client(Attach, "/attach")
            while not self.detach_cli.wait_for_service(1.0):
                self.get_logger().warn("Waiting for the service.. ")

            self.detach_req = Attach.Request()
            self.detach_req.attach = False
            self.detach_req.bot_id = bot_id
            future = self.detach_cli.call_async(self.detach_req)

        future.add_done_callback(partial(self.callback_service))

    def callback_service(self, future):
        try: 
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e,))

    def rotation_matrix(self, vx, vy, pose_theta):
        matrix = np.array([[ np.cos(pose_theta), np.sin(pose_theta)],
                           [-np.sin(pose_theta), np.cos(pose_theta)]])
        v1, v2 = matrix @ np.array([[vx],
                                    [vy]])
        return v1[0], v2[0]


    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel, bot_id, base_angle, elbow_angle):
        '''
        Purpose:
        ---
        Publishes wheel velocity commands to the robot’s motor controller.
        '''     
        topic = f"bot_cmd/{bot_id}"  

        max_w = 250.0
        normalize = 0

        if abs(wheel_vel[0]) > max_w or abs(wheel_vel[1]) > max_w or abs(wheel_vel[2]) > max_w:
            normalize = np.linalg.norm([wheel_vel[0], wheel_vel[1], wheel_vel[2]])
            wheel_vel[0] *= max_w/normalize
            wheel_vel[1] *= max_w/normalize
            wheel_vel[2] *= max_w/normalize

        w1 = int((1500 + wheel_vel[0]))
        w2 = int(1500 + wheel_vel[1]) 
        w3 = int(1500 + wheel_vel[2])

        motor_msg = f"{w1},{w2},{w3},{base_angle},{elbow_angle}"

        self.client.publish(topic, motor_msg, qos=0)

    
    def on_message(self, client, userdata, msg):
        print(f"[{msg.topic}] {msg.payload.decode()}")
        self.ir_msg = msg.payload.decode()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker")
        client.subscribe("esp/sensor/0")

    def on_disconnect(self, client, userdata, rc):
        print("Disconnected from broker")

    def inverse_kinematics(self, vel_bot):

        M_inv = np.array([
                        [-3.33333333e-01,  5.77350269e-01,  2.11117445e-01],
                        [-3.33333333e-01, -5.77350269e-01,  2.11117445e-01],
                        [ 6.66666667e-01,  4.27325041e-17,  2.11117445e-01]
                        ])
    
        return np.dot(M_inv, vel_bot)     


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()