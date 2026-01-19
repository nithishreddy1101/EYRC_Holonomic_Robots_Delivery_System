#!/usr/bin/env python3

'''
# Team ID:          1060
# Theme:            Holo Battalion
# Author List:      A.Nithish, Rohan A Khamitkar
# Filename:         multiholonomic_controller.py
# Functions:        pose_bot_cb, pose_crate_cb, control_cb, publish_wheel_velocities, inverse_kinematics
# Global variables: None
'''

# ---------------------- Import Required Libraries ----------------------------
import rclpy
from rclpy.node import Node
from hb_interfaces.msg import Pose2D, Poses2D
import numpy as np
import math
from functools import partial

import paho.mqtt.client as mqtt
from hb_control.srv import Attach #custom service 
import pyrvo
from hb_interfaces.msg import BotCmd, BotCmdArray
from scipy.optimize import linear_sum_assignment


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
        self.wait_start_time = None
        self.wait_duration = 5.0

        self.initial_poses = None
        self.task = 0

        self.ir_msg = 1

        self.current_index_0 = 0
        self.current_index_2 = 0
        self.current_index_4 = 0

        self.state0 = 0
        self.state2 = 0
        self.state4 = 0

        self.bias0 = 180.5
        self.bias2 = 180.5
        self.bias4 = 180.5

        self.initial_crate_poses = None
        self.task_allocaton = 0
        self.pose_state = 0
        self.tasks = {}


        self.current_angle0 = 90
        self.target_angle0 = 10

        self.current_angle2 = 90
        self.target_angle02 = 10
        
        self.current_angle4 = 90
        self.target_angle4 = 10

        self.ds = {"red":(1215, 1215-100.0, 0.0),
                  "green":(820, 2017.5, 0.0), 
                  "blue":(1616, 2015.5, 0.0)}


        # ---------------- Goal Definitions ----------------
        self.goals0 = [(1219.2, 1219.0, 0.0),
                      (1219.2, 1219.2, 0.0),
                      (1740, 1040, 0.0),
                      (1218.2, 205.2, 0.0)]
        
        self.goals2 = [(0, 0, 0),
                      (0.0, 0.0, 0.0),
                      (1568.0, 205.2, 0.0)]
        self.goals4 = [(0, 0, 0),
                      (0.0, 0.0, 0.0),
                      (864.0, 204.2, 0.0)]
        
        self.deliv0 = 0
        self.deliv2 = 0
        self.deliv4 = 0

        self.goals_map = {
                    0: self.goals0,
                    2: self.goals2,
                    4: self.goals4
                }

        self.unassigned_crates = None

        self.sim = pyrvo.RVOSimulator(0.1, 300, 3, 5000,5000, 160, 500.0)

        self.agent_0 = self.sim.add_agent([1218.2, 205.2])
        self.agent_2 = self.sim.add_agent([1568.0, 205.2])
        self.agent_4 = self.sim.add_agent([864.0, 204.2])

        self.preferred_velocity = np.array([-50.0, 0.0])
        self.position = np.array([0.0, 0.0])
        
        self.pid_params0 = {
            'x': {'kp': 10.8, 'ki': 0.00, 'kd':0 , 'max_out': self.max_vel},
            'y': {'kp': 10.8, 'ki': 0.00, 'kd': 0, 'max_out': self.max_vel},
            'theta': {'kp': 1000.0, 'ki': 0.0, 'kd': 0.0, 'max_out': self.max_vel*2 }
        }

        self.pid_params2 = {
            'x': {'kp': 7.8, 'ki': 0.00, 'kd':0 , 'max_out': self.max_vel},
            'y': {'kp': 7.8, 'ki': 0.00, 'kd': 0, 'max_out': self.max_vel},
            'theta': {'kp': 1000.0, 'ki': 0.0, 'kd': 0.0, 'max_out': self.max_vel*2 }
        }

        self.pid_params4 = {
            'x': {'kp': 12.8, 'ki': 0.00, 'kd':0 , 'max_out': self.max_vel},
            'y': {'kp': 12.8, 'ki': 0.00, 'kd': 0, 'max_out': self.max_vel},
            'theta': {'kp': 1000.0, 'ki': 0.0, 'kd': 0.0, 'max_out': self.max_vel*2 }
        }


        # Initialize PIDs
        self.pid_x0 = PID(**self.pid_params0['x'])
        self.pid_y0 = PID(**self.pid_params0['y'])
        self.pid_theta0 = PID(**self.pid_params0['theta'])

        self.pid_x2 = PID(**self.pid_params2['x'])
        self.pid_y2 = PID(**self.pid_params2['y'])
        self.pid_theta2 = PID(**self.pid_params2['theta'])

        self.pid_x4 = PID(**self.pid_params4['x'])
        self.pid_y4 = PID(**self.pid_params4['y'])
        self.pid_theta4 = PID(**self.pid_params4['theta'])

        # ---------------- ROS 2 Publishers & Subscribers ----------------

        self.bot_subsriber = self.create_subscription(Poses2D, "/bot_pose", self.pose_bot_cb, 10)
        self.crate_pose = self.create_subscription(Poses2D, "/crate_pose", self.pose_crate_cb, 10)
        self.sub = self.create_subscription(BotCmdArray,'/bot_cmd',self.cmd_callback,10)
        self.cmd_publisher = self.create_publisher(BotCmdArray, "/bot_cmd", 10)

        self.client = mqtt.Client()

        self.client.on_message = self.on_message
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.connect(broker_ip, 1883, 60)
        self.client.loop_start()
        
        # ---------------- Timer for Control Loop ----------------
        self.timer0 = self.create_timer(0.05, self.control_cb0)  # ~50ms = 20 Hz
        self.timer2 = self.create_timer(0.05, self.control_cb2)  # ~50ms = 20 Hz
        self.timer4 = self.create_timer(0.05, self.control_cb4)  # ~50ms = 20 Hz
        self.task_timer = self.create_timer(1.5, self.task_alloc)

        self.attach_cli = self.create_client(Attach, "/attach")
        self.detach_cli = self.create_client(Attach, "/attach")

        self.get_logger().info(f'Holonomic PID Controller started. Goals: {self.goals0}')

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


    # ---------------- Control Loop ----------------
    def control_cb0(self):
        '''
        Purpose:
        ---
        Main control loop called periodically via ROS timer.
        It computes PID-based body velocities and converts them to wheel velocities.
        '''
        bot_id = 0
        if not self.current_bot_poses:
            self.cmd_publisher.publish(BotCmdArray(cmds=[BotCmd(
                id=0, m1=1500, m2=1500, m3=1500, base=110, elbow=0)]))  
            self.get_logger().info("No bot poses received yet.")
            return
        
        if self.pose_state != 1:
            return
        
        position =  np.array([self.current_bot_poses[bot_id][0], self.current_bot_poses[bot_id][1]]) 

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        pose_x0, pose_y0, pose_w0 = self.current_bot_poses[bot_id]
    
        if self.current_index_0 >= len(self.goals0):
            self.cmd_publisher.publish(BotCmdArray(cmds=[BotCmd(
                id=0, m1=1500, m2=1500, m3=1500, base=100, elbow=0)]))  
            return
      
        goal_x0, goal_y0, goal_w0= self.goals0[self.current_index_0]
        goal_w0 = math.radians(goal_w0)
        goal_w0 = 0

        error = [goal_x0 - pose_x0,
                goal_y0 - pose_y0-self.bias0,
                angle_error_rad(goal_w0, pose_w0)]
        
        error_x, error_y = self.rotation_matrix(error[0], error[1], pose_w0) 

        # print(f"Error: {error[0]} {error[1]}")
        
        vx = self.pid_x0.compute(error_x, dt)
        vy = self.pid_y0.compute(error_y, dt)
        vw = self.pid_theta0.compute(error[2], dt)

        self.sim.set_agent_pref_velocity(self.agent_0, [vx, vy])
        self.sim.set_agent_position(self.agent_0, tuple(position))
        self.sim.do_step()
        new_velocity = self.sim.get_agent_velocity(self.agent_0)

        vel_bot = np.array([new_velocity.x, new_velocity.y, vw])

        wheel_vel = self.inverse_kinematics(vel_bot)

        # print(f"{wheel_vel}")
        # ===================== STATE 0: Move to goal =====================
        if (self.state0 == 0):
            self.publish_wheel_velocities(wheel_vel, bot_id, 110, 0)
            if self.goal_reached(error):
                    self.get_logger().info(f"Goal {self.current_index_0+1} reached.")
                    self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, 70, 50)
                    self.state0 = 1
                    self.pid_x0.reset()
                    self.pid_y0.reset()
                    self.pid_theta0.reset()

                    self.current_angle0 = 90
                    self.target_angle0 = 165

                    self.current_index_0 += 1
                    self.bias0 = 0.0
                    return
        # ===================== STATE 1: Lower gripper =====================
        if self.state0 == 1:
            done, self.current_angle0 = self.move_servo_towards_target(self.current_angle0, self.target_angle0)
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, self.current_angle0, 90)
            if done:
                self.state0 = 2
            return
        # ===================== STATE 2: Wait =====================
        if self.state0 == 2:
            if self.wait_done():
                self.call_link_service(bot_id, "attach")
                self.state0 = 3
            return
        
        # ===================== STATE 3: Lift gripper =====================
        if self.state0 == 3:
            if self.wait_done():
                self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, 90, 90)
                self.state0 = 4
            return  
        
        # ===================== STATE 4: Move to next goal =====================  
        if self.state0 == 4:
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 90)

            if self.goal_reached(error):
                self.get_logger().info(f"Goal {self.current_index_0 + 1} reached.")

                self.current_index_0 += 1
                self.bias0 = 0.0

                self.current_angle0 = 90
                self.target_angle0 = 160
                self.state0 = 5

            return  
        # ===================== STATE 5: Lower object =====================
        if self.state0 == 5:
            done, self.current_angle0 = self.move_servo_towards_target(self.current_angle0, self.target_angle0)
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, self.current_angle0, 90)

            if done:
                self.state0 = 6
            return      
        # ===================== STATE 6: Detach =====================
        if self.state0 == 6:
            if self.wait_done():
                self.call_link_service(bot_id, "detach")

                self.pid_x0.reset()
                self.pid_y0.reset()
                self.pid_theta0.reset()

                self.state0 = 7
            return
        
        # ===================== STATE 7: Final move =====================        
        if (self.state0 == 7):
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 50)
            if self.goal_reached(error):
                self.current_index_0 += 1

    def control_cb2(self):
        '''
        Purpose:
        ---
        Main control loop called periodically via ROS timer.
        It computes PID-based body velocities and converts them to wheel velocities.
        '''
        bot_id = 2
        if not self.current_bot_poses:
            self.cmd_publisher.publish(BotCmdArray(cmds=[BotCmd(
                id=bot_id, m1=1500, m2=1500, m3=1500, base=90, elbow=180)]))  
            self.get_logger().info("No bot poses received yet.")
            return
        
        if self.pose_state != 1:
            return
        position =  np.array([self.current_bot_poses[bot_id][0], self.current_bot_poses[bot_id][1]])

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        pose_x2, pose_y2, pose_w2 = self.current_bot_poses[bot_id]
    
        if self.current_index_2 >= len(self.goals2):
            self.cmd_publisher.publish(BotCmdArray(cmds=[BotCmd(
                id=bot_id, m1=1500, m2=1500, m3=1500, base=90, elbow=180)]))  
            return
      
        goal_x2, goal_y2, goal_w2= self.goals2[self.current_index_2]
        goal_w2 = math.radians(goal_w2)
        goal_w2= 0

        error = [goal_x2 - pose_x2,
                goal_y2 - pose_y2-self.bias2,
                angle_error_rad(goal_w2, pose_w2)]
        
        error_x, error_y = self.rotation_matrix(error[0], error[1], pose_w2) 

        # print(f"Error: {error[0]} {error[1]}")
        
        vx = self.pid_x2.compute(error_x, dt)
        vy = self.pid_y2.compute(error_y, dt)
        vw = self.pid_theta2.compute(error[2], dt)

        self.sim.set_agent_pref_velocity(self.agent_2, [vx, vy])
        self.sim.set_agent_position(self.agent_2, tuple(position))
        self.sim.do_step()
        new_velocity = self.sim.get_agent_velocity(self.agent_2)

        vel_bot = np.array([new_velocity.x, new_velocity.y, vw])

        wheel_vel = self.inverse_kinematics(vel_bot)

                # ===================== STATE 0: Move to goal =====================
        if (self.state2 == 0):
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 180)
            if self.goal_reached(error):
                    self.get_logger().info(f"Goal {self.current_index_2+1} reached.")
                    self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, 90, 90)
                    self.state2 = 1
                    self.pid_x2.reset()
                    self.pid_y2.reset()
                    self.pid_theta2.reset()

                    self.current_angle2 = 90
                    self.target_angle2 = 10

                    self.current_index_2 += 1
                    self.bias2 = 0.0
                    return
        # ===================== STATE 1: Lower gripper =====================
        if self.state2 == 1:
            done, self.current_angle2 = self.move_servo_towards_target(self.current_angle2, self.target_angle2)
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, self.current_angle2, 90)
            if done:
                self.state2 = 2
            return
        # ===================== STATE 2: Wait =====================
        if self.state2 == 2:
            if self.wait_done():
                self.call_link_service(bot_id, "attach")
                self.state2 = 3
            return
        
        # ===================== STATE 3: Lift gripper =====================
        if self.state2 == 3:
            if self.wait_done():
                self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, 90, 90)
                self.state2 = 4
            return  
        
        # ===================== STATE 4: Move to next goal =====================  
        if self.state2 == 4:
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 90)

            if self.goal_reached(error):
                self.get_logger().info(f"Goal {self.current_index_2 + 1} reached.")

                self.current_index_2 += 1
                self.bias2 = 0.0

                self.current_angle2 = 90
                self.target_angle2 = 10
                self.state2 = 5

            return  
        # ===================== STATE 5: Lower object =====================
        if self.state2 == 5:
            done, self.current_angle2 = self.move_servo_towards_target(self.current_angle2, self.target_angle2)
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, self.current_angle2, 90)

            if done:
                self.state2 = 6
            return      
        # ===================== STATE 6: Detach =====================
        if self.state2 == 6:
            if self.wait_done():
                self.call_link_service(bot_id, "detach")


                self.pid_x2.reset()
                self.pid_y2.reset()
                self.pid_theta2.reset()

                self.state2 = 7
            return
        
        # ===================== STATE 7: Final move =====================        
        if (self.state2 == 7):
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 180)
            if self.goal_reached(error):
                self.current_index_2 += 1

    def control_cb4(self):
        '''
        Purpose:
        ---
        Main control loop called periodically via ROS timer.
        It computes PID-based body velocities and converts them to wheel velocities.
        '''
        bot_id = 4
        if not self.current_bot_poses:
            self.cmd_publisher.publish(BotCmdArray(cmds=[BotCmd(
                id=bot_id, m1=1500, m2=1500, m3=1500, base=90, elbow=50)]))  
            self.get_logger().info("No bot poses received yet.")
            return
        
        if self.pose_state != 1:
            return
        position =  np.array([self.current_bot_poses[bot_id][0], self.current_bot_poses[bot_id][1]])

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        pose_x4, pose_y4, pose_w4 = self.current_bot_poses[bot_id]
    
        if self.current_index_4 >= len(self.goals4):
            self.cmd_publisher.publish(BotCmdArray(cmds=[BotCmd(
                id=bot_id, m1=1500, m2=1500, m3=1500, base=90, elbow=180)]))  
            return
      
        goal_x4, goal_y4, goal_w4= self.goals4[self.current_index_4]
        goal_w4 = math.radians(goal_w4)
        goal_w4= 0

        error = [goal_x4 - pose_x4+20.0,
                goal_y4 - pose_y4-self.bias4,
                angle_error_rad(goal_w4, pose_w4)]
        
        error_x, error_y = self.rotation_matrix(error[0], error[1], pose_w4) 

        # print(f"Error: {error[0]} {error[1]}")
        
        vx = self.pid_x4.compute(error_x, dt)
        vy = self.pid_y4.compute(error_y, dt)
        vw = self.pid_theta4.compute(error[2], dt)

        self.sim.set_agent_pref_velocity(self.agent_4, [vx, vy])
        self.sim.set_agent_position(self.agent_4, tuple(position))
        self.sim.do_step()
        new_velocity = self.sim.get_agent_velocity(self.agent_4)

        vel_bot = np.array([new_velocity.x, new_velocity.y, vw])

        wheel_vel = self.inverse_kinematics(vel_bot)

        # print(f"{wheel_vel}")
                 # ===================== STATE 0: Move to goal =====================
        if (self.state4 == 0):
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 180)
            if self.goal_reached(error):
                    self.get_logger().info(f"Goal {self.current_index_4+1} reached.")
                    self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, 90, 90)
                    self.state4 = 1
                    self.pid_x4.reset()
                    self.pid_y4.reset()
                    self.pid_theta4.reset()

                    self.current_angle4 = 90
                    self.target_angle4 = 10

                    self.current_index_4 += 1
                    self.bias4 = 0.0
                    return
        # ===================== STATE 1: Lower gripper =====================
        if self.state4 == 1:
            done, self.current_angle4 = self.move_servo_towards_target(self.current_angle4, self.target_angle4)
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, self.current_angle4, 85)
            if done:
                self.state4 = 2
            return
        # ===================== STATE 2: Wait =====================
        if self.state4 == 2:
            if self.wait_done():
                self.call_link_service(bot_id, "attach")
                self.state4 = 3
            return
        
        # ===================== STATE 3: Lift gripper =====================
        if self.state4 == 3:
            if self.wait_done():
                self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, 90, 90)
                self.state4 = 4
            return  
        
        # ===================== STATE 4: Move to next goal =====================  
        if self.state4 == 4:
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 90)

            if self.goal_reached(error):
                self.get_logger().info(f"Goal {self.current_index_4 + 1} reached.")

                self.current_index_4 += 1
                self.bias4 = 0.0

                self.current_angle4 = 90
                self.target_angle4 = 5
                self.state4 = 5

            return  
        # ===================== STATE 5: Lower object =====================
        if self.state4 == 5:
            done, self.current_angle4 = self.move_servo_towards_target(self.current_angle4, self.target_angle4)
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, self.current_angle4, 90)

            if done:
                self.state4 = 6
            return      
        # ===================== STATE 6: Detach =====================
        if self.state4 == 6:
            if self.wait_done():
                self.call_link_service(bot_id, "detach")

                self.pid_x4.reset()
                self.pid_y4.reset()
                self.pid_theta4.reset()

                self.state4 = 7
            return
        
        # ===================== STATE 7: Final move =====================        
        if (self.state4 == 7):
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 180)
            if self.goal_reached(error):
                self.current_index_4 += 1
                   

    def call_link_service(self, bot_id, action):

        if action == "attach":
            while not self.attach_cli.wait_for_service(1.0):
                self.get_logger().warn("Waiting for the service.. ")

            self.attach_req = Attach.Request()
            self.attach_req.attach =True
            self.attach_req.bot_id = bot_id
            future = self.attach_cli.call_async(self.attach_req)

        elif action == "detach":
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
    
    def goal_reached(self, error):
        return (
            abs(error[0]) < 5.0 and
            abs(error[1]) < 5.0 and
            abs(error[2]) < math.radians(5.0)
        )
    
    def move_servo_towards_target(self, current_angle, target_angle,step=1):

        if current_angle == target_angle:
            return True, current_angle

        if current_angle < target_angle:
            current_angle = min(current_angle + step, target_angle)
        else:
            current_angle = max(current_angle - step, target_angle)

        return False, current_angle


    def wait_done(self):
        if self.wait_start_time is None:
            self.wait_start_time = self.get_clock().now()
            return False

        elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
        if elapsed >= self.wait_duration:
            self.wait_start_time = None
            return True

        return False

    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel, bot_id, base_angle, elbow_angle):
        '''
        Purpose:
        ---
        Publishes wheel velocity commands to the robot’s motor controller.
        '''  

        max_w = 200.0
        normalize = 0

        if abs(wheel_vel[0]) > max_w or abs(wheel_vel[1]) > max_w or abs(wheel_vel[2]) > max_w:
            normalize = np.linalg.norm([wheel_vel[0], wheel_vel[1], wheel_vel[2]])
            wheel_vel[0] *= max_w/normalize
            wheel_vel[1] *= max_w/normalize
            wheel_vel[2] *= max_w/normalize

        w1 = int(1500 + wheel_vel[0])
        w2 = int(1500 + wheel_vel[1]) 
        w3 = int(1500 + wheel_vel[2])

        cmd_msg = BotCmdArray()
        cmd = BotCmd() 
        cmd.id = bot_id
        cmd.m1 = w1
        cmd.m2 = w2
        cmd.m3 = w3
        cmd.base = base_angle
        cmd.elbow = elbow_angle
        cmd_msg.cmds.append(cmd)
        self.cmd_publisher.publish(cmd_msg)

    
    def on_message(self, client, userdata, msg):
        print(f"[{msg.topic}] {msg.payload.decode()}")
        self.ir_msg = msg.payload.decode()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker")
        client.subscribe("esp/sensor/2")

    def on_disconnect(self, client, userdata, rc):
        print("Disconnected from broker")

    def inverse_kinematics(self, vel_bot):

        M_inv = np.array([
                        [-3.33333333e-01,  5.77350269e-01,  2.11117445e-01],
                        [-3.33333333e-01, -5.77350269e-01,  2.11117445e-01],
                        [ 6.66666667e-01,  4.27325041e-17,  2.11117445e-01]
                        ])
    
        return np.dot(M_inv, vel_bot)   
    
    def cmd_callback(self, msg: BotCmdArray): 
        for cmd in msg.cmds:
            bot_id = cmd.id
            motor_msg = (
                f"{cmd.m1},"
                f"{cmd.m2},"
                f"{cmd.m3},"
                f"{cmd.base},"
                f"{cmd.elbow}"
            )

            topic = f"bot_cmd/{bot_id}" 
            self.client.publish(topic, motor_msg, qos=0)
            # self.get_logger().info(
            #     f"Bot {bot_id} → {topic} : {motor_msg}"
            # )

    def task_allocator(self):

        if  len(self.current_bot_poses) !=3  and self.crate_poses is None:
            self.cmd_publisher.publish(BotCmdArray(cmds=[BotCmd(
                id=0, m1=1500, m2=1500, m3=1500, base=100, elbow=50)]))
            self.cmd_publisher.publish(BotCmdArray(cmds=[BotCmd(
                id=2, m1=1500, m2=1500, m3=1500, base=90, elbow=50)]))
            self.cmd_publisher.publish(BotCmdArray(cmds=[BotCmd(
                id=4, m1=1500, m2=1500, m3=1500, base=90, elbow=50)]))
            
            self.get_logger().info("No bot poses received yet.")
            return
        
        self.tasks = {}

        if self.task_allocaton == 0:
            self.initial_crate_poses = self.crate_poses
            print(self.crate_poses)
        elif self.task_allocaton == 1 and self.unassigned_crates is not None:
            self.initial_crate_poses = self.unassigned_crates
        else:
            return

        modified_crate_dict = {k: v[:2] for k, v in self.initial_crate_poses.items()}
        sorted_crate_poses = dict(sorted(modified_crate_dict.items()))
        crate_keys = list(sorted_crate_poses.keys())
        crate_array = np.array([item for tup in sorted_crate_poses.values() for item in tup])
        crate_reshaped_array = crate_array.reshape(len(crate_keys), 2)

        modified_bot_dict = {k: v[:2] for k, v in self.current_bot_poses.items()}
        sorted_bot_poses = dict(sorted(modified_bot_dict.items()))
        bot_keys = list(sorted_bot_poses.keys())
        bot_array = np.array([item for tup in sorted_bot_poses.values() for item in tup])
        bot_reshaped_array = bot_array.reshape(len(bot_keys), 2)
        
        cost_matrix = np.zeros((len(bot_keys), len(crate_keys)))
        for i in range(len(bot_keys)):
            for j in range(len(crate_keys)):
                cost_matrix[i, j] = np.linalg.norm(bot_reshaped_array[i] - crate_reshaped_array[j])
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        assignments = list(zip(row_ind, col_ind))
        self.get_logger().info(f"Assignments (bot -> crate): {assignments} {bot_keys} ")
        print(len(assignments))
        get_color_num = lambda i: (["red", "green", "blue"][i % 3], i)

        for i in range(len(assignments)):
            key = crate_keys[assignments[i][1]]
            self.tasks[bot_keys[assignments[i][0]]] = (self.initial_crate_poses[key], get_color_num(key))

        assigned_crates = set(col_ind)
        unassigned_crates = [j for j in range(len(crate_keys)) if j not in assigned_crates]
        if unassigned_crates:
            self.unassigned_crates = {crate_keys[unassigned_crates[0]]: self.crate_poses[crate_keys[unassigned_crates[0]]]}

        self.get_logger().info(f"Tasks: {(self.tasks)}  {self.unassigned_crates}")
        keys_ = list(self.tasks.keys())[0]
        if self.task_allocaton == 0:
            self.goals0[0] = self.tasks[0][0]
            self.goals2[0] = self.tasks[2][0]
            self.goals4[0] = self.tasks[4][0]
            # self.pose_state = 1
        elif self.task_allocaton == 1:
            if keys_ == 0:
                self.goals0[0] = self.tasks[0][0]
                self.state0 = 0
                self.current_index_0 = 0
                self.bias0 = 135.5
            if keys_ == 2:
                self.goals2[0] = self.tasks[2][0]
                self.state2 = 0
                self.current_index_2 = 0
                self.bias2 = 135.5
            if keys_ == 4:
                self.goals4[0] = self.tasks[4][0]
                self.state4 = 0
                self.current_index_4 = 0
                self.bias4 = 135.5
        red_keys = [k for k, (_, (color, _)) in self.tasks.items() if color == 'red']
        green_keys = [k for k, (_, (color, _)) in self.tasks.items() if color == 'green']
        blue_keys = [k for k, (_, (color, _)) in self.tasks.items() if color == 'blue']

        color_groups = {
            'red': red_keys,
            'green': green_keys,
            'blue': blue_keys
        }

        for color, keys in color_groups.items():
            if not keys:      
                continue
            base_x, base_y, base_w = self.ds[color]
            bias = 0
            for k in keys:
                goals = self.goals_map[k]
                goals[1] = (base_x + bias, base_y - 150 , base_w)
                bias += 80
        self.get_logger().info(f"Goals: {self.goals0} {self.goals2} {self.goals4}")

    def task_alloc(self):
        if self.pose_state == 0:
            self.task_allocator()
            self.pose_state = 1
        if self.deliv0 == 1 and self.deliv2 == 1 and self.deliv4 == 1 and self.task_allocaton == 0:
            self.task_allocaton = 1
            self.task_allocator()
            self.task_allocaton = 2

    def add_obsticle(self, x, y, r=10.0):
        self.sim.add_obstacle([
            [x - r, y - r],
            [x + r, y - r],
            [x + r, y + r],
            [x - r, y + r]
        ])
        self.sim.process_obstacles()


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()