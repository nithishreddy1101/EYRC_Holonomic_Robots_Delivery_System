#!/usr/bin/env python3

"""
* Team Id        : 1060
* Author List    : A.Nithish, Rohan A Khamitkar
* Filename       : holonomic_pid_controller.py
* Theme          : Multi-Robot Warehouse Automation
* Functions      :
*   angle_error_rad(),
*   PID.compute(), PID.reset(),
*   HolonomicPIDController.__init__(),
*   pose_bot_cb(), pose_crate_cb(),
*   control_cb0(), control_cb2(), control_cb4(),
*   call_link_service(), callback_service(),
*   rotation_matrix(), goal_reached(), error_stable(),
*   move_servo_towards_target(), wait_done(),
*   publish_wheel_velocities(),
*   inverse_kinematics(), cmd_callback(),
*   task_allocator(), task_allocator_callback(),
*   crate_priority(), box_in_pickup_zone(),
*   check_points(), main()
* Global Variables: broker_ip
"""

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


# ---------------------- MQTT Configuration ----------------------------------
# broker_ip: IP address of MQTT broker used for ESP32 motor commands
broker_ip = "10.37.195.156"

def angle_error_rad(goal_rad, current_rad):
    """
    * Function Name: angle_error_rad
    * Input:
    *   goal_rad    -> Desired angle (radians)
    *   current_rad -> Current angle (radians)
    * Output:
    *   error -> Shortest signed angular error in range [-π, π]
    * Logic:
    *   Handles angle wrapping to avoid discontinuity at 0°/360°
    * Example Call: angle_error_rad(goal_theta, current_theta)
    """ 
    error = goal_rad - current_rad 
    while error > math.pi:
        error -= 2.0 * math.pi
    while error < -math.pi:
        error += 2.0 * math.pi
    return error


# ---------------------- PID Controller Class --------------------------------
class PID:
    """
    Purpose:
    ---
    Implements a PID controller for position/orientation control.
    Separate instances are used for x, y, and theta.
    """
    def __init__(self, kp, ki, kd, max_out=1.0):
        """
        * Function Name: __init__
        * Input:
        *   kp, ki, kd -> PID gains
        *   max_out    -> Maximum allowed output
        * Output: None
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0


    def compute(self, error, dt):
        """
        * Function Name: compute
        * Input:
        *   error -> Current error
        *   dt    -> Time difference (seconds)
        * Output:
        *   v -> PID output (clipped)
        * Logic:
        *   PID = kp*e + ki*∫e dt + kd*(de/dt)
        """
        
        self.integral += error*dt
        derivative = (error - self.prev_error) / dt
        v = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.prev_error = error
        v = max(min(v, self.max_out), -self.max_out)

        return v
    
    
    def reset(self):
        """
        * Function Name: reset
        * Purpose:
        *   Resets PID internal state when switching goals
        """
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Main Node Class -------------------------------------
class HolonomicPIDController(Node):
    """
    Purpose:
    ---
    ROS2 node implementing multi-robot holonomic PID control,
    task allocation, collision avoidance (RVO),
    gripper actuation, and MQTT motor command publishing.
    """
    def __init__(self):
        """
        * Function Name: __init__
        * Input: None
        * Output: None
        * Logic:
        *   Initializes control parameters, PID controllers,
        *   ROS interfaces, MQTT client, and timers.
        """
        super().__init__('holonomic_pid_controller')  # initializing ros node

        self.max_vel = 500.0
        self.last_time = self.get_clock().now()
        self.wait_duration = 1.5
    
        self.current_bot_poses = {} 
        self.crate_poses = {}
        self.wait_start_time = {0: None, 2: None, 4: None}
        self.goal_wait_start_time = {}

        self.undelivered_crate_poses = None
    
        self.tasks = {}

        self.current_angle = {
            0: {"base" : 90, "elbow": 90},
            2: {"base" : 90, "elbow": 90},
            4: {"base" : 90, "elbow": 90}
        }
        self.target_angle = {
            0: {"base" : 10, "elbow": 90},
            2: {"base" : 10, "elbow": 90},
            4: {"base" : 10, "elbow": 90}
        }

        self.color_bias = {
            'red': 0.0,
            'green': 0.0,
            'blue': 0.0
        }

         # ---------------- State Machines ----------------

        self.state_map = {
            0: 0,
            2: 0,
            4: 0
        }

        self.current_index_map = {
            0: 0,
            2: 0,
            4: 0
        }

        self.assigned_crates = [] #Stores assigned crates

         # ---------------- Drop Zones ----------------

        self.drop_zones = {"red":(1215, 1115, 0.0),
                  "green":(820, 2017.5, 0.0), 
                  "blue":(1616, 2015.5, 0.0)}
        
        self.alter_routes  = {"red":(1215, 700.0, 0.0),
                  "green":(680.0, 1000, 0.0), 
                  "blue":(1740, 1040, 0.0)}
        
        # ---------------- Goal Definitions ----------------
        
        self.goals_map = {0:[(0, 0, 0),(0, 0, 0),
                            (680.0, 1000.0, 0.0),(1218.2, 225.2, 0.0)], 
                        2:[(0, 0, 0),(0.0, 0.0, 0.0),
                            (680.0, 1000, 0.0),(1568.0, 225.2, 0.0)], 
                       4:[(0, 0, 0),(0.0, 0.0, 0.0),
                            (680.0, 1000, 0.0), (864.0, 225.2, 0.0)]}
        
        self.orientation = {
            0: [0, 0, 0, 0],
            2: [0, 0, 0, 0],
            4: [0, 0, 0, 0]

        }

        self.wait_duration_map = {
            0: 1.5, 
            2: 1.5,
            4: 1.5
        }
    
        self.unassigned_bots = [0, 2, 4]

        self.angle_maps = {
            0: None,
            2: None,
            4: None
        }
        self.angle_maps_1 = {
            0: None,
            2: None,
            4: None
        }

        self.task_allocated = {
            0: False,
            2: False,
            4: False
        }

        self.time_shift = {
            "red":0,
            "green":0,
            "blue":0
        }

                            #(x_min, y_min), (x_max, y_max)
        self.pickup_zones = [((172, 542),(376, 1095)),   #P1 
                            ((2062, 542),(2265, 1095)),  #P2
                            ((172, 1342),(376, 1895)),   #P3
                            ((2062, 1342),(2265, 1895))] #P4
        
        # ---------------- RVO Simulator ----------------

        self.sim = pyrvo.RVOSimulator(0.1, 300, 3, 5000,5000, 160, 500.0)

        self.agent_0 = self.sim.add_agent([1218.2, 205.2])
        self.agent_2 = self.sim.add_agent([1568.0, 205.2])
        self.agent_4 = self.sim.add_agent([864.0, 204.2])

        self.preferred_velocity = np.array([-50.0, 0.0])
        self.position = np.array([0.0, 0.0])
        
        self.pid_params0 = {
            'x': {'kp': 12.0, 'ki': 0.000, 'kd':0 , 'max_out': self.max_vel},
            'y': {'kp': 13.8, 'ki': 0.000, 'kd': 00, 'max_out': self.max_vel},
            'theta': {'kp': 1000.0, 'ki': 0.000, 'kd': 00, 'max_out': self.max_vel*2 }
        }

        self.pid_params2 = {
            'x': {'kp': 12.0, 'ki': 0.000, 'kd':0 , 'max_out': self.max_vel},
            'y': {'kp': 13.8, 'ki': 0.000, 'kd': 0, 'max_out': self.max_vel},
            'theta': {'kp': 1000.0, 'ki': 0.000, 'kd': 00.0, 'max_out': self.max_vel*2 }
        }

        self.pid_params4 = {
            'x': {'kp': 12.0, 'ki': 0.000, 'kd':00 , 'max_out': self.max_vel},
            'y': {'kp': 13.8, 'ki': 0.000, 'kd': 00, 'max_out': self.max_vel},
            'theta': {'kp': 1000.0, 'ki': 0.0, 'kd': 00.0, 'max_out': self.max_vel*2 }
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

         # ---------------- MQTT Setup ----------------
        self.client = mqtt.Client()

        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.connect(broker_ip, 1883, 60)
        self.client.loop_start()
        
        # ---------------- Timer for Control Loop ----------------
        self.timer0 = self.create_timer(0.05, self.control_cb0)  # ~50ms = 20 Hz
        self.timer2 = self.create_timer(0.05, self.control_cb2)  # ~50ms = 20 Hz
        self.timer4 = self.create_timer(0.05, self.control_cb4)  # ~50ms = 20 Hz
        self.task_timer = self.create_timer(1.5, self.task_allocator_callaback) # ~1.0s = 1Hz

        self.attach_cli = self.create_client(Attach, "/attach")
        self.detach_cli = self.create_client(Attach, "/attach")

        self.get_logger().info(f'Holonomic PID Controller started')

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
        '''
        Purpose:
        ---
        Callback function for /crate_pose topic.
        Updates the current robot pose whenever a new pose message arrives.
        '''
        poses = msg.poses
        if not poses:
            self.crate_poses = {} 
            return
        
        self.crate_poses = {
            pose.id: (pose.x, pose.y, pose.w)
            for pose in poses
        }


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
                id=0, m1=1500, m2=1500, m3=1500, base=90, elbow=180)]))  
            self.get_logger().info("No bot poses received yet.")
            return
        
        if not self.task_allocated[bot_id]:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now
        pose_x, pose_y, pose_w = self.current_bot_poses[bot_id]
        position = np.array([pose_x, pose_y])
    
        if self.current_index_map[bot_id] >= len(self.goals_map[bot_id]):
            self.sim.set_agent_pref_velocity(self.agent_0, [0, 0])
            self.sim.set_agent_position(self.agent_0, tuple(position))
            self.sim.do_step()
            self.cmd_publisher.publish(BotCmdArray(cmds=[BotCmd(
                id=0, m1=1500, m2=1500, m3=1500, base=90, elbow=180)]))  
            self.task_allocated[bot_id] = False
            return
      
        goal_x, goal_y, goal_w = self.goals_map[bot_id][self.current_index_map[bot_id]]
        goal_w = self.orientation[bot_id][self.current_index_map[bot_id]]

        goal_w = np.deg2rad(goal_w)

        pose_x-=20

        error = [goal_x - pose_x,
                goal_y - pose_y,
                angle_error_rad(goal_w, pose_w)]
        
        error_x, error_y = self.rotation_matrix(error[0], error[1], pose_w) 

        # print(f"Error: {error_x} {error_y} {error[2]}")

        if self.state_map[bot_id] in [0, 4, 7]:  # motion states
            vx = self.pid_x0.compute(error_x, dt)
            vy = self.pid_y0.compute(error_y, dt)
            vw = self.pid_theta0.compute(error[2], dt)
        else:
            self.pid_x0.reset()
            self.pid_y0.reset()
            self.pid_theta0.reset()
            vx = vy = vw = 0.0

        self.sim.set_agent_pref_velocity(self.agent_0, [vx, vy])
        self.sim.set_agent_position(self.agent_0, tuple(position))
        self.sim.do_step()
        new_velocity = self.sim.get_agent_velocity(self.agent_0)

        vel_bot = np.array([new_velocity.x, new_velocity.y, vw])

        wheel_vel = self.inverse_kinematics(vel_bot)

        # ===================== STATE 0: Move to goal =====================
        if (self.state_map[bot_id] == 0):
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 180)
            if self.error_stable(bot_id, error_x, error_y, error[2]):
                    self.get_logger().info(f"Goal of BOT{bot_id} - {self.current_index_map[bot_id]+1} reached.")
                    self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, 90, 180)

                    self.state_map[bot_id] = 1
                    self.pid_x0.reset()
                    self.pid_y0.reset()
                    self.pid_theta0.reset()

                    self.current_angle[bot_id]["base"] = 90
                    self.target_angle[bot_id]["base"] = 10

                    self.current_index_map[bot_id] += 1
                    return
            
        # ===================== STATE 1: Lower gripper =====================
        if self.state_map[bot_id] == 1:
            done, self.current_angle[bot_id]["base"] = self.move_servo_towards_target(self.current_angle[bot_id]["base"], 
                                                                                      self.target_angle[bot_id]["base"])
            
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, self.current_angle[bot_id]["base"], 85)
            if done:
                self.state_map[bot_id] = 2
            return
        # ===================== STATE 2: Wait =====================
        if self.state_map[bot_id] == 2:
            if self.wait_done(bot_id):
                self.call_link_service(bot_id, "attach")
                self.state_map[bot_id] = 3
            return
        
        # ===================== STATE 3: Lift gripper =====================
        if self.state_map[bot_id] == 3:
            if self.wait_done(bot_id):
                self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, 90, 90)
                self.state_map[bot_id] = 4
            return  
        
        # ===================== STATE 4: Move to next goal =====================  
        if self.state_map[bot_id] == 4:
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 90)

            if self.error_stable(bot_id, error_x, error_y, error[2]):
                self.get_logger().info(f"Goal of BOT{bot_id} - {self.current_index_map[bot_id] + 1} reached.")

                self.current_index_map[bot_id] += 1

                self.current_angle[bot_id]["base"] = 90
                self.target_angle[bot_id]["base"] = self.angle_maps[bot_id]

                self.current_angle[bot_id]["elbow"] = 90
                self.target_angle[bot_id]["elbow"] = self.angle_maps_1[bot_id]
                self.state_map[bot_id] = 5

            return  
        # ===================== STATE 5: Lower object =====================
        if self.state_map[bot_id] == 5:
            done, self.current_angle[bot_id]["base"] = self.move_servo_towards_target(self.current_angle[bot_id]["base"], self.target_angle[bot_id]["base"])
            done1, self.current_angle[bot_id]["elbow"] = self.move_servo_towards_target(self.current_angle[bot_id]["elbow"], self.target_angle[bot_id]["elbow"])
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, self.current_angle[bot_id]["base"], self.current_angle[bot_id]["elbow"])

            if done and done1:
                self.state_map[bot_id] = 6
                self.wait_duration_map[bot_id] = 1.5
            return      
        # ===================== STATE 6: Detach =====================
        if self.state_map[bot_id] == 6:
            if self.wait_done(bot_id):
                self.call_link_service(bot_id, "detach")

                self.pid_x0.reset()
                self.pid_y0.reset()
                self.pid_theta0.reset()

                self.unassigned_bots.append(bot_id)
                
                self.state_map[bot_id] = 7
            return
        
        # ===================== STATE 7: Final move =====================        
        if (self.state_map[bot_id] == 7):
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 180, mx_w=400.0)
            if self.goal_reached(error):
                self.current_index_map[bot_id] += 1

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
        
        if not self.task_allocated[bot_id]:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        pose_x, pose_y, pose_w = self.current_bot_poses[bot_id]
        position = np.array([pose_x, pose_y])

        if self.current_index_map[bot_id] >= len(self.goals_map[bot_id]):

            self.sim.set_agent_pref_velocity(self.agent_2, [0, 0])
            self.sim.set_agent_position(self.agent_2, tuple(position))
            self.sim.do_step()
            self.cmd_publisher.publish(BotCmdArray(cmds=[BotCmd(
                id=bot_id, m1=1500, m2=1500, m3=1500, base=90, elbow=180)]))
            self.task_allocated[bot_id] = False  
            return
    
        goal_x, goal_y, goal_w= self.goals_map[bot_id][self.current_index_map[bot_id]]
        goal_w = self.orientation[bot_id][self.current_index_map[bot_id]]
        goal_w = np.deg2rad(goal_w)
        # pose_y-=20

        pose_x -= 20 

        error = [goal_x - pose_x,
                 goal_y - pose_y,
                 angle_error_rad(goal_w, pose_w)]
        
        error_x, error_y = self.rotation_matrix(error[0], error[1], pose_w) 


        if self.state_map[bot_id] in [0, 4, 7]:  # motion states
            vx = self.pid_x2.compute(error_x, dt)
            vy = self.pid_y2.compute(error_y, dt)
            vw = self.pid_theta2.compute(error[2], dt)
        else:
            self.pid_x2.reset()
            self.pid_y2.reset()
            self.pid_theta2.reset()
            vx = vy = vw = 0.0

        self.sim.set_agent_pref_velocity(self.agent_2, [vx, vy])
        self.sim.set_agent_position(self.agent_2, tuple(position))
        self.sim.do_step()
        new_velocity = self.sim.get_agent_velocity(self.agent_2)

        vel_bot = np.array([new_velocity.x, new_velocity.y, vw])

        wheel_vel = self.inverse_kinematics(vel_bot)

                # ===================== STATE 0: Move to goal =====================
        if (self.state_map[bot_id] == 0):
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 180)
            if self.error_stable(bot_id, error_x, error_y, error[2]):
                    self.get_logger().info(f"Goal of BOT{bot_id} - {self.current_index_map[bot_id]+1} reached.")
                    self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, 90, 90)
                    self.state_map[bot_id] = 1
                    self.pid_x2.reset()
                    self.pid_y2.reset()
                    self.pid_theta2.reset()

                    self.current_angle[bot_id]["base"] = 90
                    self.target_angle[bot_id]["base"] = 10

                    self.current_index_map[bot_id] += 1

                    return
        # ===================== STATE 1: Lower gripper =====================
        if self.state_map[bot_id] == 1:
            done, self.current_angle[bot_id]["base"] = self.move_servo_towards_target(self.current_angle[bot_id]["base"], self.target_angle[bot_id]["base"])
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, self.current_angle[bot_id]["base"], 90)
            if done:
                self.state_map[bot_id] = 2
            return
        # ===================== STATE 2: Wait =====================
        if self.state_map[bot_id] == 2:
            if self.wait_done(bot_id):
                self.call_link_service(bot_id, "attach")
                self.state_map[bot_id] = 3
            return
        
        # ===================== STATE 3: Lift gripper =====================
        if self.state_map[bot_id] == 3:
            if self.wait_done(bot_id):
                self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, 90, 90)
                self.state_map[bot_id] = 4
            return  
        
        # ===================== STATE 4: Move to next goal =====================  
        if self.state_map[bot_id] == 4:
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 90)

            if self.error_stable(bot_id, error_x, error_y, error[2]):
                self.get_logger().info(f"Goal of BOT{bot_id} - {self.current_index_map[bot_id] + 1} reached.")

                self.current_index_map[bot_id] += 1

                self.current_angle[bot_id]["base"] = 90
                self.target_angle[bot_id]["base"] = self.angle_maps[bot_id]

                self.current_angle[bot_id]["elbow"] = 90
                self.target_angle[bot_id]["elbow"] = self.angle_maps_1[bot_id]
                self.state_map[bot_id] = 5

            return  
        # ===================== STATE 5: Lower object =====================
        if self.state_map[bot_id] == 5:
            done, self.current_angle[bot_id]["base"] = self.move_servo_towards_target(self.current_angle[bot_id]["base"], self.target_angle[bot_id]["base"])
            done1, self.current_angle[bot_id]["elbow"] = self.move_servo_towards_target(self.current_angle[bot_id]["elbow"], self.target_angle[bot_id]["elbow"])
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, self.current_angle[bot_id]["base"], self.current_angle[bot_id]["elbow"])

            if done and done1:
                self.state_map[bot_id] = 6
                self.wait_duration_map[bot_id] = 1.5
            return      
        # ===================== STATE 6: Detach =====================
        if self.state_map[bot_id] == 6:
            if self.wait_done(bot_id):
                self.call_link_service(bot_id, "detach")

                self.pid_x2.reset()
                self.pid_y2.reset()
                self.pid_theta2.reset()

                self.state_map[bot_id] = 7
                self.unassigned_bots.append(bot_id)

            return
        
        # ===================== STATE 7: Final move =====================        
        if (self.state_map[bot_id] == 7):
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 180, mx_w=400.0)
            if self.goal_reached(error):
                self.current_index_map[bot_id] += 1

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
                id=bot_id, m1=1500, m2=1500, m3=1500, base=90, elbow=180)]))  
            self.get_logger().info("No bot poses received yet.")
            return
        
        if not self.task_allocated[bot_id]:
            return
        
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        pose_x, pose_y, pose_w = self.current_bot_poses[bot_id]
        position = np.array([pose_x, pose_y])
    
        if self.current_index_map[bot_id] >= len(self.goals_map[bot_id]):
            self.sim.set_agent_pref_velocity(self.agent_4, [0, 0])
            self.sim.set_agent_position(self.agent_4, tuple(position))
            self.sim.do_step()
            self.cmd_publisher.publish(BotCmdArray(cmds=[BotCmd(
                id=bot_id, m1=1500, m2=1500, m3=1500, base=90, elbow=180)]))  
            self.task_allocated[bot_id] = False
            return
        
      
        goal_x, goal_y, goal_w = self.goals_map[bot_id][self.current_index_map[bot_id]]
        goal_w = self.orientation[bot_id][self.current_index_map[bot_id]]
        goal_w = np.deg2rad(goal_w)
        pose_x-= 20.0
    
        error = [goal_x - pose_x,
                goal_y - pose_y,
                angle_error_rad(goal_w, pose_w)]
        
        error_x, error_y = self.rotation_matrix(error[0], error[1], pose_w) 


        if self.state_map[bot_id] in [0, 4, 7]:  # motion states
            vx = self.pid_x4.compute(error_x, dt)
            vy = self.pid_y4.compute(error_y, dt)
            vw = self.pid_theta4.compute(error[2], dt)
        else:
            self.pid_x4.reset()
            self.pid_y4.reset()
            self.pid_theta4.reset()
            vx = vy = vw = 0.0

        
        vx = self.pid_x4.compute(error_x, dt)
        vy = self.pid_y4.compute(error_y, dt)
        vw = self.pid_theta4.compute(error[2], dt)

        self.sim.set_agent_pref_velocity(self.agent_4, [vx, vy])
        self.sim.set_agent_position(self.agent_4, tuple(position))
        self.sim.do_step()
        new_velocity = self.sim.get_agent_velocity(self.agent_4)

        vel_bot = np.array([new_velocity.x, new_velocity.y, vw])

        wheel_vel = self.inverse_kinematics(vel_bot)

                 # ===================== STATE 0: Move to goal =====================
        if (self.state_map[bot_id] == 0):
            self.publish_wheel_velocities(wheel_vel, bot_id, 85, 180)
            if self.error_stable(4, error_x, error_y, error[2]):
                    self.get_logger().info(f"Goal of BOT{bot_id} - {self.current_index_map[bot_id]+1} reached.")
                    self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, 85, 90)
                    self.state_map[bot_id] = 1
                    self.pid_x4.reset()
                    self.pid_y4.reset()
                    self.pid_theta4.reset()

                    self.current_angle[bot_id]["base"] = 85
                    self.target_angle[bot_id]["base"] = 5
                    self.target_angle[bot_id]["base"] = 5
                    self.current_index_map[bot_id] += 1

                    return
        # ===================== STATE 1: Lower gripper =====================
        if self.state_map[bot_id] == 1:
            done, self.current_angle[bot_id]["base"] = self.move_servo_towards_target(self.current_angle[bot_id]["base"], self.target_angle[bot_id]["base"])
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, self.current_angle[bot_id]["base"], 85)
            if done:
                self.state_map[bot_id] = 2
            return
        # ===================== STATE 2: Wait =====================
        if self.state_map[bot_id] == 2:
            if self.wait_done(bot_id):
                self.call_link_service(bot_id, "attach")
                self.state_map[bot_id] = 3
            return
        
        # ===================== STATE 3: Lift gripper =====================
        if self.state_map[bot_id] == 3:
            if self.wait_done(bot_id):
                self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, 90, 90)
                self.state_map[bot_id] = 4
            return  
        
        # ===================== STATE 4: Move to next goal =====================  
        if self.state_map[bot_id] == 4:
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 90)

            if self.error_stable(bot_id, error_x, error_y, error[2]):
                self.get_logger().info(f"Goal {self.current_index_map[bot_id] + 1} reached.")

                self.current_index_map[bot_id] += 1

                self.current_angle[bot_id]["base"] = 90
                self.target_angle[bot_id]["base"] = self.angle_maps[bot_id]

                self.current_angle[bot_id]["elbow"] = 90
                self.target_angle[bot_id]["elbow"] = self.angle_maps_1[bot_id]
                self.state_map[bot_id] = 5

            return  
        # ===================== STATE 5: Lower object =====================
        if self.state_map[bot_id] == 5:
            done, self.current_angle[bot_id]["base"] = self.move_servo_towards_target(self.current_angle[bot_id]["base"], self.target_angle[bot_id]["base"])
            done1, self.current_angle[bot_id]["elbow"] = self.move_servo_towards_target(self.current_angle[bot_id]["elbow"], self.target_angle[bot_id]["elbow"])
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot_id, self.current_angle[bot_id]["base"], self.current_angle[bot_id]["elbow"])

            if done and done1:
                self.state_map[bot_id] = 6
                self.wait_duration_map[bot_id] = 1.5
            return      
        # ===================== STATE 6: Detach =====================
        if self.state_map[bot_id] == 6:
            if self.wait_done(bot_id):
                self.call_link_service(bot_id, "detach")

                self.pid_x4.reset()
                self.pid_y4.reset()
                self.pid_theta4.reset()
                self.state_map[bot_id] = 7
               
                self.unassigned_bots.append(bot_id)
            return
        
        # ===================== STATE 7: Final move =====================        
        if (self.state_map[bot_id] == 7):
            self.publish_wheel_velocities(wheel_vel, bot_id, 90, 180, mx_w=400)
            if self.goal_reached(error):
                self.current_index_map[bot_id] += 1
                   

    def call_link_service(self, bot_id, action):
        """
        * Function Name: call_link_service
        * Input:
        *   bot_id -> ID of the robot requesting attach/detach
        *   action -> String ("attach" or "detach")
        * Output: None
        * Logic:
        *   Calls the ROS2 Attach service asynchronously to
        *   either attach or detach the crate from the robot.
        * Example Call: call_link_service(0, "attach")
        """

        # -------- Attach Request --------
        if action == "attach":
            # Wait until attach service becomes available
            while not self.attach_cli.wait_for_service(1.0):
                self.get_logger().warn("Waiting for the attach service...")

            # Create attach request
            self.attach_req = Attach.Request()
            self.attach_req.attach = True
            self.attach_req.bot_id = bot_id

            # Call service asynchronously
            future = self.attach_cli.call_async(self.attach_req)

        # -------- Detach Request --------
        elif action == "detach":
            # Wait until detach service becomes available
            while not self.detach_cli.wait_for_service(1.0):
                self.get_logger().warn("Waiting for the detach service...")

            # Create detach request
            self.detach_req = Attach.Request()
            self.detach_req.attach = False
            self.detach_req.bot_id = bot_id

            # Call service asynchronously
            future = self.detach_cli.call_async(self.detach_req)

        # Register callback to handle service response
        future.add_done_callback(partial(self.callback_service))


    def callback_service(self, future):
        """
        * Function Name: callback_service
        * Input:
        *   future -> Service response future
        * Output: None
        * Logic:
        *   Handles success or failure of attach/detach service call.
        * Example Call: Automatically triggered after service call
        """
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))


    def rotation_matrix(self, vx, vy, pose_theta):
        """
        * Function Name: rotation_matrix
        * Input:
        *   vx, vy     -> Velocity components in world frame
        *   pose_theta -> Robot orientation (radians)
        * Output:
        *   (v1, v2) -> Velocity in robot frame
        * Logic:
        *   Applies 2D rotation matrix to transform velocities
        *   from world frame to robot frame.
        * Example Call: rotation_matrix(err_x, err_y, theta)
        """

        # Rotation matrix for transforming velocities
        matrix = np.array([
            [ np.cos(pose_theta),  np.sin(pose_theta)],
            [-np.sin(pose_theta),  np.cos(pose_theta)]
        ])

        # Apply rotation
        v1, v2 = matrix @ np.array([[vx], [vy]])

        return v1[0], v2[0]


    def goal_reached(self, error):
        """
        * Function Name: goal_reached
        * Input:
        *   error -> [x_error, y_error, theta_error]
        * Output:
        *   True if goal reached, else False
        * Logic:
        *   Checks if position and orientation errors
        *   are within acceptable thresholds.
        * Example Call: goal_reached(error)
        """
        return (
            abs(error[0]) < 40.0 and
            abs(error[1]) < 40.0 and
            abs(error[2]) < math.radians(10.0)
        )


    def error_stable(self, bot_id, error_x, error_y, error_theta,
                     pos_thresh=6.0,
                     ang_thresh=math.radians(5.0)):
        """
        * Function Name: error_stable
        * Input:
        *   bot_id       -> Robot ID
        *   error_x/y    -> Position errors
        *   error_theta  -> Orientation error
        * Output:
        *   True if error remains small for a fixed duration
        * Logic:
        *   Ensures the robot stays within error bounds
        *   continuously before declaring success.
        * Example Call: error_stable(0, ex, ey, etheta)
        """

        # Check if errors are within thresholds
        if (abs(error_x) < pos_thresh and
            abs(error_y) < pos_thresh and
            abs(error_theta) < ang_thresh):

            # Start timing when error first becomes small
            if bot_id not in self.goal_wait_start_time:
                self.goal_wait_start_time[bot_id] = self.get_clock().now()
                return False

            # Check elapsed stable time
            elapsed = (
                self.get_clock().now() -
                self.goal_wait_start_time[bot_id]
            ).nanoseconds / 1e9

            return elapsed >= self.wait_duration

        else:
            # Reset timer if error goes out of bounds
            self.goal_wait_start_time.pop(bot_id, None)
            return False


    def move_servo_towards_target(self, current_angle, target_angle, step=1):
        """
        * Function Name: move_servo_towards_target
        * Input:
        *   current_angle -> Current servo angle
        *   target_angle  -> Desired servo angle
        *   step          -> Increment per control step
        * Output:
        *   (done, angle) -> Completion status and updated angle
        * Logic:
        *   Gradually moves servo towards target angle.
        * Example Call: move_servo_towards_target(90, 10)
        """

        # Target already reached
        if current_angle == target_angle:
            return True, current_angle

        # Increment or decrement angle
        if current_angle < target_angle:
            current_angle = min(current_angle + step, target_angle)
        else:
            current_angle = max(current_angle - step, target_angle)

        return False, current_angle


    def wait_done(self, bot_id):
        """
        * Function Name: wait_done
        * Input:
        *   bot_id -> Robot ID
        * Output:
        *   True if wait duration elapsed, else False
        * Logic:
        *   Implements a timed wait state for gripper actions.
        * Example Call: wait_done(0)
        """

        # Initialize wait timer
        if self.wait_start_time[bot_id] is None:
            self.wait_start_time[bot_id] = self.get_clock().now()
            return False

        # Compute elapsed wait time
        elapsed = (
            self.get_clock().now() -
            self.wait_start_time[bot_id]
        ).nanoseconds / 1e9

        if elapsed >= self.wait_duration_map[bot_id]:
            self.wait_start_time[bot_id] = None
            return True

        return False

    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel, bot_id, base_angle, elbow_angle, mx_w=250.0):
        '''
        Purpose:
        ---
        Publishes wheel velocity commands to the robot’s motor controller.
        '''  

        max_w = 250.0
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

    def on_connect(self, client, userdata, flags, rc):
        """
        * Function Name: on_connect
        * Input:
        *   client   -> MQTT client instance
        *   userdata -> User-defined data
        *   flags    -> Response flags from broker
        *   rc       -> Connection result code
        * Output: None
        * Logic:
        *   Callback triggered when MQTT client successfully
        *   connects to the broker.
        * Example Call: Automatically invoked by MQTT client
        """
        self.get_logger().info("Connected to MQTT broker")


    def on_disconnect(self, client, userdata, rc):
        """
        * Function Name: on_disconnect
        * Input:
        *   client   -> MQTT client instance
        *   userdata -> User-defined data
        *   rc       -> Disconnect result code
        * Output: None
        * Logic:
        *   Callback triggered when MQTT client disconnects
        *   from the broker.
        * Example Call: Automatically invoked by MQTT client
        """
        print("Disconnected from broker")


    def inverse_kinematics(self, vel_bot):
        """
        * Function Name: inverse_kinematics
        * Input:
        *   vel_bot -> [vx, vy, omega] robot body velocities
        * Output:
        *   wheel_vel -> [w1, w2, w3] wheel velocities
        * Logic:
        *   Uses inverse kinematics matrix of a
        *   3-wheel holonomic (Kiwi drive) robot.
        * Example Call: inverse_kinematics(vel_bot)
        """

        # Inverse kinematics matrix (robot specific)
        M_inv = np.array([
            [-3.33333333e-01,  5.77350269e-01,  2.11117445e-01],
            [-3.33333333e-01, -5.77350269e-01,  2.11117445e-01],
            [ 6.66666667e-01,  4.27325041e-17,  2.11117445e-01]
        ])

        # Convert body velocities to wheel velocities
        return np.dot(M_inv, vel_bot)


    def cmd_callback(self, msg: BotCmdArray):
        """
        * Function Name: cmd_callback
        * Input:
        *   msg -> BotCmdArray containing motor & servo commands
        * Output: None
        * Logic:
        *   Converts ROS2 bot command messages into
        *   MQTT messages and publishes them to ESP32.
        * Example Call: Automatically called on /bot_cmd topic
        """

        for cmd in msg.cmds:
            bot_id = cmd.id

            # Format motor and servo commands as CSV string
            motor_msg = (
                f"{cmd.m1},"
                f"{cmd.m2},"
                f"{cmd.m3},"
                f"{cmd.base},"
                f"{cmd.elbow}"
            )

            # Publish to bot-specific MQTT topic
            topic = f"bot_cmd/{bot_id}"
            self.client.publish(topic, motor_msg, qos=0)

    def task_allocator(self):

        """
        * Function Name: task_allocator
        * Input: None
        * Output: None
        * Logic:
        *   Allocates pickup and drop tasks to available robots by:
        *     1. Filtering valid crates inside pickup zones
        *     2. Computing distance-based cost matrix
        *     3. Solving optimal assignment using Hungarian algorithm
        *     4. Generating robot-specific goal sequences
        * Example Call: task_allocator()
        """
        # Ensure all robot poses are available and crate data exists
        if  len(self.current_bot_poses) !=3  and self.crate_poses is None:
            self.get_logger().info("No bot poses received yet.")
            return
        
        self.tasks = {}

        undelivered_crate_poses = self.check_points(self.crate_poses)

        if not undelivered_crate_poses:
            self.get_logger().info("No crates are left.")
            return
        
        unassigned_bot_poses = {}

        for bot_id in self.unassigned_bots:
            unassigned_bot_poses[bot_id] = self.current_bot_poses[bot_id]
            # self.wait_duration_map[bot_id] = 1.5

        for color in self.time_shift.keys():
            self.time_shift[color] = 0

        dict_to_sorted_xy_array = lambda d: (
            [k for k, _ in sorted(d.items())],
            np.array([v[:2] for _, v in sorted(d.items())]))

        crate_keys, crate_reshaped_array = dict_to_sorted_xy_array(undelivered_crate_poses)
        bot_keys, bot_reshaped_array     = dict_to_sorted_xy_array(unassigned_bot_poses)
        

        # ---------------- Cost Matrix Computation ----------------
        cost_matrix = np.zeros((len(bot_keys), len(crate_keys)))
        for i in range(len(bot_keys)):
            for j in range(len(crate_keys)):
                dist = np.linalg.norm(bot_reshaped_array[i] - crate_reshaped_array[j])
                priority = self.crate_priority(crate_keys[j])
                cost_matrix[i, j] = dist * priority

        # Solve optimal assignment using Hungarian algorithm
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        assignments = list(zip(row_ind, col_ind))
        self.get_logger().info(f"Assignments (bot -> crate): {assignments} {bot_keys} ")
    
        get_color_num = lambda i: (["red", "green", "blue"][i % 3], i)

        for i in range(len(assignments)):
            key = crate_keys[assignments[i][1]]
            self.tasks[bot_keys[assignments[i][0]]] = (undelivered_crate_poses[key], get_color_num(key))

        self.get_logger().info(f"Tasks: {(self.tasks)}")

        STACK_W = 80
        STACK_H = 10
        
        for bot_id, (goal_pose , (color, crate_num)) in self.tasks.items():
            if color not in self.drop_zones:
                continue

            goal_x, goal_y, goal_w = goal_pose 
            base_x, base_y, base_w = self.drop_zones[color]

            idx = self.color_bias[color] // STACK_W

            if idx < 2:
                x = base_x + idx * STACK_W
                y = base_y - 150.0

                if bot_id == 0:
                    self.angle_maps[bot_id] = 10
                    self.angle_maps_1[bot_id] = 85
                elif bot_id == 2:
                    self.angle_maps[bot_id] = 10
                    self.angle_maps_1[bot_id] = 90
                else:
                    self.angle_maps[bot_id] = 5
                    self.angle_maps_1[bot_id] = 85
            else:
                x = base_x + STACK_W / 2 
                y = base_y - 150 + STACK_H

                if bot_id == 0:
                    self.angle_maps[bot_id] = 45
                    self.angle_maps_1[bot_id] = 120
                elif bot_id == 2:
                    self.angle_maps[bot_id] = 45
                    self.angle_maps_1[bot_id] = 120
                else:
                    self.angle_maps[bot_id] = 40
                    self.angle_maps_1[bot_id] = 120

            self.goals_map[bot_id][0] = (goal_x, goal_y - 180.0 , goal_w)
            self.goals_map[bot_id][1] = (x, y, base_w)
            self.goals_map[bot_id][2] = self.alter_routes [color]
            self.color_bias[color] += 80
            self.unassigned_bots.remove(bot_id)
            self.state_map[bot_id] = 0
            self.current_index_map[bot_id] = 0
            self.task_allocated[bot_id] = True
            self,self.assigned_crates.append(crate_num)

            self.wait_duration_map[bot_id] += self.time_shift[color]
            self.time_shift[color] += 10


        self.get_logger().info(f"Goals: {self.goals_map}")

    def task_allocator_callaback(self):
        """
        * Function Name: task_allocator_callaback
        * Input: None
        * Output: None
        * Logic:
        *   Periodic timer callback that triggers task allocation
        *   only when there are unassigned robots available.
        * Example Call: Automatically called by ROS timer
        """

        # Do nothing if all robots are already assigned
        if not self.unassigned_bots:
            return

        # Allocate tasks to free robots
        self.task_allocator()


    def crate_priority(self, crate_id):
        """
        * Function Name: crate_priority
        * Input:
        *   crate_id -> Unique crate marker ID
        * Output:
        *   priority -> Weight used in task allocation cost
        * Logic:
        *   Determines priority based on crate color
        *   (derived from crate ID modulo).
        * Example Call: crate_priority(5)
        """

        r = crate_id % 3

        if r == 1:        # Green crate
            return 1.0
        elif r == 2:      # Blue crate
            return 1.0
        else:             # Red crate
            return 1.0


    def box_in_pickup_zone(self, point):
        """
        * Function Name: box_in_pickup_zone
        * Input:
        *   point -> (x, y, yaw) of crate
        * Output:
        *   point if inside pickup zone, else None
        * Logic:
        *   Checks whether the crate lies inside any
        *   predefined pickup zone bounding boxes.
        * Example Call: box_in_pickup_zone(crate_pose)
        """

        px, py, _ = point

        # Iterate through all pickup zones
        for (x1, y1), (x2, y2) in self.pickup_zones:

            # Compute bounding box limits
            x_min, x_max = min(x1, x2), max(x1, x2)
            y_min, y_max = min(y1, y2), max(y1, y2)

            # Check if point lies within bounds
            if x_min <= px <= x_max and y_min <= py <= y_max:
                return point

        return None


    def check_points(self, points_dict):
        """
        * Function Name: check_points
        * Input:
        *   points_dict -> Dictionary of crate poses
        * Output:
        *   inside -> Dictionary of valid pickup-zone crates
        * Logic:
        *   Filters crates that:
        *     1. Are not already assigned
        *     2. Lie inside pickup zones
        * Example Call: check_points(self.crate_poses)
        """

        inside = {}
        assigned_set = set(self.assigned_crates)

        for pid, point in points_dict.items():

            # Skip crates already assigned to robots
            if pid in assigned_set:
                continue

            # Check if crate lies in a pickup zone
            if self.box_in_pickup_zone(point):
                inside[pid] = point

        return inside
    
# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()