import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as  np
import paho.mqtt.client as mqtt
from hb_interfaces.msg import BotCmd, BotCmdArray

broker_ip = "10.186.39.156"

class TeleopController(Node):
    def __init__(self):
        super().__init__('teleop_controller')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.sub = self.create_subscription(BotCmdArray,'/bot_cmd',self.cmd_callback,10)
        self.cmd_publisher = self.create_publisher(BotCmdArray, "/bot_cmd", 10)
        self.bot_id1 = 2
        self.bot_id2 = 4

        self.alpha_deg = np.array([30, 150, 270])
        self.alpha_rad = np.deg2rad(self.alpha_deg)

        self.ir_msg = 1

        self.get_logger().info('Teleop controller started. Listening to /cmd_vel')

        self.client = mqtt.Client()
        self.client.connect(broker_ip, 1883, 60)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.cli = self.client.subscribe(f"esp/sensor/{self.bot_id1}")
        self.client.loop_start()


    def cmd_vel_callback(self, msg: Twist):

        linear_x = msg.linear.x*1000
        linear_y = msg.linear.y *1000
        angular_z = msg.angular.z*1000
        
        vel_bot = np.array([linear_x, linear_y, angular_z])

        M = np.array([
            [np.cos(self.alpha_rad[0] + np.pi/2), np.cos(self.alpha_rad[1] + np.pi/2), np.cos(self.alpha_rad[2] + np.pi/2)],
            [np.sin(self.alpha_rad[0] + np.pi/2), np.sin(self.alpha_rad[1] + np.pi/2), np.sin(self.alpha_rad[2] + np.pi/2)],
            [1.5789, 1.5789, 1.5789]
            ])
        
        M_inv = np.linalg.inv(M)
        wheel_vel = np.dot(M_inv, vel_bot)

        max_w = 400.0
        normalize = 0
        
        if abs(wheel_vel[0]) > max_w or abs(wheel_vel[1]) > max_w or abs(wheel_vel[2]) > max_w:
            normalize = np.linalg.norm([wheel_vel[0], wheel_vel[1], wheel_vel[2]])
            wheel_vel[0] *= max_w/normalize
            wheel_vel[1] *= max_w/normalize
            wheel_vel[2] *= max_w/normalize

        print(f'{wheel_vel[0]}, {wheel_vel[1]}, {wheel_vel[2]}')
 

        w1 = int(1500 + wheel_vel[0])
        w2 = int(1500 + wheel_vel[1]) 
        w3 = int(1500 + wheel_vel[2])

        cmd_msg = BotCmdArray()
        cmd = BotCmd() 
        cmd.id = self.bot_id1
        cmd.m1 = w1
        cmd.m2 = w2
        cmd.m3 = w3
        cmd.base = 90
        cmd.elbow = 120
        cmd_msg.cmds.append(cmd)
        self.cmd_publisher.publish(cmd_msg)
        

    def on_message(self, client, userdata, msg):
        print(f"[{msg.topic}] {msg.payload.decode()}")
        self.ir_msg = msg.payload.decode()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker")
        client.subscribe(f"esp/sensor/{self.bot_id1}")

    def on_disconnect(self, client, userdata, rc):
        print("Disconnected from broker")

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

            self.get_logger().info(
                f"Bot {bot_id} → {topic} : {motor_msg}"
            )
                



def main():
    rclpy.init()
    node = TeleopController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
