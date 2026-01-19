import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from hb_control.srv import Attach 

broker_ip = "172.29.122.156"

class SolenoidService(Node):

    def __init__(self):
        super().__init__('solenoid_service')
        self.srv = self.create_service(Attach, '/attach', self.attach_callback)

        self.client = mqtt.Client()
        self.client.on_disconnect = self.on_disconnect
        self.client.connect(broker_ip, 1883, 60)
        self.client.loop_start()


    def attach_callback(self, request, response):
        bot_id = request.bot_id
        value = 255 if request.attach else 0

        action = "attached" if request.attach else "detached"
        topic = f"esp/sol/{bot_id}"

        self.client.publish(topic, str(value), qos=1)

        self.get_logger().info(
            f"Bot {bot_id}: Crate {action} → MQTT [{topic}] = {value}"
        )

        response.success = True
        response.message = f"Bot {bot_id}: Crate {action}"

        return response

    def on_disconnect(self, client, userdata, rc):
        print("Disconnected from broker")
    


def main():
    rclpy.init()
    node_service = SolenoidService()
    try:
        rclpy.spin(node_service)
    except KeyboardInterrupt:
        pass
    finally:
        node_service.client.disconnect()
        node_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()