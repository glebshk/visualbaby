import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool, Empty
import peripherals_node.MLX90614 as MLX90614
import peripherals_node.GPIO as GPIO
from cv_bridge import CvBridge

class PeripheralsNode(Node):
    def __init__(self):
        super().__init__('peripherals_node')
        self.get_logger().info('Peripherals node started')
        self.declare_parameter("env", "real")
        self.MLX90614_publisher_obj_temp = self.create_publisher(Float32, '/MLX90614/obj_temp', 10)
        self.MLX90614_publisher_amb_temp = self.create_publisher(Float32, '/MLX90614/amb_temp', 10)
        self.cv_bridge = CvBridge()
        self.init_timer = self.create_timer(0.1, self.init_after_init) 

    def init_after_init(self):
        self.destroy_timer(self.init_timer)
        self.env = self.get_parameter("env").value
        if self.env == "real":
            self.MLX90614_sensor = MLX90614.MLX90614()
            self.MLX90614_real_timer = self.create_timer(1, self.MLX90614_real_callback)
            self.gpio_controller = GPIO.GPIOController()
            self.create_subscription(Bool, "/cone/servo_state", self.cone_servo_callback_real, 10)
        
        elif self.env == "simulation":
            self.get_logger().info("Simulation environment selected")
            
            self.create_subscription(
                msg_type=Image,
                topic='/MLX90614/thermal_image',
                callback=self.MLX90614_simulation_callback,
                qos_profile=10
            )
            self.create_subscription(Bool, "/Cone/servo_state", self.cone_servo_callback_simulation, 10)
    
    def cone_servo_callback_real(self, msg):
        self.get_logger().debug(f"Cone servo state received: {msg.data}")
        if msg.data:
            self.get_logger().info("Attaching cone servo")
            self.gpio_controller.attach_cone_servo()
        else:
            self.get_logger().info("Detaching cone servo")
            self.gpio_controller.detach_cone_servo()

    def cone_servo_callback_simulation(self, msg):
        self.get_logger().debug(f"Cone servo state received for simulation: {msg.data}")
        if not msg.data:
            self.get_logger().info("Publishing detach command to Gazebo")
            detach_msg = Empty()
            self.create_publisher(Empty, "/cone/detach", 10).publish(detach_msg)

    def MLX90614_real_callback(self):
        self.get_logger().debug("MLX90614 real callback called")
        object_temp = self.MLX90614_sensor.get_obj_temp()
        ambient_temp = self.MLX90614_sensor.get_amb_temp()
        self.get_logger().debug(f"Object Temp: {object_temp}, Ambient Temp: {ambient_temp}")
        self.MLX90614_publisher_obj_temp.publish(Float32(data=object_temp))
        self.MLX90614_publisher_amb_temp.publish(Float32(data=ambient_temp))

    def MLX90614_simulation_callback(self, msg):
        self.get_logger().debug("MLX90614 simulation callback ")
        # Convert ROS Image message to OpenCV image
        temp_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

        avr_temp = temp_image.mean() * 0.01 - 273.15 # Kelvin to Celsius conversion
        self.get_logger().debug(f"Average Object Temp from Simulation: {avr_temp}")
        self.MLX90614_publisher_obj_temp.publish(Float32(data=avr_temp))


def main(args=None):
    rclpy.init(args=args)
    node = PeripheralsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
