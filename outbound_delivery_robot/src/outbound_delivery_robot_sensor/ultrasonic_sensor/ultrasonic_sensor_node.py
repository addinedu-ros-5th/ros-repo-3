import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        self.publisher_ = self.create_publisher(Float32, 'ultrasonic_distance', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.trig_pin = 23  # 설정한 GPIO 핀 번호
        self.echo_pin = 24  # 설정한 GPIO 핀 번호
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        
    def timer_callback(self):
        distance = self.get_distance()
        msg = Float32()
        msg.data = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{distance}" cm')
    
    def get_distance(self):
        GPIO.output(self.trig_pin, GPIO.LOW)
        time.sleep(0.1)
        
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, GPIO.LOW)
        
        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()
        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()
        
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)
        
        return distance

    def destroy(self):
        GPIO.cleanup()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
