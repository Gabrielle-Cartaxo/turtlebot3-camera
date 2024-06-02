import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
import time

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')

        #definindo os publishers
        self.publisher_image = self.create_publisher(CompressedImage, '/video_frames', 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        #definindo tempo de processamento 
        self.timer = self.create_timer(0.02, self.timer_callback) 

        #setando webcam, mas se fosse a do robô teria que trocar por 1
        self.cap = cv2.VideoCapture(0)

        #variavel que vai ajudar a calcular a latência
        self.last_publish_time = None

        self.twist = Twist()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # transformando imagem em JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            msg.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
            self.publisher_image.publish(msg)
            
            # Calculo da latência
            current_time = time.time()
            if self.last_publish_time is not None:
                latency = (current_time - self.last_publish_time) * 1000  # Convert to ms
                self.get_logger().info(f'Image latency: {latency:.2f} ms')
            self.last_publish_time = current_time

        # publicar comando de velocidade
        self.publisher_cmd_vel.publish(self.twist)

    def update_twist(self, linear_x=0.0, angular_z=0.0):
        self.twist.linear.x = linear_x
        self.twist.angular.z = angular_z

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
