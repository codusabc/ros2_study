import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist
# from 패키지.그 안의 모듈 import 그 안의 class
# from 패키지.패키지디렉토리 import 데이터 타입


class TurtlesimPublisher(Node):
    # 이 클래스는 /turtle1/cmd_vel 토픽을 500ms마다 한 번씩
    # msg.linear.x = 2.0, msg.angular.z = 2.0 로 바꿔서 발행함

    def __init__(self):
        super().__init__('turtlesim_publisher') 
        
        self.publisher = self.create_publisher(
            Twist, #타입
            '/turtle1/cmd_vel', #토픽
            10 #데이터 10개까진 쌓아둬
        )
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 2.0
        self.publisher.publish(msg)


def main(): 
    rp.init()

    turtlesim_publisher = TurtlesimPublisher()
    rp.spin(turtlesim_publisher)

    turtlesim_publisher.destroy_node()
    rp.shutdown()
    


if __name__ == '__main__':
    main() # 바로 main문으로 진입
