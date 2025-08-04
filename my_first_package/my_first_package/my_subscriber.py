import rclpy as rp
from rclpy.node import Node
from turtlesim.msg import Pose
# from 패키지.그 안의 모듈 import 그 안의 class
# from 패키지.패키지디렉토리 import 데이터 타입


class TurtlesimSubscriber(Node): #노드를 상속 받는다는 게 뭔 말이야?

    def __init__(self):
        super().__init__('turtlesim_subscriber') 
        #노드의 던더인잇 다 받아옴
        #던더 인잇할 때 'turtlesim_subscriber' 처럼 이름을 적어주면 노드 이름을 지정한 것
        self.subscription = self.create_subscription(
            Pose, #타입
            '/turtle1/pose', #토픽
            self.callback, #그때마다 콜백함수 실행해
            10 #데이터 10개까진 쌓아둬
        )
        self.subscription #prevent unused variable warning

    def callback(self, msg):
        print("X: ", msg.x, ", Y: ", msg.y)

def main(): #매개변수 args를 선언하고 기본값을 None 으로 설정
    #= 함수를 부를 때 아무것도 return 하지 않아도 된다는 뜻.
    rp.init()

    turtlesim_subscriber = TurtlesimSubscriber()
    rp.spin(turtlesim_subscriber)

    turtlesim_subscriber.destroy_node()
    rp.shutdown()
    


if __name__ == '__main__':
    main() # 바로 main문으로 진입
