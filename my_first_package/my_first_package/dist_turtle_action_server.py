import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from turtlesim.msg import Pose # turtlesim이 발행하는 pose를 구독하기 위해
from geometry_msgs.msg import Twist #cmd_vel 토픽이 발행하는 메시지 타입
from my_first_package_msgs.action import DistTurtle # 우리가 만드는 액션 서버가 사용하는 데이터 타입
from my_first_package.my_subscriber import TurtlesimSubscriber

#파라미터 변경을 실시간으로 알고싶다면 
from rcl_interfaces.msg import SetParametersResult

import math
import time 

class Turtlesub_Action(TurtlesimSubscriber):
    # Turtlesim 이 보내는 pose토픽을 구독하는 아이
    # 구독할 때마다 실행하는 callback함수를 만지작 거린 거
    def __init__(self, ac_server):
        super().__init__()
        self.ac_server = ac_server 
        # ac_server는 ac_server에서 입력 받아온 것. 
        # 여기서 만든 걸 main의 sub이 받아오게 한 것.

    def callback(self, msg): #class 입장에서 오버라이딩 시킴
        self.ac_server.current_pose = msg
        #받아온 거에 current_pose라는 속성을 만들어둠

class DistTurtleServer(Node):
    def __init__(self):
        super().__init__('dist_turtle_action_server')
        self.total_dist = 0 # 거북이가 이동한 총 거리를 저장할 변수
        self.is_first_time = True 
        # 거북이 처음 위치를 잡아주려면 지금 시작하는 곳이 처음 시작하는 곳인지를 알아야함
        self.current_pose = Pose() # 현재 거북이의 pose를 저장할 변수
        self.previous_pose = Pose() # 이전 거북이의 pose를 저장할 변수
        # total_dist를 알려면 현재 pose와 이전 pose를 비교해야함
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.action_server = ActionServer(            
            self,
            DistTurtle,
            'dist_turtle', # Action name            
            self.execute_callback # request가 들어오면 callback 함수 실행
        )

        # 파라미터의 기본값을 선언 / 코드로 파라미터 다루기에서 추가
        self.declare_parameter('quantile_time', 0.75)
        self.declare_parameter('almost_goal_time', 0.95)

        # 파라미터를 가져옴
        # get_parameters는 리스트로 파라미터 이름을 받음 (값 아님, 뒤에 대괄호)
        # 그리고 받아올 변수 이름이 앞에 있는 괄호임 (코드내에서 쓸 변수로 지정해야함)
        (quantile_time, almosts_time) = self.get_parameters(
            ['quantile_time', 'almost_goal_time'])

        # 파라미터 값을 클래스 변수로 저장
        self.quantile_time = quantile_time.value
        self.almost_time = almosts_time.value

        # # .value로 파라미터 값에 접근
        # print('quantile_time and almost_goal_time is', quantile_time.value, almosts_time.value)

        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            print(param.name, "is changed to", param.value)

            # 파라미터 값이 변경되면 해당 값을 업데이트
            if param.name == 'quantile_time':
                self.quantile_time = param.value
            if param.name == 'almost_time':
                self.almost_time = param.value

        return SetParametersResult(successful=True)

    def calc_diff_pose(self):
        if self.is_first_time:
            # 현재 포즈를 이전 포즈에 저장
            self.previous_pose.x = self.current_pose.x
            self.previous_pose.y = self.current_pose.y
            # 이제 움직였으니까 is_first_time을 False로 바꿔줌
            self.is_first_time = False

        # 현재 포즈와 이전 포즈의 차이를 계산 (거리 구하는 공식)
        diff_dist = math.sqrt(
            (self.current_pose.x - self.previous_pose.x) ** 2 +\
            (self.current_pose.y - self.previous_pose.y) ** 2
        )

        # 리턴하기 직전에 현재 포즈를 이전 포즈로 업데이트
        self.previous_pose = self.current_pose

        return diff_dist

    def execute_callback(self, goal_handle):
        feedback_msg = DistTurtle.Feedback() #피드백 선언

        msg = Twist() # 이 밑에 값들은 request 하는 사람이 지정함
        msg.linear.x = goal_handle.request.linear_x 
        msg.angular.z = goal_handle.request.angular_z

        while True:
            self.total_dist += self.calc_diff_pose()
            feedback_msg.remained_dist = goal_handle.request.dist - self.total_dist # 남은 거리 계산
            goal_handle.publish_feedback(feedback_msg) # 피드백 발행
            self.publisher.publish(msg) # 거북이에게 명령을 내림 request에서 받은 값으로
            time.sleep(0.01)

            if feedback_msg.remained_dist < 0.2: # 오차 허용 범위
                break

        goal_handle.succeed() #성공했다 치자
        result = DistTurtle.Result()

        # 액션 서버가 중단 안 됐는데 새 클라이언트 요청이 들어오면 모든 값 초기화
        result.pos_x = self.current_pose.x
        result.pos_y = self.current_pose.y
        result.pos_theta = self.current_pose.theta
        result.result_dist = self.total_dist

        self.total_dist = 0  # Reset total distance for next action
        self.is_first_time = True  # Reset for next action

        return result
    
def main(args=None):
    rp.init(args=args)

    executor = MultiThreadedExecutor() #멀티스레드 가져옴

    ac = DistTurtleServer() #액션 서버 노드
    sub = Turtlesub_Action(ac_server = ac) #구독, 액션 클라이언트 노드
    #위에 ac가 여기에 입력으로 들어간다는 점 유심히 봐야함

    executor.add_node(sub) #두 노드를 스레드 풀에 등록
    executor.add_node(ac)

    try:
        executor.spin()

    finally: #어떤 방식으로든 spin이 끝나면 finally는 무조건 실행됨
        executor.shutdown()
        ac.destroy_node()
        sub.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()