from my_first_package_msgs.srv import MultiSpawn
import rclpy as rp
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Spawn
import numpy as np # 까먹지 마라
import time # 까먹지 마라

class MultiSpawning(Node):
    def __init__(self):
        super().__init__('multi_spawn')
        self.server = self.create_service(MultiSpawn, 'multi_spawn', self.callback_service)
        self.teleport = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        self.spawn = self.create_client(Spawn, '/spawn')
        self.req_teleport = TeleportAbsolute.Request()
        self.req_spawn = Spawn.Request()
        # turtlesim 의 가운데가 0,0이 아니라서 센터를 지정해줌
        self.center_x = 5.54
        self.center_y = 5.54

    # 괄호 안에 self 까먹지 마라
    def calc_position(self, n, r):
        gap_theta = 2*np.pi/n
        theta = [gap_theta*n for n in range(n)]
        x = [r*np.cos(th) for th in theta]
        y = [r*np.sin(th) for th in theta]

        return x, y, theta

    def callback_service(self, request, response):
        x, y, theta = self.calc_position(request.num, 3)

        for n in range(len(theta)):
            self.req_spawn.x = self.center_x + x[n]
            self.req_spawn.y = self.center_y + y[n]
            self.req_spawn.theta = theta[n]
            self.spawn.call_async(self.req_spawn)
            time.sleep(0.1) #타임 딜레이가 안 맞는데

        response.x=x
        response.y=y
        response.theta=theta

        return response

    
def main(args=None):
    rp.init(args=args)
    multi_spawn = MultiSpawning()
    rp.spin(multi_spawn)
    rp.shutdown()


if __name__ == '__main__':
    main()