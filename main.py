import math
from maix import uart

device = "/dev/ttyS0"
serial = uart.UART(device, 115200)

# 定义一个机械臂节点类
class RobotArmNode:
    def __init__(self, ID, bias):
        self.ID = ID
        self.bias = bias
    def move(self, angle):
        serial.write_str('#' + str(self.ID) + 'P' + str(int((angle * 2000 / math.pi)) + 500 + self.bias).zfill(4) + 'T' + "1000" + '!')
        print('#' + str(self.ID) + 'P' + str(int((angle * 2000 / math.pi)) + 500 + self.bias).zfill(4) + 'T' + "1000" + '!')

Base = RobotArmNode(100, 0)
Shoulder = RobotArmNode(101, 0)
Elbow = RobotArmNode(102, 40)
Wrist = RobotArmNode(103, 90)
Hand = RobotArmNode(104, 0)
Grip = RobotArmNode(105, 0)

baseHeigit = 19.00

l_1 = 10.4
l_2 = 9.9
l_3 = 18.0

def inverse_kinematics(x, y, phi):
    #prepare
    x = x - l_3 * math.cos(phi)
    y = y - l_3 * math.sin(phi)
    
    c_2 = (x**2 + y**2 - l_1**2 - l_2**2) / (2 * l_1 * l_2)
    if abs(c_2) > 1:
        print("Invalid input")
        return None
    
    theta_2 = -math.acos(c_2)

    k_1 = l_1 + l_2 * c_2
    k_2 = l_2 * math.sin(theta_2)

    theta_1 = math.atan2(y, x) - math.atan2(k_2, k_1)

    theta_3 = phi - theta_1 - theta_2

    Shoulder.move(theta_1)
    Elbow.move(theta_2 + math.pi/2)
    Wrist.move(theta_3 + math.pi/2)
    print(theta_1, theta_2, theta_3)

inverse_kinematics(20, -10, -math.pi / 6)
