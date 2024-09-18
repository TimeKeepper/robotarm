import math

# 定义一个机械臂节点类
class RobotArmNode:
    def __init__(self, ID, bias, range):
        self.ID = ID
        self.bias = bias
        self.range = range
    def move(self, angle):
        angle = angle + (self.range / 2)
        print("Node" , self.ID , " move to angle " , str((angle - self.range / 2) * 180 / math.pi))
        if ((angle < 0) or (angle > self.range)):
            print("Node move failed: Invalid input")
            return
        print('#' + str(self.ID) + 'P' + str(int((angle * 2000 / self.range)) + 500 + self.bias).zfill(4) + 'T' + "1000" + '!')

Base = RobotArmNode(100, 0, (3 / 2) * math.pi)
Shoulder = RobotArmNode(101, 0, (1) * math.pi)
Elbow = RobotArmNode(102, 40, (3 / 2) * math.pi)
Wrist = RobotArmNode(103, 90, (3 / 2) * math.pi)
Hand = RobotArmNode(104, 0, (3 / 2) * math.pi)
Grip = RobotArmNode(105, 0, (1) * math.pi)

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

    Shoulder.move(theta_1 - (math.pi / 2))
    Elbow.move(theta_2)
    Wrist.move(theta_3)
    print("theta_1 =", theta_1)
    print("theta_2 =", theta_2)
    print("theta_3 =", theta_3)

    return (theta_1, theta_2, theta_3)

import matplotlib.pyplot as plt

def plot_arm(x, y, phi):
    
    result = inverse_kinematics(x, y, phi)
    
    if result is None:
        return

    theta_1, theta_2, theta_3 = result

    if theta_1 is None:
        return
    

    x_1 = l_1 * math.cos(theta_1)
    y_1 = l_1 * math.sin(theta_1)

    x_2 = x_1 + l_2 * math.cos(theta_1 + theta_2)
    y_2 = y_1 + l_2 * math.sin(theta_1 + theta_2)

    x_3 = x_2 + l_3 * math.cos(theta_1 + theta_2 + theta_3)
    y_3 = y_2 + l_3 * math.sin(theta_1 + theta_2 + theta_3)

    plt.plot([0, x_1], [0, y_1], 'b-')
    plt.plot([x_1, x_2], [y_1, y_2], 'b-')
    plt.plot([x_2, x_3], [y_2, y_3], 'b-')

    plt.plot(x, y, 'ro')
    plt.plot(x_3, y_3, 'go')

    plt.axis('equal')
    plt.show()

plot_arm(20, 0, -math.pi/3)

