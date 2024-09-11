# 定义一个机械臂节点类
class RobotArmNode:
    def __init__(self, ID, angle_range):
        self.ID = ID
        self.angle_range = angle_range
    def move(self, angle):
        print("机械臂{}移动到角度{}".format(self.ID, angle))

baseHeigit = 19.00


def anti_movecalculation(position):
    x, y, z = position


