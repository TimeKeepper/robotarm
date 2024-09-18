import math
from maix import uart

device = "/dev/ttyS0"
serial = uart.UART(device, 115200)

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
        serial.write_str('#' + str(self.ID) + 'P' + str(int((angle * 2000 / self.range)) + 500 + self.bias).zfill(4) + 'T' + "1000" + '!')

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
    print(theta_1, theta_2, theta_3)

def Grip_bite():
    Grip.move(-0.85)

def Grip_open():
    Grip.move(0)

def Grip_close():
    Grip.move(-math.pi/2)

Grip_open()

matr = [5.0, 0.0]

inverse_kinematics(matr[0], matr[1], -math.pi / 3)

from maix import camera, display, image, nn, app, time

detector = nn.YOLOv5(model="/root/models/block.mud", dual_buff=False)

cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
dis = display.Display()

while not app.need_exit():
    img = cam.read()
    objs = detector.detect(img, conf_th = 0.5, iou_th = 0.45)
    for obj in objs:
        if detector.labels[obj.class_id] == "red":
            poi_y = obj.y + obj.h / 2
            poi_x = obj.x + obj.w / 2
            bias_y = poi_y - (detector.input_height() / 2)
            bias_x = poi_x - (detector.input_width() / 2)
            if abs(bias_y) > detector.input_height() / 10:
                matr[0] -= 0.01 * (poi_y - (detector.input_height() / 2))
                inverse_kinematics(matr[0], matr[1], -math.pi / 3)
            if (abs(bias_x) > detector.input_width() / 10):
                matr[1] -= 0.01 * (poi_x - (detector.input_width() / 2))
            print(matr)
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color = image.COLOR_RED)
        msg = f'{detector.labels[obj.class_id]}: {obj.score:.2f}'
        img.draw_string(obj.x, obj.y, msg, color = image.COLOR_RED)
    display.send_to_maixvision(img)
    time.sleep_ms(300)
    # dis.show(img)

