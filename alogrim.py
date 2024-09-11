l_1 = 10.4
l_2 = 9.9
l_3 = 18.0

import math

def inverse_kinematics(x, y, phi):
    c_2 = (x**2 + y**2 - l_1**2 - l_2**2) / (2 * l_1 * l_2)
    if abs(c_2) > 1:
        print("Invalid input")
        return None
    
    theta_2 = -math.acos(c_2)

    k_1 = l_1 + l_2 * c_2
    k_2 = l_2 * math.sin(theta_2)

    theta_1 = math.atan2(y, x) - math.atan2(k_2, k_1)

    theta_3 = phi - theta_1 - theta_2

    return (theta_1, theta_2, theta_3)

# Testing the function
test = inverse_kinematics(10, 0, 0)

import matplotlib.pyplot as plt

def plot_arm(x, y, phi):

    theta_1, theta_2, theta_3 = inverse_kinematics(x, y, phi)

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


plot_arm(15, 10, -math.pi/6)