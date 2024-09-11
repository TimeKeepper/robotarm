import sympy as sp
#定义DH参数
q1,q2,q3,q4,q5=sp.symbols('g1 q2 q3 q4 q5')
a2,a3,d3,d4=sp.symbols('a2 a3 d3 d4')
# DH参数表
dh_params = sp.Matrix(  [[0, 0, 0, q1],
                        [-sp.pi/2, 0, a2, q2],
                        [0, a3, d3, q3],
                        [sp.pi/2, 0, d4, q4],
                        [-sp.pi/2, 0, 0, q5]])

#计算正运动学
def forward_kinematics(dh_params):
    T = sp.eye(4)
    for i in range(dh_params.shape[0]):
        alpha, a ,d ,theta = dh_params[i, :]
        T_i = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)], 
                         [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                         [0 ,sp.sin(alpha) ,sp.cos(alpha), d],
                         [0, 0, 0, 1]])
        T = T * T_i
    return T
#测试正运动学
T = forward_kinematics(dh_params)

# 计算运动学迎解
def inverse_kinematics(T):
    # 许算未端执行器的位置和姿态
    p=T[0:3, 3]
    R=T[0:3, 0:3]

    # 计算关节角度的解
    q1 = sp.atan2(p[1], p[0])
    q3 = sp.Symbol('q3')
    q2=sp.atan2(p[2]-d3, sp.sqrt((p[0]**2 + p[1]**2 - a2**2 - (p[2] - d3)**2)))
    q4=sp.atan2(R[2, 1], R[2, 2])
    q5=sp.atan2(-R[2,0],sp.sqrt(R[0, 0]**2 + R[1,0]**2))

    return sp.Matrix([q1, q2, q3, q4, q5])
# 测试运动学逆解
q = inverse_kinematics(T)
print(q)