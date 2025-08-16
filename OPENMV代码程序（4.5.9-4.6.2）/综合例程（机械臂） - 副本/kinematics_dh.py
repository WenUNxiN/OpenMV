import math
import time
from pyb import UART,millis

# ------------------------------------------------------------
# 常量
PI = math.pi
DEG2RAD = PI / 180.0
RAD2DEG = 180.0 / PI

# DH 参数 a, alpha(deg), d, theta(deg)
DH = [
    (0.0,   0.0, 119.0,   0.0),   # J1
    (35.0, -90.0,   0.0,   0.0),   # J2
    (130.0,  0.0,   0.0,  90.0),   # J3
    (45.0, -90.0, 130.0,   0.0),   # J4
    (0.0,   90.0,   0.0,   0.0),   # J5
    (0.0,  -90.0, 170.0,   0.0)    # J6 (工具)
]

# 关节限位 [min, max] (deg)
JOINT_LIMITS = [(-180, 180)] * 6

# ------------------------------------------------------------
# 数据结构
class Pose:
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.x, self.y, self.z = x, y, z
        self.roll, self.pitch, self.yaw = roll, pitch, yaw

class JointAngles:
    def __init__(self):
        self.theta = [0.0] * 6

# ------------------------------------------------------------
# 工具函数
def cos(x): return math.cos(x)
def sin(x): return math.sin(x)
def atan2(y, x): return math.atan2(y, x)
def sqrt(x): return math.sqrt(x) if x > 0 else 0.0
def fabs(x): return abs(x)

# ------------------------------------------------------------
# 逆运动学
def inverse_kinematics(pose: Pose) -> tuple[int, JointAngles]:
    sol = JointAngles()

    # 1. 末端姿态转旋转矩阵 (XYZ 固定轴, 顺序 Rz*Ry*Rx)
    roll  = pose.roll  * DEG2RAD
    pitch = pose.pitch * DEG2RAD
    yaw   = pose.yaw   * DEG2RAD

    cr, sr = cos(roll),  sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    cy, sy = cos(yaw),   sin(yaw)

    R = [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [  -sp,            cp*sr,            cp*cr]
    ]

    # 2. 腕部中心
    tool_len = DH[5][2]
    wx = pose.x - tool_len * R[2][2]
    wy = pose.y - tool_len * R[1][2]
    wz = pose.z - tool_len * R[0][2]

    # 3. 关节1
    theta1 = atan2(wy, wx)
    # 只取主解（如需双解可把 theta1_alt 也计算）
    theta1 = theta1

    # 4. 关节2/3
    a2  = DH[2][0]          # 大臂长
    d4  = DH[3][2]          # 连杆偏移 d4
    a3  = DH[3][0]          # 连杆 a3
    aa2 = sqrt(a3*a3 + d4*d4)

    r_proj = sqrt(wx*wx + wy*wy) - DH[1][0]  # 减去 a1
    h_proj = wz - DH[0][2]

    d13 = sqrt(r_proj*r_proj + h_proj*h_proj)

    cos_theta33 = (r_proj*r_proj + h_proj*h_proj - a2*a2 - aa2*aa2) / (2*a2*aa2)
    if fabs(cos_theta33) > 1.0:
        return 1, None  # 不可达

    theta33 = -atan2(sqrt(1 - cos_theta33*cos_theta33), cos_theta33)
    theta3_ab = atan2(a3, d4)
    theta3 = theta33 + theta3_ab

    gamma = atan2(r_proj, h_proj)
    beta  = atan2(aa2 * sin(theta33), a2 + aa2 * cos(theta33))
    theta2 = gamma + beta

    # 5. 关节4/5/6  (使用 ZYZ 欧拉角分解)
    c1  = cos(theta1)
    s1  = sin(theta1)
    c23 = cos(theta2 + theta3)
    s23 = sin(theta2 + theta3)

    # R03
    R03 = [
        [c1*c23, -c1*s23,  s1],
        [s1*c23, -s1*s23, -c1],
        [   s23,     c23,   0]
    ]

    # R36 = R03^T * R06
    R36 = [[0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            R36[i][j] = sum(R03[k][i] * R[k][j] for k in range(3))

    # 从 R36 提取 ZYZ 欧拉角
    # theta5 = atan2(sqrt(R36[0][2]**2 + R36[1][2]**2), R36[2][2])
    # 简化：直接取主值
    theta5 = math.acos(R36[2][2])
    if fabs(sin(theta5)) > 1e-3:
        theta4 = atan2(R36[1][2], R36[0][2])
        theta6 = atan2(R36[2][1], -R36[2][0])
    else:
        # 奇异位置
        theta4 = 0.0
        theta6 = atan2(-R36[0][1], R36[1][1])

    # 6. 封装结果（转回度）
    sol.theta[0] = theta1  * RAD2DEG
    sol.theta[1] = theta2  * RAD2DEG
    sol.theta[2] = theta3  * RAD2DEG
    sol.theta[3] = theta4  * RAD2DEG
    sol.theta[4] = theta5  * RAD2DEG - 90.0
    sol.theta[5] = theta6  * RAD2DEG

    # 7. 关节限位检查
    for i in range(6):
        if sol.theta[i] < JOINT_LIMITS[i][0] or sol.theta[i] > JOINT_LIMITS[i][1]:
            return 2, None

    return 0, sol
