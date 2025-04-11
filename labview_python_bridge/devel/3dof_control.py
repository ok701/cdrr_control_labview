# filename: cable_robot_step.py

import numpy as np
from scipy.optimize import lsq_linear

# ------------------------
# 전역 변수 (시뮬레이션 상태)
# ------------------------
initialized = False
time_index = 0
time_array = None

current_pos = None
current_vel = None
pos_log = []
ref_log = []
T_log = []

# 파라미터들
m = 1.0
kp = 10.0
kd = 0.0
b = 5.0
T_min = 20.0
T_max = 70.0
motor_pos = np.array([[0.0, 0.0],
                      [2.0, 0.0],
                      [1.0, 2.0]])
r = 0.3
omega = np.pi / 5
center = np.array([1.0, 1.0])
dt = 0.05

def init_simulation(t_final=10.0):
    global initialized, time_index, time_array
    global current_pos, current_vel
    global pos_log, ref_log, T_log
    
    # 상태 초기화
    time_index = 0
    time_array = np.arange(0.0, t_final + dt, dt)

    current_pos = center.copy()
    current_vel = np.array([0.0, 0.0])

    pos_log = []
    ref_log = []
    T_log = []

    initialized = True
    return "Init Done!"

def get_Jacobian(pos):
    J = np.zeros((2, 3))
    for i in range(3):
        vec = motor_pos[i] - pos
        norm_vec = np.linalg.norm(vec)
        if norm_vec > 1e-12:
            J[:, i] = vec / norm_vec
        else:
            J[:, i] = np.zeros(2)
    return J

def calc_tension_robust(F_des, J):
    res = lsq_linear(J, F_des, bounds=(T_min, T_max), method='trf', lsmr_tol='auto', verbose=0)
    if not res.success:
        T_unbounded, _, _, _ = np.linalg.lstsq(J, F_des, rcond=None)
        return np.clip(T_unbounded, T_min, T_max)
    return res.x

def step_simulation():
    global time_index
    global current_pos, current_vel
    global pos_log, ref_log, T_log
    
    if not initialized:
        return "Error: init_simulation() not called"

    if time_index >= len(time_array):
        return "Simulation Done"

    t = time_array[time_index]

    # 레퍼런스 계산
    pos_ref = center + r * np.array([np.cos(omega * t), np.sin(omega * t)])
    vel_ref = r * omega * np.array([-np.sin(omega * t), np.cos(omega * t)])
    ref_log.append(pos_ref.copy())

    # PD 제어
    pos_err = pos_ref - current_pos
    vel_err = vel_ref - current_vel
    F_des = m*(kp*pos_err + kd*vel_err) - b*current_vel

    # Jacobian, Tension
    J = get_Jacobian(current_pos)
    T = calc_tension_robust(F_des, J)
    F_real = J @ T
    acc = F_real / m

    # Update
    current_vel += acc * dt
    current_pos += current_vel * dt

    # logging
    pos_log.append(current_pos.copy())
    T_log.append(T.copy())

    # 인덱스 증가
    time_index += 1

    # 반환할 정보
    # 현재 시점 t, 위치 current_pos, 장력 T
    return (float(t), 
            float(current_pos[0]), 
            float(current_pos[1]),
            T.tolist())  # [T1, T2, T3]
