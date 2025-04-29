import numpy as np

# ------------------------
# 전역 변수 (시뮬레이션 상태)
# ------------------------
time_index = 0           # 스텝 인덱스
current_pos = np.array([0.0, 0.0])  
current_vel = np.array([0.0, 0.0])   
pos_log = []             # 위치 로그
T_log = []               # 장력 로그

# 파라미터들 (필요한 것만 남김)
m = 1
b = 20
T_min = 0.0
T_max = 100.0
# motor_pos = np.array([[-1.160, -1.000],
#                       [ 1.720, -1.000],
#                       [ 0.00,  1.140]])
motor_pos = np.array([[-1.010, -0.490],
                      [ 1.820, -0.470],
                      [ 0.13,  1.66]])
# dt = 0.01

# ------------------------
# 환경 초기 설정 (직접 대입)
#   - init_simulation 삭제했으므로,
#     외부에서 직접 current_pos, current_vel 등을 원하는 값으로 세팅하면 됨.
#   - 필요하다면 이 부분에 초기화 함수를 추가하세요.
# ------------------------

def get_Jacobian(pos):
    """
    모터 위치로부터 pos까지의 단위 벡터를 열단위로 모은 2x3 Jacobian 계산
    """
    J = np.zeros((2, 3))
    for i in range(3):
        vec = motor_pos[i] - pos
        norm_vec = np.linalg.norm(vec)
        if norm_vec > 1e-12:
            J[:, i] = vec / norm_vec
        else:
            J[:, i] = np.zeros(2)
    return J

def step_simulation(T_in, dt_ms):
    """
    외부에서 받은 장력 T_in(길이 3)을 이용해 물리 시뮬레이션 1스텝 진행.
    
    Returns:
      (t, x, y, T_list):
        t       : 스텝 시간 (time_index * dt)
        x, y    : 업데이트된 현재 위치
        T_list  : 실제 사용된 장력 (clip 적용 후)
    """
    global time_index
    global current_pos, current_vel
    global pos_log, T_log

    # # 만약 current_pos, current_vel이 아직 None이라면 에러 처리할 수도 있음
    # if current_pos is None or current_vel is None:
    #     return (0.0, 0.0, 0.0, [0.0, 0.0, 0.0])

    # 스텝 시간
    dt = dt_ms / 1000.0  # ms to s
    t = time_index * dt

    # 장력 범위 클리핑
    T_array = np.array(T_in, dtype=float)
    # T_array = np.clip(T_array, T_min, T_max)

    # 실제 힘 계산
    J = get_Jacobian(current_pos)
    F_real = J @ T_array - b * current_vel

    # 가속도
    acc = ( F_real / m )

    # 속도 업데이트
    current_vel += acc * dt

    # 위치 업데이트
    current_pos += current_vel * dt

    # 로그 저장
    pos_log.append(current_pos.copy())
    T_log.append(T_array.copy())

    # 스텝 증가
    time_index += 1

    # 결과 반환
    return (float(current_pos[0] * 1000),
            float(current_pos[1] * 1000),
            T_array.tolist())
