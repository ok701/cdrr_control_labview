import numpy as np

time_index = 0  
current_pos = np.array([0.0, 0.0])  
current_vel = np.array([0.0, 0.0])   

m = 1
b = 10
T_min = 0.0
T_max = 100.0
motor_pos = np.array([[-1.010, -0.490],
                      [ 1.820, -0.470],
                      [ 0.13,  1.66]])

def get_Jacobian(pos):
    """
    Compute the 2x3 Jacobian whose columns are unit vectors from the motor positions to pos.
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

def step_simulation(T_in, dt_ms, human_force):
    """
    Advance the physics simulation by one step using the external tension input T_in (length 3).
    
    Returns:
    (t, x, y, T_list):
    t       : Step time (time_index * dt)
    x, y    : Updated current position
    T_list  : Actual tension used (after clipping)
    """
    global time_index
    global current_pos, current_vel

    # Step time
    dt = dt_ms / 1000.0  # ms to s
    t = time_index * dt

    T_array = np.array(T_in, dtype=float)
    # T_array = np.clip(T_array, T_min, T_max)


    J = get_Jacobian(current_pos)
    F_tension = J @ T_array
    F_damping = -b * current_vel
    F_human = np.array([-human_force, human_force])
    F_total = F_tension + F_damping + F_human   

    acc = F_total / m

    current_vel += acc * dt
    current_pos += current_vel * dt

    time_index += 1
    # position to mm
    return (float(current_pos[0] * 1000),
            float(current_pos[1] * 1000),
            T_array.tolist())
