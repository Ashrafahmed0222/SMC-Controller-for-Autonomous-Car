import numpy as np

def calculate_error(xr, yr, theta_r, xd, yd, theta_d):
    """Calculate error in the local desired frame."""
    delta_x = xr - xd
    delta_y = yr - yd
    delta_theta = theta_r - theta_d

    Rot_desired = np.array([
        [np.cos(theta_d), np.sin(theta_d), 0],
        [-np.sin(theta_d), np.cos(theta_d), 0],
        [0, 0, 1]
    ])
    e_global = np.array([delta_x, delta_y, delta_theta])
    e_local = Rot_desired @ e_global

    return e_local[0], e_local[1], e_local[2]

def calculate_error_derivative(vd, phi_d, vr, phi_r, xe, ye, theta_e):
    """Calculate error derivatives."""
    L = 0.27  # Wheelbase
    xe_dot = -vd + vr * np.cos(theta_e) + ye * (vd / L) * np.tan(phi_d)
    ye_dot = vr * np.sin(theta_e) - xe * (vd / L) * np.tan(phi_d)
    thetae_dot = (vr / L) * np.tan(phi_r) - xe * (vd / L) * np.tan(phi_d)
    return xe_dot, ye_dot, thetae_dot
def map_phi_to_servo(phi):
    """
    Maps the steering angle (phi in radians) to the servo motor's range.

    Parameters:
    - phi: Steering angle in radians (output from the controller).

    Returns:
    - servo_angle: Servo angle in degrees, constrained to [50, 130].
    """
    # Convert phi to degrees
    phi_deg = np.degrees(phi)
    print("deg")
    print(phi_deg)

    # Clip phi_deg to the car's steering range [-40°, 40°]
    phi_deg = np.clip(phi_deg, -40, 40)

    # Map [-40°, 40°] to the servo's range [50°, 130°]
    servo_angle = np.interp(phi_deg, [-40, 40], [50, 130])
    print (servo_angle)

    return servo_angle
def map_to_pwm(v_c, v_min=0, v_max=0.3, pwm_min=0, pwm_max=100):
    """
    Maps a velocity command (v_c) to a PWM range.
    
    Parameters:
    - v_c: The velocity command to be mapped
    - v_min: Minimum velocity value (e.g., -0.3 m/s)
    - v_max: Maximum velocity value (e.g., 0.3 m/s)
    - pwm_min: Minimum PWM value (e.g., 0)
    - pwm_max: Maximum PWM value (e.g., 100)
    
    Returns:
    - Mapped PWM value
    """
    # Ensure v_c is within the expected range
    v_c = saturate(v_c, v_min, v_max)
    
    # Perform linear mapping
    pwm = np.interp(v_c, [0, 0.3], [0, 100])
    
    return pwm
# Update sliding_mode_control to include servo logic if necessary
def sliding_mode_control(vr_dot, xe_dot, ye_dot, thetae_dot, xe, ye, thetae, omegad, vr):
    
    """
    Sliding Mode Controller logic.
    
    Parameters:
    - vr_dot: Desired velocity derivative
    - xe_dot, ye_dot, thetae_dot: Derivatives of the error states
    - xe, ye, thetae: Current errors in position and orientation
    - omegad: Desired angular velocity
    - vr: Reference velocity

    Returns:
    - phi: Steering angle command
    - v_c: Velocity command
    """
    
    # Initialize the control parameters
    L = 0.27  # Wheelbase length
    domegad = 0  # Desired angular velocity derivative (set to zero for simplicity)
    vd_dot = 0  # Desired velocity derivative (set to zero for simplicity)

    # Control gains
    k0 = 0.05
    k1 = 0.25
    k2 = 0.5
    p1 = 1
    p2 = 1
    q1 = 1
    q2 = 1

    # Sliding surfaces
    s1 = xe_dot + k1 * xe
    s2 = ye_dot + k2 * ye + k0 * sgn(ye) * thetae

    # Maximum steering angle limit (rad)
    MAX_delta =MAX_delta = np.radians(30)  # Maximum steering angle in radian
    # Adjusted cosine for steering angle (to avoid division by zero)
    adjusted_cos = np.cos(thetae) if abs(np.cos(thetae)) > 1e-3 else 1e-3

    # Compute velocity command (v_c)
    v_c = 1 / adjusted_cos * (-q1 * s1 - p1 * saturate(s1, -0.05, 0.05) - k1 * xe_dot - domegad * ye
                              - omegad * ye_dot + vr * thetae_dot * np.sin(thetae) + vd_dot)
    v_c = saturate(v_c, 0, 0.3)
    print("{phi} / phi {v_c} /v_c")

    # Compute steering angle (phi)
    
    # Prevent division by zero
    denominator = vr * (vr * np.cos(thetae) + k0 * saturate(ye, -0.05, 0.05))
    denominator = max(denominator, 1e-6)  # Add epsilon to avoid zero

    # Calculate steering angle (phi)
    try:
        phi = np.arctan((L / vr) * omegad +
                        L / denominator *
                        (domegad * xe + omegad * xe_dot))
        phi = saturate(phi, -MAX_delta, MAX_delta)
    except Exception as e:
        phi = 0.0
        phi = saturate(phi, -MAX_delta, MAX_delta)
        print(f"Error in phi calculation: {e}")

    phi = saturate(phi, -MAX_delta, MAX_delta)
    print(phi)
    print (v_c)

    # Map phi to the servo's angle range & Map V_c to the PWM range
    servo_angle = map_phi_to_servo(phi)
    pwm_value = map_to_pwm(v_c, v_min=-0.3, v_max=0.3, pwm_min=0, pwm_max=100)

    return servo_angle, pwm_value

# Helper functions for saturation and sign approximation

def saturate(x, lb, ub):
    """Saturates the value x between a lower bound lb and an upper bound ub."""
    return min(ub, max(lb, x))

def sgn(x):
    """Approximate sign function using tanh to avoid discontinuities."""
    return np.tanh(10 * x)