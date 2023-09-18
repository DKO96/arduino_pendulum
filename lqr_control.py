#! usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import control as ct
from scipy.integrate import solve_ivp


def lqr():

    l_b = 0.075     # [m] distance between the com of the pendulum body and the pivot point
    l_w = 0.085     # [m] distance between the motor axis and the pivot point
    m_b = 0.419     # [kg] mass of the pendulum body
    m_w = 0.204     # [kg] mass of the wheel
    I_b = 3.34e-3   # [kg*m^2] moment of inertia of the pendulum body around the pivot point
    I_w = 0.57e-3   # [kg*m^2] moment of inertia of the wheel and the motor around the rotational axis of the motor
    C_b = 1.02e-3   # [kg*m^2*s^-1] dynamic friction coefficients of the pendulum body
    C_w = 0.05e-3   # [kg*m^2*s^-1] dynamic friction coefficients of the wheel
    g = 9.81        # [m*s^-2] gravitational constant

    K_m = 25.1e-3   # [Nm*A^-1] torque constant of bldc motor
    min_torque = -0.1367
    max_torque = 0.1367


    M = m_b*l_b + m_w*l_w
    I = I_b + m_w*l_w*l_w

    # State space representation
    A = np.array([[0, 1, 0],
                  [ M*g/I, -C_b/I, C_w/I],
                  [-M*g/I,  C_b/I, -C_w*(I_b + I_w + m_w*l_w**2)/(I_w*I)]])

    B = np.array([[0], 
                  [-K_m/I], 
                  [K_m*(I_b + I_w + m_w*l_w**2)/(I_w*I)]])

    # LQR tuning matrix
    theta_b = 1
    omega_b = 1
    omega_w = 1
    Q = np.array([[theta_b, 0, 0],    # penalize pendulum angular error
                  [0, omega_b, 0],    # penalize pendulum angular rate
                  [0, 0, omega_w]])   # penalize wheel angular rate
    
    R = 10                       # penalize control torque

    # LQR control gains
    K, S, E = ct.lqr(A, B, Q, R)
    print(f"LQR Gains: {K}")

    # simulation
    tspan = np.linspace(0,5,1000)
    x0 = np.array([0.5, 0, 0])
    wr = np.array([0, 0, 0])
    # wr = np.array([0, 0, desired_wheel_speed])
    u = lambda x: -K @ (x - wr)

    def invertpend(t, x, u_func):
        theta_b, omega_b, omega_w = x
        u_val = u_func(x)

        alpha_b = (M*g*np.sin(theta_b) - K_m*u_val - C_b*omega_b + C_w*omega_w) / I
        alpha_w = (((I + I_w) * (K_m*u_val - C_w*omega_w)) / (I_w*I)) - ((M*g*np.sin(theta_b) - C_b*omega_b) / I)

        return np.hstack([omega_b, alpha_b, alpha_w])

    sol = solve_ivp(invertpend, [0, 5], x0, args=(u,), t_eval=tspan)
    # control_input = np.apply_along_axis(u, 0, sol.y)
    raw_control_input = np.apply_along_axis(u, 0, sol.y)
    control_input = np.clip(raw_control_input, min_torque, max_torque)

    df = pd.DataFrame({
        'time': sol.t,
        'theta_b': sol.y[0,:],
        'omega_b': sol.y[1,:],
        'omega_w': sol.y[2,:],
        'torque': control_input[0]
    })

    # plots
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)
    ax1.plot(df['time'].values, df['theta_b'].values, color='red', linewidth=2.5)  
    ax1.set_title('Pendulum angular error')
    ax1.set(ylabel='angular error [rads]') 
    
    ax2.plot(df['time'].values, df['omega_b'].values, color='green', linewidth=2.5)   
    ax2.set_title('Pendulum angular velocity') 
    ax2.set(ylabel='angular velocity [rads/s]') 
    
    ax3.plot(df['time'].values, df['omega_w'].values, color='purple', linewidth=2.5)   
    ax3.set_title('Wheel angular velocity') 
    ax3.set(ylabel='angular velocity [rads/s]') 
    
    ax4.plot(df['time'].values, df['torque'].values, color='blue', linewidth=2.5)   
    ax4.set_title('Control torque') 
    ax4.set(ylabel='torque [Nm]', xlabel='time [s]') 
    plt.show() 


if __name__ == '__main__':
    lqr()