import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

    
def sys(y, t, A, b):
    kp = 0.5
    kd = 0.2
    
    u = kp*y[0] + kd*y[1]
    km = 1
    tau = km*u

    dy = np.dot(A,y) + b*tau

    return dy
    
def main():
    m_1 = 0.02
    m_2 = 0.3
    L = 0.125
    r = 0.05
    I_1 = 1/12*m_1*L**2
    I_2 = 1/2*m_2*r**2
    g = 9.81

    I_0 = I_1 + I_2 + 1/4*m_1*L**2 + m_2*L**2
    m_0 = 1/2*m_1 + m_2

    A = np.array([[0, 1, 0],
                  [m_0*L*g/(I_0 - I_2), 0, 0],
                  [-m_0*L*g/(I_0 - I_2), 0, 0]])
    b = np.array([0, -1/(I_0 - I_2), 1/I_2 + 1/(I_0 - I_2)])

    tspan = np.linspace(0, 20, 500)
    iniCon = np.array([2, 0, 0])

    y = odeint(sys, iniCon, tspan, args=(A,b))
    tau = np.array([sys(y[i, :], tspan[i], A, b)[1] for i in range(len(tspan))])

    plt.figure()
    plt.plot(tspan, y[:, 0])
    plt.xlabel('time (s)')
    plt.ylabel('theta (rad)')
    plt.show()

    maximum_torque = np.max(tau)
    minimum_torque = np.min(tau)

    print('Maximum torque:', maximum_torque)
    print('Minimum torque:', minimum_torque)


if __name__ == '__main__':
    main()