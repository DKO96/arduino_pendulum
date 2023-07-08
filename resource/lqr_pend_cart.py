#! usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import control as ct
from scipy.integrate import solve_ivp

m = 1
M = 5 
L = 2
g = -10 
d = 1

b = 1

A = np.array([[0, 1, 0, 0],
              [0, -d/M, b*m*g/M, 0],
              [0, 0, 0, 1],
              [0, -b*d/(M*L), -b*(m+M)*g/(M*L), 0]])
eig, _ = np.linalg.eig(A)
print(f'eig A: {eig}')

B = np.array([[0],
              [1/M],
              [0],
              [b*1/(M*L)]])
print(f'rank: {np.linalg.matrix_rank(ct.ctrb(A, B))}')

Q = np.eye(4)

R = 0.0001

K, S, E = ct.lqr(A, B, Q, R)
print(f'K: {K}')

tspan = np.linspace(0,10,1000)

x0 = np.array([-1, 0, np.pi+0.1, 0])

wr = np.array([1, 0, np.pi, 0])

def pendcart(t, x, m, M, L, g, d, u):
    sin = np.sin(x[2])
    cos = np.cos(x[2])
    D = m*L*L*(M+m*(1-cos**2))
    dx = np.zeros_like(x)
    dx[0] = x[1]
    dx[1] = (1/D)*(-m**2*L**2*g*cos*sin + m*L**2*(m*L*x[3]**2*sin - d*x[1])) + m*L*L*(1/D)*u(x)
    dx[2] = x[3]
    dx[3] = (1/D)*((m+M)*m*g*L*sin - m*L*cos*(m*L*x[3]**2*sin - d*x[1])) - m*L*cos*(1/D)*u(x)
    return dx

def pendcart_ss(t, x, A, B, u):
    return A @ x + B @ u(x)

u = lambda x : -K @ (x - wr)

sol = solve_ivp(pendcart, [0, 10], x0, args=(m, M, L, g, d, u), dense_output=True, t_eval=tspan)
sol_ss = solve_ivp(pendcart_ss, [0, 10], x0, args=(A, B, u), dense_output=True, t_eval=tspan)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 12))

ax1.plot(sol.t, sol.y.T)
ax1.legend(['x', 'v', 'theta', 'omega'])
ax1.set_title('ODE')

ax2.plot(sol_ss.t, sol_ss.y.T)
ax2.legend(['x', 'v', 'theta', 'omega'])
ax2.set_title('State Space')

plt.show()

