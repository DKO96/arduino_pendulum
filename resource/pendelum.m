clear ; clc; close all

m_1 = 0.02; %shaft mass (kg)
m_2 = 0.3; %wheel mass (kg)
L = 0.125; %shaft length (m)
r = 0.05; %wheel radius (m)
I_1 = 1/12*m_1*L^2; %shaft inertia
I_2 = 1/2*m_2*r^2; %wheel inertia

g = 9.81; %gravity

I_0 = I_1 + I_2 + 1/4*m_1*L^2 + m_2*L^2;
m_0 = 1/2*m_1 + m_2;

A = [0, 1, 0;
    m_0*L*g/(I_0 - I_2), 0, 0;
    -m_0*L*g/(I_0 - I_2), 0, 0];

B = [0; -1/(I_0 - I_2); 1/I_2 + 1/(I_0 - I_2)];

tspan = [0 20];
iniCon = [2; 0; 0];
[t, y] = ode45(@(t, y) sys(t, y, A, B), tspan, iniCon);

tau = zeros(length(t), 1);

for i = 1:length(t)
    [dy, tau1] = sys(t(i), y(i, :)', A, B);
    tau(i) = tau1;
end

figure()
plot(t, y(:, 1));
xlabel('time (s)')
ylabel('\theta (rad)')

maximum_torque = max(tau)
minimum_torque = min(tau)

function [dy, tau] = sys(t, y, A, B)

K_p = 0.5;
K_d = 0.2;

u = K_p*y(1) + K_d*y(2);
K_m = 1;
tau = K_m*u;

dy = A*y + B*tau;

end
