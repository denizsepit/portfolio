%% ELECTROMECHANICAL DRIVE SYSTEM SIMULATION (PMSM / TRACTION DRIVE)
% Author: M. Deniz Sepit — Electrical & Electronics Engineering (deniz.sepit@std.ieu.tr)
clear; clc; close all;

% 1. Electromechanical & Machine Parameters
p_poles = 8; Rs = 0.42; Ld = 4.8e-3; Lq = 5.2e-3;
psi_pm  = 0.38; J_eq = 1.85; B_visc = 0.12; r_sheave = 0.24;
zeta    = 0.75; wn = 6.0;

% 2. S-Curve Jerk-Limited Kinematic Profile
v_max = 2.5; a_max = 1.1; T_sim = 8.0; dt = 0.005;
t = (0:dt:T_sim)'; N = length(t);
t_ramp = v_max / a_max; t_cruise = max(t_ramp + 1.5, 5.2);
v_ref = zeros(N,1); a_ref = zeros(N,1);

for i = 1:N
    ti = t(i);
    if ti < t_ramp
        phase = (pi * ti) / t_ramp;
        v_ref(i) = 0.5 * v_max * (1 - cos(phase));
        a_ref(i) = (0.5 * pi * v_max / t_ramp) * sin(phase);
    elseif ti < t_cruise
        v_ref(i) = v_max; a_ref(i) = 0;
    elseif ti < (t_cruise + t_ramp)
        td = ti - t_cruise; phase = (pi * td) / t_ramp;
        v_ref(i) = 0.5 * v_max * (1 + cos(phase));
        a_ref(i) = -(0.5 * pi * v_max / t_ramp) * sin(phase);
    end
end

% 3. 4th-Order Runge-Kutta (RK4) Numerical ODE Solver
f_ode = @(vel, acc, u_ref) wn^2 * (u_ref - vel) - 2 * zeta * wn * acc;
v_motor = zeros(N,1); a_motor = zeros(N,1);

for i = 1:(N-1)
    y_c = v_motor(i); dy_c = a_motor(i);
    u_c = v_ref(i); u_m = 0.5*(v_ref(i)+v_ref(i+1)); u_n = v_ref(i+1);
    k1_y = dy_c;              k1_dy = f_ode(y_c, dy_c, u_c);
    k2_y = dy_c+0.5*dt*k1_dy; k2_dy = f_ode(y_c+0.5*dt*k1_y, dy_c+0.5*dt*k1_dy, u_m);
    k3_y = dy_c+0.5*dt*k2_dy; k3_dy = f_ode(y_c+0.5*dt*k2_y, dy_c+0.5*dt*k2_dy, u_m);
    k4_y = dy_c+dt*k3_dy;     k4_dy = f_ode(y_c+dt*k3_y, dy_c+dt*k3_dy, u_n);
    v_motor(i+1) = y_c  + (dt/6)*(k1_y  + 2*k2_y  + 2*k3_y  + k4_y);
    a_motor(i+1) = dy_c + (dt/6)*(k1_dy + 2*k2_dy + 2*k3_dy + k4_dy);
end

% 4. Electromagnetic Torque & Stator/Core Losses
omega_m  = v_motor / r_sheave; alpha_m = a_motor / r_sheave;
T_em     = J_eq * alpha_m + B_visc * omega_m + 145.0 * (v_motor > 0.01);
i_q      = T_em / (1.5 * p_poles * psi_pm);
P_copper = 3 * (i_q / sqrt(2)).^2 * Rs;
f_elec   = (p_poles * omega_m) / (2 * pi);
P_core   = 1.8 * f_elec + 0.012 * f_elec.^2;

figure('Color','w');
subplot(2,1,1); plot(t, v_ref, '--r', t, v_motor, '-k', 'LineWidth', 1.8);
grid on; ylabel('Velocity [m/s]'); title('S-Curve Velocity Tracking (RK4)');
subplot(2,1,2); plot(t, T_em, '-b', 'LineWidth', 1.8);
grid on; xlabel('Time [s]'); ylabel('Torque [N*m]'); title('Electromagnetic Torque Profile');
