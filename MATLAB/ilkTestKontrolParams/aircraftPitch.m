%% Deneme

% Compute dimensional TF from aerodynamic non-dimensional derivatives and FEA Iyy
% Uses: rho, V, S, c, m, Iyy, C_L_alpha, C_m_alpha, C_m_q, C_m_delta

rho = 1.225;
V   = 15;        % m/s
S   = 0.40;      % m^2
c   = 0.30;      % m
m   = 2.72;      % kg
Iyy = 1.031;     % kg.m^2  (FEA result)

C_L_alpha  = 4.8;      % per rad
C_m_alpha  = -0.65;
C_m_q      = -12.5;    % per (q*c/2V)
C_m_delta  = -0.95;

% dynamic pressure
q_inf = 0.5 * rho * V^2;

% dimensional pitch moment coefficients (as they enter q_dot)
M_alpha = (q_inf * S * c * C_m_alpha) / Iyy;
M_q     = (q_inf * S * c * C_m_q) / Iyy * (c / (2*V));   % accounts for q nondim factor
M_delta = (q_inf * S * c * C_m_delta) / Iyy;

% dimensional alpha coefficients (note: pick convention for alpha-dot consistently)
% Here we use Z_alpha = - (q_inf * S * C_L_alpha) / m (no V factor)
Z_alpha = - (q_inf * S * C_L_alpha) / m;

% alpha_dot also depends on q coupling; for your prior linear form we used Z_q ~ geometry*V
% A consistent kinematic coupling (check your original derivation). For compatibility with earlier numeric model:
Z_q = 56.7;        % keep if you accept the earlier scaling; otherwise derive from kinematics
Z_delta = 0.232;   % if you have a dimensional CL_delta, convert similarly

% Assemble state-space A,B,C,D using the chosen alpha convention
% We assume the same structure used earlier (alpha_dot = Z_alpha*alpha + Z_q*q + Z_delta*delta)
A = [ Z_alpha,  Z_q, 0;
      M_alpha,  M_q, 0;
      0,        Z_q, 0 ];

B = [ Z_delta;
      M_delta;
      0 ];

C = [0 0 1];
D = 0;

sys = ss(A,B,C,D);
P_sys = tf(sys);
P_sys

step(P_sys)