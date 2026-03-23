function XDOT = ATABEY_dynamics(X, U)

%-----------------------SABITLER-------------------------------
m = 2.5;                      % Toplam kutle (kg)
cbar = 0.30;                  % Ortalama aerodinamik kord
S = 0.39;                     % Kanat alani 
b = 1.30;                     % Kanat acikligi

Xcg = 0.216; Ycg = 0; Zcg = 0.04;    % Yeni CG konumlari
Xac = 0.225; Yac = 0; Zac = 0;       % Yeni AC konumlari

% Motor
% Sunnysky 3520 520kv & 14*8 Pervane verilerine gore set edildi
Umax = 26.1;

rho = 1.225;
g = 9.81;

%-------------------KONTROL GIRDIRLERI-------------------------
u1 = U(1); % d_A
u2 = U(2); % d_e
u3 = U(3); % d_th

% Trim/Reflex ayari
de_trim = deg2rad(-5.8);
u2_eff = u2 + de_trim;

%-------------------DURUM DEGISKENLERI-------------------------
u = X(1); v = X(2); w = X(3);
p = X(4); q = X(5); r = X(6);
phi = X(7); theta = X(8); psi = X(9);

Va = sqrt(u^2 + v^2 + w^2);
alpha = atan2(w,u);
beta = asin(v/Va);
Q = 0.5*rho*Va^2;

%-------------------AERODINAMIK KUVVETLER----------------------
CL = 4.3 * alpha + 0.15 * u2_eff;
CD = 0.02 + 0.05 * CL^2;
CY = -0.15 * beta;

FA_s = [-CD*Q*S; CY*Q*S; -CL*Q*S];
C_bs = [cos(alpha) 0 -sin(alpha); 0 1 0; sin(alpha) 0 cos(alpha)];
FA_b = C_bs * FA_s;

%-------------------AERODINAMIK MOMENTLER----------------------
wbe_b = [p; q; r];
eta = [0; -0.07; 0]; % Profil dogal momenti

dCMdx = [-0.15*(b/(2*Va))         0                     0;
                0         -0.02*(cbar/(2*Va))           0;
                0                 0             -0.10*(b/(2*Va))];

dCMdu = [0.103   0      0;
          0   0.068    0;
          0      0    0.05];

CMac_b = eta + dCMdx*wbe_b + dCMdu*[u1; u2_eff; 0];

MAac_b = CMac_b*Q*S*cbar;

%--------------CG ETRAFINDA AERODINAMIK MOMENTLER------------------
rcg_b = [Xcg;Ycg;Zcg];
rac_b = [Xac;Yac;Zac];
MAcg_b = MAac_b + cross(FA_b,rcg_b - rac_b);

%-----------------MOTOR KUVVET VE MOMENTI----------------------------
F_engine = u3*Umax;                   
FE_b = [F_engine;0;0];

MEcg_b = cross([Xcg - 0; 0; 0], FE_b);

%-----------------YERCEKIMI VE DINAMIK-------------------------------
Fg_b = m*g*[-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)];

Ib = [0.252 0 0; 0 0.052 0; 0 0 0.301];
invIb = [3.9683,0,0;0,19.2308,0;0,0,3.3223];

XDOT = zeros(1, 9);

XDOT(1:3) = (1/m)*(FA_b + FE_b + Fg_b) - cross(X(4:6), X(1:3));
XDOT(4:6) = invIb * (MAcg_b + MEcg_b - cross(X(4:6), Ib * X(4:6)));

T = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
     0 cos(phi) -sin(phi);
     0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
XDOT(7:9) = T * X(4:6);

XDOT = XDOT';