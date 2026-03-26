function XDOT = ATABEY_dynamics(X, U)

%-----------------------CONSTANTS-------------------------------
m = 2.5;                       %Aircraft total mass (kg)

cbar = 0.3;                     %Mean Aerodynamic Chord (m)
S = 0.38;                       %Wing planform area (m^2)
b = 1.3;                        % Kanat açıklığı (m)

Xcg = 0.216;                    %x position of CoG in Fm (m)
Ycg = 0;                        %y position of CoG in Fm (m)
Zcg = 0.04;                     %z position of CoG in Fm (m)

Xac = 0.225;                    %x position of aerodynamic center in Fm (m)
Yac = 0;                        %y position of aerodynamic center in Fm (m)
Zac = 0;                        %z position of aerodynamic center in Fm (m)

%Engine inputs
Umax  = 26.1;                   %maximum thrust provided by one engine (N)

Xapt = 0;                       %x position of engine force in Fm (m)
Yapt = 0;                       %y position of engine force in Fm (m)
Zapt = 0.04;                    %z position of engine force in Fm (m)

%Other constants
rho = 1.225;                    %Air density (kg/m^3)
g = 9.81;                       %Gravitational acceleration (m/s^2)
alpha_L0 = 0.0523;              %Zero lift angle of attack (rad)
n = 5.5;                        %Slope of linear region of lift slope

%--------------------------STATE VECTOR----------------------------------
%Extract state vector
x1 = X(1);                              %u
x2 = X(2);                              %v
x3 = X(3);                              %w
x4 = X(4);                              %p
x5 = X(5);                              %q
x6 = X(6);                              %r
x7 = X(7);                              %phi
x8 = X(8);                              %theta
x9 = X(9);                              %psi

u1 = U(1);                              %d_A (aileron)
u2 = U(2);                              %d_T (stabilizer)
u3 = U(3);                              %d_th (throttle)
u4 = U(4);

%---------------INTERMEDIATE VARIABLES------------------------
%Calculate airspeed
Va = sqrt(x1^2 + x2^2 + x3^2);

%Calculate alpha and beta
alpha = atan2(x3,x1);
beta = asin(x2/Va);

%Calculate dynamic pressure
Q = 0.5*rho*Va^2;

%Also define the vectors wbe_b and V_b 
wbe_b = [x4;x5;x6];
V_b = [x1;x2;x3];

%---------------AERODYNAMIC FORCE COEFFICIENTS----------------
%Total lift force
CL = 0.15;

%Total drag force (neglecting tail)
CD =  0.025;

%Calculate sideforce
CY = -0.15;

%--------------DIMENSIONAL AERODYNAMIC FORCES---------------------
%Calculate the actual dimensional forces.  These are in F_s (stability axis)
FA_s = [-CD*Q*S;
         CY*Q*S;
        -CL*Q*S];
    
%Rotate these forces to F_b (body axis)
C_bs = [cos(alpha) 0 -sin(alpha);
        0 1 0;
        sin(alpha) 0 cos(alpha)];
    
FA_b = C_bs*FA_s;

%--------------AERODYNAMIC MOMENT ABOUT AC-------------------
%Calculate the moments in Fb.  Define eta, dCMdx and dCMdu
eta11 = 0;
eta21 = -0.09*Q*S*cbar;
eta31 = 0;

eta = [eta11;
       eta21;
       eta31];

dCMdx = [-0.15*(b/(2*Va)),         0,                     0;
                0,         -0.03*(cbar/(2*Va)),           0;
                0,                 0,             -0.12*(b/(2*Va))];

dCMdu = [0.103,   0,      0;   
          0,    0.075,    0;   
          0,      0,    0.05];

%Now calculate CM = [Cl;Cm;Cn] about Aerodynamic center in Fb
CMac_b = eta + dCMdx*wbe_b + dCMdu*[u1;u2;u3];

%OPTIONAL: Covert this to stability axis
C_sb = C_bs';
Mac_b = CMac_b*Q*S*cbar;
Mac_s = C_sb*Mac_b;

%--------------AERODYNAMIC MOMENT ABOUT CG-------------------
%Transfer moment to cg
rcg_b = [Xcg;Ycg;Zcg];
rac_b = [Xac;Yac;Zac];
MAcg_b = C_bs*Mac_s + cross(FA_b,rcg_b - rac_b);

%-----------------ENGINE FORCE & MOMENT----------------------------
%Now effect of engine.  First, calculate the thrust of each engine
F = u4*Umax;

%Assuming that engine thrust is aligned with Fb, we have
FE_b = [F;0;0];
  
%Now engine moment due to offset of engine thrust from CoG.
mew = [Xcg - Xapt;
        Yapt - Ycg;
        Zcg - Zapt];
        
MEcg_b = cross(mew,FE_b);

%--------------------GRAVITY EFFECTS--------------------------------
%Calculate gravitational forces in the body frame.  This causes no moment
%about CoG.
g_b = [-g*sin(x8);
        g*cos(x8)*sin(x7);
        g*cos(x8)*cos(x7)];
  
Fg_b = m*g_b;

%-------------------STATE DERIVATIVES------------------------------
%Inertia matrix
Ib = [0.252 0 0; 0 0.052 0; 0 0 0.301];
    
%Inverse of inertia matrix 
invIb = [3.9683,0,0;0,19.2308,0;0,0,3.3223];

%Form F_b (all the forces in Fb) and calculate udot, vdot, wdot
F_b = Fg_b + FE_b + FA_b;
x1to3dot = (1/m)*F_b - cross(wbe_b,V_b);

%Form Mcg_b (all moments about CoG in Fb) and calculate pdot, qdot, rdot.
Mcg_b = MAcg_b + MEcg_b;
x4to6dot = invIb*(Mcg_b - cross(wbe_b,Ib*wbe_b));

%Calculate phidot,thetadot, and psidot    
H_phi = [1 sin(x7)*tan(x8) cos(x7)*tan(x8);
         0 cos(x7) -sin(x7);
         0 sin(x7)/cos(x8) cos(x7)/cos(x8)];
    
x7to9dot = H_phi*wbe_b;

%Place in first order form
XDOT = [x1to3dot;
        x4to6dot;
        x7to9dot];