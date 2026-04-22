function XDOT = ATABEY_dynamics(X, U)

% Christopher Lum - RCAM_model Baz Alınmıştır
% https://github.com/clum/YouTube/blob/main/Controls28/RCAM_model.m

%-----------------------SABİTLER-------------------------------
% Araç Sabitleri
m    = 2.5;                          % Toplam Kütle (kg)
cbar = 0.3;                          % Ortalama Aerodinamik Kord (m)
S    = 0.38;                         % Kanat Alanı (m^2)
b    = 1.3;                          % Kanat Açıklığı (m)
AR   = b^2 / S;                      % Aspect Ratio (~4.45)

Xcg = 0.18; Ycg = 0; Zcg = 0.015;    % Body Frame CoG Pozisyonu (m)
Xac = 0.206; Yac = 0; Zac = 0.03;    % Body Frame Aerodinamik Center Pozisyonu (m)

% Motor Değerleri
Umax = 30;                           % Maksimum Motor İtkisi (N)
Xapt = 0.18; Yapt = 0; Zapt = 0;     % Body Frame Motor İtki Kuvvetinin Konumu (m)

% Çevresel Sabitler
rho = 1.225;                         % Hava Yoğunluğu
g   = 9.81;                          % Yerçekimi İvmesi (m/s^2)

%------------------STATE VE GİRDİ TANIMLARI----------------------
x1 = X(1); x2 = X(2); x3 = X(3);     % u, v, w (m/s)
x4 = X(4); x5 = X(5); x6 = X(6);     % p, q, r (rad/s)
x7 = X(7); x8 = X(8); x9 = X(9);     % phi, theta, psi (rad)

u_l = U(1);                          % Sol Elevon Girdisi  [-1, 1]
u_r = U(2);                          % Sağ Elevon Girdisi  [-1, 1]

u1 = u_l - u_r;                      % Elevon -> Aileron
u2 = u_l + u_r;                      % Elevon -> Elevator
u3 = 0;                              % Rudder Olmadığından 0 Alındı
u4 = U(3);                           % Motor Gazı [0, 1]

%------------------------HIZ VE AÇILAR---------------------------
Va = sqrt(x1^2 + x2^2 + x3^2);       % Araç Hızı
Va = max(Va, 1e-3);                  % Div by Zero Koruması

alpha = atan2(x3, x1);               % Angle of Attack (rad)
% beta = asin(x2 / Va);              % Angle of Sideslip (rad), ihmal

Q = 0.5 * rho * Va^2;                % Dinamik Hava Basıncı

% State'lerin Vektör Halinde Yazılması
wbe_b = [x4; x5; x6];
V_b   = [x1; x2; x3];

%---------------------AERODİNAMİK KATSAYILAR---------------------
% Lift
% CL_a = 2*pi*AR / (2 + sqrt(4 + AR^2)) ~ 4.10/rad
% Ref: USAF DATCOM Sec 4.1.3; Pamadi Ch. 3
CL0      = 0.0428;                   % Zero-alpha Lift 
CL_alpha = 5.10 * alpha;             % Lift-Curve Etkisi
CL       = CL0 + CL_alpha;

% Drag
% Oswald e = 0.78 (Brandt et al., 2004)
e_oswald = 0.78;
CD0      = 0.0321;                   % Zero-lift Drag (Beard & McLain, 2012)
CD       = CD0 + CL^2 / (AR * e_oswald * pi);

% Side Force
CY = 0;                              % İhmal, Gövde Simetrik, Rudder Yok

% Stability -> Body
FA_s = [-CD * Q * S;
         CY * Q * S;
        -CL * Q * S];

C_bs = [cos(alpha)  0  -sin(alpha);
             0      1       0;
        sin(alpha)  0   cos(alpha)];

FA_b = C_bs * FA_s;

%------------------AERODİNAMİK MOMENT KATSAYILARI----------------

CMx0 = 0.0;    % Roll Trim Offset
CMy0 = 0.02;   % Pitch Trim Offset
CMz0 = 0.0;    % Yaw Trim Offset

% Roll Momenti (Cl)
% Ref: Beard & McLain Table 4.1; DATCOM Sec 6.1; Pamadi Ch. 6
CMx_p  = -0.42 * (b / (2*Va));            % Roll Damping
CMx_r  =  0.12 * (b / (2*Va));            % Yaw Değişimden Kaynaklı Roll
CMx_da =  0.20;                           % Aileron Effectiveness
CMx    = CMx0 + CMx_p*x4 + CMx_r*x6 + CMx_da*u1;

% Pitch Momenti (Cm)
% Ref: Pamadi Ch. 6; Brandt et al. Ch. 3
CMy_alpha = -0.30;                        % Pitch Static Stability
CMy_q     = -9.0 * (cbar / (2*Va));       % Pitch Damping
CMy_de    = -0.55;                        % Elevator Effectiveness
CMy       = CMy0 + CMy_alpha*alpha + CMy_q*x5 + CMy_de*u2;

% Yaw moment (Cn)
% Ref: Beard & McLain Table 4.1; Pamadi Ch. 6
CMz_r  = -0.12 * (b / (2*Va));            % Yaw Damping
CMz_dr =  0.02;                           % Rudder Effectiveness
CMz    = CMz0 + CMz_r*x6 + CMz_dr*u3;

% Moment Vektörleri
Mac_b = [CMx * Q * S * b;        % Roll
         CMy * Q * S * cbar;     % Pitch
         CMz * Q * S * b];       % Yaw

% CoG'ye Göre Momentler
rcg_b  = [Xcg; Ycg; Zcg];
rac_b  = [Xac; Yac; Zac];
MAcg_b = Mac_b + cross(FA_b, rcg_b - rac_b);

%-------------------------DİĞER DEĞERLER-------------------------
F_engine = Umax * u4;
FE_b     = [F_engine; 0; 0];
% CG Etrafında Motor Momenti (Simetriden Dolayı İhmal Edildi)
% MEcg_b = cross([Xcg-Xapt; Ycg-Yapt; Zcg-Zapt], FE_b);

% Yerçekimi (Body Frame)
g_b  = [-g*sin(x8);
         g*cos(x8)*sin(x7);
         g*cos(x8)*cos(x7)];
Fg_b = m * g_b;

% Eylemsizlik Matrisi
Ib = [ 0.10421065  -0.00104788   0.00123247;
      -0.00104788   0.01710749  -0.00038369;
       0.00123247  -0.00038369   0.11944482];

%-------------------------DURUM TÜREVLERİ-----------------------
% Body Frame Toplam Kuvvet
F_b = Fg_b + FE_b + FA_b;

% Body Frame Öteleme Hareketleri
x1to3dot = (1/m)*F_b - cross(wbe_b, V_b);

% Body Frame Dönme Hareketleri
Mcg_b    = MAcg_b;
x4to6dot = Ib \ (Mcg_b - cross(wbe_b, Ib*wbe_b));

% Euler Kinematiği
H_phi = [1  sin(x7)*tan(x8)  cos(x7)*tan(x8);
         0  cos(x7)          -sin(x7);
         0  sin(x7)/cos(x8)   cos(x7)/cos(x8)];

x7to9dot = H_phi * wbe_b;

% Çıktı
XDOT = [x1to3dot;
        x4to6dot;
        x7to9dot];

end