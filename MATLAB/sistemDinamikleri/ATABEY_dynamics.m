function XDOT = ATABEY_dynamics(X, U)

%-----------------------SABİTLER-------------------------------
m = 2.72;                     % Toplam kütle (kg)
cbar = 0.30;                  % Ortalama aerodinamik kord
lt = 0.35;                    % AC'ye göre Kuyruk - gövde arası mesafe
S = 0.40;                     % Kanat alanı 
St = 0.345;                   % Kuyruk alanı 

Xcg = 0.20; Ycg = 0; Zcg = 0.02;    % Fm'de CG konumları
Xac = 0.22; Yac = 0; Zac = 0;       % Fm'de AC konumları

% Motor
%%
Umax  = 0;                     % Veri elimizde yok
%%
Xapt = 0.50;                   % Motor Fm'e göre x pozisyonu
Yapt = 0;                      % Motor Fm'e göre y pozisyonu
Zapt = 0;                      % Motor Fm'e göre z pozisyonu

% Diğer değişkenler
rho = 1.225;                   % Hava yoğunluğu
g = 9.81;                      % Yerçekimi 
depsda = 0.25;                 % Alpha'ya göre downwash değişimi 
alpha_L0 = -0.035;             % 0 lift hücum açısı
n = 4.8;                       % Lift eğrisi lineer bölgedeki eğim 

%---------------------------VEKTÖR TANIMLAMALARI---------------------------
x1 = X(1); x2 = X(2); x3 = X(3);
x4 = X(4); x5 = X(5); x6 = X(6);
x7 = X(7); x8 = X(8); x9 = X(9);

u1 = U(1); u2 = U(2); u3 = U(3); u4 = U(4);

%---------------DEĞİŞKEN DEĞERLER------------------------
Va = sqrt(x1^2 + x2^2 + x3^2);  % Hız
alpha = atan2(x3,x1);
beta = asin(max(min(x2/Va,1),-1));
Q = 0.5*rho*Va^2;               % Dinamik basınç

wbe_b = [x4;x5;x6];
V_b = [x1;x2;x3];

%---------------AERODİNAMİK KUVVET KATSAYILARI----------------
CL_wb = n*(alpha - alpha_L0);
epsilon = depsda*(alpha - alpha_L0);
alpha_t = alpha - epsilon + u2 + 1.3*x5*lt/Va;
CL_t = 3.1*(St/S)*alpha_t;
CL = CL_wb + CL_t;
CD =  0.13 + 0.07*(5.5*alpha + 0.654)^2;
CY = -0.85*beta + 0.24*u3;

%--------------VEKTÖREL AERODİNAMİK KUVVETLER---------------------
FA_s = [-CD*Q*S;
         CY*Q*S;
        -CL*Q*S];
    
C_bs = [cos(alpha) 0 -sin(alpha);
        0 1 0;
        sin(alpha) 0 cos(alpha)];
    
FA_b = C_bs*FA_s;

%--------------AC ETRAFINDA AERODİNAMİK MOMENTLER-------------------
eta11 = -1.4*beta;
eta21 = -0.59 - (3.1*(St*lt)/(S*cbar))*(alpha - epsilon);
eta31 = (1 - alpha*(180/(15*pi)))*beta;

eta = [eta11; eta21; eta31];

dCMdx = (cbar/Va)*[-11 0 5;
                    0 (-4.03*(St*lt^2)/(S*cbar^2)) 0;
                    1.7 0 -11.5];
              
dCMdu = [-0.6 0 0.22;
          0 (-3.1*(St*lt)/(S*cbar)) 0;
          0 0 -0.63];

CMac_b = eta + dCMdx*wbe_b + dCMdu*[u1;u2;u3];

MAac_b = CMac_b*Q*S*cbar;

C_sb = C_bs';
Mac_b = CMac_b*Q*S*cbar;
Mac_s = C_sb*Mac_b;
CMac_s = Mac_s./(Q*S*cbar);
CMac_s = C_sb*CMac_b;

%--------------CG ETRAFINDA AERODİNAMİK MOMENTLER-------------------
rcg_b = [Xcg;Ycg;Zcg];
rac_b = [Xac;Yac;Zac];
MAcg_b = C_bs*Mac_s + cross(FA_b,rcg_b - rac_b);

%-----------------MOTOR KUVVET VE MOMENTİ----------------------------
F_engine = u4*Umax;                   
FE_b = [F_engine;0;0];

MEcg_b = cross([Xcg - Xapt; Ycg - Yapt; Zcg - Zapt], FE_b);

%--------------------YERÇEKİMİ--------------------------------
g_b = [-g*sin(x8);
        g*cos(x8)*sin(x7);
        g*cos(x8)*cos(x7)];
  
Fg_b = m*g_b;

%-------------------DURUM TÜREVLERİ------------------------------
Ib = m*[0.476 0 -0.109;
        0 1.031 0;
        -0.109 0 1.411];

F_b = Fg_b + FE_b + FA_b;
x1to3dot = (1/m)*F_b - cross(wbe_b,V_b);

Mcg_b = MAcg_b + MEcg_b;
x4to6dot = Ib\(Mcg_b - cross(wbe_b,Ib*wbe_b));

H_phi = [1 sin(x7)*tan(x8) cos(x7)*tan(x8);
         0 cos(x7) -sin(x7);
         0 sin(x7)/cos(x8) cos(x7)/cos(x8)];
    
x7to9dot = H_phi*wbe_b;

XDOT = [x1to3dot;
        x4to6dot;
        x7to9dot];

end
