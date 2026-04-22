function [phi_cmd, outDebug] = L1_Controller(States, X_NAV, chi, pathInput)

% Girişler:
x = X_NAV(1);                                   % X Pozisyonu
y = X_NAV(2);                                   % Y Pozisyonu
% chi                                           % Yönelim (rad)
Vg = sqrt(States(1)^2+States(2)^2+States(3)^2); % Yere göre hız (m/s)
s_vec = pathInput(1);                           % Path parametre vektörü
xs = pathInput(2); ys = pathInput(3);           % Path koordinatları

% Sabitler:
L1 = 1;            % Lookahead distance
phi_max = 20/57;   % Maksimum roll açısı (rad)
Vmin = 10;         % Minimum hız 

% Çıkışlar:
% phi_cmd     : Roll komutu (rad)
% debug[] =
    % eta         : Heading hatası (rad) (debug)
    % s_star      : Path üzerindeki en yakın nokta parametresi (debug)
    % s_target    : Lookahead noktası parametresi (debug)
    % a_lat       : Lateral ivme (m/s^2) (debug)
    % dist        : Path'e olan mesafe (debug)


% 1 - SPEED CLAMP

if Vg < Vmin
    Vg = Vmin;
end

% 2 - CLOSEST POINT (Projection)

min_dist = 1e9;
s_star = s_vec(1);

for i = 1:length(s_vec)
    % Araç ve path noktası arasındaki mesafe hesaplanması
    dx = xss - xs(i);
    dy = y - ys(i);

    % Kare mesafe (Karekök almamayı tercih ettim performans için)
    d = dx*dx + dy*dy;
    
    % Minimum mesafe güncellenmesi
    if d < min_dist
        min_dist = d;
        s_star = s_vec(i);
    end
end

% 3 - LOOKAHEAD POINT

s_target = s_star + L1;

% Path sınırı (boundary condition)
if s_target > s_vec(end)
    s_target = s_vec(end);
end

% Lineer İnterpolasyon (s_target tam grid olmayabilir. Ara değeri alalım)
x_L1 = interp1(s_vec, xs, s_target);
y_L1 = interp1(s_vec, ys, s_target);

% 4 - VECTOR COMPUTATION

% L1 - UAV vektörü
L1x = x_L1 - x;
L1y = y_L1 - y;

% UAV'nin yere göre hız vektörü
Vgx = Vg * cos(chi);
Vgy = Vg * sin(chi);

% 5 - ETA ANGLE

% UAV güncel yön ve aslında olması gereken yön arasındaki açı farkı
cross = Vgx * L1y - Vgy * L1x;
dot   = Vgx * L1x + Vgy * L1y;
eta = atan2(cross, dot);

% 6 - L1 GUIDANCE

a_lat = 2 * Vg^2 / L1 * sin(eta);


% 7 - ROLL COMMAND

% a_lat değeri roll açısına döndürülür.
g = 9.81;
phi_cmd = atan(a_lat / g);


% 8 - SATURATION

% Verilen komutların uçağın fiziksel sınırları arasında kalmasını
% garantilemek için gerekli kod bloğu
if phi_cmd > phi_max
    phi_cmd = phi_max;
elseif phi_cmd < -phi_max
    phi_cmd = -phi_max;
end

outDebug = [eta, s_star, s_target, a_lat, dist];

end