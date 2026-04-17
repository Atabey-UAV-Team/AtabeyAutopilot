% MATLAB Klasörü Altında Çalıştırın
clc; clear; close all

simTime = 180;
fcPosRef = [100000 100000 1000];
fcVelRef = [100 10 10];

addpath("Scriptler");
addpath("Güdüm");

% Hedeflenen uçuş hızı
V_target = 20;

%% Flight Controller Katsayıları
Ki_Ye = 0.0025;
Kp_Ye = 0.18;
Kd_Ye = 0.6;

Kp_Ze = -0.3;
Ki_Ze = -0.0001;
Kd_Ze = -0.05;

Kp_psi = 0.7;
Ki_psi = 0.001;

Kp_u = 0.5;
Ki_u = 0.15;

Kp_Xe = 0.125;

Kp_phi = -0.2;
Ki_phi = -0.1;

Ki_theta = -0.8;
Kp_theta = -0.8;

Kq_eta = -0.28;
Ktheta_eta = -1;

Kphi_xi = -0.2;

q_trim = 0;
eta_trim = 0.1;
theta_trim = 10;
xi_trim = 0;
chi_trim = 0;
tau_trim = 0;

%% Trim

% Kayıt dosyası adı. Hazır çözüm varsa onun üzerinden tekrar çözülür.
mat_file = 'ATABEY_trim_solution.mat';

if isfile(mat_file)
    disp('Mevcut trim çözümü bulundu. Z_star başlangıç tahmini olarak yükleniyor.');
    load(mat_file, 'Z_star');
    Z_guess = Z_star;  % Önceki çözümden başlangıç
else
    disp('Kayıtlı trim çözümü bulunamadı. Varsayılan başlangıç tahmini kullanılıyor.');

    % X_o: Durumlar için başlangıç tahmini (9x1)
    X_o = zeros(9, 1);
    X_o(1) = V_target; % İleri hız tahmini
    X_o(8) = 0.05;     % Havada tutunmak için ufak bir başlangıç pitch açısı

    % U_o: Girdiler için başlangıç tahmini
    U_o = zeros(3, 1);
    U_o(4) = 0.2;      % Maks thrust'ın ~%20'si civarında bir başlangıç gaz tahmini

    % Tahminleri Birleştir (12x1)
    Z_guess = [X_o; U_o];
end

% fminsearch Ayarları
options = optimset('Display', 'iter', ...
    'MaxFunEvals', 200000, ...
    'MaxIter',     100000, ...
    'TolX',        1e-6, ...
    'TolFun',      1e-6);

% Fonksiyonu Tanımla
cost_func = @(Z) cost_ATABEY(Z, V_target);

% Optimizasyon
disp('ATABEY için trim optimizasyonu başlıyor...');
[Z_star, f0] = fminsearch(cost_func, Z_guess, options);

% Sonuç kontrolü
fprintf('\nFinal Maliyet (f0): %e\n', f0);
if f0 < 1e-6
    disp('Başarılı: Trim noktası bulundu!');
else
    disp('Uyarı: Maliyet tam olarak sıfıra inmedi. Başlangıç tahminlerini (Z_guess) güncellemeyi dene.');
end

% Optimize edilmiş X ve U değerlerini ayır
XStar = Z_star(1:9);
UStar = Z_star(10:12);

% Sonuçları Kaydet
save(mat_file, 'Z_star', 'XStar', 'UStar', 'f0');
disp('Sonuçlar ATABEY_trim_solution2.mat dosyasına kaydedildi.');