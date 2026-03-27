clc; clear; close all

% Hedeflenen uçuş hızı
V_target = 85;

% Kayıt dosyası adı. Hazır çözüm varsa onun üzerinden tekrar çöz
mat_file = 'RCAM_trim_solution2.mat';

if isfile(mat_file)
    disp('Mevcut trim çözümü bulundu. Z_star başlangıç tahmini olarak yükleniyor...');
    load(mat_file, 'Z_star');
    Z_guess = Z_star;  % Önceki çözümden başlangıç
else
    disp('Kayıtlı trim çözümü bulunamadı. Varsayılan başlangıç tahmini kullanılıyor...');

    % X_o: Durumlar için başlangıç tahmini (9x1)
    X_o = zeros(9, 1);
    X_o(1) = V_target; % İleri hız tahmini
    X_o(8) = 0.05;     % Havada tutunmak için ufak bir başlangıç yunuslama açısı

    % U_o: Girdiler için başlangıç tahmini
    U_o = zeros(4, 1);
    U_o(4) = 0.2;      % Maks thrust'ın ~%20'si civarında bir başlangıç gaz tahmini

    % Tahminleri birleştir (12x1)
    Z_guess = [X_o; U_o];
end

% fminsearch ayarları
options = optimset('Display', 'iter', ...
    'MaxFunEvals', 200000, ...
    'MaxIter',     100000, ...
    'TolX',        1e-6, ...
    'TolFun',      1e-6);

% Parametreli fonksiyonu tanımla
cost_func = @(Z) cost_ATABEY(Z, V_target);

% Optimizasyonu başlat
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
UStar = Z_star(10:14);

% Sonuçları kaydet
save(mat_file, 'Z_star', 'XStar', 'UStar', 'f0');
disp('Sonuçlar ATABEY_trim_solution2.mat dosyasına kaydedildi.');