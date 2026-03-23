function Trimhane()

clc, clear

addpath(fileparts(fileparts(mfilename('fullpath'))));
% mfilename('fullpath') bu dosyanın tam yolunu verir, 
% fileparts iki kez uygulanınca bir üst klasöre çıkar.

% Parametreler
trim_type = 'level';   % 'level' | 'climb' | 'descent' | 'turn' | 'sideslip'
V_target  = 15;        % Hedef hız [m/s]  (veya trim tipine göre vektör)
%   climb/descent : V_target = [Va, gam_rad]       örn. [15, deg2rad(5)]
%   turn          : V_target = [Va, phi_rad, psi_dot_rad_s]
%   sideslip      : V_target = [Va, beta_rad]

% Pipeline
Z_guess  = build_initial_guess(V_target, trim_type);
filename = sprintf('ATABEY_trim_%s.mat', trim_type);
if isfile(filename)
    prev    = load(filename, 'Z_star');
    Z_guess = prev.Z_star;
    fprintf('Önceki trim verisi yüklendi: %s\n', filename);
end
[Z_star, f0]     = run_optimization(Z_guess, V_target, trim_type);
[X_opt, U_opt]   = parse_solution(Z_star);
evaluate_result(f0, Z_star);
save_trim_solution(Z_star, X_opt, U_opt, f0, trim_type);

end

%%  YEREL FONKSİYONLAR

function Z_guess = build_initial_guess(V_target, trim_type)
% Trim tipine göre başlangıç tahmini oluşturur.

    Va_des = V_target(1);

    X_o    = zeros(9, 1);
    X_o(1) = Va_des;
    U_o    = zeros(3, 1);
    U_o(3) = 0.2;

    switch trim_type
        case 'level'
            X_o(1) = Va_des;
            X_o(8) = 0.05;
            U_o(3) = 0.5;

        case 'climb'
            gam_des = V_target(2);
            X_o(8)  = gam_des + 0.05;
            U_o(3)  = 0.5;

        case 'descent'
            gam_des = V_target(2);
            X_o(8)  = gam_des + 0.02;
            U_o(3)  = 0.1;

        case 'sideslip'
            beta_des = V_target(2);
            X_o(2)   = Va_des * sin(beta_des);
            X_o(8)   = 0.05;

        case 'turn'
            phi_des = V_target(2);
            X_o(7)  = phi_des;
            X_o(8)  = 0.05;
            U_o(3)  = 0.3;

        otherwise
            error('build_initial_guess: Bilinmeyen trim tipi: "%s"', trim_type);
    end

    Z_guess = [X_o; U_o];
end

% ─────────────────────────────────────────────────────────────────────────

function [Z_star, f0] = run_optimization(Z_guess, V_target, trim_type)
% fminsearch() ile trim optimizasyonunu çalıştırır.

    options = optimset( ...
        'Display',     'final', ... % iter, final, off
        'MaxFunEvals', 100000,  ...
        'MaxIter',     50000,   ...
        'TolX',        1e-10,   ...
        'TolFun',      1e-10);

    cost_func = @(Z) cost_ATABEY(Z, V_target, trim_type);

    fprintf('\nATABEY trim optimizasyonu başlıyor  [tip: %s] ...\n', trim_type);
    [Z_star, f0] = fminsearch(cost_func, Z_guess, options);
end

% ─────────────────────────────────────────────────────────────────────────

function [X_opt, U_opt] = parse_solution(Z_star)
% Çözümü durum ve girdi vektörlerine ayırır.

    X_opt = Z_star(1:9);
    U_opt = Z_star(10:12);
end

% ─────────────────────────────────────────────────────────────────────────

function evaluate_result(f0, Z_star)
% Final maliyeti ekrana yazdırır ve başarı kontrolü yapar.

    fprintf('\nFinal Maliyet (f0): %e\n', f0);

    if f0 < 1e-4
        disp('Başarılı: Trim noktası bulundu!');
    else
        disp('Uyarı: Maliyet tam olarak sıfıra inmedi.');
        disp('       Z_guess veya H ağırlık matrisini güncellemeyi dene.');

        % --- Hangi Q bileşeni büyük? ---
        X = Z_star(1:9);
        U = Z_star(10:12);
        xdot   = ATABEY_dynamics(X, U);
        Va     = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
        alpha  = atan2(X(3), X(1));
        gam    = X(8) - alpha;
        Q = [xdot; -Va; gam; X(2); X(7); X(9)];
        labels = {'xdot1','xdot2','xdot3','xdot4','xdot5','xdot6', ...
                  'xdot7','xdot8','xdot9','Va_err','gam','v','phi','psi'};
        fprintf('\n  %-10s  %12s\n', 'Bileşen', 'Q^2 (katkı)');
        fprintf('  %s\n', repmat('-',1,26));
        for i = 1:14
            fprintf('  %-10s  %12.6f\n', labels{i}, Q(i)^2);
        end
    end
end

% ─────────────────────────────────────────────────────────────────────────

function save_trim_solution(Z_star, X_opt, U_opt, f0, trim_type)
% Trim sonuçlarını .mat dosyasına kaydeder.
% Optimizasyon yakınsamadıysa dosya adının başına WIP_ eklenir.

    if f0 < 1e-4
        prefix = '';
    else
        prefix = 'WIP_';
    end
    filename = sprintf('%sATABEY_trim_%s.mat', prefix, trim_type);
    save(filename, 'Z_star', 'X_opt', 'U_opt', 'f0', 'trim_type');
    fprintf('Sonuçlar "%s" dosyasına kaydedildi.\n', filename);
end