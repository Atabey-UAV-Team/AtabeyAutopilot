function [F0] = cost_ATABEY(Z, V_target)

    % Durumları (X) ve Girdileri (U) ayır
    X = Z(1:9);
    % Elevon Left, Elevon Right, Throttle
    U = Z(10:12);

    % Modelden türevleri çek
    xdot = ATABEY_dynamics(X, U);

    % Hız (Va) ve Uçuş Yolu Açısı (gamma) hesapla
    Va = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
    alpha = atan2(X(3),X(1));
    gam = X(8) - alpha; % theta - alpha

    % Kısıtlamalar (14x1 Vektör)
    % Steady-State and Level Flight trim'i için ayarlanmıştır
    Q = [xdot;          % 1-9: Türevler sıfır olmalı
         V_target - Va; % 10:  Hedef hava hızına ulaşılmalı
         gam;           % 11:  Düz uçuş (Uçuş yolu açısı sıfır)
         X(2);          % 12:  Yanal hız sıfır olmalı (Sideslip yok)
         X(7);          % 13:  Roll açısı sıfır olmalı (Kanatlar düz)
         X(9)];         % 14:  Yaw açısı sıfır olmalı (Kuzeye gidiş)

    % Ceza Ağırlık Matrisi (Şimdilik her şey eşit)
    H = diag(ones(1,14));

    % Toplam Maliyet
    F0 = Q' * H * Q;
end
