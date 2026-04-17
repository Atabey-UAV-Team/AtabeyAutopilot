function [t_out, C_out] = B_Spline(path)
% path: [N x 2] veya [N x 3] A* koordinat matrisi
% C_out: [n x 3] -> [s, x, y] formatında (L1 Controller için hazır)
% t_out: Parametre vektörü

%% 1. Ayarlar ve Ön İşleme
k = 4;      % Kübik spline (derece k-1)
n = 500;    % % Çıktı eğrisinin kaç noktadan oluşacağı.
P = double(path);

% çakışan noktaları temizle
mask = [true; any(diff(P, 1, 1) ~= 0, 2)];
P = P(mask, :);

% [Satır, Sütun] geliyorsa [X, Y] yap (Gerekirse bu satırı sil)
%P = fliplr(P); 

[m, d] = size(P);
if m < k
    % Eğer nokta sayısı dereceden azsa dereceyi düşür
    k = m; 
end

%% 2. Clamped Knot Vector (Düğüm Vektörü)
num_knots = m + k; % toplam nokta sayısı
knots = [zeros(1, k), linspace(0, 1, num_knots - 2*k + 2), ones(1, k)];

%% 3. De Boor Algoritması 
% Noktaları yaklaştırarak pürüzsüz bir nokta üretir.
t_vec = linspace(0, 1, n);
C = zeros(n, d);

for i = 1:n
    ti = t_vec(i);
    
    % Knot span bulma
    if ti == knots(end)
        r = m;
    else
        r = find(knots <= ti, 1, 'last');
        if r > m, r = m; end
    end
    
    % De Boor hesaplama
    idx = (r-k+1):r;
    dp = P(idx, :);
    
    for s_step = 1:(k-1)
        for j = k:-1:(s_step+1)
            i_knot = r - k + j;
            denom = knots(i_knot + k - s_step) - knots(i_knot);
            alpha = 0;
            if denom ~= 0
                alpha = (ti - knots(i_knot)) / denom;
            end
            dp(j, :) = (1 - alpha) * dp(j-1, :) + alpha * dp(j, :);
        end
    end
    C(i, :) = dp(k, :);
end

%% 4. L1 için Yay Uzunluğu (s) Hesaplama
% L1 kontrolcüsü path üzerinde ne kadar ilerlediğini 's' ile takip eder.
% Her noktanın bir önceki noktaya olan kümülatif mesafesini hesaplıyoruz.

segment_lens = sqrt(sum(diff(C, 1, 1).^2, 2));
s_dist = [0; cumsum(segment_lens)]; % Yol boyunca alınan toplam mesafe

% L1 için paketlenmiş çıktı: [s, x, y]
C_out = [s_dist, C(:, 1), C(:, 2)];
t_out = t_vec;

end