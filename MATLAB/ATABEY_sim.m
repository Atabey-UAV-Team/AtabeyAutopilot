%% Simülasyon Parametreleri
clc; clear; close all;

simTime   = 300;       % Saniye

% Oluşturulmuş Trim Dosyasını Yükle
load("ATABEY_trim_solution.mat")
x0 = XStar;
% x0 = [15;0;0;0;0;0;0;0;0];
u  = UStar;

% Kontrol Sınırları
uServoMax = 20*pi/180;

% Harita Oluşturulması
addpath("scripts");
generateMap();

%% Simülasyon
clc
endResults = sim("ATABEY_Model_sim.slx")

%% Plot
clc; close all;
X_ts = endResults.SimulatedOutputs;
U_ts = endResults.SimulatedInputs;
P_ts = endResults.SimulatedPositions;
time = X_ts.Time;

% 3D timeseries to 2D numeric arrays
X_data = squeeze(permute(X_ts.Data, [3 1 2]));
U_data = reshape(U_ts.Data, U_ts.Length, []);
P_data = squeeze(permute(P_ts.Data, [3 1 2]));

% Girdi
figure('Name', 'Kontrol Girdileri', 'NumberTitle', 'off')
numInputs  = size(U_data, 2);
inputNames = {'Left', 'Right', 'Motor'};
uMin       = [-uServoMax, -uServoMax,  0];
uMax       = [ uServoMax,  uServoMax,  1];
for i = 1:numInputs
    subplot(numInputs, 1, i)
    plot(time, U_data(:,i), 'LineWidth', 1.5)
    hold on
    yline(uMin(i), '--r', 'Min')
    yline(uMax(i), '--g', 'Max')
    grid on
    xlabel('Simülasyon Zamanı [s]')
    ylabel(inputNames{i})
    title(inputNames{i})
    legend('Girdi', 'Min Limit', 'Max Limit')
end

% Durum
figure('Name', 'Sistem Durumları', 'NumberTitle', 'off')
numStates  = size(X_data, 2);
stateNames = {'u', 'v', 'w', 'p', 'q', 'r', '\phi', '\theta', '\psi'};
for i = 1:numStates
    subplot(3, 3, i)
    plot(time, X_data(:,i), 'LineWidth', 1.5)
    grid on
    xlabel('Simülasyon Zamanı [s]')
    ylabel(stateNames{i}, 'Interpreter', 'tex')
    title(stateNames{i},  'Interpreter', 'tex')
end

% Konum
figure('Name', 'Simüle Edilen Pozisyonlar', 'NumberTitle', 'off')
posNames = {'X [m]', 'Y [m]', 'Z [m]'};
for i = 1:3
    subplot(3, 1, i)
    plot(time, P_data(:,i), 'LineWidth', 1.5)
    grid on
    xlabel('Simülasyon Zamanı [s]')
    ylabel(posNames{i})
    title(posNames{i})
end