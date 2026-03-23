%% Simülasyon Parametreleri
clc; clear; close all;

simTime   = 60;       % Saniye
trim_type = 'level';  % 'level' | 'climb' | 'descent' | 'turn' | 'sideslip'

% Trim Yükle
filename = sprintf('ATABEY_trim_%s.mat', trim_type);

if isfile(filename)
    fprintf('Trim verisi yüklendi: %s\n', filename);
    trim = load(filename);
    x0   = trim.X_opt;
    u    = trim.U_opt;
else
    warning('"%s" bulunamadı. Varsayılan başlangıç noktası kullanılıyor.', filename);
            x0 = [15; 0; 0; 0; 0; 0; 0; 0; 0];
            u  = [0; 0; 1];
end

% Kontrol Sınırları
uServoMax = 20*pi/180;
uServoMin = -1*uServoMax;
uMotorMin = 0;
uMotorMax = 1;

%% Simülasyon
clc
endResults = sim("ATABEY_sistem_modeli.slx")

%% Plot
clc; close all;
X_ts = endResults.SimulatedOutputs;
U_ts = endResults.SimulatedInputs;
time = X_ts.Time;

% 3D timeseries to 2D numeric arrays
X_data = squeeze(permute(X_ts.Data, [3 1 2]));
U_data = reshape(U_ts.Data, U_ts.Length, []);
U_data = U_data(:, 1:3);

% Input
figure('Name', sprintf('Kontrol Girdileri [%s]', trim_type), 'NumberTitle', 'off')
numInputs  = size(U_data, 2);
inputNames = {'Aileron', 'Elevator', 'Motor'};
uMin       = [uServoMin, uServoMin, uMotorMin];
uMax       = [uServoMax, uServoMax, uMotorMax];

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

% State
figure('Name', sprintf('Sistem Durumları [%s]', trim_type), 'NumberTitle', 'off')
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