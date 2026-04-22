function generateMap()

clc; close all;

%% Harita Parametreleri
mapSize = 500;           % Harita Tek Doğrultuda Uzunluk
resolution = 10;         % Grid Boyutu

xGrid = 0:resolution:mapSize;
yGrid = 0:resolution:mapSize;

if ~evalin('base','exist(''simulatedMapWaypoint'',''var'')')
    assignin('base','simulatedMapWaypoint',timeseries([mapSize/2 mapSize/2],0));
end

% Harita Oluşturulması
map = zeros(length(xGrid), length(yGrid));

%% Plot
fig = figure('Name','Alan Haritası');
ax = axes('Parent', fig);

imagesc(ax, xGrid, yGrid, map);
set(ax, 'YDir', 'normal');
set(ax, 'XLim', [0 mapSize]);
set(ax, 'YLim', [0 mapSize]);
axis(ax, 'equal');

axis(ax, 'tight');
colormap(gray);

xlabel('X (m)');
ylabel('Y (m)');

hold(ax, 'on');

% Grid Çizgileri
for i = 1:length(xGrid)
    xline(ax, xGrid(i), ':k');
    yline(ax, yGrid(i), ':k');
end

% Orijin (bottom-left = 0,0)
xline(ax, 0, 'r', 'LineWidth', 1.5);
yline(ax, 0, 'r', 'LineWidth', 1.5);

% Waypoint İşareti
hPoint = plot(ax, mapSize/2, mapSize/2, 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% Başlangıç Waypointi (Haritanın Tam Ortası)
simulatedMapWaypoint = timeseries([mapSize/2 mapSize/2], 0);
assignin('base', 'simulatedMapWaypoint', simulatedMapWaypoint);

%% Haritaya Tıklanmasıyla Waypoint Güncellenmesi
fig.WindowButtonDownFcn = @updatePoint;

    function updatePoint(~, ~)

    cp = get(ax, 'CurrentPoint');

    % Grid Noktalarının Tam Üstüne İşareti Koyma
    % Float Değerlere İzin Vermez
    [~, ix] = min(abs(xGrid - cp(1,1)));
    [~, iy] = min(abs(yGrid - cp(1,2)));

    x = xGrid(ix);
    y = yGrid(iy);

    % Harita Sınır Kontrolü
    if x < 0 || x > mapSize || y < 0 || y > mapSize
        return;
    end

    set(hPoint, 'XData', x, 'YData', y);
    drawnow;

    t = now;
    simulatedMapWaypoint = timeseries([x y], t);
    assignin('base','simulatedMapWaypoint',simulatedMapWaypoint);

end

end