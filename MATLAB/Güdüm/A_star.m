function optimal_paths = A_Star()
    %% HARITA PARAMETRELERI
    mapSize    = 500;           
    resolution = 10;           
    MAP_W      = mapSize/resolution; 
    MAP_H      = mapSize/resolution;  
    MAP = 2 * ones(MAP_W + 1, MAP_H + 1);
    
    % Rotaları saklayacak nesne
    optimal_paths = {}; 
    pathCounter = 0;
    
    %--------------------------------------------------------------------------
    %% 2. HARITAYI HAZIRLA
    %--------------------------------------------------------------------------
    fig = figure('Name','Sürekli Navigasyon','Color','w');
    ax = axes('Parent', fig); hold(ax, 'on');
    xGrid = 0:resolution:mapSize; yGrid = 0:resolution:mapSize;
    imagesc(ax, xGrid, yGrid, zeros(length(xGrid), length(yGrid)));
    set(ax, 'YDir', 'normal'); colormap(gray); axis(ax, 'equal'); axis(ax, 'tight');
    for i = 0:resolution:mapSize, xline(ax, i, ':k'); yline(ax, i, ':k'); end
    xline(ax, 0, 'r', 'LineWidth', 1.5); yline(ax, 0, 'r', 'LineWidth', 1.5);
    
    %--------------------------------------------------------------------------
    %% 3. İLK BAŞLANGIÇ VE ENGEL SEÇİMİ
    %--------------------------------------------------------------------------
    xStart = MAP_W/2; yStart = MAP_H/2;
    plotMarker(ax, xStart, yStart, 'bo', 'Başlangıç');
    
    msgPrompt(ax, 'Engelleri belirleyin (Bitirmek için sağ tık)', 'blue');
    but = 1;
    while but == 1
        [xc_raw, yc_raw, but] = ginput(1);
        if but ~= 1, break; end
        xc = floor(xc_raw/resolution) + 1; yc = floor(yc_raw/resolution) + 1;
        r_in = inputdlg('Yarıçap:', 'Engel', 1, {'2'});
        if isempty(r_in), continue; end
        r = str2double(r_in{1});
        for xi = 1:MAP_W
            for yi = 1:MAP_H
                if (xi - xc)^2 + (yi - yc)^2 <= (r+2)^2 % +2 buffer
                    MAP(xi, yi) = -1;
                    plot(ax, (xi-1)*resolution + 5, (yi-1)*resolution + 5, 'rs', 'MarkerFaceColor', 'r');
                end
            end
        end
    end
    
    %--------------------------------------------------------------------------
    %% 4. SÜREKLİ GÖREV DÖNGÜSÜ (ANA YAPI)
    %--------------------------------------------------------------------------
    while true
        % Pencere kapandıysa döngüyü kır ve verileri döndür
        if ~ishandle(fig), break; end
        
        msgPrompt(ax, 'YENİ HEDEF seçin (Çıkmak için figürü kapatın)', [0 0.5 0]);
        try
            [xTarget, yTarget] = selectPoint(ax, 'Hedef seçiliyor...', 'g');
        catch
            break; 
        end
        hT = plotMarker(ax, xTarget, yTarget, 'gd', 'Hedef');
    
        % A* VERİ YAPILARINI SIFIRLA
        OPEN = []; 
        CLOSED_MAP = false(MAP_W + 1, MAP_H + 1); 
        CLOSED_MAP(MAP == -1) = true;
        CLOSED = [];
        xNode = xStart; yNode = yStart;
        g0 = 0; h0 = sqrt((xNode-xTarget)^2 + (yNode-yTarget)^2);
        OPEN(1,:) = insert_open(xNode, yNode, xNode, yNode, g0, h0, g0+h0);
        OPEN(1,1) = 0; CLOSED(end+1,:) = [xNode, yNode];
        openCount = 1;
    
        % ANA A* DÖNGÜSÜ
        while ~(xNode == xTarget && yNode == yTarget)
            expanded = expand_array(xNode, yNode, OPEN(node_index(OPEN, xNode, yNode),6), ...
                            xTarget, yTarget, CLOSED_MAP, MAP_W, MAP_H);
            if isempty(expanded), break; end
            for i = 1:size(expanded,1)
                matchIdx = findInOpen(OPEN, openCount, expanded(i,1), expanded(i,2));
                if matchIdx > 0
                    if expanded(i,5) < OPEN(matchIdx,8)
                        OPEN(matchIdx,4:8) = [xNode, yNode, expanded(i,3:5)];
                    end
                else
                    openCount = openCount + 1;
                    OPEN(openCount,:) = insert_open(expanded(i,1), expanded(i,2), xNode, yNode, ...
                                                    expanded(i,3), expanded(i,4), expanded(i,5));
                end
            end
            minIdx = min_fn(OPEN, openCount);
            if minIdx == -1, break; end
            xNode = OPEN(minIdx,2); yNode = OPEN(minIdx,3);
            OPEN(minIdx,1) = 0; 
            CLOSED(end+1,:) = [xNode, yNode];
            CLOSED_MAP(xNode, yNode) = true; 
        end
    
        % SONUÇ VE GÜNCELLEME
        if xNode == xTarget && yNode == yTarget
            path = tracePath(OPEN, openCount, CLOSED, xTarget, yTarget, xStart, yStart);
            animatePath(ax, path); 
            
            % VERİYİ KAYDETME 
            pathCounter = pathCounter + 1;
            optimal_paths{pathCounter} = uint16((path - 1) * resolution); % Metre koordinatları
            fprintf('>> Rota %d kaydedildi.\n', pathCounter);
    
            % Hedef yeni başlangıçta
            xStart = xTarget;
            yStart = yTarget;
            delete(hT); 
            plotMarker(ax, xStart, yStart, 'bo', 'Yeni Başlangıç');
        else
            msgbox('Yol bulunamadı! Lütfen başka bir hedef seçin.');
        end
    end
    
    %==========================================================================
    %% YARDIMCI FONKSIYONLAR
    %==========================================================================
    
    function [xv, yv] = selectPoint(ax, prompt, col)
        xlabel(ax, prompt, 'Color', col, 'FontWeight','bold');
        but = 0;
        while but ~= 1
            if ~ishandle(ax), error('Figür kapatıldı'); end
            [xv, yv, but] = ginput(1);
        end
        res = 10; 
        xv = floor(xv/res) + 1;
        yv = floor(yv/res) + 1;
        xv = max(1, min(50, xv));
        yv = max(1, min(50, yv));
    end
    
    function h = plotMarker(ax, x, y, style, label)
        res = 10;
        h = plot(ax, (x-1)*res + 5, (y-1)*res + 5, style, 'MarkerSize',12, 'LineWidth',2);
        text(ax, (x-1)*res + 7, (y-1)*res + 5, label, 'FontWeight','bold', 'Color', 'w');
    end
    
    function msgPrompt(ax, txt, col)
        if ishandle(ax), xlabel(ax, txt, 'Color', col, 'FontWeight','bold'); drawnow; end
    end
    
    function CLOSED = buildInitialClosed(MAP, W, H)
        CLOSED = [];
        k = 1;
        for i = 1:W
            for j = 1:H
                if MAP(i,j) == -1
                    CLOSED(k,:) = [i, j]; %#ok<AGROW>
                    k = k+1;
                end
            end
        end
        if isempty(CLOSED), CLOSED = [0 0]; end % Hata önleyici
    end
    
    function idx = findInOpen(OPEN, n, x, y)
        matches = find(OPEN(1:n,2) == x & OPEN(1:n,3) == y & OPEN(1:n,1) == 1, 1);
    if isempty(matches)
        idx = 0;
    else
        idx = matches;
    end
end
    
    function path = tracePath(OPEN, ~, CLOSED, xT, yT, xS, yS)
        last = CLOSED(end,:);
        if last(1) ~= xT || last(2) ~= yT
            path = [];
            return;
        end
        path    = [xT, yT];
        ni = node_index(OPEN, xT, yT);
        px = OPEN(ni, 4); py = OPEN(ni, 5);
        while px ~= xS || py ~= yS
            path(end+1,:) = [px, py]; 
            ni = node_index(OPEN, px, py);
            px = OPEN(ni,4); py = OPEN(ni,5);
        end
        path(end+1,:) = [xS, yS];
        path = flipud(path);
    end
    
    function animatePath(ax, path)
        if isempty(path), return; end
        res = 10;
        n = size(path,1);
        p = plot(ax, (path(1,1)-1)*res + 5, (path(1,2)-1)*res + 5, 'bo', ...
                 'MarkerSize',10, 'MarkerFaceColor','b');
        for i = 2:n
            if ~ishandle(ax), return; end
            pause(0.05);
            set(p,'XData',(path(i,1)-1)*res + 5,'YData',(path(i,2)-1)*res + 5);
            drawnow;
        end
        plot(ax, (path(:,1)-1)*res + 5, (path(:,2)-1)*res + 5, 'b-', 'LineWidth',2);
    end
    
    function row = insert_open(x, y, px, py, g, h, f)
        row = [1, x, y, px, py, g, h, f];
    end
    
    function n_index = node_index(OPEN, x, y)
        matches = find(OPEN(:,2) == x & OPEN(:,3) == y, 1);
        if isempty(matches), error('Node bulunamadı'); end
        n_index = matches;
    end
    
    function i_min = min_fn(OPEN, openCount)

        tempOpen = OPEN(1:openCount, :);
        activeIdx = find(tempOpen(:,1) == 1);
        
        if isempty(activeIdx), i_min = -1; return; end
        
       
        [~, localIdx] = min(tempOpen(activeIdx, 8));
        i_min = activeIdx(localIdx);
    end
    
   function exp_array = expand_array(node_x, node_y, g_parent, ...
                                   xTarget, yTarget, CLOSED_MAP, MAX_X, MAX_Y)
        exp_array = [];
        exp_count = 1;
        for dk = -1:1
            for dj = -1:1
                if dk == 0 && dj == 0, continue; end
                sx = node_x + dk; sy = node_y + dj;
                if sx < 1 || sx > MAX_X || sy < 1 || sy > MAX_Y || CLOSED_MAP(sx, sy)
                    continue; 
                end
                
                g = g_parent + sqrt((node_x-sx)^2 + (node_y-sy)^2);
                h = sqrt((xTarget-sx)^2 + (yTarget-sy)^2);
                exp_array(exp_count,:) = [sx, sy, g, h, g+h]; 
                exp_count = exp_count + 1;
            end
        end
    end
end