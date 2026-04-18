function [optimal_paths, path_lengths] = A_Star()
%#codegen

    coder.extrinsic('figure','axes','hold','imagesc','set','colormap','axis',...
                    'xline','yline','plot','text','xlabel','drawnow','pause',...
                    'msgbox','delete','ishandle','fprintf','ginput');

    %% CONSTANTS  (all sizes must be compile-time fixed for codegen)
    mapSize    = 500;
    resolution = 10;
    MAP_W      = 50;   % mapSize / resolution
    MAP_H      = 50;
    MAX_NODES  = (MAP_W + 1) * (MAP_H + 1);   % 2601
    MAX_PATHS  = 20;    % maximum number of trips that can be stored
    MAX_PLEN   = 500;   % maximum waypoints in a single path

    %% FIXED-SIZE OUTPUTS  (replaces cell array)
    optimal_paths = zeros(MAX_PATHS, MAX_PLEN, 2, 'uint16');   % [trip, waypoint, xy]
    path_lengths  = zeros(MAX_PATHS, 1, 'uint16');              % valid waypoints per trip

    MAP = 2 * ones(MAP_W + 1, MAP_H + 1);

    CLOSED      = zeros(MAX_NODES, 2);
    closedCount = 0;
    pathCounter = 0;

    %% Build figure
    fig = figure('Name','Surekli Navigasyon','Color','w');
    ax  = axes('Parent', fig);
    hold(ax, 'on');
    xGrid = 0:resolution:mapSize;
    yGrid = 0:resolution:mapSize;
    imagesc(ax, xGrid, yGrid, zeros(length(xGrid), length(yGrid)));
    set(ax, 'YDir', 'normal');
    colormap(gray);
    axis(ax, 'equal');
    axis(ax, 'tight');
    for i = 0:resolution:mapSize
        xline(ax, i, ':k');
        yline(ax, i, ':k');
    end
    xline(ax, 0, 'r', 'LineWidth', 1.5);
    yline(ax, 0, 'r', 'LineWidth', 1.5);

    xStart = double(MAP_W) / 2;
    yStart = double(MAP_H) / 2;
    plotMarker(ax, xStart, yStart, 'bo', 'Baslangic');

    %% -----------------------------------------------------------------------
    %% OBSTACLE PLACEMENT  (left-click centre, left-click radius, right = done)
    %% -----------------------------------------------------------------------
    msgPrompt(ax, 'ENGEL: Sol tik=merkez, Sol tik=yaricap  |  Sag tik=bitir', 'blue');

    while true
        xc_mx = 0; yc_mx = 0; but_mx = 0;
        [xc_mx, yc_mx, but_mx] = ginput(1);
        if double(but_mx) ~= 1, break; end

        xc_raw = double(xc_mx);
        yc_raw = double(yc_mx);

        msgPrompt(ax, 'Simdi yaricap noktasini secin...', [0.8 0.4 0]);
        xe_mx = 0; ye_mx = 0; but_mx2 = 0;
        [xe_mx, ye_mx, but_mx2] = ginput(1);
        if double(but_mx2) ~= 1, break; end

        r  = sqrt((double(xe_mx) - xc_raw)^2 + (double(ye_mx) - yc_raw)^2) / resolution;
        xc = floor(xc_raw / resolution) + 1;
        yc = floor(yc_raw / resolution) + 1;

        for xi = 1:MAP_W
            for yi = 1:MAP_H
                if (xi - xc)^2 + (yi - yc)^2 <= (r + 2)^2
                    MAP(xi, yi) = -1;
                    plot(ax, (xi-1)*resolution + 5, (yi-1)*resolution + 5, ...
                         'rs', 'MarkerFaceColor', 'r');
                end
            end
        end
        msgPrompt(ax, 'ENGEL: Sol tik=merkez, Sol tik=yaricap  |  Sag tik=bitir', 'blue');
    end

    %% -----------------------------------------------------------------------
    %% MAIN TASK LOOP
    %% -----------------------------------------------------------------------
    while true
        fig_ok = false;
        fig_ok = ishandle(fig);
        if ~logical(double(fig_ok)), break; end
        if pathCounter >= MAX_PATHS, break; end   % storage full

        msgPrompt(ax, 'YENI HEDEF secin  |  Figuru kapat = cikis', [0 0.5 0]);

        ax_ok = false;
        ax_ok = ishandle(ax);
        if ~logical(double(ax_ok)), break; end

        [xTarget, yTarget] = selectPoint(ax, 'Hedef seciliyor...', 'g');
        if xTarget < 0, break; end

        hT = plotMarker(ax, xTarget, yTarget, 'gd', 'Hedef');

        %% A* init
        OPEN      = zeros(MAX_NODES, 8);
        openCount = 0;

        CLOSED_MAP = false(MAP_W + 1, MAP_H + 1);
        CLOSED_MAP(MAP == -1) = true;
        closedCount = 0;

        xNode = xStart; yNode = yStart;
        g0 = 0.0;
        h0 = sqrt((xNode - xTarget)^2 + (yNode - yTarget)^2);
        openCount = openCount + 1;
        OPEN(openCount,:) = insert_open(xNode, yNode, xNode, yNode, g0, h0, g0+h0);
        OPEN(openCount, 1) = 0;
        closedCount = closedCount + 1;
        CLOSED(closedCount,:) = [xNode, yNode];

        %% A* loop
        while ~(xNode == xTarget && yNode == yTarget)
            ni = node_index_fn(OPEN, openCount, xNode, yNode);
            if ni < 0, break; end

            expanded = expand_array(xNode, yNode, OPEN(ni,6), ...
                                    xTarget, yTarget, CLOSED_MAP, MAP_W, MAP_H);
            if isempty(expanded), break; end

            for i = 1:size(expanded, 1)
                matchIdx = findInOpen(OPEN, openCount, expanded(i,1), expanded(i,2));
                if matchIdx > 0
                    if expanded(i,5) < OPEN(matchIdx,8)
                        OPEN(matchIdx,4:8) = [xNode, yNode, expanded(i,3:5)];
                    end
                else
                    openCount = openCount + 1;
                    OPEN(openCount,:) = insert_open(expanded(i,1), expanded(i,2), ...
                                                    xNode, yNode, ...
                                                    expanded(i,3), expanded(i,4), expanded(i,5));
                end
            end

            minIdx = min_fn(OPEN, openCount);
            if minIdx < 0, break; end

            xNode = OPEN(minIdx, 2);
            yNode = OPEN(minIdx, 3);
            OPEN(minIdx, 1) = 0;
            closedCount = closedCount + 1;
            CLOSED(closedCount,:) = [xNode, yNode];
            CLOSED_MAP(xNode, yNode) = true;
        end

        %% Store result
        if xNode == xTarget && yNode == yTarget
            [path, pLen] = tracePath(OPEN, openCount, CLOSED, closedCount, ...
                                     xTarget, yTarget, xStart, yStart, MAX_PLEN);
            animatePath(ax, path, pLen);

            pathCounter = pathCounter + 1;
            %% Write into pre-allocated fixed array — NO cell indexing
            for w = 1:pLen
                optimal_paths(pathCounter, w, 1) = uint16((path(w,1) - 1) * resolution);
                optimal_paths(pathCounter, w, 2) = uint16((path(w,2) - 1) * resolution);
            end
            path_lengths(pathCounter) = uint16(pLen);

            fprintf('>> Rota %d kaydedildi (%d nokta).\n', pathCounter, pLen);

            xStart = xTarget; yStart = yTarget;
            delete(hT);
            plotMarker(ax, xStart, yStart, 'bo', 'Yeni Baslangic');
        else
            msgbox('Yol bulunamadi! Lutfen baska bir hedef secin.');
        end
    end

    %% =========================================================
    %% LOCAL FUNCTIONS
    %% =========================================================

    function [xv, yv] = selectPoint(ax, prompt, col)
        xlabel(ax, prompt, 'Color', col, 'FontWeight', 'bold');
        xv_mx = 0; yv_mx = 0; but_l = 0;
        while double(but_l) ~= 1
            ok = false; ok = ishandle(ax);
            if ~logical(double(ok)), xv = -1.0; yv = -1.0; return; end
            [xv_mx, yv_mx, but_l] = ginput(1);
        end
        xv = max(1.0, min(50.0, floor(double(xv_mx)/10) + 1));
        yv = max(1.0, min(50.0, floor(double(yv_mx)/10) + 1));
    end

    function h = plotMarker(ax, x, y, style, label)
        h = plot(ax,(x-1)*10+5,(y-1)*10+5,style,'MarkerSize',12,'LineWidth',2);
        text(ax,(x-1)*10+7,(y-1)*10+5,label,'FontWeight','bold','Color','w');
    end

    function msgPrompt(ax, txt, col)
        ok = false; ok = ishandle(ax);
        if logical(double(ok)), xlabel(ax,txt,'Color',col,'FontWeight','bold'); drawnow; end
    end

    function row = insert_open(x, y, px, py, g, h, f)
        row = [1.0, x, y, px, py, g, h, f];
    end

    function idx = findInOpen(OPEN, n, x, y)
        idx = 0;
        for k = 1:n
            if OPEN(k,1)==1 && OPEN(k,2)==x && OPEN(k,3)==y, idx=k; return; end
        end
    end

    function ni = node_index_fn(OPEN, n, x, y)
        ni = -1;
        for k = 1:n
            if OPEN(k,2)==x && OPEN(k,3)==y, ni=k; return; end
        end
    end

    function i_min = min_fn(OPEN, n)
        i_min = -1; minF = inf;
        for k = 1:n
            if OPEN(k,1)==1 && OPEN(k,8)<minF, minF=OPEN(k,8); i_min=k; end
        end
    end

    function exp_array = expand_array(nx, ny, gp, xT, yT, CMAP, MX, MY)
        buf = zeros(8, 5); count = 0;
        for dk = -1:1
            for dj = -1:1
                if dk==0 && dj==0, continue; end
                sx = nx+dk; sy = ny+dj;
                if sx<1||sx>MX||sy<1||sy>MY||CMAP(sx,sy), continue; end
                g = gp + sqrt(double(dk*dk + dj*dj));
                h = sqrt((xT-sx)^2 + (yT-sy)^2);
                count = count+1;
                buf(count,:) = [sx, sy, g, h, g+h];
            end
        end
        exp_array = buf(1:count,:);
    end

    function [path, pLen] = tracePath(OPEN, openCount, CLOSED, closedCount, ...
                                      xT, yT, xS, yS, MAX_PLEN)
        path = zeros(MAX_PLEN, 2);
        pLen = 0;
        if closedCount==0 || CLOSED(closedCount,1)~=xT || CLOSED(closedCount,2)~=yT
            return;
        end
        pLen = 1; path(1,:) = [xT, yT];
        ni = node_index_fn(OPEN, openCount, xT, yT);
        px = OPEN(ni,4); py = OPEN(ni,5);
        while (px~=xS || py~=yS) && pLen < MAX_PLEN
            pLen = pLen+1; path(pLen,:) = [px, py];
            ni = node_index_fn(OPEN, openCount, px, py);
            px = OPEN(ni,4); py = OPEN(ni,5);
        end
        pLen = pLen+1; path(pLen,:) = [xS, yS];
        % flip in-place (no flipud on partial array)
        for i = 1:floor(pLen/2)
            tmp = path(i,:);
            path(i,:) = path(pLen-i+1,:);
            path(pLen-i+1,:) = tmp;
        end
    end

    function animatePath(ax, path, pLen)
        if pLen < 1, return; end
        res = 10;
        p = plot(ax,(path(1,1)-1)*res+5,(path(1,2)-1)*res+5,...
                 'bo','MarkerSize',10,'MarkerFaceColor','b');
        for i = 2:pLen
            ok = false; ok = ishandle(ax);
            if ~logical(double(ok)), return; end
            pause(0.05);
            set(p,'XData',(path(i,1)-1)*res+5,'YData',(path(i,2)-1)*res+5);
            drawnow;
        end
        xp = zeros(pLen,1); yp = zeros(pLen,1);
        for i = 1:pLen
            xp(i) = (path(i,1)-1)*res+5;
            yp(i) = (path(i,2)-1)*res+5;
        end
        plot(ax, xp, yp, 'b-', 'LineWidth', 2);
    end

end
