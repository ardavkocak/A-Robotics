%% 10 Robot İçin Harita ve Yol Planlama
clc; clear; close all;

% Harita parametreleri
mapSize = 100;           % Harita boyutu (100x100)
numObstacles = 200;      % Engel sayısı
numRobots = 10;          % Robot sayısı
robotRadius = 2;         % Robot yarıçapı (birim hücre)
wheelBase = 5;           % İki tekerlek merkezi arası mesafe
maxTime = 10;            % Tüm robotların hedefe ulaşması gereken süre (saniye)

% Harita oluştur
map = zeros(mapSize, mapSize);
for i = 1:numObstacles
    x = randi([1, mapSize]);
    y = randi([1, mapSize]);
    map(x, y) = 1; % Engel
end

% Başlangıç ve hedef noktalarını rastgele ata
startPositions = zeros(numRobots, 2);
goalPositions = zeros(numRobots, 2);
for i = 1:numRobots
    while true
        start = [randi([1, mapSize]), randi([1, mapSize])];
        goal = [randi([1, mapSize]), randi([1, mapSize])];
        if map(start(1), start(2)) == 0 && map(goal(1), goal(2)) == 0
            startPositions(i, :) = start;
            goalPositions(i, :) = goal;
            break;
        end
    end
end

% Her robotun toplam yol mesafesini hesapla
pathDistances = zeros(numRobots, 1);
paths = cell(numRobots, 1);
for i = 1:numRobots
    paths{i} = planPath(map, startPositions(i, :), goalPositions(i, :));
    pathDistances(i) = size(paths{i}, 1);
end

% Robot hızlarını maksimum süreye göre ayarla
robotSpeeds = pathDistances / maxTime; % Her robotun toplam mesafesine göre hız

% Haritayı görselleştir
figure('Position', [100, 100, 800, 800]); % Genişletilmiş frame boyutu
imagesc(1 - map); % Arkaplan beyaz, engeller siyah
colormap(gray);
axis equal;
hold on;

% Kare ızgara ekle
for i = 0:mapSize
    plot([0.5, mapSize + 0.5], [i + 0.5, i + 0.5], 'k-', 'LineWidth', 0.5); % Yatay çizgiler
    plot([i + 0.5, i + 0.5], [0.5, mapSize + 0.5], 'k-', 'LineWidth', 0.5); % Dikey çizgiler
end

% Başlangıç ve hedef noktalarını çiz
colors = lines(numRobots); % Robot yolları için renkler
for i = 1:numRobots
    scatter(startPositions(i, 2), startPositions(i, 1), 100, 'filled', 'MarkerFaceColor', 'g'); % Başlangıç noktası
    scatter(goalPositions(i, 2), goalPositions(i, 1), 100, 'filled', 'MarkerFaceColor', 'b'); % Hedef noktası
end
title('Harita ve Robotların Başlangıç/Hedef Konumları');
xlabel('X'); ylabel('Y');

% Robot yollarını görselleştir
for i = 1:numRobots
    if ~isempty(paths{i})
        plot(paths{i}(:, 2), paths{i}(:, 1), '-', 'Color', colors(i, :), 'LineWidth', 2);
    end
end

% Diferansiyel sürüş simülasyonu ve tekerlek hızları hesaplama
[linearSpeeds, angularSpeeds, wheelSpeeds, collisions] = simulateRobots(map, startPositions, paths, robotRadius, wheelBase, colors, robotSpeeds, pathDistances, maxTime);

% Çarpışmaları konsola yazdır
if ~isempty(collisions)
    disp('Çarpışma Noktaları ve Robot Çiftleri:');
    for i = 1:size(collisions, 1)
        fprintf('Çarpışma Noktası: (%d, %d) - Robot %d ve Robot %d\n', collisions(i, 1), collisions(i, 2), collisions(i, 3), collisions(i, 4));
    end
end

% Doğrusal ve açısal hız profillerini çiz
figure('Position', [200, 200, 1000, 800]); % Daha geniş bir pencere boyutu
subplot(2, 1, 1);
for i = 1:numRobots
    plot(linearSpeeds(:, i), 'DisplayName', sprintf('Robot %d - Doğrusal Hız', i));
    hold on;
end
legend;
title('Tüm Robotların Doğrusal Hız Profilleri');
xlabel('Zaman Adımı');
ylabel('Doğrusal Hız (v)');
grid on;

subplot(2, 1, 2);
for i = 1:numRobots
    plot(angularSpeeds(:, i), 'DisplayName', sprintf('Robot %d - Açısal Hız', i));
    hold on;
end
legend;
title('Tüm Robotların Açısal Hız Profilleri');
xlabel('Zaman Adımı');
ylabel('Açısal Hız (\omega)');
grid on;

% Tekerlek hızlarını çiz
figure('Position', [300, 300, 1000, 800]); % Daha geniş bir pencere boyutu
for i = 1:numRobots
    subplot(numRobots, 1, i);
    plot(wheelSpeeds(:, (2*i)-1), 'DisplayName', sprintf('Robot %d - Sol Tekerlek', i));
    hold on;
    plot(wheelSpeeds(:, 2*i), 'DisplayName', sprintf('Robot %d - Sağ Tekerlek', i));
    legend;
    title(sprintf('Robot %d Tekerlek Hızları', i));
    xlabel('Zaman Adımı');
    ylabel('Hız');
    grid on;
end

% Çarpışmaları görselleştir
if ~isempty(collisions)
    figure('Position', [300, 300, 800, 800]); % Genişletilmiş frame boyutu
    imagesc(1 - map); % Arkaplan beyaz, engeller siyah
    colormap(gray);
    axis equal;
    hold on;
    for i = 0:mapSize
        plot([0.5, mapSize + 0.5], [i + 0.5, i + 0.5], 'k-', 'LineWidth', 0.5); % Yatay çizgiler
        plot([i + 0.5, i + 0.5], [0.5, mapSize + 0.5], 'k-', 'LineWidth', 0.5); % Dikey çizgiler
    end
    title('Çarpışma Noktaları');
    xlabel('X'); ylabel('Y');
    for i = 1:size(collisions, 1)
        scatter(collisions(i, 2), collisions(i, 1), 150, 'x', 'LineWidth', 2, 'MarkerEdgeColor', 'r');
    end
end

%% Yardımcı Fonksiyonlar
function path = planPath(map, start, goal)
    % A* algoritması ile yol planlama
    mapSize = size(map, 1);
    openList = start;
    closedList = [];
    gCosts = inf(mapSize, mapSize);
    gCosts(start(1), start(2)) = 0;
    fCosts = inf(mapSize, mapSize);
    fCosts(start(1), start(2)) = heuristic(start, goal);
    cameFrom = zeros(mapSize, mapSize, 2);
    
    while ~isempty(openList)
        [current, idx] = findLowestFCost(openList, fCosts);
        openList(idx, :) = [];
        closedList = [closedList; current];
        
        if isequal(current, goal)
            path = reconstructPath(cameFrom, goal);
            return;
        end
        
        neighbors = getNeighbors(current, map, mapSize);
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            if isMember(neighbor, closedList)
                continue;
            end
            
            tentativeGCost = gCosts(current(1), current(2)) + 1;
            if tentativeGCost < gCosts(neighbor(1), neighbor(2))
                gCosts(neighbor(1), neighbor(2)) = tentativeGCost;
                fCosts(neighbor(1), neighbor(2)) = tentativeGCost + heuristic(neighbor, goal);
                cameFrom(neighbor(1), neighbor(2), :) = current;
                
                if ~isMember(neighbor, openList)
                    openList = [openList; neighbor];
                end
            end
        end
    end
    
    path = [];
end

function h = heuristic(node, goal)
    % Manhattan mesafesi
    h = abs(node(1) - goal(1)) + abs(node(2) - goal(2));
end

function neighbors = getNeighbors(node, map, mapSize)
    % Komşuları bul
    directions = [0 1; 1 0; 0 -1; -1 0];
    neighbors = [];
    for i = 1:size(directions, 1)
        neighbor = node + directions(i, :);
        if neighbor(1) > 0 && neighbor(1) <= mapSize && neighbor(2) > 0 && neighbor(2) <= mapSize
            if map(neighbor(1), neighbor(2)) == 0
                neighbors = [neighbors; neighbor];
            end
        end
    end
end

function isIn = isMember(node, list)
    % Liste kontrolü
    isIn = any(ismember(list, node, 'rows'));
end

function [lowest, idx] = findLowestFCost(openList, fCosts)
    % En düşük F maliyetli düğüm
    lowest = openList(1, :);
    idx = 1;
    minCost = fCosts(lowest(1), lowest(2));
    for i = 2:size(openList, 1)
        node = openList(i, :);
        cost = fCosts(node(1), node(2));
        if cost < minCost
            lowest = node;
            idx = i;
            minCost = cost;
        end
    end
end

function path = reconstructPath(cameFrom, goal)
    % Geri takip ile yol oluştur
    path = goal;
    current = goal;
    while any(cameFrom(current(1), current(2), :))
        current = reshape(cameFrom(current(1), current(2), :), [1, 2]);
        path = [current; path];
    end
end

function [linearSpeeds, angularSpeeds, wheelSpeeds, collisions] = simulateRobots(map, startPositions, paths, robotRadius, wheelBase, colors, robotSpeeds, pathDistances, maxTime)
    % Simülasyon: Robotları diferansiyel sürüşle hareket ettir
    numRobots = size(startPositions, 1);
    robotMarkers = gobjects(numRobots, 1);
    linearSpeeds = zeros(0, numRobots);
    angularSpeeds = zeros(0, numRobots);
    wheelSpeeds = zeros(0, 2 * numRobots);
    collisions = [];
    
    % Robotları başlangıç pozisyonlarına yerleştir
    for i = 1:numRobots
        robotMarkers(i) = plot(startPositions(i, 2), startPositions(i, 1), ...
            'o', 'MarkerSize', 8, 'MarkerFaceColor', colors(i, :));
    end
    
    robotPositions = startPositions; % Mevcut pozisyon
    timeStep = maxTime / max(pathDistances); % Simülasyon adımı
    isGoalReached = false(numRobots, 1);
    
    % Animasyon döngüsü
    while ~all(isGoalReached)
        for i = 1:numRobots
            if ~isempty(paths{i}) && ~isGoalReached(i)
                currentPos = robotPositions(i, :);
                nextPos = paths{i}(1, :);
                
                % Diferansiyel sürüş için doğrusal ve açısal hız hesapla
                dx = nextPos(1) - currentPos(1);
                dy = nextPos(2) - currentPos(2);
                v = pathDistances(i) / maxTime; % Hız mesafeye göre ayarlanır
                omega = atan2(dy, dx);
                
                % Tekerlek hızları hesapla
                vLeft = v - omega * wheelBase / 2;
                vRight = v + omega * wheelBase / 2;
                
                % Robot pozisyonunu güncelle
                robotPositions(i, :) = nextPos;
                paths{i}(1, :) = []; % Geçilen noktayı kaldır
                
                set(robotMarkers(i), 'XData', nextPos(2), 'YData', nextPos(1));
                
                % Çarpışma kontrolü
                for j = 1:numRobots
                    if j ~= i && ~isGoalReached(j)
                        dist = sqrt((robotPositions(j, 1) - robotPositions(i, 1))^2 + (robotPositions(j, 2) - robotPositions(i, 2))^2);
                        if dist < 2 * robotRadius
                            collisions = [collisions; [robotPositions(i, :) i j]];
                        end
                    end
                end
                
                % Hız profillerini kaydet
                linearSpeeds(end + 1, i) = v;
                angularSpeeds(end + 1, i) = omega;
                wheelSpeeds(end + 1, (2*i-1):(2*i)) = [vLeft, vRight];
            else
                isGoalReached(i) = true;
            end
        end
        pause(timeStep);
    end
end
