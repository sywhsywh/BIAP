% 环境初始化
clear;
clc;

% 定义环境的参数
x_max = 1000; % 地图行数
y_max = 1000; % 地图列数

% 定义起点和目标点
start_pose = [30, 960]; % 起点位置
goal_pose = [960, 30];  % 目标位置
step = 30;              % 每一步的步长
r = step;               % 每次扩展的步长
numNodes = 10000;       % 最大节点数
neighbor_radius = 50;   % 邻域半径，用于路径优化

% 势场参数
K_att = 0.22;    % 引力系数
K_rep = 500;  % 斥力系数
d_0 = 50;       % 斥力作用范围

% 定义障碍物
obstacle_list = [
    250, 550, 150, 150;  % 障碍物1：位置(250,550)，大小(150,150)
    550, 300, 150, 180;  % 障碍物2：位置(550,300)，大小(150,180)
];

% 定义障碍物的影响范围
or = 20; % 障碍物影响范围
obstacles = zeros(size(obstacle_list)); % 初始化障碍物矩阵
for i = 1:size(obstacle_list, 1)
    % 扩大障碍物范围
    obstacles(i, :) = [obstacle_list(i, 1)-or, obstacle_list(i, 2)-or, ...
                       obstacle_list(i, 3)+2*or, obstacle_list(i, 4)+2*or];
end

% 画图初始化
figure;
hold on;
axis([0 x_max 0 y_max]);
axis on;
box on;
grid on;

% 绘制障碍物（扩大后的范围）
for i = 1:size(obstacle_list, 1)
    rectangle('Position', obstacle_list(i, :), 'FaceColor', [0 0 0]);
end

% 绘制起点和目标点
plot(start_pose(1), start_pose(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10); % 绿色圆点为起点
plot(goal_pose(1), goal_pose(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10); % 红色圆点为目标

% RRT*算法主体
path_V = start_pose; % 维护树的节点
path_E = [];         % 维护树的边
parent_nodes = -1;   % 父节点数组，记录每个节点的父节点索引（初始节点无父节点）
cost = 0;          % 每个节点的代价（起点的代价为0）

% 运行RRT*算法
goal_reached = false; % 是否到达目标
for k = 1:numNodes
    % 1. 在地图中随机生成一个点
    rand_point = [rand() * x_max, rand() * y_max]; % 随机生成点
    
    % 2. 使用人工势场调整随机点
    rand_point = applyArtificialPotentialField(rand_point, goal_pose, obstacles, K_att, K_rep, d_0);
    
    % 3. 找到最近的树节点
    [nearest_node, nearest_idx] = findNearestNode(path_V, rand_point);
    
    % 4. 从最近的节点扩展一个新节点
    new_point = extend(nearest_node, rand_point, r);
    
    % 5. 检查新节点是否与障碍物碰撞
    if isCollision(new_point, nearest_node, obstacles)
        continue;
    end
    
    % 6. 将新节点添加到树中
    path_V = [path_V; new_point];
    parent_nodes = [parent_nodes; nearest_idx]; % 初始父节点为最近节点
    new_node_idx = size(path_V, 1);
    cost = [cost; cost(nearest_idx) + norm(new_point - nearest_node)]; % 计算新节点的代价
    
    % 7. 在邻域范围内优化路径（寻找更优的父节点）
    neighbors = findNeighbors(path_V, new_point, neighbor_radius);
    for i = 1:length(neighbors)
        neighbor_idx = neighbors(i);
        neighbor_node = path_V(neighbor_idx, :);
        
        % 检查是否通过邻近节点连接会更短
        if ~isCollision(new_point, neighbor_node, obstacles)
            new_cost = cost(neighbor_idx) + norm(new_point - neighbor_node);
            if new_cost < cost(new_node_idx)
                % 更新父节点
                parent_nodes(new_node_idx) = neighbor_idx;
                cost(new_node_idx) = new_cost;
            end
        end
    end
    
    % 8. 更新邻居节点的父节点（检查是否可以通过新节点优化）
    for i = 1:length(neighbors)
        neighbor_idx = neighbors(i);
        neighbor_node = path_V(neighbor_idx, :);
        
        % 检查是否通过新节点连接会更短
        if ~isCollision(neighbor_node, new_point, obstacles)
            new_cost = cost(new_node_idx) + norm(neighbor_node - new_point);
            if new_cost < cost(neighbor_idx)
                % 更新邻居节点的父节点
                parent_nodes(neighbor_idx) = new_node_idx;
                cost(neighbor_idx) = new_cost;
            end
        end
    end
    
    % 绘制树的扩展过程
    line([nearest_node(1), new_point(1)], [nearest_node(2), new_point(2)], 'Color', 'm', 'LineWidth', 0.5);
    drawnow;
    
    % 9. 如果新节点接近目标点，则终止
    if norm(new_point - goal_pose) < r
        disp('找到路径!');
        goal_reached = true;
        break;
    end
end

if ~goal_reached
    disp('未能找到路径！');
    return;
end

% 回溯路径
final_node_idx = size(path_V, 1); % 最后一个节点的索引
path = [];
while final_node_idx > 0
    path = [path; path_V(final_node_idx, :)]; % 将当前节点添加到路径
    final_node_idx = parent_nodes(final_node_idx); % 获取父节点的索引
end

% 绘制最终路径
for i = 1:size(path, 1)-1
    line([path(i, 1), path(i+1, 1)], [path(i, 2), path(i+1, 2)], 'Color', 'b', 'LineWidth', 2);
end

% ====================== 子函数 ======================

% 找到最近节点
function [nearest_node, nearest_idx] = findNearestNode(path_V, rand_point)
    distances = vecnorm(path_V - rand_point, 2, 2);
    [~, nearest_idx] = min(distances);
    nearest_node = path_V(nearest_idx, :);
end

% 从最近节点扩展新节点
function new_point = extend(nearest_node, rand_point, step_size)
    direction = rand_point - nearest_node;
    direction = direction / norm(direction); % 单位化方向
    new_point = nearest_node + direction * step_size;
end

% 检查碰撞
function collision = isCollision(new_point, nearest_node, obstacles)
    collision = false;
    for i = 1:size(obstacles, 1)
        obs_x = obstacles(i, 1);
        obs_y = obstacles(i, 2);
        obs_width = obstacles(i, 3);
        obs_height = obstacles(i, 4);
        
        % 判断是否碰撞障碍物
        if (min(nearest_node(1), new_point(1)) < obs_x + obs_width && ...
            max(nearest_node(1), new_point(1)) > obs_x && ...
            min(nearest_node(2), new_point(2)) < obs_y + obs_height && ...
            max(nearest_node(2), new_point(2)) > obs_y)
            collision = true;
            break;
        end
    end
end

% 找到邻域范围内的节点
function neighbors = findNeighbors(path_V, new_point, radius)
    distances = vecnorm(path_V - new_point, 2, 2);
    neighbors = find(distances <= radius);
end

% 人工势场
function adjusted_point = applyArtificialPotentialField(rand_point, goal_pose, obstacles, K_att, K_rep, d_0)
    attractive_force = K_att * (goal_pose - rand_point);
    repulsive_force = [0, 0];
    for i = 1:size(obstacles, 1)
        obs_x = obstacles(i, 1);
        obs_y = obstacles(i, 2);
        obs_width = obstacles(i, 3);
        obs_height = obstacles(i, 4);
        obs_center = [obs_x + obs_width / 2, obs_y + obs_height / 2];
        distance = norm(rand_point - obs_center);
        if distance < d_0
            repulsive_force = repulsive_force + K_rep * (1 / distance - 1 / d_0) * (1 / distance^2) * (rand_point - obs_center) / distance;
        end
    end
    adjusted_point = rand_point + attractive_force + repulsive_force;
end
