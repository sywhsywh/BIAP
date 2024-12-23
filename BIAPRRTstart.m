clear;
clc;
x_max = 1000; 
y_max = 1000; 
start_pose = [30, 960]; 
goal_pose = [960, 30];  
step = 30;            
r = step;               
numNodes = 10000;       
neighbor_radius = 50;   
K_att = 0.22;  
K_rep = 500;    
d_0 = 50;       
obstacle_list = [
    250, 550, 150, 150;  
    550, 300, 150, 180;
];
or = 20; 
obstacles = zeros(size(obstacle_list)); 
for i = 1:size(obstacle_list, 1)
    obstacles(i, :) = [obstacle_list(i, 1)-or, obstacle_list(i, 2)-or, ...
                       obstacle_list(i, 3)+2*or, obstacle_list(i, 4)+2*or];
end
figure;
hold on;
axis([0 x_max 0 y_max]);
axis on;
box on;
grid on;
for i = 1:size(obstacle_list, 1)
    rectangle('Position', obstacles(i, :), 'FaceColor', [0 0 0]);
end
plot(start_pose(1), start_pose(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
plot(goal_pose(1), goal_pose(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
tree1.nodes = start_pose; 
tree1.parent = -1;        
tree1.cost = 0;           
tree2.nodes = goal_pose;  
tree2.parent = -1;        
tree2.cost = 0;           
goal_reached = false; 
counter = 1;
M = struct('cdata', [], 'colormap', []);
for k = 1:numNodes
    rand_point = [rand() * x_max, rand() * y_max];
    rand_point = applyArtificialPotentialField(rand_point, goal_pose, obstacles, K_att, K_rep, d_0);
    [nearest_node1, nearest_idx1] = findNearestNode(tree1.nodes, rand_point);
    new_point1 = extend(nearest_node1, rand_point, r);
    if ~isCollision(new_point1, nearest_node1, obstacles)
        tree1.nodes = [tree1.nodes; new_point1];
        tree1.parent = [tree1.parent; nearest_idx1];
        tree1.cost = [tree1.cost; tree1.cost(nearest_idx1) + norm(new_point1 - nearest_node1)];
        line([nearest_node1(1), new_point1(1)], [nearest_node1(2), new_point1(2)], ...
            'Color', 'm', 'LineWidth', 1);
        M(counter) = getframe;
        counter = counter + 1;
        [nearest_node2, nearest_idx2] = findNearestNode(tree2.nodes, new_point1);
        if ~isPathCollision(new_point1, nearest_node2, obstacles)
            tree2.nodes = [tree2.nodes; nearest_node2];
            tree2.parent = [tree2.parent; nearest_idx2];
            goal_reached = true;
            break;
        end
    end
    [nearest_node2, nearest_idx2] = findNearestNode(tree2.nodes, rand_point);
    new_point2 = extend(nearest_node2, rand_point, r);
    if ~isCollision(new_point2, nearest_node2, obstacles)
        tree2.nodes = [tree2.nodes; new_point2];
        tree2.parent = [tree2.parent; nearest_idx2];
        tree2.cost = [tree2.cost; tree2.cost(nearest_idx2) + norm(new_point2 - nearest_node2)];
        line([nearest_node2(1), new_point2(1)], [nearest_node2(2), new_point2(2)], ...
            'Color', 'c', 'LineWidth', 1);
        M(counter) = getframe;
        counter = counter + 1;
        [nearest_node1, nearest_idx1] = findNearestNode(tree1.nodes, new_point2);
        if ~isPathCollision(new_point2, nearest_node1, obstacles)
            tree1.nodes = [tree1.nodes; nearest_node1];
            tree1.parent = [tree1.parent; nearest_idx1];
            goal_reached = true;
            break;
        end
    end
end
found_connection = false;
for i = 1:size(tree1.nodes, 1)
    for j = 1:size(tree2.nodes, 1)
        if ~isPathCollision(tree1.nodes(i, :), tree2.nodes(j, :), obstacles)
            connect_idx1 = i;
            connect_idx2 = j;
            found_connection = true;
            break;
        end
    end
    if found_connection
        break;
    end
end
function [nearest_node, nearest_idx] = findNearestNode(nodes, point)
    distances = vecnorm(nodes - point, 2, 2);
    [~, nearest_idx] = min(distances);
    nearest_node = nodes(nearest_idx, :);
end
function new_point = extend(nearest_node, rand_point, step_size)
    direction = rand_point - nearest_node;
    direction = direction / norm(direction);
    new_point = nearest_node + direction * step_size;
end
function collision = isCollision(new_point, nearest_node, obstacles)
    collision = isPathCollision(nearest_node, new_point, obstacles);
end
function collision = isPathCollision(point1, point2, obstacles)
    collision = false;
    for i = 1:size(obstacles, 1)
        obs_x = obstacles(i, 1);
        obs_y = obstacles(i, 2);
        obs_width = obstacles(i, 3);
        obs_height = obstacles(i, 4);
        if checkLineRect(point1, point2, obs_x, obs_y, obs_width, obs_height)
            collision = true;
            break;
        end
    end
end
function collision = checkLineRect(p1, p2, rx, ry, rw, rh)
    collision = false;
    edges = [
        rx, ry, rx + rw, ry;
        rx, ry, rx, ry + rh;
        rx + rw, ry, rx + rw, ry + rh;
        rx, ry + rh, rx + rw, ry + rh;
    ];
    for i = 1:size(edges, 1)
        if lineIntersect(p1, p2, edges(i, 1:2), edges(i, 3:4))
            collision = true;
            return;
        end
    end
end
function intersect = lineIntersect(p1, p2, q1, q2)
    intersect = false;
    u = p2 - p1;
    v = q2 - q1;
    w = p1 - q1;
    A = [-u; v]';
    B = w';
    if rank(A) < 2
        return;
    end
    t_s = A \ B;
    t = t_s(1);
    s = t_s(2);
    if t >= 0 && t <= 1 && s >= 0 && s <= 1
        intersect = true;
    end
end
function path = backtracePath(nodes, parent, idx)
    path = [];
    while idx > 0
        path = [path; nodes(idx, :)];
        idx = parent(idx);
    end
end
