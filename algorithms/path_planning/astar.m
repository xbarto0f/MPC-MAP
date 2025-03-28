function [path] = astar(read_only_vars, public_vars)

path_start = max(round(public_vars.estimated_pose(1:2)), [1 1])*5;
goal = max(round(read_only_vars.discrete_map.goal), [1 1]);
occupancy_grid = read_only_vars.discrete_map.map;

%% Expand obstacles by 2 (0.4)
ex_by = 2;
expanded_grid = occupancy_grid;
[m, n] = size(occupancy_grid);
for i = 1:m
    for j = 1:n
        if occupancy_grid(i, j) == 1
            for ii = -ex_by:ex_by
                for jj = -ex_by:ex_by
                    xi = i + ii;
                    xj = j + jj;
                    if xi > 0 && xi <= m && xj > 0 && xj <= n
                        expanded_grid(xi, xj) = 1;
                    end
                end
            end
        end
    end
end
occupancy_grid = expanded_grid;
%%
fprintf('Starting A* from [%d, %d] to [%d, %d]\n', path_start(1), path_start(2), goal(1), goal(2));

% Define movement directions (4-way or 8-way)
directions = [0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1];

% Initialize open and closed lists
open_list = []; % [x, y, g, h, f, parent_x, parent_y]
closed_list = containers.Map;

% figure(2);
% clf;
% imagesc(occupancy_grid);
% hold on;
% axis equal;
% colormap(gray);

% Start node
start_node = [path_start, 0, heuristic(path_start, goal), 0 + heuristic(path_start, goal), -1, -1];
open_list = [open_list; start_node];

while ~isempty(open_list)
    % Select node with lowest f-cost
    [~, idx] = min(open_list(:, 5));
    current_node = open_list(idx, :);
    open_list(idx, :) = [];
    
    x = current_node(1);
    y = current_node(2);
    g = current_node(3);
    
    % fprintf('Processing node [%d, %d] with f %.2f\n', x, y, current_node(5));
    % plot(x, y, 'bo'); drawnow;
    
    % Validate indices before accessing closed_list
    if x < 1 || y < 1 || x > size(occupancy_grid, 2) || y > size(occupancy_grid, 1)
        continue;
    end
    
    % Check if node is already visited
    key = sprintf('%d,%d', x, y);
    if isKey(closed_list, key)
        continue;
    end
    
    closed_list(key) = current_node;
    
    % Check if goal is reached
    if isequal([x, y], goal)
        % fprintf('Goal reached at [%d, %d]\n', x, y);
        path = reconstruct_path(current_node, closed_list);
        path = path/5;
        return;
    end
    
    % Expand neighbors
    for d = 1:size(directions, 1)
        neighbor = [x, y] + directions(d, :);
        
        if is_valid(neighbor, occupancy_grid, closed_list)
            new_g = g + norm(directions(d, :));
            h = heuristic(neighbor, goal);
            f = new_g + h;
            
            % fprintf('Adding neighbor [%d, %d] with f %.2f\n', neighbor(1), neighbor(2), f);
            % plot(neighbor(1), neighbor(2), 'go'); drawnow;
            
            open_list = [open_list; [neighbor, new_g, h, f, x, y]];
        end
    end
end

fprintf('No path found :( \n');
path = [];
end

function h = heuristic(node, goal)
    h = norm(node - goal); % Euclidean distance
end

function valid = is_valid(node, occupancy_grid, closed_list)
    x = node(1);
    y = node(2);
    key = sprintf('%d,%d', x, y);
    
    valid = x > 0 && y > 0 && x <= size(occupancy_grid, 2) && y <= size(occupancy_grid, 1) ...
            && occupancy_grid(y, x) == 0 && ~isKey(closed_list, key);
end

function path = reconstruct_path(current_node, closed_list)
    path = [current_node(1:2)];
    parent_x = current_node(6);
    parent_y = current_node(7);
    
    % fprintf('Reconstructing path\n');
    while parent_x ~= -1 && parent_y ~= -1
        key = sprintf('%d,%d', parent_x, parent_y);
        if ~isKey(closed_list, key)
            fprintf('Parent node [%d, %d] not found in closed list!\n', parent_x, parent_y);
            break;
        end
        current_node = closed_list(key);
        path = [current_node(1:2); path];
        parent_x = current_node(6);
        parent_y = current_node(7);
    end
    % fprintf('Path reconstruction complete.\n');
end

