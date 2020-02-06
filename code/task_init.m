function task = task_init(task_num, last_data, task_pos)
% tstate: new, assigned, to_exit, from_exit, done;
global zone map;
[m, n] = size(map);

shelf_num = length(find(map==1));
if shelf_num < task_num
    fprintf("Warning: too many tasks for the map!\n\n");
    task_num = shelf_num;
end

task_space = zeros(size(map, 1), size(map, 2));

task = repmat(struct('tstate', {"new"}, 'position', {[]}, ...
    'exit', {[]}, 'zone', {[]}, 'exit_zone', {[]}, 'executer', {[]}, 'edge', 0), ...
    1, task_num);

[exit1, exit2] = find(map == 0.5);

for i = 1:task_num
    if last_data == 0
        position = [unidrnd(m-4)+2, unidrnd(n-4)+2];
        while map(position(1), position(2)) ~= 1 || task_space(position(1), position(2)) == 1
            position = [unidrnd(m-4)+2, unidrnd(n-4)+2];
        end
    else
        position = task_pos(i, :);
    end
    
    % position = [3,8+i];
    task(i).position = position;
    task_space(position(1), position(2)) = 1;
    
    [~, ind] = min(sum(abs([exit1, exit2] - position), 2));
    task(i).exit = [exit1(ind), exit2(ind)];
    
    % defining the initial zone accordingly
    for j = 1: numel(zone)
        [row, column] = ind2sub(size(zone), j);
        if position(1) >= zone(row, column).corner(1) && ...
                position(2) >= zone(row, column).corner(2) && ...
                position(1) <= zone(row, column).corner(3) && ...
                position(2) <= zone(row, column).corner(4)
            task(i).zone = [row, column];
        end
        
        if task(i).exit(1) >= zone(row, column).corner(1) && ...
                task(i).exit(2) >= zone(row, column).corner(2) && ...
                task(i).exit(1) <= zone(row, column).corner(3) && ...
                task(i).exit(2) <= zone(row, column).corner(4)
            task(i).exit_zone = [row, column];
        end
    end
    
    if task(i).zone(1) > 1 && ... % not the first row of zones
            task(i).position(1) == zone(task(i).zone(1),task(i).zone(2)).corner(1) && ...  % the top row in a zone
            task(i).position(2) > 3 && ...  % not the first column
            map(task(i).position(1)+1, task(i).position(2)) == 1 % tolerance
        
        if task(i).zone(2) == 1
            if task(i).position(2) ~= zone(task(i).zone(1),task(i).zone(2)).corner(2)+4 && ...
                    task(i).position(2) ~= zone(task(i).zone(1),task(i).zone(2)).corner(2)+6 && ...
                    task(i).position(2) ~= zone(task(i).zone(1),task(i).zone(2)).corner(2)+9
                task(i).edge = 1;
            end
        elseif task(i).zone(2) > 1
            if task(i).position(2) ~= zone(task(i).zone(1),task(i).zone(2)).corner(2)+3 && ...
                    task(i).position(2) ~= zone(task(i).zone(1),task(i).zone(2)).corner(2)+8
                task(i).edge = 1;
            end
        end
    end
end

fprintf("%d tasks generated!\n\n", task_num);