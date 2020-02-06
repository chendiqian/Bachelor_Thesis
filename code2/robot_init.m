function robot = robot_init(robot_num, last_data, robot_pos)
% rstate: idle, to_task, to_exit, from_exit;
global map zone;
robot_space = zeros(size(map, 1), size(map, 2));

robot = repmat(struct('spoint', {[]}, 'epoint',{[]}, 'rstate', {"idle"}, ...
    'cur_task', {[]}, 'target_pos', {[]}, 'path', {[]}, ...
    'cur_zone', {[]}, 'target_zone', {[]}, 'path_get', 1), ...
    robot_num, 1);

for i = 1:robot_num
    if last_data == 0
        % randomize the initial position of a robot
        position = [unidrnd(size(map,1)-2)+1, unidrnd(size(map,2)-2)+1];
        while robot_space(position(1), position(2)) == 1 || map(position(1), position(2)) ~= 1
            position = [unidrnd(size(map,1)-2)+1, unidrnd(size(map,2)-2)+1];
        end
    else
        position = robot_pos(i, :);
    end
    
    robot(i).path = position;
    robot_space(position(1), position(2)) = 1;
    
    for j = 1: numel(zone)-1
        [row, column] = ind2sub(size(zone), j);
        if robot(i).path(1,1) >= zone(row, column).corner(1) && ...
                robot(i).path(1,2) >= zone(row, column).corner(2) && ...
                robot(i).path(1,1) <= zone(row, column).corner(3) && ...
                robot(i).path(1,2) <= zone(row, column).corner(4)
            
            robot(i).cur_zone = [row, column];
            zone(row, column).robot = [zone(row, column).robot, i];
            break;
        end
    end
end

fprintf("%d robots generated!\n\n", robot_num);