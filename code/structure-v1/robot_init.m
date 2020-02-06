function robot = robot_init(robot_num, map)
% rstate: idle, to_task, to_exit, from_exit; 
robot_space = zeros(size(map, 1), size(map, 2));
for i = 1:robot_num
    robot(i) = struct('spoint',{[]}, 'epoint',{[]}, 'rstate',{"idle"}, ...
        'cur_task', {[]}, 'cur_position', {[]});
    
    position = [unidrnd(size(map,1)-2)+1, unidrnd(size(map,2)-2)+1];
    while robot_space(position(1), position(2)) == 1
        position = [unidrnd(size(map,1)-2)+1, unidrnd(size(map,2)-2)+1];
    end
    robot(i).spoint = position;
    robot(i).cur_position = position;
    robot_space(position(1), position(2)) = 1;
end

fprintf("%d robots generated!\n", robot_num);