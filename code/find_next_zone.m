function find_next_zone(i)
global robot zone task

if robot(i).rstate == "idle"
    return
end

if task(robot(i).cur_task).edge == 1 && robot(i).rstate == "to_exit" && isequal(robot(i).path(1,:), task(robot(i).cur_task).position)
    robot(i).next_zone = robot(i).cur_zone - [1,0];
    robot(i).epoint = robot(i).path(1,:);
    return
end

if task(robot(i).cur_task).edge == 1 && robot(i).rstate == "from_exit" && isequal(robot(i).cur_zone, robot(i).target_zone) && ~isequal(robot(i).path(1,:), task(robot(i).cur_task).position)
    robot(i).next_zone = robot(i).cur_zone - [1,0];
    [~, ind] = min(sum(abs(zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).uout - robot(i).path(1,:)), 2));
    robot(i).epoint = zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).uout(ind, :);
    return
end

if task(robot(i).cur_task).edge == 1 && robot(i).rstate == "from_exit" && isequal(robot(i).cur_zone+[1,0], robot(i).target_zone)
    robot(i).next_zone = robot(i).cur_zone + [1,0];
    robot(i).epoint = task(robot(i).cur_task).position - [1,0];
    return
end

select_direction = inf * ones(1, 4);
% up
if robot(i).cur_zone(1) > robot(i).target_zone(1)
    select_direction(1) = length(zone(robot(i).cur_zone(1)-1,robot(i).cur_zone(2)).robot);
end
% right
if robot(i).cur_zone(2) < robot(i).target_zone(2)
    select_direction(2) = length(zone(robot(i).cur_zone(1),robot(i).cur_zone(2)+1).robot);
end
% down
if robot(i).cur_zone(1) < robot(i).target_zone(1)
    select_direction(3) = length(zone(robot(i).cur_zone(1)+1,robot(i).cur_zone(2)).robot);
end
% left
if robot(i).cur_zone(2) > robot(i).target_zone(2)
    select_direction(4) = length(zone(robot(i).cur_zone(1),robot(i).cur_zone(2)-1).robot);
end

[~, ind] = min(select_direction);

switch ind
    case 1   % up
        robot(i).next_zone = robot(i).cur_zone - [1,0];
        
        if robot(i).rstate == "to_task"
            [~, ind] = min(sum(abs(zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).Uout - robot(i).path(1,:)), 2));
            robot(i).epoint = zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).Uout(ind, :);
        else
            % find the nearest exit
            [~, ind] = min(sum(abs(zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).uout - robot(i).path(1,:)), 2));
            robot(i).epoint = zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).uout(ind, :);
        end
        
    case 2   % right
        robot(i).next_zone = robot(i).cur_zone + [0,1];
        
        if robot(i).rstate == "to_task"
            [~, ind] = min(sum(abs(zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).Rout - robot(i).path(1,:)), 2));
            robot(i).epoint = zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).Rout(ind, :);
        else
            [~, ind] = min(sum(abs(zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).rout - robot(i).path(1,:)), 2));
            robot(i).epoint = zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).rout(ind, :);
        end
        
    case 3   % down
        robot(i).next_zone = robot(i).cur_zone + [1,0];
        
        if robot(i).rstate == "to_task"
            [~, ind] = min(sum(abs(zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).Dout - robot(i).path(1,:)), 2));
            robot(i).epoint = zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).Dout(ind, :);
        else
            [~, ind] = min(sum(abs(zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).dout - robot(i).path(1,:)), 2));
            robot(i).epoint = zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).dout(ind, :);
        end
        
    case 4   % left
        robot(i).next_zone = robot(i).cur_zone - [0,1];
        
        if robot(i).rstate == "to_task"
            [~, ind] = min(sum(abs(zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).Lout - robot(i).path(1,:)), 2));
            robot(i).epoint = zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).Lout(ind, :);
        else
            [~, ind] = min(sum(abs(zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).lout - robot(i).path(1,:)), 2));
            robot(i).epoint = zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).lout(ind, :);
        end
end
