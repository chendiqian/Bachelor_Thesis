function find_exit(i)
global robot zone task

%{
if robot(i).target_zone(2) > robot(i).cur_zone(2)  % go right
    if robot(i).rstate == "to_task"
        robot(i).epoint = [robot(i).path(1,1), zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner(4)];
    else  % if rstate == "to_exit"
        if task(robot(i).cur_task).edge == 0
            robot(i).epoint = [zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner(1)+2, ...
                zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner(4)];
        else
            robot(i).epoint = task(robot(i).cur_task).position;
        end
    end
    return
elseif robot(i).target_zone(2) < robot(i).cur_zone(2)  % go left
    if robot(i).rstate == "to_task"
        robot(i).epoint = [robot(i).path(1,1), zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner(2)];
    else  % if rstate == "to_exit"
        if task(robot(i).cur_task).edge == 0
            robot(i).epoint = [zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner(1)+2, ...
                zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner(2)];
        else
            robot(i).epoint = task(robot(i).cur_task).position;
        end
    end
    return
end


if robot(i).target_zone(1) > robot(i).cur_zone(1)  % go down
    if robot(i).rstate == "to_task"
        robot(i).epoint = [zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner(3), robot(i).path(1,2)];
    else  % if rstate == "to_exit"
        if task(robot(i).cur_task).edge == 0
            robot(i).epoint = [zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner(3), ...
                zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner(2)+4];
        else
            robot(i).epoint = task(robot(i).cur_task).position;
        end
    end
elseif robot(i).target_zone(1) < robot(i).cur_zone(1)  % go up
    if robot(i).rstate == "to_task"
        robot(i).epoint = [zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner(1), robot(i).path(1,2)];
    else  % if rstate == "to_exit"
        if task(robot(i).cur_task).edge == 0
            robot(i).epoint = [zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner(1), ...
                zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner(2)+4];
        else
            robot(i).epoint = task(robot(i).cur_task).position;
        end
    end
end
%}

cur_zone = zone(robot(i).cur_zone(1), robot(i).cur_zone(2));
if robot(i).rstate == "to_task"
    if robot(i).target_zone(2) > robot(i).cur_zone(2)  % go right
        [~, p] = min(sum(abs(cur_zone.Rout - robot(i).path), 2));
        robot(i).epoint = cur_zone.Rout(p, :);
    elseif robot(i).target_zone(2) < robot(i).cur_zone(2)  % go left
        [~, p] = min(sum(abs(cur_zone.Lout - robot(i).path), 2));
        robot(i).epoint = cur_zone.Lout(p, :);
    elseif robot(i).target_zone(1) > robot(i).cur_zone(1)  % go down
        [~, p] = min(sum(abs(cur_zone.Dout - robot(i).path), 2));
        robot(i).epoint = cur_zone.Dout(p, :);
    elseif robot(i).target_zone(1) < robot(i).cur_zone(1)  % go up
        [~, p] = min(sum(abs(cur_zone.Uout - robot(i).path), 2));
        robot(i).epoint = cur_zone.Uout(p, :);
    end
    
else  % if rstate == "to_exit"
    if task(robot(i).cur_task).edge == 0
        robot(i).epoint = [cur_zone.corner(1)+2, cur_zone.corner(4)];
    else
        robot(i).epoint = task(robot(i).cur_task).position;
    end
end