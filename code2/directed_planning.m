function directed_planning(i)
global robot direction zone task

dist_to_zone = [-(robot(i).path(1,1) - zone(robot(i).target_zone(1), robot(i).target_zone(2)).corner(1)),... % up edge
    robot(i).path(1,1) - zone(robot(i).target_zone(1), robot(i).target_zone(2)).corner(3),... % down
    -(robot(i).path(1,2) - zone(robot(i).target_zone(1), robot(i).target_zone(2)).corner(2)),... % left
    robot(i).path(1,2) - zone(robot(i).target_zone(1), robot(i).target_zone(2)).corner(4)]; % right

adjacent_edge = find(dist_to_zone == 1);

if ~isequal(robot(i).target_zone, [1,size(zone, 2)])
    if length(adjacent_edge) == 1
        switch adjacent_edge
            case 1 % go down to enter the zone
                %                 % in this case, a robot without the task goes along lanes
                %                 if robot(i).rstate == "to_task" && dist_to_zone(4) <= 0 && dist_to_zone(4) >= -3
                %
                %                 % in this case, allows the robot to enter another zone
                %                 % if robot(i).rstate == "to_task" && sum(direction{robot(i).path(1,1), robot(i).path(1,2)}) == 1
                %                     safety_entrance("down", i);
                %                     return
                %                 elseif robot(i).rstate ~= "to_task"
                if task(robot(i).cur_task).edge == 0
                    % consider whether the task is at the edge of zone
                    if robot(i).path(1,2) == zone(robot(i).target_zone(1),robot(i).target_zone(2)).corner(2)+4
                        safety_entrance("down", i);
                        return
                    end
                else
                    if sum(abs(robot(i).path(1,:)-robot(i).target_pos)) == 1
                        robot(i).path = robot(i).target_pos;
                        return
                    end
                end
                %                 end
                
            case 2 % go up
                %                 if robot(i).rstate == "to_task" && dist_to_zone(3) <= 0 && dist_to_zone(3) >= -3
                %                 % if robot(i).rstate == "to_task" && sum(direction{robot(i).path(1,1), robot(i).path(1,2)}) == 1
                %                     safety_entrance("up", i);
                %                     return
                %                 elseif robot(i).rstate ~= "to_task"
                if task(robot(i).cur_task).edge == 0
                    % consider whether the task is at the edge of zone
                    if robot(i).path(1,2) == zone(robot(i).target_zone(1),robot(i).target_zone(2)).corner(2)+4
                        safety_entrance("up", i);
                        return
                    end
                else
                    if sum(abs(robot(i).path(1,:)-robot(i).target_pos)) == 1
                        robot(i).path = robot(i).target_pos;
                        return
                    end
                end
                %                 end
                
            case 3 % go right
                %                 if robot(i).rstate == "to_task" && dist_to_zone(1) <= 0 && dist_to_zone(1) >= -1
                %                 % if robot(i).rstate == "to_task" && sum(direction{robot(i).path(1,1), robot(i).path(1,2)}) == 1
                %                     safety_entrance("right", i);
                %                     return
                %                 elseif robot(i).rstate ~= "to_task"
                if task(robot(i).cur_task).edge == 0
                    % consider whether the task is at the edge of zone
                    if robot(i).path(1,1) == zone(robot(i).target_zone(1),robot(i).target_zone(2)).corner(1)+2
                        safety_entrance("right", i);
                        return
                    end
                else
                    if sum(abs(robot(i).path(1,:)-robot(i).target_pos)) == 1
                        robot(i).path = robot(i).target_pos;
                        return
                    end
                end
                %                 end
                
            case 4 % go left
                %                 if robot(i).rstate == "to_task" && dist_to_zone(2) <= 0 && dist_to_zone(2) >= -1
                %                 % if robot(i).rstate == "to_task" && sum(direction{robot(i).path(1,1), robot(i).path(1,2)}) == 1
                %                     safety_entrance("left", i);
                %                     return
                %                 elseif robot(i).rstate ~= "to_task"
                %{
                    if task(robot(i).cur_task).edge == 0
                        % consider whether the task is at the edge of zone
                        if robot(i).path(1,1) == zone(robot(i).target_zone(1),robot(i).target_zone(2)).corner(1)+2
                            safety_entrance("left", i);
                            return
                        end
                    else
                        if sum(abs(robot(i).path(1,:)-robot(i).target_pos)) == 1
                            robot(i).path = robot(i).target_pos;
                            return
                        end
                    end
                %}
                % not allowed!
                %                 end
        end
    end
else
    if length(adjacent_edge) == 1  % near the target zone
        if robot(i).path(1,1) == robot(i).target_pos(1)
            safety_entrance("right", i);
            return
        else
            if direction{robot(i).path(1,1), robot(i).path(1,2)}(2) == 1  % must be at the cross, but dont go right
                safety_entrance("left", i);
                return
            end
        end
    end
    % else: the same as below
end

% at lane
if sum(direction{robot(i).path(1,1), robot(i).path(1,2)}) == 1
    switch find(direction{robot(i).path(1,1), robot(i).path(1,2)}==1)
        case 1
            safety_entrance("up", i);
        case 2
            safety_entrance("right", i);
        case 3
            safety_entrance("down", i);
        case 4
            safety_entrance("left", i);
    end
    return
end

% at cross
if sum(direction{robot(i).path(1,1), robot(i).path(1,2)}) == 2
    if isequal(direction{robot(i).path(1,1), robot(i).path(1,2)}, [1,0,1,0]) % up and down
        if robot(i).target_pos(1) >= robot(i).path(1,1) % down
            safety_entrance("down", i);
        else
            safety_entrance("up", i);
        end
    elseif isequal(direction{robot(i).path(1,1), robot(i).path(1,2)}, [0,1,0,1]) % left and right
        if robot(i).target_pos(2) >= robot(i).path(1,2) % right
            safety_entrance("right", i);
        else
            safety_entrance("left", i);
        end
    end
end