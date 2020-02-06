function replan = undirected_planning(i, replan)
global robot task zone map

% always needs to refresh the spoint, in case the zone is replanned
robot(i).spoint = robot(i).path(1, :);

switch robot(i).rstate
    case "idle"
        robot(i).epoint = robot(i).path(1, :);
        
    case "to_task"
        if size(robot(i).path, 1) <= 1
            % needs planning
            replan(robot(i).cur_zone(1), robot(i).cur_zone(2)) = 1;
            
            if isequal(robot(i).target_zone, robot(i).cur_zone)
                if isequal(robot(i).path(1, :), robot(i).target_pos)
                    robot(i).rstate = "to_exit";
                    task(robot(i).cur_task).tstate = "to_exit";
                    robot(i).target_pos = task(robot(i).cur_task).exit;
                    robot(i).target_zone = [1, size(zone, 2)];
                    find_exit(i);
                    map(task(robot(i).cur_task).position(1), task(robot(i).cur_task).position(2)) = 0;
                else
                    robot(i).epoint = robot(i).target_pos;
                end
            else
                find_exit(i);
                if isequal(robot(i).path(1, :), robot(i).epoint)
                    enter_road(i);
                end
            end
        end
        
    case "to_exit"
        if size(robot(i).path, 1) <= 1
            % needs planning
            replan(robot(i).cur_zone(1), robot(i).cur_zone(2)) = 1;
            
            find_exit(i);
            if isequal(robot(i).path(1, :), robot(i).epoint)
                enter_road(i);
            end
        end
        
    case "from_exit"
        if size(robot(i).path, 1) <= 1
            % needs planning
            replan(robot(i).cur_zone(1), robot(i).cur_zone(2)) = 1;
            
            if isequal(robot(i).path(1, :), task(robot(i).cur_task).position)
                robot(i).rstate = "idle";
                task(robot(i).cur_task).tstate = "done";
                map(task(robot(i).cur_task).position(1), task(robot(i).cur_task).position(2)) = 1;
            else
                robot(i).epoint = robot(i).target_pos;
            end
            
        end
end