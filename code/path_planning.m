function path_planning(obj, ~)
global robot zone task map

% robots to zones re-allocation
replan = zeros(size(zone));
RobotInZone = cell(size(zone));

for i = 1: length(robot)
    for j = 1: numel(zone)
        [row, column] = ind2sub(size(zone), j);
        
        if robot(i).path(1,1) >= zone(row, column).corner(1) && ...
                robot(i).path(1,2) >= zone(row, column).corner(2) && ...
                robot(i).path(1,1) <= zone(row, column).corner(3) && ...
                robot(i).path(1,2) <= zone(row, column).corner(4)
            
            RobotInZone{row, column} = [RobotInZone{row, column}, i];
            robot(i).cur_zone = [row, column];
            break;
        end
    end
end

for j = 1: numel(zone)
    [row, column] = ind2sub(size(zone), j);
    
    if ~isequal(zone(row, column).robot, RobotInZone{row, column})
        replan(row, column) = 1;
        zone(row, column).robot = RobotInZone{row, column};
    end
end

for i = 1: length(robot)
    % always needs to refresh the spoint, in case the zone is replanned
    robot(i).spoint = robot(i).path(1, :);
    robot(i).path_get = 1;
    
    switch robot(i).rstate
        case "idle"
            robot(i).epoint = robot(i).path(1, :);
            
        case "to_task"
            if size(robot(i).path, 1) <= 1
                % needs planning
                replan(robot(i).cur_zone(1), robot(i).cur_zone(2)) = 1;
                robot(i).epoint = robot(i).path(1, :);
                
                if isequal(robot(i).target_zone, robot(i).cur_zone)
                    if isequal(robot(i).path(1, :), robot(i).target_pos)
                        robot(i).rstate = "to_exit";
                        task(robot(i).cur_task).tstate = "to_exit";
                        robot(i).target_zone = task(robot(i).cur_task).exit_zone;
                        map(robot(i).path(1,1), robot(i).path(1,2)) = 0;
                        robot(i).target_pos = task(robot(i).cur_task).exit;
                    else
                        robot(i).epoint = task(robot(i).cur_task).position;
                        robot(i).next_zone = robot(i).cur_zone;
                    end
                    
                elseif ~isequal(robot(i).target_zone, robot(i).cur_zone)
                    find_next_zone(i);
                    
                    if isequal(robot(i).path(1, :), robot(i).epoint)
                        replan = safety_entrance(replan, i);
                    end
                end
            end
            
        case "to_exit"
            if size(robot(i).path, 1) <= 1
                % needs planning
                replan(robot(i).cur_zone(1), robot(i).cur_zone(2)) = 1;
                robot(i).epoint = robot(i).path(1, :);
                
                if isequal(robot(i).target_zone, robot(i).cur_zone)
                    if isequal(robot(i).path(1, :), robot(i).target_pos)
                        robot(i).rstate = "from_exit";
                        task(robot(i).cur_task).tstate = "from_exit";
                        robot(i).target_zone = task(robot(i).cur_task).zone;
                        robot(i).target_pos = task(robot(i).cur_task).position;
                    else
                        if ~(task(robot(i).cur_task).edge == 1 && isequal(robot(i).path(1,:), task(robot(i).cur_task).position))
                            robot(i).epoint = task(robot(i).cur_task).exit;
                            robot(i).next_zone = robot(i).cur_zone;
                        else
                            find_next_zone(i);
                            
                            if isequal(robot(i).path(1, :), robot(i).epoint)
                                replan = safety_entrance(replan, i);
                            end
                        end
                    end
                elseif ~isequal(robot(i).target_zone, robot(i).cur_zone)
                    find_next_zone(i);
                    
                    if isequal(robot(i).path(1, :), robot(i).epoint)
                        replan = safety_entrance(replan, i);
                    end
                end
            end
            
            
        case "from_exit"
            if size(robot(i).path, 1) <= 1
                % needs planning
                replan(robot(i).cur_zone(1), robot(i).cur_zone(2)) = 1;
                robot(i).epoint = robot(i).path(1, :);
                
                if isequal(robot(i).target_zone, robot(i).cur_zone)
                    
                    if isequal(robot(i).path(1, :), robot(i).target_pos)
                        robot(i).rstate = "idle";
                        task(robot(i).cur_task).tstate = "done";
                        map(robot(i).path(1,1), robot(i).path(1,2)) = 1;
                    else
                        if task(robot(i).cur_task).edge == 0
                            robot(i).epoint = task(robot(i).cur_task).position;
                            robot(i).next_zone = robot(i).cur_zone;
                        else
                            find_next_zone(i);
                            
                            if isequal(robot(i).path(1, :), robot(i).epoint)
                                replan = safety_entrance(replan, i);
                            end
                        end
                    end
                else
                    find_next_zone(i);
                    
                    if isequal(robot(i).path(1, :), robot(i).epoint)
                        replan = safety_entrance(replan, i);
                    end
                end
            end
    end
end

% replan the paths of robots in a zone if the robots are changed
for i = 1: size(zone, 1)
    for j = 1: size(zone, 2)
        if replan(i, j) == 1
            fprintf("Planning for zone(%d, %d).\n", i, j);
            MIP(i, j);
        end
    end
end


% if all tasks are done, stop the timer
task_state = [task.tstate];
done_index = strcmp(task_state, "done");
if sum(done_index) == length(done_index)
    fprintf("All tasks finished, stopping the map_plot timer.\n\n");
    stop(obj);
end