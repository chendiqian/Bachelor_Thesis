function path_planning(obj, ~)
global robot zone task

% robots to zones re-allocation
replan = zeros(size(zone));
RobotInZone = cell(size(zone));

for i = 1: length(robot)
    robot(i).cur_zone = [0, 0]; % on the road
    
    for j = 1: size(zone,1) * (size(zone,2)-1) + 1
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
    robot(i).path_get = 1;
    if isequal(robot(i).cur_zone, [0, 0]) % on the road
        directed_planning(i);
    elseif isequal(robot(i).cur_zone, [1,size(zone, 2)]) % in the exit zone
        exit_planning(i);
    else
        replan = undirected_planning(i, replan); % in zone with goods
    end
end

% replan the paths of robots in a zone if the robots are changed
for i = 1: size(zone, 1)
    for j = 1: size(zone, 2)-1
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