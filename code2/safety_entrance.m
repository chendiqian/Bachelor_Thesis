function safety_entrance(arg, i)
global robot total_path_len

switch arg
    case "up"
        for r = 1: length(robot)
            if ~isequal(robot(r).path(1,:), robot(i).path(1,:) - [1,0])
                if size(robot(r).path, 1) > 1 && ~isequal(robot(r).path(2,:), robot(i).path(1,:) - [1,0])
                    continue;
                elseif size(robot(r).path, 1) > 1 && isequal(robot(r).path(2,:), robot(i).path(1,:) - [1,0])
                    return;
                end
            else
                return;
            end
        end
        
        robot(i).path = robot(i).path - [1,0];
        total_path_len = total_path_len + 1;
        
    case "down"
        for r = 1: length(robot)
            if ~isequal(robot(r).path(1,:), robot(i).path(1,:) + [1,0])
                if size(robot(r).path, 1) > 1 && ~isequal(robot(r).path(2,:), robot(i).path(1,:) + [1,0])
                    continue;
                elseif size(robot(r).path, 1) > 1 && isequal(robot(r).path(2,:), robot(i).path(1,:) + [1,0])
                    return;
                end
            else
                return;
            end
        end
        
        robot(i).path = robot(i).path + [1,0];
        total_path_len = total_path_len + 1;
        
    case "left"
        for r = 1: length(robot)
            if ~isequal(robot(r).path(1,:), robot(i).path(1,:) - [0,1])
                if size(robot(r).path, 1) > 1 && ~isequal(robot(r).path(2,:), robot(i).path(1,:) - [0,1])
                    continue;
                elseif size(robot(r).path, 1) > 1 && isequal(robot(r).path(2,:), robot(i).path(1,:) - [0,1])
                    return;
                end
            else
                return;
            end
        end
        
        robot(i).path = robot(i).path - [0,1];
        total_path_len = total_path_len + 1;
        
    case "right"
        for r = 1: length(robot)
            if ~isequal(robot(r).path(1,:), robot(i).path(1,:) + [0,1])
                if size(robot(r).path, 1) > 1 && ~isequal(robot(r).path(2,:), robot(i).path(1,:) + [0,1])
                    continue;
                elseif size(robot(r).path, 1) > 1 && isequal(robot(r).path(2,:), robot(i).path(1,:) + [0,1])
                    return;
                end
            else
                return;
            end
        end
        
        robot(i).path = robot(i).path + [0,1];
        total_path_len = total_path_len + 1;
end