function dynamic_move(obj, event, map)
global robot task direction_mat;
robot_num = length(robot);
task_num = length(task);

plot_map(map);

for i = 1:robot_num
    if robot(i).rstate ~= "idle"
        cur = robot(i).cur_position;
        epoint = robot(i).epoint;
        if isempty(find((cur - epoint) ~= 0, 1))
            % need to change the state
            % continue;  
            if robot(i).rstate == "to_task"
                fprintf("The %dth robot has fetched the %dth task!\n", ...
                    i, robot(i).cur_task);
                task(robot(i).cur_task).tstate = "to_exit";
                robot(i).rstate = "to_exit";
                robot(i).epoint = task(robot(i).cur_task).exit;
                stop(obj);
                
            elseif robot(i).rstate == "from_exit"
                fprintf("The %dth robot has finished the %dth task!\n", ...
                    i, robot(i).cur_task);
                robot(i).rstate = "idle";
                task(robot(i).cur_task).tstate = "done";
            end
        end
        
        if sum(abs(epoint - cur)) <= 1
            next_cur = epoint;
            
        elseif mod(cur(1)-1, 3)==1 && mod(cur(2)-1, 5)==1   % at crossroad
            if epoint(2) > cur(2) && contains(direction_mat(cur(1), cur(2)), 'r')
                next_cur = cur + [0, 1];
            elseif epoint(2) < cur(2) && contains(direction_mat(cur(1), cur(2)), 'l')
                next_cur = cur + [0, -1];
            elseif epoint(1) > cur(1) && contains(direction_mat(cur(1), cur(2)), 'd') % downwards
                next_cur = cur + [1, 0];
            elseif epoint(1) < cur(1) && contains(direction_mat(cur(1), cur(2)), 'u')
                next_cur = cur + [-1, 0];
            else
                % judge which direction has the least robot.
                if contains(direction_mat(cur(1), cur(2)), 'l') && ...
                        map(cur(1), cur(2)-1) ~= -1
                    next_cur = cur + [0, -1];
                elseif contains(direction_mat(cur(1), cur(2)), 'r') && ...
                        map(cur(1), cur(2)+1) ~= -1
                    next_cur = cur + [0, 1];
                elseif contains(direction_mat(cur(1), cur(2)), 'u') && ...
                        map(cur(1)-1, cur(2)) ~= -1
                    next_cur = cur + [-1, 0];
                elseif contains(direction_mat(cur(1), cur(2)), 'd') && ...
                        map(cur(1)+1, cur(2)) ~= -1
                    next_cur = cur + [1, 0];
                end
            end
            
            
        elseif map(cur(1), cur(2)) == 0   % normal road
            if direction_mat(cur(1), cur(2)) == 'l'
                next_cur = cur + [0, -1];
            elseif direction_mat(cur(1), cur(2)) == 'r'
                next_cur = cur + [0, 1];
            elseif direction_mat(cur(1), cur(2)) == 'u'
                next_cur = cur + [-1, 0];
            elseif direction_mat(cur(1), cur(2)) == 'd'
                next_cur = cur + [1, 0];
            end
        elseif map(cur(1), cur(2)) == 1  % in the position of a shelf
            if map(cur(1)-1, cur(2)) ~= 1
                next_cur = cur + [-1, 0];
            elseif map(cur(1)+1, cur(2)) ~= 1
                next_cur = cur + [1, 0];
            end
        end
        
        robot(i).cur_position = next_cur;
    end
end