function dynamic_move_debugging(spoint, epoint, map, direction_mat)
cur = spoint;
while true
    plot_map(map);
    % plot a robot
    rectangle('Position', [cur(2)-0.5, cur(1)-0.5, 1, 1], 'Facecolor', 'r');
    pause(0.3);
    
    if isempty(find((cur - epoint) ~= 0, 1))
        break;
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
    
    cur = next_cur;
end