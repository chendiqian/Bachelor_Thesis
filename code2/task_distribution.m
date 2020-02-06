function task_distribution(obj, event)
% simply assign the tasks to the nearest robots
global robot task;

while(1)
    new_index = strcmp([task.tstate], "new");
    if sum(new_index) == 0
        fprintf("All tasks allocated.\n\n");
        stop(obj);
        return;
    end
    
    new = find(new_index ~= 0);
    new = new(1);
    
    robot_state = [robot.rstate];
    idle_index = strcmp(robot_state, "idle");
    
    if sum(idle_index) == 0
        fprintf("All robots busy.\n\n");
        return;
    end
    
    % find the nearest robot
    idle_index = find(idle_index == 1);
    % robot_position = reshape([robot(idle_index).path], 2, length(idle_index))';
    robot_position = zeros(length(idle_index), 2);
    for i = 1: length(idle_index)
        robot_position(i, :) = robot(idle_index(i)).path(1, :);
    end
    
    [~, min_index] = min(sum(abs(robot_position - task(new).position), 2));
    min_index = idle_index(min_index);
    
    robot(min_index).rstate = "to_task";
    task(new).tstate = "assigned";
    fprintf("The %dth task is assigned to robot %d!\n\n", new, min_index);
    robot(min_index).cur_task = new;
    task(new).executer = min_index;
    robot(min_index).target_zone = task(new).zone;
    robot(min_index).target_pos = task(new).position;
end