function exit_planning(i)
global robot direction task

if isequal(robot(i).path(1,:), task(robot(i).cur_task).exit)
    robot(i).rstate = "from_exit";
    task(robot(i).cur_task).tstate = "from_exit";
    robot(i).target_zone = task(robot(i).cur_task).zone;
    robot(i).target_pos = task(robot(i).cur_task).position;
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
end