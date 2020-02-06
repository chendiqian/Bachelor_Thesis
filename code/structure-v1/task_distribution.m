function task_distribution(obj, event)
% simply assign the tasks to robots
% ud = obj.UserData;
global robot task;
% for i = 1:length(ud.task)
%     if ud.task(i).tstate == "new"
%         for j = 1:length(ud.robot)
%             if ud.robot(j).rstate == "idle"
%                 ud.robot(j).rstate = "to_task";
%                 ud.task(i).tstate = "assigned";
%                 fprintf("The %dth task assigned!\n", i);
%                 ud.robot(j).cur_task = i;
%                 ud.robot(j).epoint = ud.task(i).position;
%                 set(obj,'UserData',ud);
%                 return;
%             end
%         end
%     end
% end

for i = 1:length(task)
    if task(i).tstate == "new"
        for j = 1:length(robot)
            if robot(j).rstate == "idle"
                robot(j).rstate = "to_task";
                task(i).tstate = "assigned";
                fprintf("The %dth task is assigned to robot %d!\n", i, j);
                robot(j).cur_task = i;
                robot(j).epoint = task(i).position;
                return;
            end
        end
    end
end

% if the task list is empty, stop the timer
stop(obj); % stop timer
return