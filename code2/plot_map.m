function plot_map(obj, ~)
global robot task map zone total_path_len
clf;

imagesc(abs(map));
colormap(flipud(gray));
axis square;
set(gca,'XTick',0.5:size(map,2),'YTick',0.5:size(map,1),...
    'XTickLabel','','YTicklabel','','dataaspect',[1 1 1],...
    'XGrid','on','YGrid','on','GridColor','k','GridAlpha',1)

hold on;

for i = 1 : size(zone, 1)
    for j = 1 : size(zone, 2)
        if ~isempty(zone(i, j).corner)
            rectangle('Position', [zone(i,j).corner(2) - 0.5, zone(i,j).corner(1) - 0.5, ...
                zone(i,j).corner(4) - zone(i,j).corner(2) + 1, zone(i,j).corner(3) - zone(i,j).corner(1) + 1], ...
                'LineWidth', 2, 'EdgeColor','r');
        end
    end
end

for i = 1:length(task)
    if task(i).tstate == "new" || task(i).tstate == "assigned"
        rectangle('Position', [task(i).position(2) - 0.5, task(i).position(1)-0.5, 1, 1], ...
            'Facecolor', 'r');
    %elseif task(i).tstate ~= "done"
        %rectangle('Position', [task(i).position(2) - 0.5, task(i).position(1)-0.5, 1, 1], ...
        %    'Facecolor', 'w');
    end
end

for i = 1:length(robot)
    % clear current step
    if size(robot(i).path, 1) > 1
        robot(i).path(1, :) = [];
        total_path_len = total_path_len + 1;
    end
    
    if robot(i).rstate == "idle"
        plot(robot(i).path(1,2), robot(i).path(1,1), 'go', ...
            'MarkerSize', 6, 'MarkerFaceColor', 'g');
    elseif robot(i).rstate == "to_task"
        plot(robot(i).path(1,2), robot(i).path(1,1), 'mo', ...
            'MarkerSize', 6, 'MarkerFaceColor', 'm');
    elseif robot(i).rstate == "to_exit"
        rectangle('Position', [robot(i).path(1,2) - 0.5, robot(i).path(1,1)-0.5, 1, 1], ...
            'Facecolor', 'r');
    elseif robot(i).rstate == "from_exit"
        rectangle('Position', [robot(i).path(1,2) - 0.5, robot(i).path(1,1)-0.5, 1, 1], ...
            'Facecolor', 'b');
    end
end

% if all tasks are done, stop the timer
task_state = [task.tstate];
done_index = strcmp(task_state, "done");
if sum(done_index) == length(done_index)
    fprintf("All tasks finished, stopping the map_plot timer.\n\n");
    stop(obj);
    return;
end