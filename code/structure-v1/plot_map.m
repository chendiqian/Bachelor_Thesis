function plot_map(map)
global robot task;
clf;
imagesc(abs(map));
colormap(flipud(gray));
axis square;
set(gca,'XTick',0.5:size(map,2),'YTick',0.5:size(map,1),...
    'XTickLabel','','YTicklabel','','dataaspect',[1 1 1],...
    'XGrid','on','YGrid','on','GridColor','k','GridAlpha',1)

hold on;

for i = 1:length(robot)
    if robot(i).rstate == "idle"
        plot(robot(i).cur_position(2), robot(i).cur_position(1), 'go', ...
            'MarkerSize', 10, 'MarkerFaceColor', 'g');
    elseif robot(i).rstate == "to_task"
        plot(robot(i).cur_position(2), robot(i).cur_position(1), 'ro', ...
            'MarkerSize', 10, 'MarkerFaceColor', 'r');
    elseif robot(i).rstate == "to_exit"
        rectangle('Position', [robot(i).cur_position(2) - 0.5, robot(i).cur_position(1)-0.5, 1, 1], ...
            'Facecolor', 'r');
    elseif robot(i).rstate == "from_exit"
        rectangle('Position', [robot(i).cur_position(2) - 0.5, robot(i).cur_position(1)-0.5, 1, 1], ...
            'Facecolor', 'b');
    end
end

for i = 1:length(task)
    if task(i).tstate == "new" || task(i).tstate == "assigned"
        rectangle('Position', [task(i).position(2) - 0.5, task(i).position(1)-0.5, 1, 1], ...
            'Facecolor', 'r');
    else
        rectangle('Position', [task(i).position(2) - 0.5, task(i).position(1)-0.5, 1, 1], ...
            'Facecolor', 'w');
    end
end