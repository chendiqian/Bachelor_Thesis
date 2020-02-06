function keyreleasefcn(~, evt, t_dyna_plot, t_path_plan, t_task_alloc)
global task map zone

switch evt.Key
    case 'p'
        disp("==================Pause all the threads!===================");
        stop(t_dyna_plot);
        stop(t_path_plan);
        stop(t_task_alloc);
        
    case 'c'
        disp("==================Continue all the threads!================");
        start(t_dyna_plot);
        start(t_path_plan);
        start(t_task_alloc);
        
    case 'a'
        disp("==================Add new tasks===========================");
        [x, y] = find(map == 1);
        empty_shelf = [x, y];
        
        for i = 1: length(task)
            [Lia, Locb] = ismember(task(i).position, empty_shelf, 'rows');
            % Lia: whether, Locb: location in B
            if Lia == 1 && task(i).tstate ~= "done"  % if done, allow it
                empty_shelf(Locb, :) = [];
            end
        end
        
        if size(empty_shelf, 1) <= 0
            disp("No empty shelves for new tasks!")
            return
        end
        
        disp("Available positions as follows: ");
        disp(empty_shelf);
        new_position = input("Select a position for a new task:\n");
        
        new_index = strcmp([task.tstate], "new");
        new = find(new_index ~= 0);
        if isempty(new)
            least_index = length(task)+1;
        else
            least_index = new(1);
        end
        priority = input(strcat("Available priority: ", num2str(least_index), " - ", num2str(length(task)+1), "\n"));
        
        new_task = repmat(struct('tstate', {"new"}, 'position', {[]}, ...
            'exit', {[]}, 'zone', {[]}, 'exit_zone', {[]}, 'executer', {[]}, 'edge', 0), ...
            1, length(task)+1);
        new_task(1:priority-1) = task(1:priority-1);
        new_task(priority+1: end) = task(priority: end);
        new_task(priority).position = new_position;
        
        [exit1, exit2] = find(map == 0.5);
        [~, ind] = min(sum(abs([exit1, exit2] - new_position), 2));
        new_task(priority).exit = [exit1(ind), exit2(ind)];
        
        for j = 1: numel(zone)
            [row, column] = ind2sub(size(zone), j);
            if new_position(1) >= zone(row, column).corner(1) && ...
                    new_position(2) >= zone(row, column).corner(2) && ...
                    new_position(1) <= zone(row, column).corner(3) && ...
                    new_position(2) <= zone(row, column).corner(4)
                new_task(priority).zone = [row, column];
                
            end
            
            if new_task(priority).exit(1) >= zone(row, column).corner(1) && ...
                    new_task(priority).exit(2) >= zone(row, column).corner(2) && ...
                    new_task(priority).exit(1) <= zone(row, column).corner(3) && ...
                    new_task(priority).exit(2) <= zone(row, column).corner(4)
                new_task(priority).exit_zone = [row, column];
            end
        end
        
        if new_task(priority).zone(1) > 1 && ... % not the first row of zones
                new_task(priority).position(1) == zone(new_task(priority).zone(1),new_task(priority).zone(2)).corner(1) && ...  % the top row in a zone
                new_task(priority).position(2) > 3 && ...  % not the first column
                map(new_task(priority).position(1)+1, new_task(priority).position(2)) == 1 % tolerance
            
            if new_task(priority).zone(2) == 1
                if new_task(priority).position(2) ~= zone(new_task(priority).zone(1),new_task(priority).zone(2)).corner(2)+4 && ...
                        new_task(priority).position(2) ~= zone(new_task(priority).zone(1),new_task(priority).zone(2)).corner(2)+6 && ...
                        new_task(priority).position(2) ~= zone(new_task(priority).zone(1),new_task(priority).zone(2)).corner(2)+9
                    new_task(priority).edge = 1;
                end
            elseif new_task(priority).zone(2) > 1
                if new_task(priority).position(2) ~= zone(new_task(priority).zone(1),new_task(priority).zone(2)).corner(2)+3 && ...
                        new_task(priority).position(2) ~= zone(new_task(priority).zone(1),new_task(priority).zone(2)).corner(2)+5 && ...
                        new_task(priority).position(2) ~= zone(new_task(priority).zone(1),new_task(priority).zone(2)).corner(2)+8
                    new_task(priority).edge = 1;
                end
            end
        end
        
        task = new_task;
        disp("Successfully added a new task!");
        
    case 'd'
        new_index = strcmp([task.tstate], "new");
        
        if sum(new_index) == 0
            disp("All tasks at least assigned, not able to delete!");
            return
        end
        
        disp("Unassigned tasks as follows:");
        disp(find(new_index == 1));
        ind = input("Select a new task to delete: \n");
        task(ind) = [];
        
        disp("Successfully deleted a task!");
end