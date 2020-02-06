function replan = safety_entrance(replan, i)
global robot zone total_path_len

zone_diff = robot(i).next_zone - robot(i).cur_zone;
% [-1,0], up
% [1, 0], down
% [0, 1], right
% [0, -1], left

all_cur_pos = cellfun(@(x)x(1,:),{robot(zone(robot(i).next_zone(1),robot(i).next_zone(2)).robot).path},'un',0);
all_cur_pos = reshape(cell2mat(all_cur_pos),2,length(zone(robot(i).next_zone(1),robot(i).next_zone(2)).robot))';

[Lia, Locb] = ismember(robot(i).path(1,:) + zone_diff, all_cur_pos, 'rows');

if Lia == 1
    rob = robot(zone(robot(i).next_zone(1),robot(i).next_zone(2)).robot(Locb));
    if rob.rstate ~= "idle"
        if isequal(rob.next_zone, robot(i).cur_zone) && size(rob.path, 1) == 1
            if ~ismember(rob.path(1,:) + zone_diff, all_cur_pos, 'rows')
                rob.path = rob.path(1,:) + zone_diff;
                rob.spoint = rob.path(1,:);
                rob.epoint = rob.spoint;
                robot(zone(robot(i).next_zone(1),robot(i).next_zone(2)).robot(Locb)) = rob;
            end
        else
            return
        end
    else
        if ~ismember(rob.path(1,:) + zone_diff, all_cur_pos, 'rows')
            rob.path = rob.path(1,:) + zone_diff;
            rob.spoint = rob.path(1,:);
            rob.epoint = rob.spoint;
            robot(zone(robot(i).next_zone(1),robot(i).next_zone(2)).robot(Locb)) = rob;
        else
            return
        end
    end
end

total_path_len = total_path_len + 1;

robot(i).path = robot(i).path(1,:) + zone_diff;
robot(i).spoint = robot(i).path(1,:);
robot(i).epoint = robot(i).spoint;

zone(robot(i).next_zone(1), robot(i).next_zone(2)).robot = sort([zone(robot(i).next_zone(1), robot(i).next_zone(2)).robot, i]);

zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).robot(zone(robot(i).cur_zone(1), robot(i).cur_zone(2)).robot == i) = [];
robot(i).cur_zone = robot(i).next_zone;
replan(robot(i).cur_zone(1),robot(i).cur_zone(2)) = 1;