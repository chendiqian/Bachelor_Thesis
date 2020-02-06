function enter_road(i)
global robot zone

robot(i).path_get = 0;
corner = [robot(i).path(1,1),robot(i).path(1,2),robot(i).path(1,1),robot(i).path(1,2)] ...
    == zone(robot(i).cur_zone(1),robot(i).cur_zone(2)).corner;

if corner(1) == 1  % upper edge
    robot(i).path = robot(i).path(1,:) - [1, 0];
elseif corner(2) == 1  % left edge
    robot(i).path = robot(i).path(1,:) - [0, 1];
elseif corner(3) == 1  % lower edge
    robot(i).path = robot(i).path(1,:) + [1, 0];
elseif corner(4) == 1  % right edge
    robot(i).path = robot(i).path(1,:) + [0, 1];
end
