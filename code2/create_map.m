function [map, map_full] = create_map(carrier_num, carrier_absence_rate, task_num)

% initializing the map with roads, carriers and exits
length = carrier_num * 4 + carrier_num + 1;
hight = carrier_num * 2 + carrier_num + 1;

map = zeros(hight, length);

for i = 1:hight
    % carriers definition
    for j = 1:length
        if mod(i, 3) ~= 1 && mod(j, 5) ~= 1
            map(i, j) = 1;
        end
    end
end

% add enclosure around the map
map = [ones(1, length+5)*(-1);
       ones(hight, 1)*(-1), map, zeros(hight, 2), ones(hight, 1)*0.5, ones(hight, 1)*(-1);
       ones(1, length+5)*(-1)];
fprintf("The map has been created successfully!\n\n");

map_full = map;

% define the exits
[m, n] = size(map);
i = 1:m;
map(mod(i,2)==0, n-1) = 0;

num = carrier_num * carrier_num * 8;
% randomly remove some carriers
for i = 1:m
    for j = 1:n
        if map(i, j) == 1
            if rand() < carrier_absence_rate
                map(i, j) = 0;
                num = num - 1;
                if num <= task_num
                    fprintf("Too few carriers, the carriers are at the fewest possible number.\n");
                    return;
                end
            end
        end
    end
end