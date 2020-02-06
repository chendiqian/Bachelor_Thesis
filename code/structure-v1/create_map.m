function [map, crossroad] = create_map(carrier)
length = carrier * 4 + carrier + 1;
hight = carrier * 2 + carrier + 1;

map = zeros(hight, length);
crossroad = [];
for i = 1:hight
    for j = 1:length
        if mod(i, 3) ~= 1 && mod(j, 5) ~= 1
            map(i, j) = 1;
        elseif mod(i, 3) == 1 && mod(j, 5) == 1
            crossroad = [crossroad; [i, j]]; %#ok<*AGROW>
        end
    end
    if mod(i, 3) == 1
        map(i, length) = 0.5;
    end
end
map = [ones(1, length+2)*(-1); ones(hight, 1)*(-1), map, ones(hight, 1)*(-1); ones(1, length+2)*(-1)];
crossroad = crossroad + ones(size(crossroad, 1), 2);