function direction_mat = direction_init(map, crossroad)
str = "";
direction_mat = str(ones(size(map, 1), size(map, 2)), :);
direction_mat = reshape(direction_mat, size(map, 1), size(map, 2));

for i = 1:size(crossroad, 1)
    if mod(sum(crossroad(i, :)),2) == 0
        direction_mat(crossroad(i, 1), crossroad(i, 2)) = "lr";
    elseif mod(sum(crossroad(i, :)),2) == 1
        direction_mat(crossroad(i, 1), crossroad(i, 2)) = "ud";
    end
end

for i = 2:size(map, 1)-1
    for j = 2:size(map, 2)-1
        if mod(i-1, 3) ~= 1 && mod(j-1, 5) == 1  % vertical road
            [vernode1, vernode2] = find_ver_node([i, j]);
            if contains(direction_mat(vernode1(1),vernode1(2)), 'd') && ...
                    ~contains(direction_mat(vernode2(1),vernode2(2)), 'u')
                direction_mat(i, j) = 'd';
            elseif ~contains(direction_mat(vernode1(1),vernode1(2)), 'd') && ...
                    contains(direction_mat(vernode2(1),vernode2(2)), 'u')
                direction_mat(i, j) = 'u';
            end
            
        elseif mod(i-1, 3) == 1 && mod(j-1, 5) ~= 1  % horizontal road
            [horinode1, horinode2] = find_hori_node([i, j]);
            if contains(direction_mat(horinode1(1),horinode1(2)), 'r') && ...
                    ~contains(direction_mat(horinode2(1),horinode2(2)), 'l')
                direction_mat(i, j) = 'r';
            elseif ~contains(direction_mat(horinode1(1),horinode1(2)), 'r') && ...
                    contains(direction_mat(horinode2(1),horinode2(2)), 'l')
                direction_mat(i, j) = 'l';
            end
            
        end
    end
end