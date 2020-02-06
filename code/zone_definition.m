function zone = zone_definition(carrier_num, map_full, zone_batch)
global map
zone = repmat(struct('corner', {[]}, 'robot', {[]}, 'submap', {[]}, ...
    ... % exit for robot carrying shelf
    'lout', {[]}, 'rout', {[]}, 'uout', {[]}, 'dout', {[]}, ...
    ... % exit for robot carrying no shelf
    'Lout', {[]}, 'Rout', {[]}, 'Uout', {[]}, 'Dout', {[]}), ...
    carrier_num / zone_batch, carrier_num / zone_batch);

[~,~,rows] = unique(map_full, 'rows');
rows = find(rows == 2);
[~,~,cols] = unique(map_full', 'rows');
cols = find(cols == 2);

for i = 1 : carrier_num / zone_batch
    for j = 1 : carrier_num / zone_batch
        zone(i, j).corner = [rows(2*i-1)+1, cols(2*j-1)+1, rows(2*i+1), cols(2*j+1)];
                  
        if i == 1
            zone(i, j).corner = zone(i, j).corner + [-1, 0, 0, 0];
        end
        
        if j == 1
            zone(i, j).corner = zone(i, j).corner + [0, -1, 0, 0];
        end
        
        if j == carrier_num / zone_batch
            zone(i, j).corner = zone(i, j).corner + [0, 0, 0, 1];
        end
        
        zone(i, j).submap = map(zone(i, j).corner(1):zone(i, j).corner(3), ...
            zone(i, j).corner(2):zone(i, j).corner(4));
        
        
        submap = map_full(zone(i, j).corner(1):zone(i, j).corner(3), ...
            zone(i, j).corner(2):zone(i, j).corner(4));
        
        % row wise
        [~, ~, ic] = unique(submap, 'rows');
        ind = [find(ic == 1), ones(length(find(ic == 1)), 1)];
        zone(i, j).lout = ind(end, :) + zone(i, j).corner(1:2) - [1, 1];
        ind(:, 2) = ind(:, 2) * size(submap, 2);
        zone(i, j).rout = ind(1:(end-1), :) + zone(i, j).corner(1:2) - [1, 1];
        
        ind = [find(ic ~= 1), ones(length(find(ic ~= 1)), 1)];
        zone(i, j).Lout = ind(3:4, :) + zone(i, j).corner(1:2) - [1, 1];
        ind(:, 2) = ind(:, 2) * size(submap, 2);
        zone(i, j).Rout = ind(1:2, :) + zone(i, j).corner(1:2) - [1, 1];
        
        
        % column wise
        [~, ~, ic] = unique(submap', 'rows');
        ind = [ones(length(find(ic == 1)), 1), find(ic == 1)];
        zone(i, j).uout = ind(1:(end-1), :) + zone(i, j).corner(1:2) - [1, 1];
        ind(:, 1) = ind(:, 1) * size(submap, 1);
        zone(i, j).dout = ind(end, :) + zone(i, j).corner(1:2) - [1, 1];
        
        ind = [ones(length(find(ic ~= 1)), 1), find(ic ~= 1)];  % size must be 8 or 9
        zone(i, j).Uout = ind(1:4, :) + zone(i, j).corner(1:2) - [1, 1];
        ind(:, 1) = ind(:, 1) * size(submap, 1);
        zone(i, j).Dout = ind(5:8, :) + zone(i, j).corner(1:2) - [1, 1];
    end
end