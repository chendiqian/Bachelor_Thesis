function [zone, direction] = ZoneDirection(carrier_num, fullmap, zone_batch)
global map;
zone = repmat(struct('corner', {[]}, 'robot', {[]}, 'submap', {[]}, ...
    'Lout', {[]}, 'Rout', {[]}, 'Uout', {[]}, 'Dout', {[]}), ...
    carrier_num / zone_batch, carrier_num / zone_batch + 1);

[~,~,rows] = unique(fullmap, 'rows');
rows = find(rows == 2);
[~,~,cols] = unique(fullmap', 'rows');
cols = find(cols == 2);

% zone definition
for i = 1 : carrier_num / zone_batch
    for j = 1 : carrier_num / zone_batch
        zone(i, j).corner = [rows(2*i-1)+1, cols(2*j-1)+1, rows(2*i+1)-1, cols(2*j+1)-1];
        
        zone(i, j).submap = map(zone(i, j).corner(1):zone(i, j).corner(3), ...
            zone(i, j).corner(2):zone(i, j).corner(4));
        
        zone(i, j).Lout = [zone(i, j).corner(1)+1, zone(i, j).corner(2);
            zone(i, j).corner(3)-1, zone(i, j).corner(2)];
        
        zone(i, j).Rout = [zone(i, j).corner(1)+1, zone(i, j).corner(4);
            zone(i, j).corner(3)-1, zone(i, j).corner(4)];
        
        zone(i, j).Uout = [zone(i, j).corner(1), zone(i, j).corner(2)+3;
            zone(i, j).corner(1), zone(i, j).corner(2)+5];
        
        zone(i, j).Dout = [zone(i, j).corner(3), zone(i, j).corner(2)+3;
            zone(i, j).corner(3), zone(i, j).corner(2)+5];
    end
end

% exit_zone
zone(1, end).corner = [2,size(map,2)-3,size(map,1)-1,size(map,2)-1];
zone(1, end).submap = map(zone(1, end).corner(1):zone(1, end).corner(3), ...
    zone(1, end).corner(2):zone(1, end).corner(4));

% direction definition
r_ind = 1:2:length(rows);
rows = rows(r_ind);
c_ind = 1:2:length(cols);
cols = cols(c_ind);
[x, y] = meshgrid(rows, cols(1:(end-1))); % the last column is in the exits
cross = [x(:), y(:)];
cross_norm = [(cross(:,1)-2)/6+1, (cross(:,2)-2)/10+1]; % normalization

direction = cell(size(map));
% 1 up; 2 right; 3 down; 4 left
for i = 1: size(cross, 1)
    if mod(sum(cross_norm(i, :)), 2) == 0
        direction{cross(i,1), cross(i,2)} = [1, 0, 1, 0];
    else
        direction{cross(i,1), cross(i,2)} = [0, 1, 0, 1];
    end
end

for i = 2: size(direction,1)-1
    for j = 2: size(direction,2)-4
        if isempty(direction{i,j})
            if ~isempty(direction{i-1,j})
                if isequal(direction{i-1,j}, [1,0,1,0])
                    direction{i,j} = [0,0,1,0];
                elseif isequal(direction{i-1,j}, [0,1,0,1])
                    direction{i,j} = [1,0,0,0];
                elseif isequal(direction{i-1,j}, [0,0,1,0]) || isequal(direction{i-1,j}, [1,0,0,0])
                    direction{i,j} = direction{i-1,j};
                end
            end
            
            if ~isempty(direction{i,j-1})
                if isequal(direction{i,j-1}, [0,1,0,1])
                    direction{i,j} = [0,1,0,0];
                elseif isequal(direction{i,j-1}, [1,0,1,0])
                    direction{i,j} = [0,0,0,1];
                elseif isequal(direction{i,j-1}, [0,1,0,0]) || isequal(direction{i,j-1}, [0,0,0,1])
                    direction{i,j} = direction{i,j-1};
                end
            end
        end
    end
end

[x, y] = ind2sub(size(map), find(map==0.5));
exit = [x, y];
for i = 1: size(exit, 1)
    direction{exit(i,1), exit(i,2)} = [0,0,1,0];
    direction(exit(i,1), exit(i,2)-2:exit(i,2)-1) = {[0,1,0,0]};
    direction([exit(i,1)-1,exit(i,1)+1], exit(i,2)-2:exit(i,2)) = {[0,0,0,1]};
end