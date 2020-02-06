clear; clc; close all;

%% map and direction initialization
% carrier_num = unidrnd(5);
carrier_num = 4;

[map, crossroad] = create_map(carrier_num);
plot_map(map);
direction_mat = direction_init(map, crossroad);


%% execution
spoint = [unidrnd(size(map,1)-2)+1, unidrnd(size(map,2)-2)+1];
epoint = [unidrnd(size(map,1)-2)+1, unidrnd(size(map,2)-2)+1];
dynamic_move_debugging(spoint, epoint, map, direction_mat);