function [horinode1, horinode2] = find_hori_node(node)
cur_node = node;
while mod(cur_node(2)-1, 5) ~= 1
    cur_node(2) = cur_node(2) - 1;
end
horinode1 = cur_node;

cur_node = node;
while mod(cur_node(2)-1, 5) ~= 1
    cur_node(2) = cur_node(2) + 1;
end
horinode2 = cur_node;